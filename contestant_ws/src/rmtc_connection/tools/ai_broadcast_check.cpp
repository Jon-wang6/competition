/**
 * @file ai_broadcast_check.cpp
 * @brief 显式运行的广播审计工具：真实模型回复→现有JSON校验→现有策略接收检查。
 *
 * 不注册为CTest，避免普通编译/测试消耗真实API额度。输入是带期望结果的公开感知
 * 测试JSONL；只将frame交给模型，expected不会进入提示词。默认从本进程环境读取
 * DEEPSEEK_API_KEY，不输出密钥、请求头或模型服务原始响应，不接入实际比赛ROS域。
 */
#include "rmtc_connection/ai_client.hpp"

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <stdexcept>

namespace {
using Cells = std::set<std::pair<int, int>>;

/// 读取测试期望的精确格子集合；仅供审计比对，绝不用于比赛中的广播解析。
Cells cells(const rmtc::Json& points) {
  Cells result;
  for (const auto& p : points)
    result.emplace(p.at(0).get<int>(), p.at(1).get<int>());
  return result;
}

/// 对单条广播做一次真实调用，记录结构、时间/区域和策略是否接受，不模拟实际比赛得分。
rmtc::Json check(const rmtc::Json& sample, const rmtc::AiClientConfig& config) {
  const auto frame = rmtc::parse_public_perception(sample.at("frame").dump(), "A");
  const auto reply = rmtc::call_ai(config, frame, 500);
  auto result = reply.to_json();
  result["id"] = sample.at("id");
  result["family"] = sample.at("family");
  result["phase"] = sample.at("phase");
  result["expected"] = sample.at("expected");
  const bool valid = reply.completion_received && !reply.mock && reply.interpretation_valid;
  bool active = reply.constraints && reply.constraints->end_tick > frame.tick;
  bool correct = valid;
  if (sample.at("expected").at("mode") == "timeline") {
    correct = valid && reply.event_understood && reply.constraints && reply.constraints->timeline;
    if (correct) {
      const auto actual = rmtc::encode_ai_event_timeline(*reply.constraints).at("events");
      for (const auto& expected : sample.at("expected").at("events")) {
        bool found = false;
        for (const auto& effect : actual) {
          bool same = true;
          for (auto it = expected.begin(); it != expected.end(); ++it) {
            if (!effect.contains(it.key())) {
              same = false;
              continue;
            }
            if (it.key() == "cells" || it.key() == "points") {
              if (cells(effect.at(it.key())) != cells(it.value()))
                same = false;
            } else if (effect.at(it.key()) != it.value())
              same = false;
          }
          if (same)
            found = true;
        }
        correct = correct && found;
      }
    }
  } else if (sample.at("expected").at("mode") == "danger") {
    const auto& expected = sample.at("expected");
    correct = valid && reply.event_understood && active;
    if (correct) {
      Cells actual;
      for (const auto p : reply.constraints->dangerous_cells)
        actual.emplace(p.x, p.y);
      correct = reply.constraints->begin_tick == expected.at("begin_tick") &&
                reply.constraints->end_tick == expected.at("end_tick") &&
                actual == cells(expected.at("cells"));
    }
  } else if (sample.at("expected").at("mode") == "abstain") {
    correct = valid && !active;
  } else if (sample.at("expected").at("mode") == "optional_danger" && active) {
    const auto& expected = sample.at("expected");
    Cells actual;
    for (const auto p : reply.constraints->dangerous_cells)
      actual.emplace(p.x, p.y);
    correct = valid && reply.constraints->begin_tick == expected.at("begin_tick") &&
              reply.constraints->end_tick == expected.at("end_tick") &&
              actual == cells(expected.at("cells"));
  }
  result["active_constraints"] = active;
  result["expectation_passed"] = correct;
  result["strategy_accepted"] = false;
  if (valid && active) {
    rmtc::Stage2Strategy strategy;
    const auto action = strategy.decide(frame, 500 - frame.tick, &*reply.constraints);
    result["strategy_accepted"] = !strategy.plan().event_pending;
    result["strategy_action"] = action.to_json(frame.tick, frame.session_id);
    result["strategy_goal"] = strategy.plan().goal;
    const auto start = frame.self.position;
    const auto& danger = reply.constraints->dangerous_cells;
    const bool starts_inside = std::any_of(
        danger.begin(), danger.end(), [&](auto p) { return p.x == start.x && p.y == start.y; });
    bool crosses = false;
    if (!starts_inside)
      for (const auto p : strategy.plan().route.path)
        for (const auto d : danger)
          if (p.x == d.x && p.y == d.y)
            crosses = true;
    result["outside_route_crosses_danger"] = crosses;
    result["expectation_passed"] = correct && !crosses && !strategy.plan().event_pending;
  }
  return result;
}
}  // namespace

/// 显式--live才请求真实服务；按高峰价和最大输出预留费用，不重试或覆盖旧账本。
int main(int argc, char** argv) {
  try {
    if ((argc != 4 && argc != 5) || std::string(argv[1]) != "--live")
      throw std::runtime_error("usage: ai_broadcast_check --live CASES.jsonl RESULTS.jsonl [BUDGET_CNY]");
    const double budget = argc == 5 ? std::stod(argv[4]) : 0.5;
    if (!std::isfinite(budget) || budget <= 0 || std::filesystem::exists(argv[3]))
      throw std::runtime_error("invalid budget or existing audit ledger");
    const char* key = std::getenv("DEEPSEEK_API_KEY");
    if (!key || !*key)
      throw std::runtime_error("DEEPSEEK_API_KEY is missing");
    rmtc::AiClientConfig config;
    config.token = key;
    config.url = "https://api.deepseek.com/chat/completions";
    config.model = "deepseek-flash";
    config.service_mode = "official";
    config.timeout_ms = 30000;
    config.connect_timeout_ms = 5000;
    config.json_mode = true;
    config.disable_thinking = true;
    std::ifstream input(argv[2]);
    std::ofstream output(argv[3]);
    if (!input || !output)
      throw std::runtime_error("cannot open audit files");
    std::string line;
    double reserved = 0;
    double estimated_peak = 0;
    std::size_t calls = 0;
    while (std::getline(input, line)) {
      if (line.empty())
        continue;
      const auto sample = rmtc::Json::parse(line);
      const auto frame = rmtc::parse_public_perception(sample.at("frame").dump(), "A");
      const auto request = rmtc::build_ai_request(frame, 500, config.model,
                                                 config.json_mode, config.disable_thinking);
      // 2026-10-28官方Flash高峰价：输入2元/百万，输出8元/百万。
      // 用两倍UTF-8字节数再加1024协议token估计输入上界，禁用思考、工具与重试。
      // 超时也不退预留，避免服务端已生成却没有返回usage时漏计。
      const double reserve = rmtc::ai_request_reserved_cny(request);
      if (reserved + reserve > budget) {
        std::cout << rmtc::Json{{"event", "budget_stop"}, {"calls", calls},
                                 {"reserved_upper_cny", reserved}, {"budget_cny", budget}}
                         .dump() << std::endl;
        break;
      }
      reserved += reserve;
      auto result = check(sample, config);
      ++calls;
      const auto& usage = result.at("usage");
      const double cost = usage.contains("prompt_tokens") && usage.contains("completion_tokens")
          ? (usage.at("prompt_tokens").get<double>() * 2.0 +
              usage.at("completion_tokens").get<double>() * 8.0) / 1e6 : reserve;
      estimated_peak += cost;
      result["budget_cny"] = budget;
      result["reserved_upper_cny"] = reserve;
      result["reserved_total_cny"] = reserved;
      result["estimated_peak_cny"] = cost;
      result["estimated_peak_total_cny"] = estimated_peak;
      output << result.dump() << '\n';
      output.flush();
      std::cout << rmtc::Json{{"id", result["id"]},
                              {"http", result["http_status"]},
                              {"pass", result["expectation_passed"]},
                              {"ms", result["elapsed_ms"]}}
                       .dump()
                << std::endl;
    }
    return 0;
  } catch (const std::exception&) {
    std::cerr << "Broadcast audit failed; check input schema and local configuration; no "
                 "credentials printed.\n";
    return 1;
  }
}
