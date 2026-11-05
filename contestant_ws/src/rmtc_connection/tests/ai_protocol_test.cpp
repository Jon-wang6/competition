/**
 * @file ai_protocol_test.cpp
 * @brief 检查公开提示词、模拟隔离、模型格式异常及危险区域校验，不调用真实模型。
 *
 * 固定回复只作为离线协议夹具，不能作为比赛中的自然语言事件解析器。
 *
 * @details 模块协作与维护约定
 * 验证对象：ai_client.hpp。
 * 运行约定：由独立测试可执行文件/CTest执行，断言失败用于定位回归，不是正式选手的逐帧逻辑。
 * 结果边界：通过表示这些构造场景满足约束，不证明所有随机地图均最优或稳定达到某个总分。
 */
#include "rmtc_connection/ai_client.hpp"

#include <iostream>
#include <stdexcept>

namespace {

/// 条件失败时给出协议原因，Release构建也不关闭校验。
void require(bool condition, const std::string& why) {
  if (!condition)
    throw std::runtime_error(why);
}

/// 构造仅含本队公开状态的输入；公告内容故意不对应任何关键词分支。
rmtc::PublicPerception sample() {
  rmtc::PublicPerception p;
  p.session_id = "ai-test";
  p.tick = 10;
  p.map = {9, 9};
  p.vision_radius = 4;
  p.self.team = "A";
  p.self.position = {1, 1};
  p.self.alive = true;
  p.self.hp = 100;
  p.event_text = "测试公告原文，由模型解释，执行器只检查JSON";
  p.mission_text = "动态任务";
  return p;
}

/// 将测试JSON包装为标准Chat Completions格式；不代表真实LLM服务。
rmtc::Json envelope(const rmtc::Json& answer, bool mock = false) {
  auto typed = answer;
  if (!typed.contains("event_kind"))
    typed["event_kind"] = "explosion";
  return {{"id", mock ? "chatcmpl-practice-mock" : "test-fixture"},
          {"model", mock ? "practice-mock" : "test-fixture"},
          {"choices", rmtc::Json::array({{{"message", {{"content", typed.dump()}}}}})}};
}

/// 验证模拟响应不能伪装成理解成功，真实格式的测试回复仍须严格检查区域边界。
void check_reply_validation() {
  auto p = sample();
  rmtc::Json plan{{"begin_tick", 5},
                  {"end_tick", 20},
                  {"dangerous_rectangles", rmtc::Json::array({rmtc::Json::array({2, 1, 3, 2})})}};
  rmtc::Json answer{
      {"summary", "测试结构化结果"}, {"event_understood", true}, {"event_plan", plan}};
  auto reply = rmtc::decode_ai_completion(envelope(answer), p, 500, false);
  require(reply.completion_received && reply.interpretation_valid && !reply.mock &&
              reply.constraints && reply.constraints->dangerous_cells.size() == 4 &&
              reply.constraints->begin_tick == 5,
          "ongoing event rectangle did not decode");
  auto metered = envelope(answer);
  metered["usage"] = {{"prompt_tokens", 1234}, {"completion_tokens", 80},
                       {"prompt_cache_hit_tokens", 1000}, {"prompt_cache_miss_tokens", 234},
                       {"total_tokens", -1}, {"private_response", "must not propagate"}};
  const auto usage = rmtc::decode_ai_completion(metered, p, 500, false).to_json().at("usage");
  require(usage.size() == 4 && usage.at("prompt_tokens") == 1234 &&
              usage.at("completion_tokens") == 80 && !usage.contains("private_response") &&
              !usage.contains("total_tokens"),
          "billing audit must retain only valid nonnegative token counters");
  auto mock = rmtc::decode_ai_completion(envelope(answer, true), p, 500, false);
  require(mock.completion_received && mock.mock && !mock.event_understood && !mock.constraints,
          "mock response must never become understood event");
  mock = rmtc::decode_ai_completion(envelope(answer), p, 500, true);
  require(mock.mock && !mock.constraints, "configured mock mode must remain isolated");
  require(!reply.to_json().contains("token") && !reply.to_json().contains("url"),
          "IPC must not include HTTP credentials/configuration");
  auto unsupported = answer;
  unsupported["event_kind"] = "unrivaled";
  auto blocked = rmtc::decode_ai_completion(envelope(unsupported), p, 500, false);
  require(!blocked.interpretation_valid && !blocked.constraints,
          "beneficial fortress must not be executed as a danger region");

  for (auto bad : {rmtc::Json::array({-1, 0, 3, 2}), rmtc::Json::array({2, 1, 9, 2}),
                   rmtc::Json::array({3, 1, 2, 2}), rmtc::Json::array({2.5, 1, 3, 2})}) {
    answer["event_plan"]["dangerous_rectangles"] = rmtc::Json::array({bad});
    auto rejected = rmtc::decode_ai_completion(envelope(answer), p, 500, false);
    require(rejected.completion_received && !rejected.interpretation_valid && !rejected.constraints,
            "invalid model coordinates were silently accepted");
  }
  answer["event_plan"] = {{"begin_tick", 5},
                          {"end_tick", 20},
                          {"dangerous_cells", rmtc::Json::array()},
                          {"dangerous_rectangles", rmtc::Json::array()}};
  auto empty = rmtc::decode_ai_completion(envelope(answer), p, 500, false);
  require(empty.completion_received && !empty.interpretation_valid && !empty.constraints,
          "empty danger plan must not pretend an event has been handled");
  answer["event_plan"] = nullptr;
  require(!rmtc::decode_ai_completion(envelope(answer), p, 500, false).interpretation_valid,
          "nonempty understood event requires supported plan");
  answer["event_understood"] = false;
  require(rmtc::decode_ai_completion(envelope(answer), p, 500, false).interpretation_valid,
          "explicit unsupported event should remain a valid nonexecuting reply");
  auto no_event = p;
  no_event.event_text.clear();
  rmtc::Json no_event_answer{{"summary", "当前无事件"},
                             {"event_kind", "none"},
                             {"event_understood", true},
                             {"event_plan", {{"events", rmtc::Json::array()}}}};
  auto none = rmtc::decode_ai_completion(envelope(no_event_answer), no_event, 500, false);
  require(none.interpretation_valid && none.constraints && none.constraints->events.empty(),
          "empty public broadcast with an empty timeline must not waste retries");
  no_event_answer["event_plan"]["events"].push_back({{"kind", "pan"}});
  require(!rmtc::decode_ai_completion(envelope(no_event_answer), no_event, 500, false)
               .interpretation_valid,
          "empty broadcast must not accept invented effects");
  auto invalid = envelope(answer);
  invalid["choices"][0]["message"]["content"] = "not JSON";
  auto rejected = rmtc::decode_ai_completion(invalid, p, 500, false);
  require(rejected.completion_received && !rejected.interpretation_valid,
          "malformed model output is not an HTTP transport failure");
}

/// 确认当前帧可直接接受晚到的在生效期内结果，不多延后一Tick，也拒绝旧局/过期结果。
void check_strategy_delivery() {
  auto p = sample();
  rmtc::VisibleEntity resource;
  resource.type = "resource_normal";
  resource.id = "target";
  resource.position = {3, 1};
  resource.value = 10;
  p.visible_entities = {resource};
  rmtc::Stage2EventConstraints event{p.session_id, p.event_text, 9, 5, 20, {{2, 1}}, {}, false};
  rmtc::Stage2Strategy strategy;
  auto action = strategy.decide(p, 490, &event);
  require(!strategy.plan().event_pending && action.direction != "right",
          "incoming ongoing event must affect this decision immediately");
  p.tick = 20;
  require(strategy.decide(p).direction == "right", "expired AI constraint remained active");
  strategy.reset();
  event.session_id = "wrong-session";
  require(strategy.decide(p, 480, &event).direction == "right" && strategy.plan().event_pending,
          "cross-session AI event must be ignored");
  strategy.reset();
  p.tick = 10;
  event = {p.session_id, p.event_text, 9, 12, 13, {}, {}, false};
  for (int y = 0; y <= 3; ++y)
    for (int x = 0; x <= 3; ++x)
      event.dangerous_cells.push_back({x, y});
  action = strategy.decide(p, 490, &event);
  require(action.type == "move" && strategy.plan().goal == "evade" && strategy.plan().target &&
              (strategy.plan().target->x > 3 || strategy.plan().target->y > 3),
          "inside a danger region must allow crossing its cells to reach the exit");
}

/// 提示词输入只能由本队公开信息组成；事件原文作为JSON数据保留，不在本地分析。
void check_prompt() {
  const auto p = sample();
  const auto body = rmtc::build_ai_request(p, 500);
  require(body.at("stream") == false && !body.contains("model"), "use nonstreaming/default model");
  const auto snapshot = rmtc::Json::parse(body.at("messages")[1].at("content").get<std::string>());
  require(snapshot.at("event_text") == p.event_text && snapshot.at("tick") == 10 &&
              !snapshot.contains("token") && !snapshot.contains("map_view"),
          "AI input must retain public event without private state");
  require(rmtc::build_ai_request(p, 500, "selected-model").at("model") == "selected-model",
          "explicit model override missing");
  require(!body.contains("response_format") && !body.contains("thinking"),
          "generic/official requests must not contain provider-specific options by default");
  const auto deepseek = rmtc::build_ai_request(p, 500, "deepseek-flash", true, true);
  const double reserve = rmtc::ai_request_reserved_cny(deepseek);
  require(reserve > 4096 * 8.0 / 1e6 && reserve < 0.5,
          "bounded call must reserve maximum output and a conservative input allowance");
  auto unbounded = deepseek;
  unbounded["thinking"]["type"] = "enabled";
  bool rejected = false;
  try { (void)rmtc::ai_request_reserved_cny(unbounded); }
  catch (const std::invalid_argument&) { rejected = true; }
  require(rejected, "thinking requests must not use the disabled-thinking cost bound");
  require(deepseek.at("response_format").at("type") == "json_object" &&
              deepseek.at("thinking").at("type") == "disabled" &&
              deepseek.at("model") == "deepseek-flash",
          "DeepSeek request must ask for JSON output without consuming budget on thinking");
}

}  // namespace

/// 执行离线模型协议测试，失败返回非零，供CTest发现回归。
int main() {
  try {
    check_reply_validation();
    check_strategy_delivery();
    check_prompt();
    std::cout << "AI protocol/mock/constraint/public-prompt checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
