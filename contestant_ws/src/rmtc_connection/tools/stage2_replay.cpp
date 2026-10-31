/**
 * @file stage2_replay.cpp
 * @brief 离线回放本队公开感知，统计各100 Tick动作/目标、地图覆盖和决策耗时。
 *
 * 输入每行一份公开perception JSON；输出逐帧计划JSONL和终端汇总JSON。
 * 不连接ROS、不读取全图；固定感知轨迹只能诊断决策，不能替代闭环比赛评估。
 */
#include "rmtc_connection/stage2_strategy.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <numeric>
#include <stdexcept>

/// 读取并依次回放感知；输入/输出路径由调用方显式给出，失败返回非零退出码。
int main(int argc, char** argv) {
  try {
    if (argc != 3 && argc != 4)
      throw std::runtime_error(
          "usage: stage2_replay PUBLIC_FRAMES.jsonl PLANS.jsonl [COLLECTION_MODE]");
    std::ifstream input(argv[1]);
    std::ofstream output(argv[2]);
    if (!input || !output)
      throw std::runtime_error("cannot open replay input/output");
    rmtc::Stage2Weights weights;
    if (argc == 4)
      weights.collection_mode = std::stoi(argv[3]);
    rmtc::Stage2Strategy strategy(weights);
    rmtc::MapMemory memory;
    rmtc::Json buckets = rmtc::Json::object();
    std::vector<double> times;
    std::string line;
    for (; std::getline(input, line);) {
      if (line.empty())
        continue;
      const auto raw = rmtc::Json::parse(line);
      const auto frame = rmtc::parse_public_perception(line, raw.at("self").at("team"));
      if (!memory.update(frame) || frame.tick >= 500)
        continue;
      const auto started = std::chrono::steady_clock::now();
      const auto action = strategy.decide(frame, 500 - frame.tick);
      const double ms =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started)
              .count();
      times.push_back(ms);
      const auto& plan = strategy.plan();
      auto& bucket = buckets[std::to_string(frame.tick / 100 * 100)];
      auto increment = [&](const char* category, const std::string& key) {
        auto& value = bucket[category][key];
        value = value.is_number_integer() ? value.get<int>() + 1 : 1;
      };
      increment("actions", action.type);
      increment("goals", plan.goal);
      rmtc::Json row{{"tick", frame.tick},
                     {"action", action.to_json(frame.tick, frame.session_id)},
                     {"goal", plan.goal},
                     {"target_id", plan.target_id},
                     {"goal_utility", plan.goal_utility},
                     {"event_pending", plan.event_pending},
                     {"observed_cells", memory.observed_count()},
                     {"map_cells", frame.map.width * frame.map.height},
                     {"decision_ms", ms},
                     {"action_scores", plan.action_scores}};
      output << row.dump() << '\n';
    }
    if (times.empty())
      throw std::runtime_error("no actionable public frames in replay");
    std::sort(times.begin(), times.end());
    std::cout << rmtc::Json{{"frames", times.size()},
                            {"buckets", buckets},
                            {"decision_ms",
                             {{"mean",
                               std::accumulate(times.begin(), times.end(), 0.0) / times.size()},
                              {"p95", times[static_cast<std::size_t>(.95 * (times.size() - 1))]},
                              {"max", times.back()}}}}
                     .dump(2)
              << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
