/**
 * @file resource_tuning.cpp
 * @brief 固定种子资源策略调参执行器；批量测试真实C++策略，不启动ROS或影响当前比赛。
 *
 * 仅测试第一阶段单车静态场景：执行器持有生成地图真值，策略每帧只接收方形视野。
 * 检查动作不越界、不穿墙、不进入尚未观察的格子，并按移动后的实际位置自动采集。
 * 输入为JSON参数列表，输出为逐行JSON指标；可为指定组合保存逐Tick轨迹以便复核。
 * 同时记录分段覆盖率和错过的一步探查机会，仅用于评价，不反馈隐藏信息给策略。
 * 批量请求可传seeds数组，直接汇总成功率/资源分布，避免百万场测试产生重复的大日志。
 */
#include "rmtc_connection/map_generator.hpp"
#include "rmtc_connection/resource_strategy.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <numeric>
#include <set>
#include <stdexcept>
#include <string>

namespace {
using Json = nlohmann::json;
using namespace rmtc;

/** @brief 读取完整JSON文件，错误时抛出文件名，禁止悄悄换用其他地图或参数。 */
Json read_json(const std::string& path) {
  std::ifstream input(path);
  if (!input)
    throw std::runtime_error("cannot open " + path);
  Json result;
  input >> result;
  return result;
}

/** @brief 显式指定全部权重，使批量结果不受后续默认参数改动影响。 */
planning::ResourceWeights weights_from_json(const Json& input) {
  planning::ResourceWeights weights;
  weights.distance = input.at("distance").get<double>();
  weights.edge = input.at("edge").get<double>();
  weights.value = input.at("value").get<double>();
  weights.unknown_cell_value = input.at("unknown").get<double>();
  weights.exploration_fade_ticks = input.at("fade").get<std::int64_t>();
  weights.unknown_edge_bonus = input.at("bonus").get<double>();
  weights.unknown_edge_band = input.at("band").get<int>();
  weights.peek_reward = input.value("peek", 0.0);
  weights.peek_detour_limit = input.value("peek_limit", 2);
  weights.peek_patch_limit = input.value("peek_patch", 0);
  planning::validate_resource_weights(weights);
  return weights;
}

/** @brief 将裁判地图实体转换为测试场景；完整列表仅归执行器持有，不传给策略。 */
std::vector<VisibleEntity> scene_from_config(const Json& config) {
  std::vector<VisibleEntity> result;
  for (const auto& p : config.at("map").at("obstacles")) {
    VisibleEntity entity;
    entity.type = "obstacle";
    entity.position = {p.at(0).get<int>(), p.at(1).get<int>()};
    result.push_back(entity);
  }
  for (const auto& item : config.at("map").at("resources")) {
    VisibleEntity entity;
    entity.id = item.at("id").get<std::string>();
    entity.value = item.at("value").get<double>();
    entity.type = *entity.value >= 20 ? "resource_high" : "resource_normal";
    entity.position = {item.at("position").at(0).get<int>(), item.at("position").at(1).get<int>()};
    result.push_back(entity);
  }
  return result;
}

/** @brief 按当前位置裁剪公开9×9感知，不能向策略暴露远处障碍/资源或场景剩余总数。 */
PublicPerception observe(int tick, Position position, const std::vector<VisibleEntity>& scene,
                         MapSize size) {
  PublicPerception frame;
  frame.session_id = "parameter-tuning";
  frame.tick = tick;
  frame.map = size;
  frame.vision_radius = 4;
  frame.self.team = "A";
  frame.self.robot_id = "A";
  frame.self.position = position;
  frame.self.alive = true;
  frame.self.hp = 100;
  for (const auto& entity : scene)
    if (std::abs(entity.position.x - position.x) <= 4 &&
        std::abs(entity.position.y - position.y) <= 4)
      frame.visible_entities.push_back(entity);
  return frame;
}

/**
 * @brief 按给定时限运行一次策略，统计资源、覆盖率、回头路及主动/兜底探索。
 * 时限和剩余窗口与正式节点保持一致；第0帧提交首动作，最后一帧仅用于覆盖率统计。
 * 收齐资源后仍运行到时限，以免用提前停止的覆盖率与其他组合的100Tick覆盖率比较。
 */
Json simulate(const Json& config, const Json& request) {
  const int horizon = request.value("ticks", 100);
  if (horizon <= 0 || horizon > 1000)
    throw std::invalid_argument("invalid tuning horizon");
  const auto& map = config.at("map");
  const MapSize size{map.at("width").get<int>(), map.at("height").get<int>()};
  Position position{config.at("robots").at(0).at("spawn").at(0).get<int>(),
                    config.at("robots").at(0).at("spawn").at(1).get<int>()};
  auto scene = scene_from_config(config);
  const auto count =
      std::count_if(scene.begin(), scene.end(), [](const auto& e) { return e.type != "obstacle"; });
  const auto ordering_name = request.value("ordering", "weighted");
  if (ordering_name != "weighted" && ordering_name != "tour" && ordering_name != "nearest" &&
      ordering_name != "joint")
    throw std::invalid_argument("invalid tuning resource ordering");
  const auto ordering = ordering_name == "tour"      ? ResourceOrdering::Tour
                        : ordering_name == "nearest" ? ResourceOrdering::Nearest
                        : ordering_name == "joint"   ? ResourceOrdering::Joint
                                                     : ResourceOrdering::Weighted;
  ResourceStrategy strategy(ordering, weights_from_json(request.at("weights")));
  if (request.contains("coverage")) {
    const auto& c = request.at("coverage");
    const std::set<std::string> supported{"offset",          "exponent",       "starts",
                                          "expected",        "urgency",        "information_paths",
                                          "path_edge_bonus", "path_edge_band", "path_exponent",
                                          "future",          "causal",         "refresh",
                                          "relocate"};
    for (auto item = c.begin(); item != c.end(); ++item)
      if (!supported.count(item.key()))
        throw std::invalid_argument(
            "archived experimental option: " + item.key() +
            "; use runtime/large-scale-tuning/experimental/resource_tuning.experimental");
    planning::CoverageWeights joint;
    joint.insertion_offset = c.value("offset", kJointInsertionOffset);
    joint.gain_exponent = c.value("exponent", kJointGainExponent);
    joint.starts = c.value("starts", 1);
    joint.expected_resources = c.value("expected", kJointExpectedResources);
    joint.discovery_urgency = c.value("urgency", kJointDiscoveryUrgency);
    joint.prefer_information_paths = c.value("information_paths", kJointPreferInformationPaths);
    joint.path_edge_bonus = c.value("path_edge_bonus", kJointPathEdgeBonus);
    joint.path_edge_band = c.value("path_edge_band", kJointPathEdgeBand);
    joint.path_potential_exponent = c.value("path_exponent", kJointPathPotentialExponent);
    joint.future_resource_weight = c.value("future", kJointFutureResourceWeight);
    joint.causal_future_fraction = c.value("causal", kJointCausalFutureFraction);
    joint.refresh_margin = c.value("refresh", kJointRefreshMargin);
    joint.relocate = c.value("relocate", kJointRelocate);
    strategy.set_coverage_weights(joint);
  }
  MapMemory coverage;
  std::set<std::pair<int, int>> visited{{position.x, position.y}};
  const bool tracing = request.value("trace", false);
  Json trace = Json::array(), checkpoints = Json::object(), observed_at = Json::object();
  // 标记当时能从一个已知相邻格看到的未知格，最后仍未观察的部分就是错过的补看机会。
  std::vector<bool> peek_opportunities(static_cast<std::size_t>(size.width * size.height), false);
  std::set<std::string> discovered_resources;
  double discovered_value = 0;
  Json discovered_at = Json::object();
  double value = 0;
  int collected = 0, finish = -1, moves = 0, explores = 0, proactive = 0, revisits = 0, peeks = 0;
  std::int64_t decision_total_us = 0, decision_max_us = 0;
  for (int tick = 0; tick < horizon; ++tick) {
    auto frame = observe(tick, position, scene, size);
    frame.score.resource_value = value;  // 与真实裁判一致，用实际采集后的分数更新补看预算。
    coverage.update(frame);
    for (const auto& entity : frame.visible_entities) {
      if (entity.type == "obstacle" || !entity.id ||
          !discovered_resources.insert(*entity.id).second)
        continue;
      discovered_value += entity.value.value_or(0);
    }
    const auto& known = coverage.cells();
    for (const auto direction : std::vector<Position>{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}) {
      const Position neighbor{position.x + direction.x, position.y + direction.y};
      if (neighbor.x < 0 || neighbor.y < 0 || neighbor.x >= size.width || neighbor.y >= size.height)
        continue;
      const auto offset =
          static_cast<std::size_t>(size.height - 1 - neighbor.y) * size.width + neighbor.x;
      if (known[offset] != 0)
        continue;  // 只把当时已知可走的一步计为机会，不猜墙后通路。
      for (int y = std::max(0, neighbor.y - 4); y <= std::min(size.height - 1, neighbor.y + 4); ++y)
        for (int x = std::max(0, neighbor.x - 4); x <= std::min(size.width - 1, neighbor.x + 4);
             ++x) {
          const auto cell = static_cast<std::size_t>(size.height - 1 - y) * size.width + x;
          if (known[cell] == -1)
            peek_opportunities[cell] = true;
        }
    }
    const auto decision_start = std::chrono::steady_clock::now();
    const auto action = strategy.decide(frame, horizon - tick);
    const auto decision_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                 std::chrono::steady_clock::now() - decision_start)
                                 .count();
    decision_total_us += decision_us;
    decision_max_us = std::max<std::int64_t>(decision_max_us, decision_us);
    const auto& plan = strategy.plan();
    if (plan.peek)
      ++peeks;
    if (plan.state == ResourceState::Exploring) {
      ++explores;
      if (plan.best_resource_cost)
        ++proactive;
    }
    const auto before = position;
    if (action.type == "move") {
      if (action.direction == "left")
        --position.x;
      else if (action.direction == "right")
        ++position.x;
      else if (action.direction == "up")
        --position.y;
      else if (action.direction == "down")
        ++position.y;
      else
        throw std::runtime_error("invalid direction in tuning simulation");
      ++moves;
      if (!visited.emplace(position.x, position.y).second)
        ++revisits;
    } else if (action.type != "wait")
      throw std::runtime_error("unsupported phase-one action");
    if (position.x < 0 || position.y < 0 || position.x >= size.width || position.y >= size.height)
      throw std::runtime_error("tuning route left map");
    const auto offset =
        static_cast<std::size_t>(size.height - 1 - position.y) * size.width + position.x;
    if (strategy.memory().cells().at(offset) != 0)
      throw std::runtime_error("tuning route entered unknown or blocked terrain");
    Json collected_ids = Json::array();
    scene.erase(
        std::remove_if(scene.begin(), scene.end(),
                       [&](const auto& entity) {
                         if (entity.position.x != position.x || entity.position.y != position.y)
                           return false;
                         if (entity.type == "obstacle")
                           throw std::runtime_error("tuning route crossed wall");
                         value += entity.value.value_or(0);
                         ++collected;
                         collected_ids.push_back(entity.id.value_or(""));
                         return true;
                       }),
        scene.end());
    if (collected == count && finish < 0)
      finish = tick + 1;
    if ((tick + 1) % 25 == 0) {
      checkpoints[std::to_string(tick + 1)] = value;
      // 补上本次动作后的视野，只用于指标；策略仍在下一Tick才收到这份新观察。
      auto after = coverage;
      after.update(observe(tick + 1, position, scene, size));
      observed_at[std::to_string(tick + 1)] = after.observed_count();
      discovered_at[std::to_string(tick + 1)] = discovered_value;
    }
    if (tracing) {
      Json item{{"tick", tick},
                {"from", {before.x, before.y}},
                {"to", {position.x, position.y}},
                {"action", action.type},
                {"direction", action.direction},
                {"state", resource_state_name(plan.state)},
                {"resource_value", value},
                {"collected", collected_ids},
                {"observed", coverage.observed_count()}};
      if (plan.target)
        item["target"] = {plan.target->position.x, plan.target->position.y};
      item["known_resource_value"] = discovered_value;
      item["planned_cost"] = plan.total_cost;
      if (!plan.route.path.empty())
        item["route_goal"] = {plan.route.path.back().x, plan.route.path.back().y};
      if (plan.best_resource_cost)
        item["resource_cost"] = *plan.best_resource_cost;
      if (plan.best_exploration_cost)
        item["exploration_cost"] = *plan.best_exploration_cost;
      if (plan.peek)
        item["peek"] = {{"extra_steps", plan.peek->extra_steps},
                        {"extra_cells", plan.peek->extra_cells},
                        {"reward", plan.peek->reward},
                        {"used", plan.peek_detour_used}};
      trace.push_back(std::move(item));
    }
  }
  const auto terminal = observe(horizon, position, scene, size);
  coverage.update(terminal);
  // 资源发现的终局指标与覆盖率一样包含最后一步后的视野；历史分段快照保留原口径。
  for (const auto& entity : terminal.visible_entities)
    if (entity.type != "obstacle" && entity.id && discovered_resources.insert(*entity.id).second)
      discovered_value += entity.value.value_or(0);
  std::size_t skipped_peek_cells = 0;
  for (std::size_t i = 0; i < peek_opportunities.size(); ++i)
    if (peek_opportunities[i] && coverage.cells()[i] == -1)
      ++skipped_peek_cells;
  Json result{{"name", request.value("name", "")},
              {"seed", request.at("seed")},
              {"ticks", horizon},
              {"weights", request.at("weights")},
              {"resource_value", value},
              {"collected", collected},
              {"finish_tick", finish},
              {"moves", moves},
              {"explores", explores},
              {"proactive_explores", proactive},
              {"revisits", revisits},
              {"observed_cells", coverage.observed_count()},
              {"peeks", peeks},
              {"map_cells", size.width * size.height},
              {"value_at", checkpoints},
              {"observed_at", observed_at},
              {"discovered_value_at", discovered_at},
              {"terminal_discovered_value", discovered_value},
              {"skipped_peek_cells", skipped_peek_cells}};
  result["ordering"] = ordering_name;
  result["mean_decision_us"] = static_cast<double>(decision_total_us) / horizon;
  result["max_decision_us"] = decision_max_us;
  if (request.contains("coverage"))
    result["coverage"] = request.at("coverage");
  if (tracing)
    result["trace"] = std::move(trace);
  return result;
}

/** @brief 缓存每个种子的完整裁判场景，仅供执行器裁剪感知；拒绝不可复现的随机种子。 */
const Json& cached_config(const Json& base, std::map<std::int64_t, Json>& configs,
                          std::int64_t seed) {
  if (seed == -1)
    throw std::invalid_argument("random seed is not reproducible for tuning");
  if (!configs.count(seed))
    configs.emplace(seed, rmtc::practice::generate_practice_map(base, 1, seed).config);
  return configs.at(seed);
}

/**
 * @brief 同一组显式权重评估多张地图，成功定义为时限内收齐场景中的全部资源。
 * 汇总最差20%的资源均值、覆盖率及漏区；不以最短收齐时间作为排序目标。
 * save_details仅用于独立验证，保存每图结果以便配对比较；训练阶段只输出聚合指标。
 */
Json simulate_batch(const Json& base, std::map<std::int64_t, Json>& configs, const Json& request) {
  const auto& seeds = request.at("seeds");
  if (!seeds.is_array() || seeds.empty())
    throw std::invalid_argument("batch seeds must be a nonempty array");
  std::vector<double> values;
  double observed = 0, observed50 = 0, skipped = 0, peeks = 0;
  double decision_mean_us = 0;
  std::int64_t decision_max_us = 0;
  std::size_t successes = 0;
  Json histogram = Json::object(), details = Json::array();
  auto single = request;
  single.erase("seeds");
  single["trace"] = false;
  for (const auto& seed_json : seeds) {
    const auto seed = seed_json.get<std::int64_t>();
    single["seed"] = seed;
    auto result = simulate(cached_config(base, configs, seed), single);
    const auto value = result.at("resource_value").get<double>();
    values.push_back(value);
    if (result.at("finish_tick").get<int>() >= 0)
      ++successes;
    observed += result.at("observed_cells").get<double>();
    observed50 += result.at("observed_at").value("50", 0.0);
    skipped += result.at("skipped_peek_cells").get<double>();
    peeks += result.at("peeks").get<double>();
    decision_mean_us += result.at("mean_decision_us").get<double>();
    decision_max_us = std::max(decision_max_us, result.at("max_decision_us").get<std::int64_t>());
    const auto key = std::to_string(static_cast<int>(std::lround(value)));
    histogram[key] = histogram.value(key, 0) + 1;
    if (request.value("save_details", false)) {
      result.erase("weights");
      result.erase("name");
      details.push_back(std::move(result));
    }
  }
  std::sort(values.begin(), values.end());
  const auto tail_count = (values.size() + 4) / 5;
  Json result{
      {"name", request.value("name", "")},
      {"weights", request.at("weights")},
      {"maps", values.size()},
      {"successes", successes},
      {"success_rate", static_cast<double>(successes) / values.size()},
      {"mean_resource_value", std::accumulate(values.begin(), values.end(), 0.0) / values.size()},
      {"tail20_resource_value",
       std::accumulate(values.begin(), values.begin() + tail_count, 0.0) / tail_count},
      {"mean_observed_cells", observed / values.size()},
      {"mean_observed50", observed50 / values.size()},
      {"mean_skipped_peek_cells", skipped / values.size()},
      {"mean_peeks", peeks / values.size()},
      {"resource_histogram", histogram}};
  result["ordering"] = request.value("ordering", "weighted");
  result["mean_decision_us"] = decision_mean_us / values.size();
  result["max_decision_us"] = decision_max_us;
  if (request.contains("coverage"))
    result["coverage"] = request.at("coverage");
  if (request.contains("regression_seed")) {
    single["seed"] = request.at("regression_seed");
    result["regression"] =
        simulate(cached_config(base, configs, single.at("seed").get<std::int64_t>()), single);
  }
  if (request.value("save_details", false))
    result["details"] = std::move(details);
  return result;
}
}  // namespace

/** @brief 按参数列表逐次执行并立即落盘；复用同种子真值，失败打印原因且返回非零。 */
int main(int argc, char** argv) {
  try {
    if (argc != 4)
      throw std::invalid_argument(
          "usage: resource_tuning stage1-template.json requests.json results.jsonl");
    const auto base = read_json(argv[1]);
    const auto requests = read_json(argv[2]);
    if (!requests.is_array())
      throw std::invalid_argument("requests must be an array");
    std::ofstream output(argv[3]);
    if (!output)
      throw std::runtime_error("cannot open tuning results");
    std::map<std::int64_t, Json> configs;
    std::size_t completed = 0;
    for (const auto& request : requests) {
      if (request.contains("seeds")) {
        output << simulate_batch(base, configs, request).dump() << '\n';
        output.flush();
        if (!output)
          throw std::runtime_error("cannot write batch tuning result");
        ++completed;
        if (completed % 25 == 0)
          std::cout << "completed models " << completed << '/' << requests.size() << std::endl;
        continue;
      }
      const auto seed = request.at("seed").get<std::int64_t>();
      output << simulate(cached_config(base, configs, seed), request).dump() << '\n';
      if (!output)
        throw std::runtime_error("cannot write tuning result");
      ++completed;
      if (completed % 1000 == 0)
        std::cout << "completed " << completed << '/' << requests.size() << std::endl;
    }
    std::cout << "finished " << completed << " parameter runs\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
