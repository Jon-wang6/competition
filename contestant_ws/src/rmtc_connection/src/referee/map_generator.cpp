/**
 * @file map_generator.cpp
 * @brief 生成可复现、全自由区域连通的练习地图，保留官方游戏规则与结算逻辑。
 *
 * 先保护出生点、任务与事件关键点，再逐个尝试放置障碍并检查连通性，
 * 最后从剩余自由格无放回抽取16个资源位置；不将任何全图数据接入选手节点。
 */
#include "rmtc_connection/map_generator.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <numeric>
#include <queue>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace rmtc::practice {
namespace {

using Json = nlohmann::json;

/** @brief 从引擎均匀抽取[0,bound)；拒绝余数偏置，结果不依赖标准库分布实现。 */
std::uint64_t bounded_random(std::mt19937_64& engine, std::uint64_t bound) {
  const auto threshold = (std::uint64_t{0} - bound) % bound;
  for (;;) {
    const auto value = engine();
    if (value >= threshold)
      return value % bound;
  }
}

/** @brief 对整数格索引执行确定性的Fisher–Yates洗牌。 */
void shuffle_cells(std::vector<int>& cells, std::mt19937_64& engine) {
  for (std::size_t count = cells.size(); count > 1; --count)
    std::swap(cells[count - 1], cells[static_cast<std::size_t>(bounded_random(engine, count))]);
}

/** @brief 从系统随机源选择非负实际种子，返回值可直接用于下次seed参数。 */
std::int64_t random_seed() {
  std::random_device entropy;
  const auto bits =
      (static_cast<std::uint64_t>(entropy()) << 32) ^ static_cast<std::uint64_t>(entropy());
  // 随机种子仍选正整数，保持已有随机种子的记录和重放约定。
  const auto value = static_cast<std::int64_t>(
      bits & static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()));
  return value == 0 ? 1 : value;
}

/** @brief 校验JSON坐标并转换成行优先格索引；关键点越界时拒绝模板而非静默移动。 */
int position_index(const Json& position, int size) {
  if (!position.is_array() || position.size() != 2 || !position[0].is_number_integer() ||
      !position[1].is_number_integer())
    throw std::invalid_argument("template position must be [x,y] integers");
  const auto x = position[0].get<std::int64_t>(), y = position[1].get<std::int64_t>();
  if (x < 0 || y < 0 || x >= size || y >= size)
    throw std::invalid_argument("template mission/event point outside generated map");
  return static_cast<int>(y * size + x);
}

/** @brief 检查所有非障碍格都能从首个出生点四方向到达，不只检查资源所在分量。 */
bool connected(const std::vector<bool>& blocked, int size, int start) {
  std::vector<bool> visited(blocked.size(), false);
  std::queue<int> queue;
  queue.push(start);
  visited[static_cast<std::size_t>(start)] = true;
  std::size_t count = 0;
  constexpr std::array<std::array<int, 2>, 4> directions{
      {{{1, 0}}, {{-1, 0}}, {{0, 1}}, {{0, -1}}}};
  while (!queue.empty()) {
    const auto from = queue.front();
    queue.pop();
    ++count;
    for (const auto& direction : directions) {
      const auto x = from % size + direction[0], y = from / size + direction[1];
      if (x < 0 || y < 0 || x >= size || y >= size)
        continue;
      const auto to = static_cast<std::size_t>(y * size + x);
      if (!blocked[to] && !visited[to]) {
        visited[to] = true;
        queue.push(static_cast<int>(to));
      }
    }
  }
  return count == static_cast<std::size_t>(std::count(blocked.begin(), blocked.end(), false));
}

/** @brief 保护任务到达坐标、天使/辣椒点、分裂交点以及中央堡垒活动区域。 */
void reserve_objectives(const Json& config, int stage, int size, std::vector<bool>& reserved) {
  if (stage == 1)
    return;
  for (const auto& mission : config.at("missions"))
    if (mission.value("type", "") == "reach")
      reserved[static_cast<std::size_t>(position_index(mission.at("position"), size))] = true;
  const auto& events = config.at("events");
  if (!events.value("enabled", false) || !events.contains("definitions"))
    return;
  const auto& definitions = events.at("definitions");
  for (const auto* name : {"angel", "pepper"}) {
    if (definitions.contains(name) && definitions.at(name).contains("points"))
      for (const auto& point : definitions.at(name).at("points"))
        reserved[static_cast<std::size_t>(position_index(point, size))] = true;
  }
  if (definitions.contains("world_split") && definitions.at("world_split").contains("point"))
    reserved[static_cast<std::size_t>(
        position_index(definitions.at("world_split").at("point"), size))] = true;
  if (definitions.contains("unrivaled")) {
    const auto& fortress = definitions.at("unrivaled");
    const int center = fortress.value("center_mode", "map_center") == "map_center"
                           ? (size / 2) * size + size / 2
                           : position_index(fortress.at("center"), size);
    const int radius = fortress.value("size", 3) / 2;
    for (int y = std::max(0, center / size-radius); y <= std::min(size-1, center / size+radius); ++y)
      for (int x = std::max(0, center % size-radius); x <= std::min(size-1, center % size+radius); ++x)
        reserved[static_cast<std::size_t>(y*size+x)] = true;
  }
}

}  // namespace

/** @brief 组合模板、确定种子、生成连通障碍与不重叠资源，并返回可复现元信息。 */
GeneratedPracticeMap generate_practice_map(const Json& base, int stage,
                                           std::int64_t requested_seed) {
  if (!base.is_object() || (stage != 1 && stage != 2))
    throw std::invalid_argument("stage must be 1 or 2 and template must be an object");
  GeneratedPracticeMap result;
  result.seed = requested_seed == -1 ? random_seed() : requested_seed;
  result.config = base;
  auto& config = result.config;
  // 0与其他固定种子走同一生成流程，任何阶段都不直接使用官方18×18练习布局。
  const int size = stage == 1 ? 20 : 24;
  config["stage"] = stage;
  config["vision_radius"] = 4;
  config["respawn_wait"] = 3;
  config["spawn_protection"] = 2;
  if (!config.contains("events") || !config["events"].is_object())
    throw std::invalid_argument("template events must be an object");
  auto& events = config["events"];
  events["max_hp"] = 100;
  events["respawn_protection_ticks"] = 2;
  events["base_attack_damage"] = 20;
  events["base_defended_damage"] = 10;
  if (stage == 1) {
    config["missions"] = Json::array();
    config["mission_text"] = "练习采集：自主收集资源。";
    events["enabled"] = false;
  }
  if (!config.contains("missions") || !config["missions"].is_array())
    throw std::invalid_argument("template missions must be an array");

  const std::array<std::array<int, 2>, 4> spawns{
      {{{1, 1}}, {{size - 2, 1}}, {{1, size - 2}}, {{size - 2, size - 2}}}};
  config["robots"] = Json::array();
  std::vector<bool> reserved(static_cast<std::size_t>(size * size), false),
      spawn_cells(reserved.size(), false);
  const int seats = stage == 1 ? 1 : 4;
  for (int i = 0; i < seats; ++i) {
    const auto x = spawns[static_cast<std::size_t>(i)][0],
               y = spawns[static_cast<std::size_t>(i)][1];
    config["robots"].push_back(
        {{"team", std::string(1, static_cast<char>('A' + i))}, {"spawn", {x, y}}});
    spawn_cells[static_cast<std::size_t>(y * size + x)] = true;
    for (int dy = -1; dy <= 1; ++dy)
      for (int dx = -1; dx <= 1; ++dx)
        reserved[static_cast<std::size_t>((y + dy) * size + x + dx)] = true;
  }
  reserve_objectives(config, stage, size, reserved);
  std::mt19937_64 engine(static_cast<std::uint64_t>(result.seed) ^
                         (0x9e3779b97f4a7c15ULL * static_cast<std::uint64_t>(stage)));
  std::vector<int> candidates;
  for (int i = 0; i < size * size; ++i)
    if (!reserved[static_cast<std::size_t>(i)])
      candidates.push_back(i);
  shuffle_cells(candidates, engine);
  std::vector<bool> blocked(reserved.size(), false);
  auto obstacles = Json::array();
  const int obstacle_count = size * size / 10;
  for (const auto index : candidates) {
    if (obstacles.size() == static_cast<std::size_t>(obstacle_count))
      break;
    blocked[static_cast<std::size_t>(index)] = true;
    if (!connected(blocked, size, size + 1)) {
      blocked[static_cast<std::size_t>(index)] = false;
      continue;
    }
    obstacles.push_back({index % size, index / size});
  }
  if (obstacles.size() != static_cast<std::size_t>(obstacle_count))
    throw std::runtime_error("cannot place requested connected practice obstacles");
  candidates.clear();
  for (int i = 0; i < size * size; ++i)
    if (!blocked[static_cast<std::size_t>(i)] && !spawn_cells[static_cast<std::size_t>(i)])
      candidates.push_back(i);
  shuffle_cells(candidates, engine);
  if (candidates.size() < 16)
    throw std::runtime_error("not enough free cells for practice resources");
  auto resources = Json::array();
  for (int i = 0; i < 16; ++i) {
    const auto index = candidates[static_cast<std::size_t>(i)];
    resources.push_back(
        {{"id", (i < 12 ? "normal_" : "high_") + std::to_string(i < 12 ? i : i - 12)},
         {"position", {index % size, index / size}},
         {"value", i < 12 ? 10 : 20}});
  }
  config["map"] = {{"width", size},
                   {"height", size},
                   {"obstacles", std::move(obstacles)},
                   {"resources", std::move(resources)}};
  result.metadata = {{"generator_version", 4},
                     {"requested_seed", requested_seed},
                     {"seed", result.seed},
                     {"official_template", false},
                     {"randomized", requested_seed == -1},
                     {"stage", stage},
                     {"width", size},
                     {"height", size},
                     {"obstacle_count", obstacle_count},
                     {"normal_resources", 12},
                     {"high_resources", 4},
                     {"resource_value", 200},
                     {"connected", true}};
  return result;
}

}  // namespace rmtc::practice
