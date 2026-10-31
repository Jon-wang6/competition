/**
 * @file map_generator_test.cpp
 * @brief 离线验证两个阶段的种子复现、资源规则、不重叠与初始全图连通性。
 *
 * 使用独立DFS检查生成结果的性质，覆盖128张固定种子地图和随机种子重放。
 * 这里只测试裁判部署工具，不启动ROS，也不将地图真值传入参赛策略。
 */
#include "rmtc_connection/map_generator.hpp"
#include <array>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using Json = nlohmann::json;
using Point = std::pair<int, int>;

/** @brief Release构建同样检查性质，失败时抛出明确说明。 */
void require(bool condition, const std::string& message) {
  if (!condition)
    throw std::runtime_error(message);
}

/** @brief 构造含到达任务及事件关键点的测试模板，不依赖正式比赛隐藏布局。 */
Json base_template() {
  return {{"mission_text", "到达[9,9]"},
          {"missions", Json::array({{{"id", "reach"}, {"type", "reach"}, {"position", {9, 9}}}})},
          {"events",
           {{"enabled", true},
            {"definitions",
             {{"angel", {{"points", {{3, 9}, {14, 9}}}}},
              {"pepper", {{"points", {{5, 6}, {12, 11}}}}},
              {"world_split", {{"point", {8, 8}}}},
              {"unrivaled", {{"center_mode", "map_center"}}}}}}},
          {"marker", "template-content-must-stay"}};
}

/** @brief 把合法JSON坐标转为测试使用的点，同时独立检查地图边界。 */
Point point(const Json& value, int size) {
  require(value.is_array() && value.size() == 2 && value[0].is_number_integer() &&
              value[1].is_number_integer(),
          "generated coordinate is not an integer pair");
  const int x = value[0].get<int>(), y = value[1].get<int>();
  require(x >= 0 && y >= 0 && x < size && y < size, "generated position outside map");
  return {x, y};
}

/** @brief 独立检查资源数量/价值、点位唯一、出生安全、任务可达及全部自由格连通。 */
void check_map(const rmtc::practice::GeneratedPracticeMap& generated, int stage) {
  const auto& config = generated.config;
  const int size = stage == 1 ? 20 : 24;
  require(config.at("map").at("width") == size && config.at("map").at("height") == size,
          "stage dimensions incorrect");
  require(config.at("vision_radius") == 4 && config.at("respawn_wait") == 3 &&
              config.at("spawn_protection") == 2,
          "base rule parameters changed");
  std::set<Point> blocked, occupied, spawns;
  for (const auto& obstacle : config.at("map").at("obstacles"))
    require(blocked.insert(point(obstacle, size)).second, "duplicate obstacle");
  require(blocked.size() == static_cast<std::size_t>(size * size / 10),
          "practice obstacle density incorrect");
  require(config.at("robots").size() == static_cast<std::size_t>(stage == 1 ? 1 : 4),
          "stage seat count incorrect");
  for (const auto& robot : config.at("robots")) {
    const auto spawn = point(robot.at("spawn"), size);
    require(!blocked.count(spawn) && spawns.insert(spawn).second, "spawn blocked or overlapping");
    for (int dy = -1; dy <= 1; ++dy)
      for (int dx = -1; dx <= 1; ++dx)
        require(!blocked.count({spawn.first + dx, spawn.second + dy}),
                "spawn neighborhood blocked");
  }
  std::set<std::string> ids;
  int normal = 0, high = 0, value = 0;
  for (const auto& resource : config.at("map").at("resources")) {
    const auto position = point(resource.at("position"), size);
    require(!blocked.count(position) && !spawns.count(position) && occupied.insert(position).second,
            "resource overlaps wall, spawn or another resource");
    require(ids.insert(resource.at("id").get<std::string>()).second, "resource ID repeated");
    const int amount = resource.at("value").get<int>();
    require(amount == 10 || amount == 20, "resource value differs from rules");
    if (amount == 10)
      ++normal;
    else
      ++high;
    value += amount;
  }
  require(normal == 12 && high == 4 && value == 200, "resource count or total value incorrect");
  std::vector<Point> stack{*spawns.begin()};
  std::set<Point> reached;
  while (!stack.empty()) {
    const auto from = stack.back();
    stack.pop_back();
    if (!reached.insert(from).second)
      continue;
    for (const auto& direction : std::array<Point, 4>{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}}) {
      const Point to{from.first + direction.first, from.second + direction.second};
      if (to.first < 0 || to.second < 0 || to.first >= size || to.second >= size ||
          blocked.count(to) || reached.count(to))
        continue;
      stack.push_back(to);
    }
  }
  require(reached.size() + blocked.size() == static_cast<std::size_t>(size * size),
          "free terrain disconnected");
  if (stage == 1)
    require(config.at("missions").empty() && !config.at("events").at("enabled").get<bool>(),
            "stage1 enabled combat missions/events");
  else {
    require(!blocked.count({9, 9}) && !blocked.count({3, 9}) && !blocked.count({14, 9}) &&
                !blocked.count({5, 6}) && !blocked.count({12, 11}) && !blocked.count({8, 8}),
            "task/event objective blocked");
  }
  require(config.at("marker") == "template-content-must-stay", "unrelated template data lost");
}

/** @brief 对每阶段64个种子重复生成并检查性质，要求障碍和资源布局均随种子变化。 */
void check_fixed_seeds() {
  const auto base = base_template();
  for (int stage : {1, 2}) {
    std::set<std::string> obstacles, resources;
    for (std::int64_t seed = 1; seed <= 64; ++seed) {
      const auto generated = rmtc::practice::generate_practice_map(base, stage, seed);
      check_map(generated, stage);
      const auto replay = rmtc::practice::generate_practice_map(base, stage, seed);
      require(generated.config == replay.config && generated.metadata == replay.metadata,
              "fixed seed not reproducible");
      obstacles.insert(generated.config.at("map").at("obstacles").dump());
      resources.insert(generated.config.at("map").at("resources").dump());
    }
    require(obstacles.size() == 64 && resources.size() == 64,
            "different test seeds reused same map layout");
  }
  require(base == base_template(), "generator modified input template");
}

/** @brief 随机实际种子可重放，除-1以外的负数及64位边界也属于固定种子。 */
void check_random_and_boundaries() {
  const auto base = base_template();
  const auto generated = rmtc::practice::generate_practice_map(base, 2, -1);
  require(generated.seed > 0 && generated.metadata.at("randomized") == true,
          "random seed metadata incorrect");
  require(generated.config == rmtc::practice::generate_practice_map(base, 2, generated.seed).config,
          "random map cannot be replayed by recorded seed");
  for (auto seed : {std::int64_t{-2}, std::numeric_limits<std::int64_t>::min(),
                    std::numeric_limits<std::int64_t>::max()})
    require(rmtc::practice::generate_practice_map(base, 1, seed).config ==
                rmtc::practice::generate_practice_map(base, 1, seed).config,
            "signed fixed seed not reproducible");
  bool rejected = false;
  try {
    (void)rmtc::practice::generate_practice_map(base, 0);
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  require(rejected, "invalid stage accepted");
  auto bad = base;
  bad["missions"][0]["position"] = {24, 0};
  rejected = false;
  try {
    (void)rmtc::practice::generate_practice_map(bad, 2, 1);
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  require(rejected, "out-of-map mission silently changed");
}

/** @brief 种子0同样生成标准尺寸，不沿用模板的18×18布局；任务和事件定义继续保留。 */
void check_zero_seed() {
  for (int stage : {1, 2}) {
    auto base = base_template();
    base["stage"] = stage;
    base["robots"] = Json::array({{{"team", "A"}, {"spawn", {1, 1}}}});
    base["map"] = {
        {"width", 18},
        {"height", 18},
        {"obstacles", {{6, 3}, {6, 4}}},
        {"resources", Json::array({{{"id", "official"}, {"position", {2, 1}}, {"value", 10}}})}};
    const auto result = rmtc::practice::generate_practice_map(base, stage, 0);
    check_map(result, stage);
    require(result.config.at("map") != base.at("map") &&
                result.metadata.at("official_template") == false &&
                result.metadata.at("generator_version") == 4 && result.metadata.at("seed") == 0 &&
                result.metadata.at("randomized") == false,
            "seed0 used official layout or wrong metadata");
    require(result.config == rmtc::practice::generate_practice_map(base, stage, 0).config,
            "seed0 is not reproducible");
    if (stage == 2)
      require(
          result.config.at("missions") == base.at("missions") &&
              result.config.at("events").at("definitions") == base.at("events").at("definitions"),
          "stage2 objectives changed when replacing official layout");
  }
}

/** @brief 非中央堡垒也要保护其实际公告区域，避免生成障碍覆盖活动格。 */
void check_custom_fortress() {
  auto base = base_template();
  base["events"]["definitions"]["unrivaled"] = {{"center_mode","fixed"},{"center",{17,6}},{"size",3}};
  for (int seed = 0; seed < 32; ++seed) {
    const auto generated = rmtc::practice::generate_practice_map(base,2,seed);
    check_map(generated,2);
    for (const auto& value : generated.config.at("map").at("obstacles")) {
      const auto p = point(value,24);
      require(!(p.first >= 16 && p.first <= 18 && p.second >= 5 && p.second <= 7), "off-center fortress blocked");
    }
  }
}

/** @brief 执行全部离线性质检查，失败打印原因并返回1。 */
int main() {
  try {
    check_fixed_seeds();
    check_random_and_boundaries();
    check_zero_seed();
    check_custom_fortress();
    std::cout << "128 seeded maps: rules, connectivity, uniqueness and replay checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
