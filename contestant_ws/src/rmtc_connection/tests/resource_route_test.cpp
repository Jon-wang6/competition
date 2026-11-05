/**
 * @file resource_route_test.cpp
 * @brief 离线验证整体访问最优性、地图绕行、滚动缓存与16资源计算开销。
 *
 * 用独立全排列枚举核对动态规划的总代价；全部地图均为测试构造，
 * 不启动ROS/裁判/RViz，不读取实际比赛地图或向运行中的程序发送数据。
 *
 * @details 模块协作与维护约定
 * 验证对象：resource_route.hpp。
 * 运行约定：由独立测试可执行文件/CTest执行，断言失败用于定位回归，不是正式选手的逐帧逻辑。
 * 结果边界：通过表示这些构造场景满足约束，不证明所有随机地图均最优或稳定达到某个总分。
 */
#include "rmtc_connection/resource_route.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

using namespace rmtc::planning;

/** @brief 条件失败时抛出明确说明，Release模式也执行检查。 */
void require(bool condition, const std::string& message) {
  if (!condition)
    throw std::runtime_error(message);
}

/** @brief 穷举全部访问顺序作为独立真值，不调用动态规划或路线缓存。 */
std::int64_t exhaustive_cost(const std::vector<std::vector<std::int64_t>>& distance) {
  std::vector<std::size_t> permutation(distance.size() - 1);
  std::iota(permutation.begin(), permutation.end(), 1);
  auto best = std::numeric_limits<std::int64_t>::max();
  do {
    std::int64_t cost = 0;
    std::size_t previous = 0;
    for (const auto next : permutation) {
      cost += distance[previous][next];
      previous = next;
    }
    best = std::min(best, cost);
  } while (std::next_permutation(permutation.begin(), permutation.end()));
  return best;
}

/** @brief 对1至7个资源的56张固定随机距离表，用全排列逐一核对完整访问最优性。 */
void check_against_permutations() {
  std::mt19937 random(20261023);
  for (std::size_t n = 1; n <= 7; ++n) {
    for (int trial = 0; trial < 8; ++trial) {
      std::vector<std::vector<std::int64_t>> distance(n + 1, std::vector<std::int64_t>(n + 1));
      for (std::size_t i = 0; i <= n; ++i)
        for (std::size_t j = 0; j <= n; ++j)
          if (i != j)
            distance[i][j] = 1 + random() % 30;
      const auto order = plan_resource_visit_order(distance);
      require(order.cost == exhaustive_cost(distance),
              "dynamic programming disagrees with permutation oracle");
      auto unique = order.indices;
      std::sort(unique.begin(), unique.end());
      std::vector<std::size_t> expected(n);
      std::iota(expected.begin(), expected.end(), 0);
      require(unique == expected, "visit order repeats or omits a resource");
      std::int64_t reconstructed = 0;
      std::size_t previous = 0;
      for (const auto index : order.indices) {
        reconstructed += distance[previous][index + 1];
        previous = index + 1;
      }
      require(reconstructed == order.cost, "parent reconstruction disagrees with optimal cost");
    }
  }
}

/** @brief 校验空资源、非法输入、指数规模上限、溢出保护及首资源的稳定偏好。 */
void check_validation_and_ties() {
  require(plan_resource_visit_order({{0}}).indices.empty(), "empty tour rejected");
  const auto tie = plan_resource_visit_order({{0, 1, 1}, {1, 0, 2}, {1, 2, 0}});
  require(tie.cost == 3 && tie.indices.front() == 0,
          "equal-cost first-resource preference unstable");
  std::vector<std::vector<std::vector<std::int64_t>>> invalid{
      {},
      {{0, 1}, {1}},
      {{0, -1}, {1, 0}},
      {{1}},
      {{0, std::numeric_limits<std::int64_t>::max()}, {1, 0}},
      std::vector<std::vector<std::int64_t>>(18, std::vector<std::int64_t>(18, 0))};
  for (const auto& table : invalid) {
    bool rejected = false;
    try {
      (void)plan_resource_visit_order(table);
    } catch (const std::invalid_argument&) {
      rejected = true;
    }
    require(rejected, "invalid or oversized resource table accepted");
  }
}

/** @brief 在三资源折返场景检查最优首目标、路线后缀复用及动态变化导致的重算。 */
void check_rolling_routes() {
  GridMap map{9, 3, std::vector<std::int8_t>(27, 0)};
  std::vector<ResourceGoal> goals{
      {"left", {2, 0}, 10, 0}, {"near", {5, 0}, 10, 0}, {"right", {8, 0}, 10, 0}};
  ResourceRoutePlanner planner;
  auto tour = planner.plan(map, {4, 0}, goals);
  require(tour.exact && tour.replanned && tour.cost == 8 && tour.keys.front() == "left" &&
              tour.first_leg.cost == 2,
          "whole-route optimizer still selected the nearest resource");
  goals[0].last_seen_tick = 1;
  tour = planner.plan(map, {3, 0}, goals);
  require(!tour.replanned && tour.cost == 7 && tour.first_leg.cost == 1,
          "normal movement reran exponential search");
  tour = planner.plan(map, {3, 0}, goals);
  require(!tour.replanned && tour.cost == 7, "failed move advanced cached path");
  tour = planner.plan(map, {4, 1}, goals);
  require(tour.replanned && tour.first_leg.path.front() == GridIndex{4, 1},
          "off-route position reused stale tour");
  tour = planner.plan(map, {4, 0}, goals, {{3, 0}});
  require(tour.replanned && tour.cost == 12 && tour.first_leg.path[1] != GridIndex{3, 0},
          "new enemy did not trigger an A* detour");
  tour = planner.plan(map, {4, 0}, goals);
  require(tour.replanned && tour.cost == 8, "departed enemy remained blocked");
  map.cells[3] = 100;
  tour = planner.plan(map, {4, 0}, goals);
  require(tour.replanned && tour.cost == 12, "terrain change did not invalidate distances");
  goals.erase(goals.begin());
  tour = planner.plan(map, {4, 0}, goals);
  require(tour.replanned && tour.keys.size() == 2 && tour.cost == 4,
          "missing resource retained in tour");
  goals.push_back({"new-left", {2, 0}, 20, 4});
  require(planner.plan(map, {4, 0}, goals).keys.size() == 3,
          "newly observed resource was not inserted");
  planner.reset();
  require(planner.plan(map, {4, 0}, goals).replanned, "reset retained previous match's cache");
  GridMap unknown{3, 1, {0, -1, 0}};
  require(planner.plan(unknown, {0, 0}, {{"hidden-route", {2, 0}, 10, 0}}).keys.empty(),
          "resource tour assumed unknown cells were passable");
  unknown.cells[1] = 0;
  require(planner.plan(unknown, {0, 0}, {{"hidden-route", {2, 0}, 10, 0}}).cost == 2,
          "new known corridor did not unlock remembered resource");
}

/** @brief 覆盖16资源精确规划、缓存耗时与17资源安全回退；耗时只报告，不作脆弱限时断言。 */
void check_scale() {
  GridMap map{18, 1, std::vector<std::int8_t>(18, 0)};
  std::vector<ResourceGoal> goals;
  for (int x = 1; x <= 16; ++x)
    goals.push_back({"resource-" + std::to_string(x), {x, 0}, 10, 0});
  ResourceRoutePlanner planner;
  const auto begin = std::chrono::steady_clock::now();
  const auto tour = planner.plan(map, {0, 0}, goals);
  const auto built = std::chrono::steady_clock::now();
  require(tour.exact && tour.keys.size() == 16 && tour.cost == 16,
          "16-resource exact route incorrect");
  const auto cached = planner.plan(map, {1, 0}, goals);
  const auto reused = std::chrono::steady_clock::now();
  require(!cached.replanned && cached.cost == 15, "16-resource cache not reused");
  goals.push_back({"extra", {17, 0}, 10, 0});
  const auto fallback = planner.plan(map, {0, 0}, goals);
  require(!fallback.exact && fallback.reachable == 17 && fallback.keys.size() == 1 &&
              fallback.cost == 1,
          "custom oversized map did not safely fall back to nearest resource");
  std::cout << "16-resource planning: "
            << std::chrono::duration<double, std::milli>(built - begin).count()
            << " ms; cached step: "
            << std::chrono::duration<double, std::milli>(reused - built).count() << " ms\n";
}

/** @brief 执行独立规划验证，异常时打印原因并返回1。 */
int main() {
  try {
    check_against_permutations();
    check_validation_and_ties();
    check_rolling_routes();
    check_scale();
    std::cout << "resource tours: permutation oracle, rolling replans and safety checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
