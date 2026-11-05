/**
 * @file astar_test.cpp
 * @brief 离线验证四方向A*的最短路、未知格、地图行序和失败状态。
 *
 * 在3x3地图上枚举所有起终点为空的障碍布局，用独立BFS核对最少步数；
 * 另检验未知格加权代价、临时禁行、重复调用及大代价整数安全。
 * 不初始化ROS，不读取比赛地图，不调用选手控制程序。
 *
 * @details 模块协作与维护约定
 * 验证对象：astar.hpp。
 * 运行约定：由独立测试可执行文件/CTest执行，断言失败用于定位回归，不是正式选手的逐帧逻辑。
 * 结果边界：通过表示这些构造场景满足约束，不证明所有随机地图均最优或稳定达到某个总分。
 */
#include "rmtc_connection/astar.hpp"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <vector>

using namespace rmtc::planning;

/** @brief 条件失败时抛出具体原因，在Release构建中也执行检查。 */
void require(bool condition, const std::string& message) {
  if (!condition)
    throw std::runtime_error(message);
}

/** @brief 构造按比赛从上到下存储的全自由地图，测试坐标使用格子而非米。 */
GridMap free_map(int width, int height) {
  return {width, height, std::vector<std::int8_t>(static_cast<std::size_t>(width) * height, 0)};
}

/** @brief 检查成功路线的起终点、每步四邻接、边界及实际经过格子的通行代价。 */
void check_path(const GridMap& map, const SearchResult& result, GridIndex start, GridIndex goal,
                const SearchOptions& options = {}) {
  require(result.success() && !result.path.empty(), "expected successful complete path");
  require(result.path.front() == start && result.path.back() == goal, "path endpoints incorrect");
  std::int64_t cost = 0;
  for (std::size_t i = 0; i < result.path.size(); ++i) {
    const auto p = result.path[i];
    require(p.x >= 0 && p.y >= 0 && p.x < map.width && p.y < map.height, "path outside map");
    const auto row = map.row_order == RowOrder::TopToBottom ? p.y : map.height - 1 - p.y;
    const auto value = map.cells[static_cast<std::size_t>(row * map.width + p.x)];
    require(value == -1 ? options.allow_unknown : value < options.occupied_threshold,
            "path crossed forbidden terrain");
    require(std::find(options.blocked_cells.begin(), options.blocked_cells.end(), p) ==
                options.blocked_cells.end(),
            "path crossed temporary blocked cell");
    if (i == 0)
      continue;
    const auto previous = result.path[i - 1];
    require(std::abs(p.x - previous.x) + std::abs(p.y - previous.y) == 1,
            "path contains diagonal/jumping step");
    cost += value == -1 ? options.unknown_cost : 1;
  }
  require(cost == result.cost, "reported cost disagrees with traversed cells");
}

/** @brief 对3x3普通自由/障碍网格运行BFS，独立于A*的启发项与优先队列。 */
int bfs_distance(const GridMap& map) {
  std::vector<int> distance(9, -1);
  std::queue<int> queue;
  queue.push(0);
  distance[0] = 0;
  while (!queue.empty()) {
    const int from = queue.front();
    queue.pop();
    if (from == 8)
      return distance[8];
    for (int to = 0; to < 9; ++to) {
      if (distance[static_cast<std::size_t>(to)] >= 0 ||
          map.cells[static_cast<std::size_t>(to)] != 0)
        continue;
      if (std::abs(from % 3 - to % 3) + std::abs(from / 3 - to / 3) != 1)
        continue;
      distance[static_cast<std::size_t>(to)] = distance[static_cast<std::size_t>(from)] + 1;
      queue.push(to);
    }
  }
  return -1;
}

/** @brief 枚举128种有效障碍组合，要求A*可达性与路径长度均和BFS一致。 */
void check_against_bfs() {
  AStarPlanner planner;
  int checked = 0;
  for (unsigned mask = 0; mask < 512; ++mask) {
    if ((mask & 1U) || (mask & 256U))
      continue;
    auto map = free_map(3, 3);
    for (int i = 0; i < 9; ++i)
      if (mask & (1U << i))
        map.cells[static_cast<std::size_t>(i)] = 100;
    const auto result = planner.searchAStar(map, {0, 0}, {2, 2});
    const auto expected = bfs_distance(map);
    if (expected < 0) {
      require(result.status == SearchStatus::NoPath && result.path.empty(),
              "BFS-unreachable map got a path");
    } else {
      check_path(map, result, {0, 0}, {2, 2});
      require(result.cost == expected, "A* disagreed with BFS shortest distance");
    }
    ++checked;
  }
  require(checked == 128, "obstacle enumeration incomplete");
}

/** @brief 检验已知障碍绕行、两种地图行存储顺序和起终点同格。 */
void check_geometry() {
  AStarPlanner planner;
  auto map = free_map(7, 5);
  for (int y = 0; y < 4; ++y)
    map.cells[static_cast<std::size_t>(y * 7 + 3)] = 100;
  auto result = planner.searchAStar(map, {0, 2}, {6, 2});
  check_path(map, result, {0, 2}, {6, 2});
  require(result.cost == 10, "barrier detour was not optimal");

  map = {3, 2, {0, 100, 0, 0, 0, 0}, RowOrder::TopToBottom};
  result = planner.searchAStar(map, {0, 0}, {2, 0});
  check_path(map, result, {0, 0}, {2, 0});
  require(result.cost == 4, "top-first asymmetric map was flipped");
  const auto top_path = result.path;
  map = {3, 2, {0, 0, 0, 0, 100, 0}, RowOrder::BottomToTop};
  result = planner.searchAStar(map, {0, 0}, {2, 0});
  check_path(map, result, {0, 0}, {2, 0});
  require(result.path == top_path && result.cost == 4,
          "ROS row storage changed competition coordinates");

  result = planner.searchAStar(map, {0, 0}, {0, 0});
  check_path(map, result, {0, 0}, {0, 0});
  require(result.path.size() == 1 && result.cost == 0, "same-cell search failed");
}

/** @brief 检查未知格阻挡/加权、占据阈值及临时禁行不会污染后续搜索。 */
void check_options() {
  AStarPlanner planner;
  GridMap map{3, 1, {0, -1, 0}};
  require(planner.searchAStar(map, {0, 0}, {2, 0}).status == SearchStatus::NoPath,
          "unknown terrain passable by default");
  SearchOptions options;
  options.allow_unknown = true;
  options.unknown_cost = 7;
  auto result = planner.searchAStar(map, {0, 0}, {2, 0}, options);
  check_path(map, result, {0, 0}, {2, 0}, options);
  require(result.cost == 8, "unknown entry cost ignored");
  options.unknown_cost = std::numeric_limits<int>::max();
  result = planner.searchAStar(map, {0, 0}, {2, 0}, options);
  require(result.success() &&
              result.cost == static_cast<std::int64_t>(std::numeric_limits<int>::max()) + 1,
          "large path cost overflowed int32");

  map = free_map(5, 3);
  for (int x = 1; x <= 3; ++x)
    map.cells[static_cast<std::size_t>(5 + x)] = -1;
  options.unknown_cost = 5;
  result = planner.searchAStar(map, {0, 1}, {4, 1}, options);
  check_path(map, result, {0, 1}, {4, 1}, options);
  require(result.cost == 6, "A* ignored cheaper known-free detour");
  options.unknown_cost = 1;
  result = planner.searchAStar(map, {0, 1}, {4, 1}, options);
  check_path(map, result, {0, 1}, {4, 1}, options);
  require(result.cost == 4, "equal-cost unknown route did not use shortest path");

  map = {3, 1, {0, 60, 0}};
  require(planner.searchAStar(map, {0, 0}, {2, 0}).status == SearchStatus::NoPath,
          "occupied threshold ignored");
  options = {};
  options.occupied_threshold = 80;
  require(planner.searchAStar(map, {0, 0}, {2, 0}, options).cost == 2,
          "custom occupancy threshold failed");

  map = free_map(3, 1);
  options = {};
  options.blocked_cells = {{1, 0}};
  const auto snapshot = map.cells;
  require(planner.searchAStar(map, {0, 0}, {2, 0}, options).status == SearchStatus::NoPath,
          "temporary blocked cell ignored");
  require(map.cells == snapshot && planner.searchAStar(map, {0, 0}, {2, 0}).cost == 2,
          "search changed map or retained old temporary obstacle");
}

/** @brief 检查地图、参数、起终点、起终点障碍的失败结果能明确区分。 */
void check_invalid_inputs() {
  AStarPlanner planner;
  auto map = free_map(2, 2);
  require(planner.searchAStar({}, {0, 0}, {1, 1}).status == SearchStatus::InvalidMap,
          "empty map accepted");
  auto broken = map;
  broken.cells.pop_back();
  require(planner.searchAStar(broken, {0, 0}, {1, 1}).status == SearchStatus::InvalidMap,
          "bad data size accepted");
  broken = map;
  broken.cells[0] = -2;
  require(planner.searchAStar(broken, {0, 0}, {1, 1}).status == SearchStatus::InvalidMap,
          "invalid cell accepted");
  require(planner.searchAStar(map, {-1, 0}, {1, 1}).status == SearchStatus::StartOutOfBounds,
          "bad start accepted");
  require(planner.searchAStar(map, {0, 0}, {2, 1}).status == SearchStatus::GoalOutOfBounds,
          "bad goal accepted");
  map.cells[0] = 100;
  require(planner.searchAStar(map, {0, 0}, {1, 1}).status == SearchStatus::StartBlocked,
          "blocked start accepted");
  map.cells[0] = 0;
  map.cells[3] = 100;
  const auto result = planner.searchAStar(map, {0, 0}, {1, 1});
  require(result.status == SearchStatus::GoalBlocked && !result.success() && result.path.empty(),
          "blocked goal accepted");
  SearchOptions options;
  options.unknown_cost = 0;
  require(planner.searchAStar(map, {0, 0}, {1, 1}, options).status == SearchStatus::InvalidOptions,
          "zero entry cost accepted");
  options = {};
  options.blocked_cells = {{5, 5}};
  require(planner.searchAStar(map, {0, 0}, {1, 1}, options).status == SearchStatus::InvalidOptions,
          "out-of-map obstacle accepted");
}

/** @brief 执行全部离线验证，任一失败打印原因并返回非零退出码。 */
int main() {
  try {
    check_against_bfs();
    check_geometry();
    check_options();
    check_invalid_inputs();
    std::cout << "A*: 128 BFS comparisons, row order, weighted unknown cells and failure statuses "
                 "passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
