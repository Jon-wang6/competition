/**
 * @file astar.cpp
 * @brief 四方向A*：开放队列、g_score松弛、关闭节点和父节点回溯。
 *
 * 参考源：/home/jon/ws11/src/grid_path_searcher_2d/src/grid_search_astar.cpp
 * 及同包grid_map_2d.cpp。保留其搜索主线，移除ROS地图/米制坐标依赖；
 * 八邻域改为比赛的四邻域，八角启发改为曼哈顿距离，增加输入状态校验、
 * 行序转换、未知格代价与临时禁行。没有节点入口，不会自己启动或发布动作。
 */
#include "rmtc_connection/astar.hpp"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <limits>
#include <queue>

namespace rmtc::planning {
namespace {

/// 优先队列保留入队时的g，便于丢弃已被更短路线替代的旧队列条目。
struct OpenNode {
  int index;
  std::int64_t g;
  std::int64_t h;
  std::uint64_t sequence;
};

struct HigherPriority {
  /// std::priority_queue默认取最大值；反向比较让较小f优先，同f时优先较小h。
  bool operator()(const OpenNode& left, const OpenNode& right) const noexcept {
    if (left.g + left.h != right.g + right.h)
      return left.g + left.h > right.g + right.h;
    if (left.h != right.h)
      return left.h > right.h;
    return left.sequence > right.sequence;
  }
};

/// 等代价时固定尝试下、右、上、左；不依赖随机数，也不允许对角移动。
constexpr std::array<GridIndex, 4> kDirections{{{0, 1}, {1, 0}, {0, -1}, {-1, 0}}};

/** @brief 校验尺寸、数据长度、占据值和行序，防止索引越界或int线性下标溢出。 */
bool valid_map(const GridMap& map) {
  if (map.width <= 0 || map.height <= 0)
    return false;
  const auto count = static_cast<std::int64_t>(map.width) * map.height;
  if (count > std::numeric_limits<int>::max() ||
      static_cast<std::size_t>(count) != map.cells.size())
    return false;
  if (map.row_order != RowOrder::TopToBottom && map.row_order != RowOrder::BottomToTop)
    return false;
  return std::all_of(map.cells.begin(), map.cells.end(),
                     [](std::int8_t value) { return value >= -1 && value <= 100; });
}

/** @brief 判断比赛格坐标是否位于地图内，尚不判断障碍或未知格规则。 */
bool in_bounds(const GridMap& map, GridIndex point) {
  return point.x >= 0 && point.y >= 0 && point.x < map.width && point.y < map.height;
}

/** @brief 把合法比赛格坐标转换为cells下标；ROS存储方式在这里翻转行。 */
int to_linear(const GridMap& map, GridIndex point) {
  const int row = map.row_order == RowOrder::TopToBottom ? point.y : map.height - 1 - point.y;
  return row * map.width + point.x;
}

/** @brief 把合法cells下标还原为比赛格坐标；路径输出统一保持y向下。 */
GridIndex from_linear(const GridMap& map, int index) {
  const int row = index / map.width;
  return {index % map.width, map.row_order == RowOrder::TopToBottom ? row : map.height - 1 - row};
}

/** @brief 估计四方向最少步数；用64位运算保存距离和后续总代价。 */
std::int64_t heuristic(GridIndex from, GridIndex to) {
  return std::abs(static_cast<std::int64_t>(from.x) - to.x) +
         std::abs(static_cast<std::int64_t>(from.y) - to.y);
}

/** @brief 根据占据值、未知格选项与本次临时禁行表判断合法下标是否可进入。 */
bool traversable(const GridMap& map, int index, const SearchOptions& options,
                 const std::vector<bool>& blocked) {
  if (blocked[static_cast<std::size_t>(index)])
    return false;
  const auto value = map.cells[static_cast<std::size_t>(index)];
  return value == -1 ? options.allow_unknown : value < options.occupied_threshold;
}

/** @brief 从终点沿父节点回溯并反转，返回含起点/终点的逐格路线。 */
std::vector<GridIndex> reconstruct_path(const GridMap& map, const std::vector<int>& parent,
                                        int goal) {
  std::vector<GridIndex> path;
  for (int index = goal; index >= 0; index = parent[static_cast<std::size_t>(index)]) {
    path.push_back(from_linear(map, index));
  }
  std::reverse(path.begin(), path.end());
  return path;
}

}  // namespace

/**
 * @brief 校验地图/选项后搜索四方向最短路径，返回状态、代价和包含两端的格子序列。
 * 使用曼哈顿启发式；未知格能否通行及其代价由options控制，临时障碍不写回地图。
 * 无路和非法输入通过SearchStatus区分；同起终点仍需通过地图和可通行性校验。
 */
SearchResult AStarPlanner::searchAStar(const GridMap& map, GridIndex start, GridIndex goal,
                                       const SearchOptions& options) const {
  SearchResult result;
  if (!valid_map(map)) {
    result.status = SearchStatus::InvalidMap;
    return result;
  }
  if (options.occupied_threshold < 1 || options.occupied_threshold > 100 ||
      options.unknown_cost < 1 ||
      std::any_of(
          options.blocked_cells.begin(), options.blocked_cells.end(),
          [&](GridIndex point) { return !in_bounds(map, point); })) {
    result.status = SearchStatus::InvalidOptions;
    return result;
  }
  if (!in_bounds(map, start)) {
    result.status = SearchStatus::StartOutOfBounds;
    return result;
  }
  if (!in_bounds(map, goal)) {
    result.status = SearchStatus::GoalOutOfBounds;
    return result;
  }

  const auto count = map.cells.size();
  std::vector<bool> blocked(count, false);
  for (const auto point : options.blocked_cells)
    blocked[static_cast<std::size_t>(to_linear(map, point))] = true;
  const int start_index = to_linear(map, start);
  const int goal_index = to_linear(map, goal);
  if (!traversable(map, start_index, options, blocked)) {
    result.status = SearchStatus::StartBlocked;
    return result;
  }
  if (!traversable(map, goal_index, options, blocked)) {
    result.status = SearchStatus::GoalBlocked;
    return result;
  }

  // 与ws11相同：g_score存当前最短代价，parent记录前驱，closed避免重复扩展。
  // 总格数与单步代价均不超过INT_MAX，最优简单路径代价和启发项可安全用int64保存。
  std::vector<std::int64_t> g_score(count, std::numeric_limits<std::int64_t>::max());
  std::vector<int> parent(count, -1);
  std::vector<bool> closed(count, false);
  std::priority_queue<OpenNode, std::vector<OpenNode>, HigherPriority> open_queue;
  std::uint64_t sequence = 0;
  g_score[static_cast<std::size_t>(start_index)] = 0;
  open_queue.push({start_index, 0, heuristic(start, goal), sequence++});

  while (!open_queue.empty()) {
    const auto entry = open_queue.top();
    open_queue.pop();
    const auto current_index = static_cast<std::size_t>(entry.index);
    if (closed[current_index] || entry.g != g_score[current_index])
      continue;
    closed[current_index] = true;
    const auto current = from_linear(map, entry.index);
    result.visited.push_back(current);
    if (entry.index == goal_index) {
      result.path = reconstruct_path(map, parent, goal_index);
      result.cost = entry.g;
      result.status = SearchStatus::Found;
      return result;
    }

    for (const auto direction : kDirections) {
      const GridIndex neighbor{current.x + direction.x, current.y + direction.y};
      if (!in_bounds(map, neighbor))
        continue;
      const auto neighbor_index = static_cast<std::size_t>(to_linear(map, neighbor));
      if (closed[neighbor_index] ||
          !traversable(map, static_cast<int>(neighbor_index), options, blocked))
        continue;
      const std::int64_t step_cost = map.cells[neighbor_index] == -1 ? options.unknown_cost : 1;
      const auto new_g = entry.g + step_cost;
      if (new_g >= g_score[neighbor_index])
        continue;
      g_score[neighbor_index] = new_g;
      parent[neighbor_index] = entry.index;
      open_queue.push(
          {static_cast<int>(neighbor_index), new_g, heuristic(neighbor, goal), sequence++});
    }
  }
  return result;  // 默认NoPath；失败时不返回可能误导调用方的半条路线。
}

}  // namespace rmtc::planning
