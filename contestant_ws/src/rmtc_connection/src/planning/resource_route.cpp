/**
 * @file resource_route.cpp
 * @brief 实现已知资源的最短访问顺序和静态路线后缀复用。
 *
 * 单段路径沿用现有A*，整体顺序使用状态压缩动态规划。地图和资源变化才
 * 重建距离表；资源超过官方初始16个时安全回退最近资源，不扩大指数状态。
 * 路程是已知自由格上的移动步数，不等同于受事件、敌人干扰后的实际耗时。
 *
 * @details 模块协作与维护约定
 * 输入目标来自调用方已经发现的资源；距离矩阵只表达当前静态通行图。
 * 小规模子集动态规划选择开放终点访问顺序，不要求返回出生点；缓存只可在其前提仍成立时复用。
 */
#include "rmtc_connection/resource_route.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace rmtc::planning {
namespace {

/** @brief 比较资源几何/价值快照，最后观察Tick变化不使最短路缓存失效。 */
bool same_goals(const std::vector<ResourceGoal>& left, const std::vector<ResourceGoal>& right) {
  if (left.size() != right.size())
    return false;
  for (std::size_t i = 0; i < left.size(); ++i) {
    if (left[i].key != right[i].key || left[i].position != right[i].position ||
        left[i].value != right[i].value)
      return false;
  }
  return true;
}

/** @brief 区分正常无路与非法输入；后者须报告给控制层，不能静默丢弃目标。 */
void check_search(const SearchResult& result) {
  if (result.status == SearchStatus::InvalidMap || result.status == SearchStatus::InvalidOptions ||
      result.status == SearchStatus::StartOutOfBounds ||
      result.status == SearchStatus::GoalOutOfBounds)
    throw std::invalid_argument("invalid map, resource position or blocked cell in resource tour");
}

}  // namespace

/**
 * @brief 用状态压缩动态规划求固定起点、开放终点的最短访问顺序，最多16个目标。
 * distance的0号为起点，其余为目标；返回indices对应目标下标，不包含起点。
 * 同总长优先较小的首目标下标；输入为非负有限距离方阵，主对角线为0，可非对称。
 */
ResourceVisitOrder plan_resource_visit_order(
    const std::vector<std::vector<std::int64_t>>& distance) {
  if (distance.empty() || distance.size() > kExactResourceLimit + 1)
    throw std::invalid_argument(
        "resource distance table must contain a start and at most 16 resources");
  const auto n = distance.size() - 1;
  constexpr auto infinity = std::numeric_limits<std::int64_t>::max() / 4;
  for (std::size_t i = 0; i < distance.size(); ++i) {
    if (distance[i].size() != distance.size())
      throw std::invalid_argument("resource distance table is not square");
    for (std::size_t j = 0; j < distance.size(); ++j) {
      if (distance[i][j] < 0 ||
          distance[i][j] > (infinity - 1) / static_cast<std::int64_t>(distance.size()) ||
          (i == j && distance[i][j] != 0))
        throw std::invalid_argument("invalid or overflowing resource distance");
    }
  }
  if (n == 0)
    return {};
  const std::size_t states = std::size_t{1} << n;
  const auto absent = std::numeric_limits<std::uint8_t>::max();
  std::vector<std::int64_t> dp(states * n, infinity);
  std::vector<std::uint8_t> parent(states * n, absent), first(states * n, absent);
  for (std::size_t i = 0; i < n; ++i) {
    const auto entry = (std::size_t{1} << i) * n + i;
    dp[entry] = distance[0][i + 1];
    first[entry] = static_cast<std::uint8_t>(i);
  }
  for (std::size_t mask = 1; mask < states; ++mask) {
    for (std::size_t last = 0; last < n; ++last) {
      const auto source = mask * n + last;
      if (dp[source] == infinity)
        continue;
      for (std::size_t next = 0; next < n; ++next) {
        if (mask & (std::size_t{1} << next))
          continue;
        const auto destination = (mask | (std::size_t{1} << next)) * n + next;
        const auto cost = dp[source] + distance[last + 1][next + 1];
        if (cost < dp[destination] ||
            (cost == dp[destination] && first[source] < first[destination])) {
          dp[destination] = cost;
          parent[destination] = static_cast<std::uint8_t>(last);
          first[destination] = first[source];
        }
      }
    }
  }
  const auto full = states - 1;
  std::size_t last = 0;
  for (std::size_t candidate = 1; candidate < n; ++candidate) {
    const auto left = full * n + candidate, right = full * n + last;
    if (std::make_pair(dp[left], first[left]) < std::make_pair(dp[right], first[right]))
      last = candidate;
  }
  ResourceVisitOrder result;
  result.cost = dp[full * n + last];
  for (auto mask = full; mask != 0;) {
    result.indices.push_back(last);
    const auto previous = parent[mask * n + last];
    mask &= ~(std::size_t{1} << last);
    last = previous;
  }
  std::reverse(result.indices.begin(), result.indices.end());
  return result;
}

/// 清空Tour的地图、目标和访问顺序缓存；不影响外部地图记忆。
void ResourceRoutePlanner::reset() {
  cached_ = false;
  grid_ = {};
  start_ = {};
  goals_.clear();
  blocked_.clear();
  tour_ = {};
}

/**
 * @brief 按公开已知地形规划整批资源；条件未变且仍在首段路径上时复用访问顺序。
 * blocked先排序去重，避免消息顺序影响缓存；目标间距离由A*计算。
 * 小规模使用精确访问顺序，大规模最近邻兜底；返回引用由本规划器保存，下次调用会更新。
 */
const ResourceTour& ResourceRoutePlanner::plan(const GridMap& grid, GridIndex start,
                                               const std::vector<ResourceGoal>& goals,
                                               std::vector<GridIndex> blocked,
                                               const std::string& preferred_key) {
  // 排序后比较临时禁行集合，同一组敌车的消息顺序改变不触发无意义重算。
  std::sort(blocked.begin(), blocked.end(), [](GridIndex a, GridIndex b) {
    return std::make_pair(a.x, a.y) < std::make_pair(b.x, b.y);
  });
  blocked.erase(std::unique(blocked.begin(), blocked.end()), blocked.end());
  SearchOptions options;
  options.blocked_cells = blocked;
  const bool unchanged = cached_ && grid.width == grid_.width && grid.height == grid_.height &&
                         grid.row_order == grid_.row_order && grid.cells == grid_.cells &&
                         blocked == blocked_ && same_goals(goals, goals_);
  if (unchanged &&
      ((tour_.keys.empty() && start == start_) ||
       (!tour_.keys.empty() && std::find(tour_.first_leg.path.begin(), tour_.first_leg.path.end(),
                                         start) != tour_.first_leg.path.end()))) {
    if (!tour_.keys.empty() && start != start_) {
      auto leg = astar_.searchAStar(grid, start, tour_.first_leg.path.back(), options);
      check_search(leg);
      if (!leg.success())
        throw std::logic_error("cached resource route became unreachable without map changes");
      tour_.cost += leg.cost - tour_.first_leg.cost;
      tour_.first_leg = std::move(leg);
    }
    start_ = start;
    tour_.replanned = false;
    return tour_;
  }

  // 先验证空目标场景的地图/起点，不能因没有资源而掩盖非法地图。
  const auto origin = astar_.searchAStar(grid, start, start, options);
  check_search(origin);
  std::set<std::string> unique_keys;
  for (const auto& goal : goals) {
    if (goal.key.empty() || !unique_keys.insert(goal.key).second || !std::isfinite(goal.value) ||
        goal.value < 0)
      throw std::invalid_argument(
          "resource tour requires unique keys and finite nonnegative values");
  }
  std::vector<std::size_t> ranked(goals.size());
  std::iota(ranked.begin(), ranked.end(), 0);
  std::sort(ranked.begin(), ranked.end(), [&](std::size_t a, std::size_t b) {
    const auto& left = goals[a];
    const auto& right = goals[b];
    return std::make_tuple(left.position != start, left.key != preferred_key, -left.last_seen_tick,
                           -left.value, left.key) <
           std::make_tuple(right.position != start, right.key != preferred_key,
                           -right.last_seen_tick, -right.value, right.key);
  });
  std::vector<std::size_t> reachable;
  std::vector<SearchResult> legs;
  for (const auto index : ranked) {
    auto leg = astar_.searchAStar(grid, start, goals[index].position, options);
    check_search(leg);
    if (!leg.success())
      continue;
    reachable.push_back(index);
    legs.push_back(std::move(leg));
  }
  ResourceTour result;
  result.replanned = true;
  result.reachable = reachable.size();
  if (reachable.size() > kExactResourceLimit) {
    // 自定义练习可额外放置资源；超限时保持原最近资源能力，绝不分配2^n状态。
    std::size_t best = 0;
    for (std::size_t i = 1; i < legs.size(); ++i)
      if (legs[i].cost < legs[best].cost)
        best = i;
    result.exact = false;
    result.keys = {goals[reachable[best]].key};
    result.cost = legs[best].cost;
    result.first_leg = std::move(legs[best]);
  } else if (!reachable.empty()) {
    const auto n = reachable.size();
    std::vector<std::vector<std::int64_t>> distance(n + 1, std::vector<std::int64_t>(n + 1, 0));
    for (std::size_t i = 0; i < n; ++i) {
      distance[0][i + 1] = distance[i + 1][0] = legs[i].cost;
      for (std::size_t j = i + 1; j < n; ++j) {
        const auto leg = astar_.searchAStar(grid, goals[reachable[i]].position,
                                            goals[reachable[j]].position, options);
        check_search(leg);
        if (!leg.success())
          throw std::logic_error("resources reachable from the same start are disconnected");
        // 未知格禁行且每步代价1，当前地图距离对称，可以只计算一半。
        distance[i + 1][j + 1] = distance[j + 1][i + 1] = leg.cost;
      }
    }
    const auto order = plan_resource_visit_order(distance);
    result.cost = order.cost;
    for (const auto index : order.indices)
      result.keys.push_back(goals[reachable[index]].key);
    result.first_leg = std::move(legs[order.indices.front()]);
  }
  // 所有计算成功后才提交缓存；异常不会把半成品标记为可沿用的计划。
  cached_ = true;
  grid_ = grid;
  start_ = start;
  goals_ = goals;
  blocked_ = std::move(blocked);
  tour_ = std::move(result);
  return tour_;
}

}  // namespace rmtc::planning
