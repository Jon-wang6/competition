/**
 * @file resource_peek.cpp
 * @brief 资源路线的一步观察改进：比较额外绕路与额外信息，保持采集目标及预算约束。
 *
 * @details 模块协作与维护约定
 * 输入已有资源目标/路径，在保持采集目标的前提下评估下一步能多看多少未知区域。
 * 绕路预算与原路线可见集合共同限制探查，不将同一批新格子在每一步重复计入收益。
 */
#include "rmtc_connection/resource_peek.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <tuple>

namespace rmtc::planning {
/** @brief 标记原路线所有预期方形视野，再对可走邻格作增量比较，不推测未知格内容。 */
std::optional<ResourcePeek> plan_resource_peek(const GridMap& map, const SearchResult& direct,
                                               int vision_radius,
                                               const std::vector<GridIndex>& blocked_cells,
                                               const ResourceWeights& weights,
                                               std::int64_t detour_budget,
                                               std::optional<std::int64_t> remaining_ticks) {
  validate_resource_weights(weights);
  if (detour_budget < 0 || (remaining_ticks && *remaining_ticks < 0))
    throw std::invalid_argument("invalid peek budget");
  if (weights.peek_reward == 0 || !direct.success() || direct.path.size() < 2)
    return std::nullopt;
  const auto start = direct.path.front(), goal = direct.path.back();
  (void)new_visibility_gain(map, start, vision_radius, 0, 0);  // 复用地图/半径/行顺序校验。
  if (direct.cost != static_cast<std::int64_t>(direct.path.size()) - 1)
    throw std::invalid_argument("peek requires a unit-cost resource route");
  for (const auto point : direct.path)
    if (point.x < 0 || point.y < 0 || point.x >= map.width || point.y >= map.height)
      throw std::invalid_argument("peek resource route outside map");
  const auto index = [&](GridIndex point) {
    const int row = map.row_order == RowOrder::BottomToTop ? map.height - 1 - point.y : point.y;
    return static_cast<std::size_t>(row) * map.width + point.x;
  };
  std::vector<bool> covered(map.cells.size(), false);
  for (const auto point : direct.path)
    for (int y = std::max(0, point.y - vision_radius);
         y <= std::min(map.height - 1, point.y + vision_radius); ++y)
      for (int x = std::max(0, point.x - vision_radius);
           x <= std::min(map.width - 1, point.x + vision_radius); ++x)
        covered[index({x, y})] = true;
  // 以公开记忆里的未知连通块区分“小漏区”和“大片未探索区域”；不查询实际地形。
  // 付费补看只奖励小漏区；不增加采集路程的观察仍可面向任意未知区域。
  std::vector<std::size_t> component_size(map.cells.size(), 0);
  constexpr std::array<GridIndex, 4> directions{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};
  if (weights.peek_patch_limit > 0) {
    for (int y = 0; y < map.height; ++y)
      for (int x = 0; x < map.width; ++x) {
        if (map.cells[index({x, y})] != -1 || component_size[index({x, y})] != 0)
          continue;
        std::vector<GridIndex> component{{x, y}};
        component_size[index({x, y})] = 1;
        for (std::size_t head = 0; head < component.size(); ++head)
          for (const auto dir : directions) {
            const GridIndex next{component[head].x + dir.x, component[head].y + dir.y};
            if (next.x < 0 || next.y < 0 || next.x >= map.width || next.y >= map.height ||
                map.cells[index(next)] != -1 || component_size[index(next)] != 0)
              continue;
            component_size[index(next)] = 1;
            component.push_back(next);
          }
        for (const auto point : component)
          component_size[index(point)] = component.size();
      }
  }
  AStarPlanner planner;
  SearchOptions options;
  options.blocked_cells = blocked_cells;
  std::optional<ResourcePeek> best;
  for (const auto direction : directions) {
    const GridIndex neighbor{start.x + direction.x, start.y + direction.y};
    if (neighbor.x < 0 || neighbor.y < 0 || neighbor.x >= map.width || neighbor.y >= map.height ||
        map.cells[index(neighbor)] != 0 ||
        std::find(blocked_cells.begin(), blocked_cells.end(), neighbor) != blocked_cells.end())
      continue;
    if (neighbor == direct.path[1])
      continue;
    auto route = planner.searchAStar(map, neighbor, goal, options);
    if (!route.success())
      continue;
    const auto extra = 1 + route.cost - direct.cost;
    if (extra < 0)
      throw std::logic_error("peek found a shorter route than A*");
    if (extra > detour_budget || (remaining_ticks && 1 + route.cost >= *remaining_ticks))
      continue;
    ResourcePeek candidate;
    candidate.extra_steps = extra;
    for (int y = std::max(0, neighbor.y - vision_radius);
         y <= std::min(map.height - 1, neighbor.y + vision_radius); ++y)
      for (int x = std::max(0, neighbor.x - vision_radius);
           x <= std::min(map.width - 1, neighbor.x + vision_radius); ++x) {
        const auto cell = index({x, y});
        if (covered[cell] || map.cells[cell] != -1)
          continue;
        if (extra > 0 && weights.peek_patch_limit > 0 &&
            component_size[cell] > static_cast<std::size_t>(weights.peek_patch_limit))
          continue;
        ++candidate.extra_cells;
        candidate.weighted_extra_cells += unknown_cell_edge_factor(
            {x, y}, map.width, map.height, weights.unknown_edge_bonus, weights.unknown_edge_band);
      }
    if (candidate.extra_cells == 0)
      continue;
    candidate.reward = weights.peek_reward * candidate.weighted_extra_cells *
                       exploration_reward_scale(1, remaining_ticks, weights.exploration_fade_ticks);
    candidate.total = weights.distance * extra - candidate.reward;
    if (!std::isfinite(candidate.total) || !std::isfinite(candidate.reward))
      throw std::invalid_argument("peek reward overflow");
    if (candidate.total >= 0)
      continue;
    route.path.insert(route.path.begin(), start);
    ++route.cost;
    candidate.route = std::move(route);
    const auto priority = [](const ResourcePeek& p) {
      return std::make_tuple(p.total, p.extra_steps, -static_cast<std::int64_t>(p.extra_cells),
                             p.route.path[1].y, p.route.path[1].x);
    };
    if (!best || priority(candidate) < priority(*best))
      best = std::move(candidate);
  }
  return best;
}
}  // namespace rmtc::planning
