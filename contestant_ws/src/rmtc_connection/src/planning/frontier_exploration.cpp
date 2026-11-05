/**
 * @file frontier_exploration.cpp
 * @brief 从已知空地选择有信息增益的探索目标，按实际路程及地图外边缘距离排序。
 *
 * 不读取裁判配置，不猜未知地形或资源分布。每帧按真实位置重算，发现资源后由
 * 上层策略决定采集或探索；新增未知格越靠地图四条外边，信息收益可越高。
 * 全图已知或未知区域无法安全接近时返回空目标，已见格不会反复获奖。
 *
 * @details 模块协作与维护约定
 * 候选是可达的观察位置，奖励来自该位置能新增看到的未知格，而不是猜出的真实隐藏资源。
 * 评价时区分路程成本、地图外边缘距离和新增观察面积；无有效候选须由调用方决定等待或其他模式。
 */
#include "rmtc_connection/frontier_exploration.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <tuple>

namespace rmtc::planning {

/** @brief 拒绝负权重、零路程权重及非有限数，避免无效评分或无距离约束的漫游。 */
void validate_exploration_weights(ExplorationWeights weights) {
  if (!std::isfinite(weights.distance) || weights.distance <= 0 || !std::isfinite(weights.edge) ||
      weights.edge < 0 || !std::isfinite(weights.new_cell_reward) || weights.new_cell_reward < 0 ||
      weights.fade_ticks < 0 || !std::isfinite(weights.unknown_edge_bonus) ||
      weights.unknown_edge_bonus < 0 || weights.unknown_edge_band < 0)
    throw std::invalid_argument("invalid exploration distance/edge/reward/fade weights");
}

/** @brief 按未知格自身的外边距离计算有上限的线性加成，四角不会重复叠加两条边。 */
double unknown_cell_edge_factor(GridIndex cell, int width, int height, double bonus, int band) {
  if (width <= 0 || height <= 0 || cell.x < 0 || cell.y < 0 || cell.x >= width ||
      cell.y >= height || !std::isfinite(bonus) || bonus < 0 || band < 0)
    throw std::invalid_argument("invalid unknown cell edge weights or position");
  if (band == 0 || bonus == 0)
    return 1;
  const int edge = std::min({cell.x, cell.y, width - 1 - cell.x, height - 1 - cell.y});
  return 1 + bonus * std::max(0.0, 1 - static_cast<double>(edge) / band);
}

/** @brief 扫描裁剪到地图内的方形视野，分别累计新增格数和按未知格外边距离计算的权重。 */
VisibilityGain new_visibility_gain(const GridMap& map, GridIndex target, int vision_radius,
                                   double bonus, int band) {
  if (map.width <= 0 || map.height <= 0 ||
      static_cast<std::uint64_t>(map.width) * map.height != map.cells.size() ||
      (map.row_order != RowOrder::TopToBottom && map.row_order != RowOrder::BottomToTop) ||
      target.x < 0 || target.y < 0 || target.x >= map.width || target.y >= map.height ||
      vision_radius < 0 || vision_radius > 4 || !std::isfinite(bonus) || bonus < 0 || band < 0)
    throw std::invalid_argument("invalid map/target/radius for visibility gain");
  VisibilityGain result;
  const int right = target.x + std::min(vision_radius, map.width - 1 - target.x);
  const int bottom = target.y + std::min(vision_radius, map.height - 1 - target.y);
  for (int y = std::max(0, target.y - vision_radius); y <= bottom; ++y) {
    const auto row = map.row_order == RowOrder::BottomToTop ? map.height - 1 - y : y;
    for (int x = std::max(0, target.x - vision_radius); x <= right; ++x) {
      if (map.cells[static_cast<std::size_t>(row) * map.width + x] != -1)
        continue;
      ++result.new_cells;
      result.weighted_cells += unknown_cell_edge_factor({x, y}, map.width, map.height, bonus, band);
    }
  }
  if (!std::isfinite(result.weighted_cells))
    throw std::invalid_argument("visibility gain overflow");
  return result;
}

/** @brief 保留原始格数查询接口，不将边缘加成伪装成实际观察到的格数。 */
std::size_t count_new_visible_cells(const GridMap& map, GridIndex target, int vision_radius) {
  return new_visibility_gain(map, target, vision_radius, 0, 0).new_cells;
}

/** @brief 依据实际到达所需步数折扣探索收益，真实资源价值由上层保持不变。 */
double exploration_reward_scale(std::int64_t distance, std::optional<std::int64_t> remaining_ticks,
                                std::int64_t fade_ticks) {
  if (distance < 0 || fade_ticks < 0 || (remaining_ticks && *remaining_ticks < 0))
    throw std::invalid_argument("invalid exploration time budget");
  if (!remaining_ticks)
    return 1;
  if (distance >= *remaining_ticks || *remaining_ticks - distance <= 1)
    return 0;
  if (fade_ticks == 0)
    return 1;
  return std::min(1.0, static_cast<double>(*remaining_ticks - distance - 1) / fade_ticks);
}

/** @brief 遍历已知连通区域，统计候选新增视野，按评分选择目标并生成A*路径。 */
ExplorationPlan plan_frontier_exploration(const GridMap& map, GridIndex start, int vision_radius,
                                          const std::vector<GridIndex>& blocked_cells,
                                          ExplorationWeights weights,
                                          std::optional<GridIndex> previous_target,
                                          std::optional<std::int64_t> max_steps,
                                          std::optional<std::int64_t> remaining_ticks) {
  validate_exploration_weights(weights);
  if (vision_radius < 0 || vision_radius > 4 || (max_steps && *max_steps < 0) ||
      (remaining_ticks && *remaining_ticks < 0))
    throw std::invalid_argument("invalid exploration vision radius or step budget");
  SearchOptions options;
  options.blocked_cells = blocked_cells;
  AStarPlanner planner;
  // 同格搜索复用A*的地图、格值、行顺序及起点校验，不接受未知或被封锁的起点。
  if (!planner.searchAStar(map, start, start, options).success())
    throw std::invalid_argument("invalid exploration map or start");
  const int width = map.width, height = map.height;
  const auto count = map.cells.size();
  const auto index = [width](GridIndex point) {
    return static_cast<std::size_t>(point.y) * width + point.x;
  };
  const auto inside = [width, height](GridIndex point) {
    return point.x >= 0 && point.y >= 0 && point.x < width && point.y < height;
  };
  // 统一还原成比赛y向下的逻辑索引，禁止将RViz的倒序行误当地图方向。
  std::vector<std::int8_t> cells(count);
  for (int y = 0; y < height; ++y) {
    const int source_y = map.row_order == RowOrder::BottomToTop ? height - 1 - y : y;
    std::copy_n(map.cells.begin() + static_cast<std::size_t>(source_y) * width, width,
                cells.begin() + static_cast<std::size_t>(y) * width);
  }
  ExplorationPlan result;
  result.unknown_cells = std::count(cells.begin(), cells.end(), -1);
  if (result.unknown_cells == 0 || vision_radius == 0 || (max_steps && *max_steps == 0))
    return result;
  std::vector<bool> blocked(count, false);
  for (const auto point : blocked_cells)
    if (inside(point))
      blocked[index(point)] = true;

  // 数量和价值各建积分图：边缘加成按未知格自身坐标计算，候选视野仍可O(1)求和。
  const auto stride = static_cast<std::size_t>(width) + 1;
  std::vector<std::int64_t> prefix(stride * (static_cast<std::size_t>(height) + 1), 0);
  std::vector<double> weighted_prefix(prefix.size(), 0);
  for (int y = 0; y < height; ++y)
    for (int x = 0; x < width; ++x) {
      const auto p = (static_cast<std::size_t>(y) + 1) * stride + x + 1;
      prefix[p] = (cells[index({x, y})] < 0 ? 1 : 0) + prefix[p - 1] + prefix[p - stride] -
                  prefix[p - stride - 1];
      const double cell_gain =
          cells[index({x, y})] == -1
              ? unknown_cell_edge_factor({x, y}, width, height, weights.unknown_edge_bonus,
                                         weights.unknown_edge_band)
              : 0;
      weighted_prefix[p] = cell_gain + weighted_prefix[p - 1] + weighted_prefix[p - stride] -
                           weighted_prefix[p - stride - 1];
      if (!std::isfinite(weighted_prefix[p]))
        throw std::invalid_argument("visibility prefix sum overflow");
    }
  const auto gain = [&](GridIndex point) {
    const int left = std::max(0, point.x - vision_radius),
              top = std::max(0, point.y - vision_radius);
    const int right = point.x + std::min(vision_radius, width - 1 - point.x) + 1;
    const int bottom = point.y + std::min(vision_radius, height - 1 - point.y) + 1;
    const auto raw =
        static_cast<std::size_t>(prefix[bottom * stride + right] - prefix[top * stride + right] -
                                 prefix[bottom * stride + left] + prefix[top * stride + left]);
    const double weighted =
        weighted_prefix[bottom * stride + right] - weighted_prefix[top * stride + right] -
        weighted_prefix[bottom * stride + left] + weighted_prefix[top * stride + left];
    // 浮点积分图相减可能产生微小误差；每个格权重至少1，零格视野必须严格为0。
    return VisibilityGain{raw, raw == 0 ? 0.0 : std::max(static_cast<double>(raw), weighted)};
  };
  std::vector<int> distance(count, -1);
  std::vector<GridIndex> queue{start};
  distance[index(start)] = 0;
  constexpr std::array<GridIndex, 4> directions{{{0, 1}, {1, 0}, {0, -1}, {-1, 0}}};
  for (std::size_t head = 0; head < queue.size(); ++head) {
    const auto from = queue[head];
    const int steps = distance[index(from)];
    const auto visibility = gain(from);
    const auto new_cells = visibility.new_cells;
    // 起点已被本帧完整观察过；排除它可以防止输入不完整时反复输出同格探索。
    if (steps > 0 && new_cells > 0) {
      ++result.candidates;
      const int edge = std::min({from.x, from.y, width - 1 - from.x, height - 1 - from.y});
      ExplorationTarget candidate{from, steps, edge, new_cells,
                                  weights.distance * steps + weights.edge * edge};
      candidate.weighted_new_cells = visibility.weighted_cells;
      candidate.reward_scale = exploration_reward_scale(steps, remaining_ticks, weights.fade_ticks);
      candidate.reward =
          weights.new_cell_reward * candidate.weighted_new_cells * candidate.reward_scale;
      candidate.total -= candidate.reward;
      if (!std::isfinite(candidate.total))
        throw std::overflow_error("exploration score overflow");
      const auto priority = [&](const ExplorationTarget& target) {
        return std::make_tuple(target.total, -static_cast<std::int64_t>(target.new_cells),
                               !previous_target || target.position != *previous_target,
                               target.distance, target.position.y, target.position.x);
      };
      if (!result.target || priority(candidate) < priority(*result.target))
        result.target = candidate;
    }
    if (max_steps && steps >= *max_steps)
      continue;
    for (const auto direction : directions) {
      const GridIndex to{from.x + direction.x, from.y + direction.y};
      if (!inside(to))
        continue;
      const auto next = index(to);
      if (distance[next] >= 0 || blocked[next] || cells[next] < 0 ||
          cells[next] >= options.occupied_threshold)
        continue;
      distance[next] = steps + 1;
      queue.push_back(to);
    }
  }
  if (result.target) {
    result.route = planner.searchAStar(map, start, result.target->position, options);
    if (!result.route.success() || result.route.cost != result.target->distance)
      throw std::logic_error("exploration BFS and A* distance disagree");
  }
  return result;
}

}  // namespace rmtc::planning
