/**
 * @file weighted_resource.cpp
 * @brief 资源与新视野统一评分；边缘指地图四条外边，不包括墙边或未知区边界。
 *
 * 所有数据都由调用方传入。本函数是启发式评分，不承诺整场最短路或最大收益。
 */
#include "rmtc_connection/weighted_resource.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rmtc::planning {

/** @brief 拒绝负权重、NaN及无穷值，保持清边/价值偏好的约定并保证比较有效。 */
void validate_resource_weights(const ResourceWeights& weights) {
  if (!std::isfinite(weights.edge) || !std::isfinite(weights.value) || weights.edge < 0 ||
      weights.value < 0 || !std::isfinite(weights.distance) || weights.distance <= 0 ||
      !std::isfinite(weights.unknown_cell_value) || weights.unknown_cell_value < 0 ||
      weights.exploration_fade_ticks < 0 || !std::isfinite(weights.unknown_edge_bonus) ||
      weights.unknown_edge_bonus < 0 || weights.unknown_edge_band < 0 ||
      !std::isfinite(weights.peek_reward) || weights.peek_reward < 0 ||
      weights.peek_detour_limit < 0 || weights.peek_patch_limit < 0)
    throw std::invalid_argument(
        "resource weights require finite distance>0, other coefficients>=0 and fade_ticks>=0");
}

/** @brief 校验候选输入，计算到地图四边的最小格数，再生成代价及其分项。 */
WeightedResourceCost score_resource_candidate(std::int64_t astar_distance, GridIndex position,
                                              int width, int height, double resource_value,
                                              const ResourceWeights& weights, std::size_t new_cells,
                                              double exploration_scale,
                                              std::optional<double> weighted_new_cells) {
  validate_resource_weights(weights);
  const double gain = weighted_new_cells.value_or(static_cast<double>(new_cells));
  if (width <= 0 || height <= 0 || position.x < 0 || position.y < 0 || position.x >= width ||
      position.y >= height || astar_distance < 0 || !std::isfinite(resource_value) ||
      resource_value < 0 || !std::isfinite(exploration_scale) || exploration_scale < 0 ||
      exploration_scale > 1 || new_cells > static_cast<std::uint64_t>(width) * height ||
      !std::isfinite(gain) || gain < new_cells || (new_cells == 0 && gain != 0))
    throw std::invalid_argument("invalid resource position, dimensions, A* distance or value");
  WeightedResourceCost result;
  result.astar_distance = astar_distance;
  result.edge_distance =
      std::min({position.x, position.y, width - 1 - position.x, height - 1 - position.y});
  result.value_units = resource_value / 10.0;
  result.edge_cost = weights.edge * result.edge_distance;
  result.value_reward = weights.value * result.value_units;
  result.distance_cost = weights.distance * static_cast<double>(astar_distance);
  result.new_cells = new_cells;
  result.weighted_new_cells = gain;
  result.exploration_scale = exploration_scale;
  result.exploration_reward =
      weights.value * weights.unknown_cell_value / 10.0 * gain * exploration_scale;
  result.total =
      result.distance_cost + result.edge_cost - result.value_reward - result.exploration_reward;
  if (!std::isfinite(result.edge_cost) || !std::isfinite(result.value_reward) ||
      !std::isfinite(result.distance_cost) || !std::isfinite(result.exploration_reward) ||
      !std::isfinite(result.total))
    throw std::invalid_argument("resource weighted score overflowed");
  return result;
}

}  // namespace rmtc::planning
