/**
 * @file weighted_resource.hpp
 * @brief 资源与探索统一评分：路程＋边缘偏好－资源奖励－新增视野奖励。
 *
 * 只给候选资源评分，不决定路径、不订阅ROS、不读取裁判配置。
 * 路程、边缘、价值、未知格价值及探索时间折扣均由resource_config.hpp配置。
 * 不惩罚资源记忆的新旧，避免旧资源因久未观察而一直被遗留。
 */
#pragma once

#include "rmtc_connection/astar.hpp"
#include "rmtc_connection/resource_config.hpp"
#include <cstdint>
#include <optional>

namespace rmtc::planning {

/// 系数为有限非负数，distance必须>0；前两个字段保留旧调用的聚合初始化顺序。
struct ResourceWeights {
  double edge{kDefaultEdgeWeight};          ///< 每多一格边缘距离增加的代价，越大越偏好靠边资源。
  double value{kDefaultValueWeight};        ///< 每10点资源价值抵扣的代价，越大越偏好高价值资源。
  double distance{kDefaultDistanceWeight};  ///< 实际路程的系数，越大越重视近目标。
  double unknown_cell_value{
      kDefaultUnknownCellValue};  ///< 每个新可见未知格的虚拟资源价值，0关闭竞争。
  std::int64_t exploration_fade_ticks{
      kDefaultExplorationFadeTicks};  ///< 新信息可用窗口不足此值时线性折扣。
  double unknown_edge_bonus{
      kDefaultUnknownEdgeBonus};  ///< 贴边未知格的额外价值倍数B，不影响真实资源价值。
  int unknown_edge_band{kDefaultUnknownEdgeBand};  ///< 地图外边奖励带宽度L，0关闭加成。
  double peek_reward{
      kDefaultPeekReward};  ///< 每个原采集路线看不到的加权未知格，可抵扣多少额外绕路代价。
  int peek_detour_limit{
      kDefaultPeekDetourLimit};  ///< 两次实际采集之间的累计绕路上限，0仅允许等长路线补看。
  int peek_patch_limit{
      kDefaultPeekPatchLimit};  ///< 额外绕路只奖励小型未知连通块，0不限制；等长路线不受限。
};

/// 代价明细；value_reward与exploration_reward为正数，计算total时减去。
struct WeightedResourceCost {
  std::int64_t astar_distance{0};
  int edge_distance{0};
  double value_units{0};
  double edge_cost{0};
  double value_reward{0};
  double distance_cost{0};
  std::size_t new_cells{0};
  double exploration_scale{1};
  double exploration_reward{0};
  double total{0};
  double weighted_new_cells{0};  ///< 新视野未知格边缘权重之和，不含基础价值及时间折扣。
};

/** @brief 检查距离系数为正，其余权重及折扣窗口非负；非法时抛出std::invalid_argument。 */
void validate_resource_weights(const ResourceWeights& weights);

/**
 * @brief 计算K_distance·d_astar＋K_edge·d_edge－K_value·(value+未知虚拟价值)/10。
 * @param astar_distance 从当前实际位置到资源的A*移动步数，须非负。
 * @param position 资源绝对坐标；d_edge=min(x,y,width-1-x,height-1-y)。
 * @param width,height 当前公开感知提供的地图尺寸。
 * @param resource_value 单个资源的value，不是score.resource_value累计值。
 * @param weights 集中的资源/探索权重，distance为正，其余系数非负。
 * @param new_cells 到达位置后新增可见的未知格数；以前已观察过的格子不计奖励。
 * @param exploration_scale 0..1的时间折扣，只降低探索奖励，不削减真实资源价值。
 * @param weighted_new_cells 调用方按未知格位置算出的加权和；省略时按new_cells等值计分。
 * @throws std::invalid_argument 尺寸、坐标、距离、价值、权重或结果非法。
 * 可达性、资源是否消失及剩余Tick须由调用方先过滤；负总分是合法的。
 */
WeightedResourceCost score_resource_candidate(
    std::int64_t astar_distance, GridIndex position, int width, int height, double resource_value,
    const ResourceWeights& weights, std::size_t new_cells = 0, double exploration_scale = 1.0,
    std::optional<double> weighted_new_cells = std::nullopt);

}  // namespace rmtc::planning
