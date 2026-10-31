/**
 * @file frontier_exploration.hpp
 * @brief 独立探索候选规划：可作为采集兜底，也可按统一代价与资源目标竞争。
 *
 * 仅使用公开地图记忆；候选为能看见未知格的已知可达空地，路线不穿未知格。
 * 视野按比赛的方形范围计算，障碍不遮挡视野；靠地图外边的未知格可获得额外奖励。
 * 新增视野只是信息量，不猜资源位置；已观察格、地图外区域均不奖励。
 */
#pragma once

#include "rmtc_connection/astar.hpp"
#include "rmtc_connection/resource_config.hpp"

#include <cstddef>
#include <optional>

namespace rmtc::planning {

/// 底层探索权重；Weighted由统一ResourceWeights映射，旧策略仍使用独立兜底参数。
struct ExplorationWeights {
  double distance{kExplorationDistanceWeight};  ///< 有限正数，控制实际路径长度的侧重。
  double edge{kExplorationEdgeWeight};          ///< 有限非负数，控制靠近地图外边缘的侧重。
  double new_cell_reward{0};                    ///< 每个新未知格抵扣的路程代价，0保留旧兜底排序。
  std::int64_t fade_ticks{0};  ///< 到达并预留采集后不足此窗口数时线性减奖，0不衰减。
  double unknown_edge_bonus{
      kDefaultUnknownEdgeBonus};                   ///< B：贴边未知格的额外倍数，0恢复等值未知格。
  int unknown_edge_band{kDefaultUnknownEdgeBand};  ///< L：外边奖励带宽度，0关闭加成。
};

/// 新视野原始格数与加权格数分开保存；weighted_cells不含基础价值K或时间折扣。
struct VisibilityGain {
  std::size_t new_cells{0};
  double weighted_cells{0};
};

/// 探索目标评分；total=路程项+边缘项-时间折扣后的新增视野奖励，越小越优先。
struct ExplorationTarget {
  GridIndex position;
  std::int64_t distance{0};
  int edge_distance{0};
  std::size_t new_cells{0};  ///< 目标位置的方形视野内，当前尚未观察过的格数。
  double total{0};
  double reward_scale{1};
  double reward{0};
  double weighted_new_cells{0};  ///< Σ[1+B×max(0,1-d/L)]，每个未知格按自身位置计算。
};

/// 没有目标表示没有安全且有信息增益的候选，或剩余步数不足。
struct ExplorationPlan {
  std::optional<ExplorationTarget> target;
  SearchResult route;
  std::size_t candidates{0};  ///< 距离预算内、能揭开未知格的可达候选数。
  std::size_t unknown_cells{0};
};

/** @brief 检查路程权重>0，其余权重与折扣窗口>=0且有限；非法时抛invalid_argument。 */
void validate_exploration_weights(ExplorationWeights weights);

/** @brief 校验坐标/加成，返回单格1+B×max(0,1-d/L)；B或L为0时返回1。 */
double unknown_cell_edge_factor(GridIndex cell, int width, int height, double bonus, int band);

/**
 * @brief 统计目标方形视野内未知格数量及边缘加权和，支持两种行顺序。
 * 只读取记忆中的-1格，已见空地/障碍及地图外不计；奖励依据未知格位置而非目标位置。
 * bonus>=0且有限，band>=0；0关闭加成。输入或累计结果非法时抛invalid_argument。
 */
VisibilityGain new_visibility_gain(const GridMap& map, GridIndex target, int vision_radius,
                                   double bonus = kDefaultUnknownEdgeBonus,
                                   int band = kDefaultUnknownEdgeBand);

/** @brief 统计目标方形视野内尚未知的格子，支持两种行顺序；已见格及地图外不计。 */
std::size_t count_new_visible_cells(const GridMap& map, GridIndex target, int vision_radius);

/**
 * @brief 计算探索奖励时间折扣：预留到达路程及一个采集窗口后，按fade_ticks线性折扣。
 * 不提供remaining_ticks或fade_ticks=0时不衰减；没有后续采集时间时返回0。
 */
double exploration_reward_scale(std::int64_t distance, std::optional<std::int64_t> remaining_ticks,
                                std::int64_t fade_ticks);

/**
 * @brief 按距离/外边缘评分选择探索位置，再用A*返回到该位置的最短路线。
 * @param map 当前公开记忆，允许两种行存储顺序；未知格始终不可走。
 * @param start 裁判提供的实际位置，不使用上帧预测坐标。
 * @param vision_radius 官方方形视野半径，取0..4；半径0时无法安全扩展视野。
 * @param blocked_cells 当前可见敌车等临时禁行格，视野外敌人不永久封路。
 * @param weights 独立探索权重；先比分数，再比新增视野，最后同分保留旧探索目标。
 * @param previous_target 上一Tick的探索目标，仅同分时用于稳定选择。
 * @param max_steps 可用的探索步数；调用方预留后续采集窗口，省略时不限制。
 * @param remaining_ticks 当前剩余动作窗口，用于按到达所需路程折扣探索奖励。
 * 用一次BFS取得全部候选的真实四方向距离，与单位代价A*距离一致；用积分图
 * 同时统计原始新增格数及边缘加权和。仅对最终目标调用A*，避免逐个候选重复搜索。
 * @throws std::exception 地图/起点/权重/预算非法，或内部路线不一致时抛出异常。
 */
ExplorationPlan plan_frontier_exploration(
    const GridMap& map, GridIndex start, int vision_radius,
    const std::vector<GridIndex>& blocked_cells = {}, ExplorationWeights weights = {},
    std::optional<GridIndex> previous_target = std::nullopt,
    std::optional<std::int64_t> max_steps = std::nullopt,
    std::optional<std::int64_t> remaining_ticks = std::nullopt);

}  // namespace rmtc::planning
