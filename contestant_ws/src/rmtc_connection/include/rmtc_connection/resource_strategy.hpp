/**
 * @file resource_strategy.hpp
 * @brief 资源跟踪策略：记住公开资源，规划整批访问顺序，每Tick只走一步。
 *
 * 地图与实体历史由本策略自己的MapMemory保存，不依赖RViz是否开启。
 * 排除确认消失的资源，避开当前可见敌车；历史敌车不作为永久障碍。
 * Weighted/Tour/Nearest只规划已知地形；Joint估算未知远期路线，但只执行已知安全首步。
 * Weighted统一比较采集与探索，Joint把观察点与资源安排进一条开放路线。
 */
#pragma once

#include "rmtc_connection/astar.hpp"
#include "rmtc_connection/frontier_exploration.hpp"
#include "rmtc_connection/map_memory.hpp"
#include "rmtc_connection/resource_route.hpp"
#include "rmtc_connection/resource_peek.hpp"
#include "rmtc_connection/weighted_resource.hpp"
#include "rmtc_connection/coverage_route.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace rmtc {

/// 当前资源决策状态；等待采集不需要额外交互动作，裁判会自动结算资源。
enum class ResourceState {
  Moving,
  AtTarget,
  Exploring,
  NoReachableResource,
  NoResourceWithinBudget,
  Dead
};

/// 当前Tick的计划快照，便于日志检查；route.path包含起点和目标，不直接执行整条路径。
struct ResourcePlan {
  ResourceState state{ResourceState::NoReachableResource};
  std::string resource_key;
  std::optional<VisibleEntity> target;
  std::int64_t last_seen_tick{0};
  planning::SearchResult route;
  std::size_t candidates{0};             ///< 本帧考虑的未确认消失资源数，包含暂时不可达的资源。
  std::vector<std::string> visit_order;  ///< 实际资源键的访问顺序；Joint的纯观察点不写入此列表。
  std::int64_t total_cost{0};  ///< Tour为整批已知路线；Joint为乐观整条路线；单目标模式为当前段。
  std::string order_method{
      "none"};  ///< 日志方法名：如optimal_tour、weighted、frontier、joint_coverage_estimate。
  bool order_replanned{false};  ///< 是否重算整体顺序，便于核对缓存与计算频率。
  std::optional<planning::WeightedResourceCost>
      weighted_cost;  ///< 直接资源路线的选择评分；补看增量另记peek。
  std::optional<std::int64_t>
      remaining_ticks;  ///< 实际剩余动作窗口数；Joint仅在0时停止，不保证整条估算路线来得及。
  std::size_t budget_filtered{0};  ///< 已知可达但最低采集耗时超过剩余Tick的候选数。
  std::optional<planning::ExplorationPlan> exploration;  ///< 无资源目标时的探索评分及候选统计。
  std::optional<double>
      best_resource_cost;  ///< Weighted最佳真实资源代价，探索胜出时仍保留以便比较。
  std::optional<double> best_exploration_cost;  ///< Weighted探索候选代价，同分优先真实资源。
  std::optional<double>
      exploration_cost_limit;  ///< 有资源时探索须严格低于min(0,资源代价)，无资源时不限制。
  std::optional<planning::ResourcePeek>
      peek;                          ///< 保留采集目标的一步补看，独立记录增量收益和额外距离。
  std::int64_t peek_detour_used{0};  ///< 两次实际采集之间已确认执行的额外绕路步数。
};

class ResourceStrategy {
 public:
  /// 选择算法并校验两组旧策略权重；Joint参数独立存放在CoverageWeights中。
  explicit ResourceStrategy(ResourceOrdering ordering = kDefaultResourceOrdering,
                            planning::ResourceWeights weights = {},
                            planning::ExplorationWeights exploration_weights = {});

  /// 清空上一场地图、资源目标和动作缓存；由控制节点确认新场次后调用。
  void reset();

  /**
   * @brief 更新公开记忆，按所选策略规划，只返回当前Tick的一步动作。
   * @param frame 已通过parse_public_perception校验的本队完整感知。
   * @param remaining_ticks 剩余动作窗口数；省略时不推测比赛总长度。
   * Weighted用它过滤来不及采集的资源，并逐渐降低探索奖励；Tour/Nearest只在
   * 无资源时的探索兜底使用此预算。Joint仅在剩余0时等待，不限制整条路线长度。
   * @return move/wait意图，不附带Tick和场次，也不创建任何ROS发布器。
   * Joint把采集和观察点合排，未知地形按空地估算远期距离，但执行首步必须已知可走。
   * Joint用BFS计算单位步长距离，小规模顺序用精确规划，大规模用局部优化；
   * 计划随观察滚动更新，估算不代表真实全图最优，也不保证在时限内完成。
   * Tour优先整批已知路线，小规模精确、大规模最近邻兜底；Nearest优先最近单段。
   * Tour在目标或地形等发生变化时重排，常规沿路移动复用顺序。
   * 每帧按self.position重新计算；移动失败、重生或方向效果不会错误推进路径下标。
   * 相同场次和Tick直接返回缓存动作；旧Tick/已结束场次返回wait，不污染最新计划。
   * Weighted按统一代价比较资源与探索，二者均计入新增视野奖励，末段按剩余窗口
   * 降低探索奖励；有资源时，探索收益须超过自身路程/边缘成本且优于采集。
   * 脚下资源优先采集、收支相抵或同分优先真实资源；未知价值接近0自然优先采集。
   * 按peek_reward选择等长多看路线或有限的小绕路；只奖励原采集路线看不到的格。
   * 两次实际采集之间累计额外绕路；失败移动不扣预算，实际资源分数上升才补充预算。
   * Tour/Nearest保持有资源先采集、没有资源再探索，其探索兜底为后续采集预留时间。
   * 所有策略实际执行的首步都只走已知空地；无安全目标时等待。
   * @throws std::exception 地图输入异常或A*内部输入校验失败时交给控制节点回退WAIT。
   */
  Action decide(const PublicPerception& frame,
                std::optional<std::int64_t> remaining_ticks = std::nullopt);

  /// 返回最近一个被接受Tick的目标、完整路线和状态；调用decide前为空目标。
  const ResourcePlan& plan() const noexcept {
    return plan_;
  }

  /// 返回本策略独立保存的公开记忆，仅供查询，不修改RViz节点的地图。
  const MapMemory& memory() const noexcept {
    return memory_;
  }

  /// 设置Joint专用参数，下次规划时校验和使用；不改其他策略、不清空地图记忆。
  /// 默认值来自resource_config.hpp，离线评估和ROS节点可使用同一份配置。
  void set_coverage_weights(planning::CoverageWeights weights) {
    coverage_weights_ = weights;
  }

 private:
  /**
   * @brief 执行Joint分支；调用前decide已更新记忆、清空本帧计划并检查存活状态。
   * grid/blocked_cells来自公开观察；返回动作同时更新plan_和cached_action_。
   */
  Action decide_joint(const PublicPerception& frame, const planning::GridMap& grid,
                      planning::GridIndex start,
                      const std::vector<planning::GridIndex>& blocked_cells,
                      std::optional<std::int64_t> remaining_ticks);

  MapMemory memory_;
  planning::AStarPlanner planner_;
  planning::ResourceRoutePlanner route_planner_;
  planning::CoverageRoutePlanner coverage_planner_;
  planning::CoverageWeights coverage_weights_;
  ResourceOrdering ordering_;
  planning::ResourceWeights weights_;
  planning::ExplorationWeights exploration_weights_;
  ResourcePlan plan_;
  Action cached_action_{Action::wait()};
  std::int64_t peek_detour_used_{0};
  std::optional<planning::GridIndex> pending_peek_position_;
  std::int64_t pending_peek_extra_{0};
};

/// 将资源状态转换成稳定日志名称，不依赖终端语言或自然语言解析。
const char* resource_state_name(ResourceState state) noexcept;

}  // namespace rmtc
