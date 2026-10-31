/**
 * @file resource_route.hpp
 * @brief 多资源访问顺序规划：A*距离表、开放终点动态规划及滚动计划缓存。
 *
 * 与ROS、裁判和资源观察接口独立，只接受调用方提供的已知地图与资源。
 * 16个以内资源求当前静态快照下访问全部可达资源的最短路程，不返回起点。
 * 不宣称未知地图下整场最优；不主动探索，也不预测视野外资源是否已消失。
 */
#pragma once

#include "rmtc_connection/astar.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace rmtc::planning {

inline constexpr std::size_t kExactResourceLimit = 16;

/// 动态规划结果：indices为资源的零起始下标，cost不包含返回出生点的距离。
struct ResourceVisitOrder {
  std::vector<std::size_t> indices;
  std::int64_t cost{0};
};

/**
 * @brief 根据完整距离表，求从当前位置出发访问全部资源的最短顺序。
 * @param distance 方阵；第0项为当前位置，第i+1项为第i个资源。
 * 所有边须为非负有限整数，主对角线为0；允许非对称距离，不要求回到0。
 * @return 全部资源的顺序及最少总代价；没有资源时返回空顺序、代价0。
 * 同总代价时优先选择下标较小的首资源，其余并列结果保持确定性。
 * 状态为dp[已访问资源集合][最后一个资源]，复杂度O(n²·2^n)。
 * @throws std::invalid_argument 表尺寸/距离非法，或资源超过16个时抛出，
 * 防止意外创建指数级内存；调用方可显式回退其他策略。
 */
ResourceVisitOrder plan_resource_visit_order(
    const std::vector<std::vector<std::int64_t>>& distance);

/// 一个合法观察得到的候选资源；last_seen_tick只用于等长计划的首次目标偏好。
struct ResourceGoal {
  std::string key;
  GridIndex position;
  double value{0};
  std::int64_t last_seen_tick{0};
};

/// 本次滚动规划快照；first_leg只包含到首目标的路，cost为整批访问总代价。
struct ResourceTour {
  std::vector<std::string> keys;
  SearchResult first_leg;
  std::int64_t cost{0};
  std::size_t reachable{0};
  bool exact{true};       ///< 超过16个可达资源时仅选最近资源，明确标记为非精确。
  bool replanned{false};  ///< false表示沿用仍然有效的最优计划，仅更新首段路线。
};

class ResourceRoutePlanner {
 public:
  /** @brief 清空地图、资源与路线缓存；换场时调用，不保留上一局的访问顺序。 */
  void reset();

  /**
   * @brief 用A*构建实际步数距离表，再调用独立函数plan_resource_visit_order。
   * 未知格禁行，当前可见敌车通过blocked临时禁行；排除当前不可达资源。
   * 地图、资源位置/价值或临时禁行变化时重新规划；仅观察时间变化不重算。
   * 静态信息不变、实际位置仍在首段路线时，最优计划的后缀仍最优，复用顺序；
   * 移动失败会从实际原位置继续，幻术偏离或复活到其他位置会重新规划。
   * preferred_key只影响等总路程的首目标，不能压过更短的完整资源路线。
   * 返回的引用在下次plan/reset时可能失效；本对象不发送动作、不访问ROS。
   * @throws std::exception 地图/资源输入不合法或距离表违反连通性约束。
   */
  const ResourceTour& plan(const GridMap& grid, GridIndex start,
                           const std::vector<ResourceGoal>& goals,
                           std::vector<GridIndex> blocked = {},
                           const std::string& preferred_key = {});

 private:
  AStarPlanner astar_;
  bool cached_{false};
  GridMap grid_;
  GridIndex start_;
  std::vector<ResourceGoal> goals_;
  std::vector<GridIndex> blocked_;
  ResourceTour tour_;
};

}  // namespace rmtc::planning
