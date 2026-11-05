/**
 * @file stage2_joint_planner.hpp
 * @brief 第二阶段联合采集提案：复用第一阶段CoverageRoutePlanner，独立评价近期收益。
 *
 * 只读本队公开记忆；长路线只用于估计，执行段截断到已知安全格。资源与任务收益
 * 分别封顶，资源被抢、敌车换位或事件约束变化后重新规划；不修改第一阶段参数。
 *
 * @details 模块协作与维护约定
 * 第一阶段路线负责提出访问顺序，第二阶段再按剩余时间、风险和收益解释该提案。
 * 复用规划器不等于把第一阶段目标直接照搬；敌人抢夺与任务等仍由第二阶段评价。
 */
#pragma once
#include "rmtc_connection/coverage_route.hpp"
#include "rmtc_connection/map_memory.hpp"
#include "rmtc_connection/stage2_config.hpp"

namespace rmtc {

/// 一项经过第二阶段边际计分、记忆老化和争抢折扣后的资源机会。
struct JointOpportunity {
  std::string key;
  Position position;
  double value{0}, resource_points{0}, priority{0};
  std::map<std::string, double> mission_points;
  int nearest_enemy_steps{-1}, direct_steps{-1};  ///< 仅由当前可见敌车及本队已知地形推算。
  double preference{1.0};                        ///< 当前可见资源的偏好，不改变真实分数预算。
};

/// 执行段、访问顺序与后续收益率；并非已经获得的比赛分数。
struct Stage2JointPlan {
  planning::SearchResult route;
  std::string resource_key;
  std::vector<std::string> visit_order;
  double resource_points{0}, mission_points{0}, future_utility{0};
  int estimated_cost{0};
};

class Stage2JointPlanner {
 public:
  /// 新场次清除第一阶段规划器的路线与距离缓存。
  void reset() {
    planner_.reset();
  }
  /**
   * @brief 为当前Tick生成联合路线提案；blocked包含敌车和已校验事件约束。
   * 不产生攻击或防御动作；由第二阶段统一比较。后续收益只计已知可达、时限内的
   * 至多三个资源，并按任务ID、资源计分上限扣除已计入部分，避免重复许诺同一奖励。
   */
  Stage2JointPlan propose(const PublicPerception& frame, const MapMemory& memory,
                          std::vector<JointOpportunity> opportunities,
                          const std::vector<planning::GridIndex>& blocked,
                          std::int64_t remaining_ticks, const Stage2Weights& weights);

 private:
  planning::CoverageRoutePlanner planner_;
};
}  // namespace rmtc
