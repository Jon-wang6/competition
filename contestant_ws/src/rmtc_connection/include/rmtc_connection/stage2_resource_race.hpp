/**
 * @file stage2_resource_race.hpp
 * @brief 整局一次性资源优先：按对手到达期限选择可采集子集，并优化采集次序。
 *
 * 只使用本队已知地形、资源记忆和可见对手的最短路。对手期限是保守估计，
 * 不是对手意图或隐藏地图真值；每帧重算，只执行第一段的一步。
 *
 * @details 模块协作与维护约定
 * 按已知资源子集和对手预计到达期限组织路线；不能把预计期限当作裁判预约或保证。
 * 资源只计一次，下一Tick资源被确认吃掉或路径失效时需按新快照重新决策。
 */
#pragma once

#include "rmtc_connection/astar.hpp"
#include "rmtc_connection/public_data.hpp"
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace rmtc {

/// 所有系数独立于第一阶段；不存在后期停止检查已知资源的Tick开关。
struct ResourceRaceWeights {
  bool enabled{false};        ///< 多图验证后再选择默认；关闭时原决策完全不变。
  int target_limit{16};       ///< 公开规则最多16块；子集动态规划不枚举隐藏资源。
  int horizon{80};            ///< 多段路线的预测步数；还会受比赛剩余Tick限制。
  int deadline_slack{2};      ///< 可见对手最早到达估计允许的余量，不是实际抢到保证。
  double distance_cost{0.1};  ///< 整条路线每步的资源价值成本；0为价值优先、等价值最短路。
  double move_priority{1.5};  ///< 有采集路线时的移动机会价值；动作层仍扣真实生存/事件风险。
  double budget_ms{35.0};     ///< 软计算预算；保留已找到的可行路线，不等待用满。
  bool coverage_fallback{true}; ///< 无可行已知资源时，开局复用第一阶段联合覆盖规划。
  int explore_ticks{80};      ///< 仅约束纯探索；晚期看见资源仍立即调用资源路线规划。
  double coverage_offset{12.0}; ///< 第一阶段同口径的观察点插入成本。
  int explore_method{1};      ///< 0联合覆盖，1按新视野与争抢机会的密度选择观察目标。
  double explore_offset{3.0}; ///< 探索距离分母的固定成本，增大后愿意为更大视野多走。
  double explore_distance_power{1.0}; ///< 距离惩罚指数，不以固定近邻/远邻规则截断。
  double explore_edge_bonus{0.0}; ///< 边界未知格的额外比重。
  double explore_competition{0.5}; ///< 可见对手抢先概率对未知格探索价值的影响。
  double explore_urgency{0.5}; ///< 双方到达时间接近的争夺区额外权重。
  double explore_race_scale{3.0}; ///< 到达步数差的平滑尺度。
  double explore_capture_floor{0.2}; ///< 对手可能更快的区域仍保留的探索价值比例。
  double explore_move_priority{0.3}; ///< 纯探索移动机会价值下限，不等于真实已得分。
  double terminal_information{0.0}; ///< 采集路线终点尚可开启的未知格价值，影响后续发现。
  double mission_weight{0.0}; ///< 采集任务的额外路线价值，按任务分别封顶。
  int task_deadline_buffer{2}; ///< 到达任务的最晚出发缓冲，避免为采集平白丢掉任务。
};

/// 原始资源价值与记忆可信度分离，避免将可见偏好或任务分重复计入整条路线。
struct RaceResource {
  std::string key;
  Position position;
  double value{0}, confidence{1};
  int enemy_steps{-1};        ///< 最快当前可见敌人的已知地形距离；-1为未知。
  std::map<std::string,double> mission_values; ///< 换算成资源价值单位的任务机会。
};

/// forecast_value是规划估计，不计入裁判分；complete=false表示预算内的可行解。
struct ResourceRacePlan {
  planning::SearchResult route;
  std::vector<std::string> order;
  std::vector<int> arrivals;
  double forecast_value{0}, objective{0}, elapsed_ms{0};
  int targets{0}, expanded{0}, route_steps{0};
  bool complete{true}, contested_fallback{false};
};

/**
 * @brief 在当前可通行图上解带预计抢夺期限的子集采集问题。
 * 对固定资源子集，保留每个末资源的最早到达标签；资源价值不随路线重复领取。
 * 在固定距离与期限模型内，比较全部可行子集的“记忆加权价值-路程成本”，
 * 同收益优先短路线。最多16块、复杂度O(2^n*n^2)，超预算返回已找到的路线。
 * passable含已知墙、当前车辆和临时事件墙；information只在等长路径间选择。
 */
ResourceRacePlan plan_resource_race(
    MapSize size, Position start, std::vector<RaceResource> resources,
    int remaining_ticks, const ResourceRaceWeights& weights,
    const std::function<bool(Position)>& passable,
    const std::function<double(Position)>& information,
    double mission_budget = 0.0);

}  // namespace rmtc
