/**
 * @file stage2_collection.hpp
 * @brief 第二阶段独立的时间插值与资源争抢估计；不改变规则分数和第一阶段配置。
 */
#pragma once
#include "rmtc_connection/stage2_config.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace rmtc {
/// 在[0,duration]内从起值平滑过渡至终值；零窗口直接使用终值。
inline double collection_linear(double begin, double end, std::int64_t tick, int duration) {
  const double t = duration > 0 ? std::clamp(static_cast<double>(tick) / duration, 0.0, 1.0) : 1.0;
  return begin + (end - begin) * t;
}
/// 采集任务的机会偏好可随资源争抢窗口减弱；零窗口严格保留原固定系数。
inline double resource_mission_weight_at(const Stage2Weights& weights, std::int64_t tick) {
  return weights.resource_mission_ramp_ticks > 0
      ? collection_linear(weights.resource_mission_weight, weights.resource_mission_weight_end,
                          tick, weights.resource_mission_ramp_ticks)
      : weights.resource_mission_weight;
}
/**
 * @brief 到达任务随时间恢复优先级；资源已确认耗尽时提前恢复，近截止时按路程兜底。
 * travel_steps=-1只计算基础偏好；未连通目标可传曼哈顿下界，不能声称一定能及时到达。
 * 权重只影响决策，不修改真实任务分数，也不会强迫车辆忽略当前致死风险。
 */
inline double reach_priority_at(const Stage2Weights& weights, std::int64_t tick,
                                std::int64_t remaining_ticks, int travel_steps,
                                bool resources_possible) {
  double result = resources_possible
      ? collection_linear(weights.opening_reach_weight, weights.reach_weight_end, tick,
                          weights.reach_ramp_ticks)
      : weights.reach_weight_end;
  if (travel_steps >= 0 && remaining_ticks <= travel_steps + weights.reach_deadline_buffer)
    result = std::max(result, weights.reach_deadline_weight);
  return result;
}
/// 只调整第二阶段联合规划的副本；关闭开关时完全保持固定参数。
inline Stage2Weights collection_weights_at(const Stage2Weights& weights, std::int64_t tick) {
  auto result = weights;
  if (weights.joint_linear_weights) {
    result.joint_future_weight = collection_linear(
        weights.joint_future_weight, weights.joint_future_weight_end, tick, weights.joint_ticks);
    result.joint_observation_offset =
        collection_linear(weights.joint_observation_offset, weights.joint_observation_offset_end,
                          tick, weights.joint_ticks);
  }
  return result;
}
/**
 * @brief 用双方已知地形最短步数估计资源收益折扣，不声称是真实抢到概率。
 * enemy_steps=-1代表已知区域无法连通，不据此断言敌人无路，保持收益不变。
 * 对手越早到，折扣越强；平局连续过渡。仅取最快可见敌车，不叠乘多个敌人。
 */
inline double collection_race_factor(int own_steps, int enemy_steps, std::int64_t tick,
                                     const Stage2Weights& weights) {
  if (enemy_steps < 0 || own_steps < 0)
    return 1.0;
  const double strength = collection_linear(
      weights.competition_weight, weights.competition_weight_end, tick, weights.joint_ticks);
  const double gap = std::clamp(
      (static_cast<double>(own_steps) - enemy_steps) / weights.competition_softness, -60.0, 60.0);
  const double factor = weights.contested_resource_factor +
                        (1.0 - weights.contested_resource_factor) / (1.0 + std::exp(gap));
  return 1.0 - strength * (1.0 - factor);
}
}  // namespace rmtc
