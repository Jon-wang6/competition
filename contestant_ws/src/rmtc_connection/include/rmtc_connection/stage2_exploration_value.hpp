/**
 * @file stage2_exploration_value.hpp
 * @brief 开图收益的平滑限幅，保留超过旧上限后“多看一些”的差别。
 *
 * 仅处理公开记忆计算出的机会价值，不推测或写入隐藏资源。
 */
#pragma once
#include <algorithm>
#include <cmath>

namespace rmtc {
/**
 * @brief 混合旧硬上限与平滑上限；softness=0逐值复现旧行为，1完整平滑。
 * @param raw 本次观察的非负资源、任务与信息机会价值之和。
 * @param cap 奖励尺度及上界；0关闭奖励，避免除零。
 * @param softness [0,1]内平滑程度，由策略构造函数统一校验。
 * @return 不超过cap的收益；完整平滑时超过旧上限仍保持单调，防止密度差别丢失。
 */
inline double exploration_value(double raw, double cap, double softness) {
  if (raw <= 0 || cap <= 0)
    return 0;
  const double hard = std::min(cap, raw);
  if (softness <= 0)
    return hard;
  const double smooth = -cap * std::expm1(-raw / cap);
  return hard + std::clamp(softness, 0.0, 1.0) * (smooth - hard);
}
}  // namespace rmtc
