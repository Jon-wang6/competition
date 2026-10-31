/**
 * @file stage2_resource_economy.hpp
 * @brief 第二阶段资源机会估计：用公开地图覆盖、本人采集和确认消失记录调节偏好。
 *
 * 所有数值都是本队观察得到的估计，不读取敌方得分或裁判全图。已经采集的资源
 * 也会进入缺失记忆，故用max(本人采集值,确认消失值)作为移除量保守下界，避免相加
 * 重复扣除。地图覆盖只降低继续搜索的偏好，不削减已知资源本身的实际计分收益。
 */
#pragma once

#include "rmtc_connection/map_memory.hpp"
#include "rmtc_connection/stage2_config.hpp"

#include <algorithm>
#include <cmath>

namespace rmtc {

/// 用于决策和日志的资源机会估计；factor为偏好系数，绝非裁判真实剩余资源量。
struct ResourceEconomy {
  double coverage{0};
  double confirmed_removed_value{0};
  double known_available_value{0};
  double collection_factor{1};
  double phase_factor{1};
  double exploration_factor{1};
};

/**
 * @brief 根据本队本场记忆估计采集需求和未知区域价值；不修改地图、分数或任务。
 * 已知仍存在的资源不因地图开完而失去价值；采集满分后的任务收益由调用方单独保留。
 * unknown格本身已决定每个候选能新增多少视野，本函数仅补充全局阶段的平滑调节。
 */
inline ResourceEconomy assess_resource_economy(const MapMemory& memory,
                                               const PublicPerception& frame,
                                               const Stage2Weights& weights) {
  ResourceEconomy result;
  result.coverage = static_cast<double>(memory.observed_count()) / memory.cells().size();
  double missing = 0;
  for (const auto& [key, remembered] : memory.entities()) {
    (void)key;
    const auto& e = remembered.entity;
    if ((e.type != "resource_normal" && e.type != "resource_high") || !e.value)
      continue;
    if (remembered.first_missing_tick)
      missing += *e.value;
    else
      result.known_available_value += *e.value;
  }
  result.confirmed_removed_value = std::max(frame.score.resource_value, missing);
  if (!weights.adaptive_resources)
    return result;
  const double need =
      std::pow(std::clamp(1.0 - frame.score.resource_value / weights.resource_cap, 0.0, 1.0),
               weights.resource_need_power);
  const double supply = std::pow(
      std::clamp(1.0 - result.confirmed_removed_value / weights.initial_resource_value, 0.0, 1.0),
      weights.resource_depletion_power);
  const double unknown = std::clamp(1.0 - result.coverage, 0.0, 1.0);
  const double opportunities =
      std::clamp(unknown + result.known_available_value / weights.initial_resource_value, 0.0, 1.0);
  result.collection_factor = need * supply;
  result.phase_factor = need * supply * std::pow(opportunities, weights.resource_coverage_power);
  result.exploration_factor =
      weights.exploration_floor + (1.0 - weights.exploration_floor) * need * supply *
                                      std::pow(unknown, weights.resource_coverage_power);
  return result;
}

}  // namespace rmtc
