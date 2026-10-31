/**
 * @file stage2_frontier_value.hpp
 * @brief 用资源发现概率与未完成采集任务评估未知格；只使用本队公开记忆。
 *
 * 未曾见过的资源可能已被他人吃掉，因此先按时间折扣，再分配到剩余未知格。
 * 这是探索机会价值，不会将估计资源写入地图，不会许诺一定能完成任务。
 */
#pragma once
#include "rmtc_connection/map_memory.hpp"
#include "rmtc_connection/stage2_config.hpp"
#include <algorithm>
#include <cmath>

namespace rmtc {
class FrontierOpportunityValue {
 public:
  /**
   * @brief 每Tick准备一次资源/任务缺口；曾见过且已消失的资源也从未见价值中扣除。
   * 初始总价值使用已有规则参数initial_resource_value；不从裁判配置读取资源点位。
   * normal/high仅取本车连续公开帧确认的采集下界，不从总分猜测高/普通资源数量。
   */
  FrontierOpportunityValue(const PublicPerception& frame, const MapMemory& memory,
                           const Stage2Weights& weights, int normal, int high)
      : weights_(weights) {
    if (weights.frontier_resource_weight <= 0 && weights.frontier_mission_weight <= 0)
      return;
    unknown_ = std::count(memory.cells().begin(), memory.cells().end(), std::int8_t{-1});
    double seen_value = 0, normal_value = 10, high_value = 20;
    int normal_seen = 1, high_seen = 1;
    for (const auto& [id, item] : memory.entities()) {
      (void)id;
      const auto& e = item.entity;
      if (!e.value || *e.value <= 0)
        continue;
      if (e.type == "resource_normal") {
        seen_value += *e.value;
        normal_value += *e.value;
        ++normal_seen;
      }
      if (e.type == "resource_high") {
        seen_value += *e.value;
        high_value += *e.value;
        ++high_seen;
      }
    }
    unseen_value_ = std::max(0.0, weights.initial_resource_value - seen_value);
    high_fraction_ = static_cast<double>(high_seen) / (normal_seen + high_seen + 3);
    mean_value_ =
        (1 - high_fraction_) * normal_value / normal_seen + high_fraction_ * high_value / high_seen;
    value_per_cell_ = unknown_ > 0 ? unseen_value_ *
                                         std::exp2(-static_cast<double>(frame.tick) /
                                                   weights.frontier_resource_half_life) /
                                         unknown_
                                   : 0;
    resource_left_ = std::max(0.0, weights.resource_cap - frame.score.resource_value);
    for (const auto& m : frame.missions) {
      if (!m.target || *m.target <= 0 ||
          std::find(frame.score.mission_completed_ids.begin(),
                    frame.score.mission_completed_ids.end(),
                    m.id) != frame.score.mission_completed_ids.end())
        continue;
      double left = 0, rate = 0;
      if (m.type == "normal_resources") {
        left = *m.target - normal;
        rate = (1 - high_fraction_) / mean_value_;
      }
      if (m.type == "high_resources") {
        left = *m.target - high;
        rate = high_fraction_ / mean_value_;
      }
      if (m.type == "resource_value") {
        left = *m.target - frame.score.resource_value;
        rate = 1;
      }
      if (left > 0 && rate > 0)
        missions_.push_back({left, rate});
    }
  }
  /// 给一次视野能新揭示的格子估计额外价值；资源真实计分及每项任务分别封顶。
  double extra(double new_cells) const {
    const double expected = std::clamp(new_cells, 0.0, static_cast<double>(unknown_)) * value_per_cell_;
    double gain = weights_.frontier_resource_weight * std::min(resource_left_, expected) *
                  weights_.resource_points / weights_.resource_cap;
    for (const auto& [left, rate] : missions_)
      gain += weights_.frontier_mission_weight * weights_.mission_points *
              weights_.mission_potential * std::min(1.0, expected * rate / left);
    return gain;
  }
  /// 日志公开估计量，便于分辨探索奖励来自任务缺口还是对未知资源的先验。
  Json describe() const {
    return {{"unknown_cells", unknown_},
            {"unseen_value_upper_bound", unseen_value_},
            {"discounted_value_per_cell", value_per_cell_},
            {"collection_tasks", missions_.size()}};
  }

 private:
  const Stage2Weights& weights_;
  int unknown_{0};
  double unseen_value_{0}, value_per_cell_{0}, mean_value_{10}, high_fraction_{0.2},
      resource_left_{0};
  std::vector<std::pair<double, double>> missions_;
};
}  // namespace rmtc
