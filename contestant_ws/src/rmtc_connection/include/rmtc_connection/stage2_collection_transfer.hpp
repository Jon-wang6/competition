/**
 * @file stage2_collection_transfer.hpp
 * @brief 将历史高采集参数仅用于前期，再平滑恢复当前参数；不改战斗、风险、任务或事件权重。
 *
 * 开局保留hold_ticks，随后用fade_ticks退出。后期资源价值不归零，恢复的是当前
 * 固定采集权重；每帧只计算配置副本，重复Tick和跨局不会积累衰减。
 *
 * @details 模块协作与维护约定
 * 历史采集参数通过保持期和淡出期平滑过渡，过渡结束恢复当前采集配置。
 * 这是参数调度，不是模型训练；应分别检查前期资源与后期战斗指标是否被影响。
 */
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace rmtc {
/// 默认目标取自历史purpose_46采集部分；开关关闭时不改变现有策略。
struct Stage2CollectionTransferWeights {
  bool enabled{false};
  int hold_ticks{20};           ///< 完整使用历史采集参数的Tick数。
  int fade_ticks{60};           ///< 随后退回当前参数的时长，结束后严格恢复原值。
  double shape{1.0};            ///< 1线性退出；较大值较早退出，较小值较晚退出。
  double resource_weight{3.63877}; ///< 只替换前期采集偏好，不修改实际资源价值或计分。
  double explore_peak{4.57999}; ///< 原探索半衰曲线的前期起值。
  double explore_half_ticks{45.88984}; ///< 历史探索时间尺度。
  double silence_half_ticks{20.29022}; ///< 历史资源空窗的探索时间尺度。
  double contested_factor{.26668}; ///< 争抢资源仍保留的相对价值。
  double competition_softness{1.1375}; ///< 到达距离差的平滑尺度。
  double travel_offset{1.85861}; ///< 仅资源/开图目标使用；战斗目标分母保持当前值。
  double resource_floor{.42361}; ///< 资源候选至少达到统一最优收益的此比例，才优先采集。

  /// 拒绝非法时间窗、非有限数或无界候选收益，不让搜索产生退化曲线。
  void validate() const {
    for(double v:{shape,resource_weight,explore_peak,explore_half_ticks,silence_half_ticks,
                  contested_factor,competition_softness,travel_offset,resource_floor})
      if(!std::isfinite(v)||v<0)throw std::invalid_argument("invalid collection transfer parameter");
    if(hold_ticks<0||fade_ticks<1||hold_ticks+fade_ticks>200||shape<.25||shape>4||
       resource_weight>100||explore_peak>100||explore_half_ticks<=0||silence_half_ticks<=0||
       contested_factor>1||competition_softness<=0||travel_offset<.25||travel_offset>20||resource_floor>1)
      throw std::invalid_argument("collection transfer outside bounds");
  }
};

/** @brief 历史采集部分的剩余比例；战斗状态或到达退出Tick时直接返回0，不留浮点尾项。 */
inline double collection_transfer_alpha(const Stage2CollectionTransferWeights& w,
                                        std::int64_t tick,bool eligible) {
  if(!w.enabled||!eligible||tick>=w.hold_ticks+w.fade_ticks)return 0;
  if(tick<=w.hold_ticks)return 1;
  return std::pow(1-static_cast<double>(tick-w.hold_ticks)/w.fade_ticks,w.shape);
}

/**
 * @brief 只混合采集/探索与资源争抢参数，返回独立副本；alpha=0时完整返回原配置。
 * 资源专用路程分母与资源优先门槛由候选层单独使用，禁止改变全局travel_offset。
 */
template<class Weights>
Weights collection_transfer_weights_at(const Weights& source,double alpha) {
  if(alpha<=0)return source;
  auto result=source;const auto& w=source.collection_transfer;
  const auto blend=[&](double current,double donor){return current+(donor-current)*alpha;};
  result.acquisition.resource_weight=blend(source.acquisition.resource_weight,w.resource_weight);
  result.acquisition.explore_peak=blend(source.acquisition.explore_peak,w.explore_peak);
  result.acquisition.explore_half_ticks=blend(source.acquisition.explore_half_ticks,w.explore_half_ticks);
  result.acquisition.silence_half_ticks=blend(source.acquisition.silence_half_ticks,w.silence_half_ticks);
  result.contested_resource_factor=blend(source.contested_resource_factor,w.contested_factor);
  result.competition_softness=blend(source.competition_softness,w.competition_softness);
  return result;
}
} // namespace rmtc
