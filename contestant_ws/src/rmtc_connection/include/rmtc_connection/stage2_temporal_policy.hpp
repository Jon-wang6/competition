/**
 * @file stage2_temporal_policy.hpp
 * @brief 按参数含义生成每Tick独立副本；固定采集权重不参与衰减。
 *
 * 任务、攻击已有独立时间线；本层为记忆可信度、争抢、资源前瞻和交火机会
 * 提供可训练的起值/终值过渡。原配置不被逐帧改写，避免指数式累乘或跨局残留。
 */
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace rmtc {
/// 起值读取对应原配置字段；只保存终值与独立过渡时间，避免重复参数彼此覆盖。
struct Stage2TemporalWeights {
  bool enabled{false};
  int transition_ticks{100}; ///< 各终值到位的时间，不改变500Tick比赛长度。
  double shape{1.0};         ///< 1为线性；大于1偏后过渡，小于1偏早过渡。
  double resource_age_end{8.0}; ///< 后期资源记忆可信度的时间尺度；当帧可见仍为完整可信度。
  double competition_end{1.0}; ///< 后期对可见对手抢夺估计的采纳强度。
  double lookahead_end{0.0}; ///< 后期已知多资源的前瞻权重。
  double route_information_end{0.0}; ///< 后期等长路径的顺路开图偏好。
  double third_party_attack_end{0.4}; ///< 后期可攻击交火对手的机会偏好。
  double third_party_risk_end{0.35}; ///< 后期转火风险折扣；不改变最坏致死上界。

  /// 起值由Stage2Weights校验，这里只校验本层终值及形状。
  void validate() const {
    for (double v:{shape,resource_age_end,competition_end,lookahead_end,
                   route_information_end,third_party_attack_end,third_party_risk_end})
      if (!std::isfinite(v) || v<0) throw std::invalid_argument("invalid temporal parameter");
    if (transition_ticks<1 || transition_ticks>500 || shape<.25 || shape>4 ||
        resource_age_end<=0 || competition_end>1 || route_information_end>1 ||
        third_party_attack_end>1 || third_party_risk_end>1 || lookahead_end>3)
      throw std::invalid_argument("temporal parameter outside bounds");
  }
};

/// 参数曲线的进度；不共享原始参数，所以对相同Tick重复调用不产生额外衰减。
inline double temporal_progress(const Stage2TemporalWeights& w,std::int64_t tick) {
  return std::pow(std::clamp(static_cast<double>(tick)/w.transition_ticks,0.0,1.0),w.shape);
}

/**
 * @brief 为本帧创建参数副本；参数类型用模板隔离头文件依赖。
 * 资源采集的acquisition.resource_weight与攻击/任务的独立时间线保持原值。
 */
template<class Weights>
Weights temporal_weights_at(const Weights& source,std::int64_t tick) {
  auto result=source;
  const auto& w=source.temporal;
  if (!w.enabled) return result;
  const double alpha=temporal_progress(w,tick);
  const auto blend=[&](double begin,double end){return begin+(end-begin)*alpha;};
  result.resource_age_ticks=blend(source.resource_age_ticks,w.resource_age_end);
  // 原争抢模块也有线性插值：设为相同的本帧值，避免重复套两次曲线。
  result.competition_weight=result.competition_weight_end=blend(source.competition_weight,w.competition_end);
  result.resource_lookahead_weight=blend(source.resource_lookahead_weight,w.lookahead_end);
  result.route_information_weight=blend(source.route_information_weight,w.route_information_end);
  result.third_party_attack_bonus=blend(source.third_party_attack_bonus,w.third_party_attack_end);
  result.third_party_risk_discount=blend(source.third_party_risk_discount,w.third_party_risk_end);
  return result;
}
} // namespace rmtc
