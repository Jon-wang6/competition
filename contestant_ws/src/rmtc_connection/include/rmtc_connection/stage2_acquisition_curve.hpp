/**
 * @file stage2_acquisition_curve.hpp
 * @brief 全程固定的采集偏好与独立探索衰减；后期看到资源仍保持采集欲望。
 *
 * 资源权重 = resource_weight，整局固定；探索才随Tick与资源空窗衰减。
 * 探索在相同时间曲线上再乘资源空窗折扣；空窗不是资源已耗尽的事实。
 * 本文件只计算决策偏好，不改变资源价值、任务分、视野或攻防结算规则。
 *
 * @details 模块协作与维护约定
 * 采集权重与探索衰减分开：后期出现可拿资源仍可采集，资源空窗只是探索偏好的依据。
 * Tick及空窗长度决定本帧权重，不把上帧已衰减权重反复相乘。
 */
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace rmtc {

/// 采集固定权重、探索半衰Tick、形状及上下限均可独立调参。
struct AcquisitionCurveWeights {
  bool enabled{false};           ///< 关闭时完全沿用原线性加权，方便逐局配对验证。
  double resource_weight{6.0};    ///< 整局固定的资源/采集任务偏好，不随时间降低。
  double explore_peak{1.5};       ///< 开局新增未知视野收益倍率。
  double explore_floor{0.05};     ///< 后期纯资源探索的最低倍率，不影响找人/到达任务本身收益。
  double explore_half_ticks{25.0}; ///< 纯资源探索的时间半衰Tick。
  double explore_shape{1.0};      ///< 探索时间曲线指数。
  double silence_half_ticks{12.0}; ///< 无资源活动的空窗期折扣半衰Tick。
  int silence_grace_ticks{5};     ///< 开局或刚完成采集后的搜索宽限，避免立即放弃邻近区域。

  /// 拒绝非有限数和不合法的曲线，保证可解释的单调时间衰减。
  void validate() const {
    for (double v:{resource_weight,
                   explore_peak,explore_floor,explore_half_ticks,explore_shape,
                   silence_half_ticks})
      if (!std::isfinite(v) || v<0) throw std::invalid_argument("invalid acquisition curve value");
    if (explore_peak<explore_floor || explore_half_ticks<=0 || silence_half_ticks<=0 ||
        explore_shape<0.25 || explore_shape>4 ||
        silence_grace_ticks<0 || silence_grace_ticks>500 || resource_weight>100 ||
        explore_peak>100)
      throw std::invalid_argument("acquisition curve outside bounds");
  }
};

/// 单调的半衰曲线；负Tick按开局处理，有限参数由构造/配置入口校验。
inline double acquisition_decay(double peak,double floor,double half_ticks,double shape,
                                 std::int64_t tick) {
  return floor+(peak-floor)*std::exp2(-std::pow(std::max<std::int64_t>(0,tick)/half_ticks,shape));
}

/// 每帧只计算一次基础权重；已知资源与纯探索分别使用独立分量。
struct AcquisitionPreference {
  double resource{1}, exploration{1};
  std::int64_t silence_ticks{0};
};

/**
 * @brief 按时间与最近一次可达资源活动计算偏好；不将“没看到”解释为地图资源为0。
 * 有当前可达资源时空窗折扣为1；只降低未知资源搜索，不能抹掉后期新资源。
 */
inline AcquisitionPreference acquisition_preference(const AcquisitionCurveWeights& w,
    std::int64_t tick,std::int64_t last_activity,bool visible_resource) {
  AcquisitionPreference p;
  if (!w.enabled) return p;
  p.silence_ticks=std::max<std::int64_t>(0,tick-last_activity);
  p.resource=w.resource_weight; // 用户要求：后期看到资源仍保持同样的采集偏好。
  const auto quiet=std::max<std::int64_t>(0,p.silence_ticks-w.silence_grace_ticks);
  const double time=std::exp2(-std::pow(std::max<std::int64_t>(0,tick)/w.explore_half_ticks,w.explore_shape));
  const double silence=visible_resource?1.0:std::exp2(-quiet/w.silence_half_ticks);
  p.exploration=w.explore_floor+(w.explore_peak-w.explore_floor)*time*silence;
  return p;
}
} // namespace rmtc
