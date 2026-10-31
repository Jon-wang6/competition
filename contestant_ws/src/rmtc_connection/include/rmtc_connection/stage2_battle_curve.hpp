/**
 * @file stage2_battle_curve.hpp
 * @brief 采集高峰之后的独立战斗时间曲线，不修改资源价值、采集权重或真实伤害。
 *
 * 每帧只修改参数副本：资源仍可达时可退回原判断，避免战斗调参直接挤掉采集。
 * 伤害越接近满分，曲线可按需求缺口收敛，避免为最后一点伤害承担过大死亡风险。
 */
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace rmtc {
struct Stage2BattleCurveWeights {
  bool enabled{false};             ///< 实验验证通过前默认关闭，旧参数组可完整复现。
  bool protect_resources{true};    ///< 有已知可达资源时使用原攻防/搜索偏好。
  int start_tick{80};              ///< 起效前行为保持原值；不改变开局抢资源的动作。
  int ramp_ticks{120};             ///< 从起效至完整终值的过渡长度。
  double shape{1.0};               ///< 1线性，较大值将变化推迟，较小值提前变化。
  double damage_need_power{0.0};   ///< 0只看时间；正数使接近伤害上限时逐渐恢复原偏好。
  double combat_end{1.0};          ///< 攻击/攻击位置收益的最终倍率。
  double search_end{1.0};          ///< 找人与巡查收益的最终倍率，不放大资源探索。
  double hp_risk_end{1.0};         ///< 每点预计掉血代价的最终倍率，死亡上界仍完整保留。
  double future_risk_end{1.0};     ///< 行动后暴露代价的最终倍率。
  double route_risk_end{1.0};      ///< 路线近期交火代价的最终倍率。
  double third_party_end{1.0};     ///< 对手正与第三方交战时，进攻机会奖励的最终倍率。

  /// 启动时拒绝非法窗口与无界倍率；不能通过本曲线把死亡风险和规则伤害改成零。
  void validate() const {
    if(start_tick<0||start_tick>499||ramp_ticks<1||ramp_ticks>500)
      throw std::invalid_argument("invalid battle curve window");
    if(!std::isfinite(shape)||shape<.25||shape>4||!std::isfinite(damage_need_power)||
        damage_need_power<0||damage_need_power>3)
      throw std::invalid_argument("invalid battle curve shape");
    for(double v:{combat_end,search_end,hp_risk_end,future_risk_end,route_risk_end,third_party_end})
      if(!std::isfinite(v)||v<.25||v>4)
        throw std::invalid_argument("invalid battle curve multiplier");
  }
};

/** @brief 用绝对Tick、可达资源与当前伤害缺口计算进度；重复调用不累乘、不跨局残留。 */
inline double battle_curve_progress(const Stage2BattleCurveWeights& w,std::int64_t tick,
                                    double effective_damage,double cap,bool reachable_resource) {
  if(!w.enabled||(w.protect_resources&&reachable_resource)||tick<=w.start_tick) return 0;
  double alpha=std::pow(std::clamp(static_cast<double>(tick-w.start_tick)/w.ramp_ticks,0.,1.),w.shape);
  if(w.damage_need_power>0 && cap>0)
    alpha*=std::pow(std::clamp(1-effective_damage/cap,0.,1.),w.damage_need_power);
  return alpha;
}

/**
 * @brief 平滑调整战斗/找人/软风险副本；采集、探索、到达任务、死亡代价和事件参数保持原值。
 * 该函数在本帧地形BFS与资源可达性检查之后调用；只改后续评分使用的字段。
 */
template<class Weights>
Weights battle_curve_weights_at(const Weights& source,std::int64_t tick,double damage,bool resource) {
  const auto& w=source.battle_curve;
  const double alpha=battle_curve_progress(w,tick,damage,source.damage_cap,resource);
  if(alpha<=0) return source;
  auto result=source;
  const auto factor=[&](double end){return 1+(end-1)*alpha;};
  result.opening_combat_weight*=factor(w.combat_end);
  result.combat_weight_end*=factor(w.combat_end);
  result.recent_enemy_search_weight*=factor(w.search_end);
  result.patrol_value*=factor(w.search_end);
  result.hp_loss_cost*=factor(w.hp_risk_end);
  result.future_exposure_cost*=factor(w.future_risk_end);
  result.route_risk*=factor(w.route_risk_end);
  result.third_party_attack_bonus*=factor(w.third_party_end);
  return result;
}
} // namespace rmtc
