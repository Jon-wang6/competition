/**
 * @file stage2_duel_stalemate.hpp
 * @brief 识别可观察到的连续互防僵持，为有限攻击试探提供证据；不修改真实伤害上界。
 *
 * 上帧防御不代表本帧必然防御。这里要求连续帧、双方位置/血量和本车得分不变，
 * 且所有当前威胁者持续防御。只给动作层一个可调的软风险折扣，绝不强制攻击。
 */
#pragma once
#include "rmtc_connection/stage2_opponent_intent.hpp"
#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace rmtc {
/// 仅用于无资源可采、伤害未满的普通交战；默认关闭，保留旧版本作严格对照。
struct DuelStalemateWeights {
  bool enabled{false};
  int ticks{12};             ///< 连续多少个无进展互防间隔后允许试探。
  int start_tick{80};        ///< 避开开局资源争夺时间。
  double maximum_hp{40};     ///< 高血量没有必要利用此层改变原攻防比较。
  double death_risk_factor{.15}; ///< 仅攻击候选的预计死亡代价倍率；不修改真实扣分或伤害上界。

  /// 拒绝非法门槛，不允许负风险或无界的等待长度。
  void validate() const {
    if(ticks<2||ticks>80||start_tick<0||start_tick>499||
       !std::isfinite(maximum_hp)||maximum_hp<=0||maximum_hp>100||
       !std::isfinite(death_risk_factor)||death_risk_factor<0||death_risk_factor>1)
      throw std::invalid_argument("invalid duel stalemate parameter");
  }
};

/// 只记住连续证据的长度；新增敌人、移动、掉血、得分、漏帧或复活均重新判断。
class DuelStalemateTracker {
 public:
  /// 新局或显式重置时清除证据，不把上一局的互防状态带入下一局。
  void reset(){ticks_=0;last_tick_=-1;session_.clear();}

  /**
   * @brief 使用公开的上帧状态和本车实际提交的物理动作更新互防证据。
   * threats由上层当前射线/射程判断提供，不把视野内但无法命中的敌人算成威胁者。
   * 重复Tick不重复累计；未知血量或缺失防御姿态不当作“确认稳定”。
   */
  void observe(const PublicPerception& now,const std::optional<PublicPerception>& old,
               const Action& previous_action,const std::vector<const VisibleEntity*>& threats) {
    if(now.session_id==session_&&now.tick==last_tick_)return;
    const bool contiguous=now.session_id==session_&&now.tick==last_tick_+1;
    if(!contiguous)ticks_=0;
    session_=now.session_id;last_tick_=now.tick;
    bool stable=old&&old->session_id==now.session_id&&old->tick+1==now.tick&&
        old->self.alive&&now.self.alive&&!old->self.spawn_protected&&!now.self.spawn_protected&&
        previous_action.type=="defend"&&old->self.position.x==now.self.position.x&&
        old->self.position.y==now.self.position.y&&old->self.hp==now.self.hp&&
        old->score.effective_damage==now.score.effective_damage&&
        old->score.resource_value==now.score.resource_value&&
        old->score.mission_completed_ids==now.score.mission_completed_ids&&!threats.empty();
    for(const auto* enemy:threats) {
      if(!stable)break;
      const auto found=std::find_if(old->visible_entities.begin(),old->visible_entities.end(),
          [&](const auto& e){return same_opponent(e,*enemy);});
      stable=enemy->defending.value_or(false)&&!enemy->spawn_protected.value_or(false)&&
          enemy->hp&&*enemy->hp>0&&found!=old->visible_entities.end()&&found->hp&&
          found->defending.value_or(false)&&found->position.x==enemy->position.x&&
          found->position.y==enemy->position.y&&found->hp==enemy->hp;
    }
    ticks_=stable?std::min(ticks_+1,1000):0;
  }

  /// 返回连续无进展的互防间隔数，仅用于日志和当前帧门槛。
  int ticks()const{return ticks_;}

  /// 机会与事件门槛由策略提供；本层不自行宣称敌人本Tick不会攻击。
  bool active(const DuelStalemateWeights& w,const PublicPerception& frame,bool eligible)const {
    return w.enabled&&eligible&&frame.tick>=w.start_tick&&frame.self.hp<=w.maximum_hp&&ticks_>=w.ticks;
  }
 private:
  int ticks_{0};
  std::int64_t last_tick_{-1};
  std::string session_;
};
} // namespace rmtc
