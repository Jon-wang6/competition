/**
 * @file stage2_battle_curve_test.cpp
 * @brief 验证战斗曲线保护采集、按时生效、收敛于伤害需求且不累乘参数；不连接裁判或API。
 */
#include "rmtc_connection/stage2_strategy.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
/// 使用Release构建也有效的检查，错误文字直接说明被破坏的策略约束。
void require(bool ok,const char* message) {if(!ok)throw std::runtime_error(message);}

/** @brief 开局和可达资源存在时严格保留旧值；风险系数变化不改变资源、事件及死亡代价。 */
void check_curve_invariants() {
  rmtc::Stage2Weights w;w.battle_curve.enabled=true;
  w.battle_curve.start_tick=80;w.battle_curve.ramp_ticks=100;
  w.battle_curve.combat_end=2;w.battle_curve.search_end=1.5;
  w.battle_curve.hp_risk_end=.5;w.battle_curve.future_risk_end=.5;w.battle_curve.route_risk_end=.5;
  for(int tick:{0,40,80}) {
    const auto p=rmtc::battle_curve_weights_at(w,tick,0,false);
    require(p.combat_weight_end==w.combat_weight_end&&p.hp_loss_cost==w.hp_loss_cost,
            "battle curve changed the protected opening");
  }
  auto p=rmtc::battle_curve_weights_at(w,400,0,true);
  require(p.combat_weight_end==w.combat_weight_end&&p.future_exposure_cost==w.future_exposure_cost,
          "battle curve displaced reachable resource collection");
  p=rmtc::battle_curve_weights_at(w,180,0,false);
  require(p.combat_weight_end==2*w.combat_weight_end&&p.hp_loss_cost==.5*w.hp_loss_cost,
          "late battle curve did not reach its configured preference");
  require(p.acquisition.resource_weight==w.acquisition.resource_weight&&
          p.visible_resource_weight==w.visible_resource_weight&&p.death_cost==w.death_cost&&
          p.damage_cap==w.damage_cap&&p.events.unrivaled_risk_multiplier==w.events.unrivaled_risk_multiplier,
          "battle curve altered scoring, resources or event rules");
  require(rmtc::battle_curve_weights_at(w,180,0,false).combat_weight_end==p.combat_weight_end&&
          w.combat_weight_end!=p.combat_weight_end,"curve mutated source or multiplied across calls");
  w.battle_curve.damage_need_power=1;
  require(rmtc::battle_curve_progress(w.battle_curve,180,250,500,false)==.5,
          "damage need feedback failed at half remaining damage");
  require(rmtc::battle_curve_progress(w.battle_curve,180,500,500,false)==0,
          "full damage score still inherited aggressive soft-risk reductions");
  w.battle_curve.enabled=false;
  require(rmtc::battle_curve_weights_at(w,499,0,false).combat_weight_end==w.combat_weight_end,
          "disabled curve did not restore exact old parameters");
}

/** @brief 真实策略在后期看见资源时仍沿原动作采集，资源被确认消失后才切换到曲线。 */
void check_strategy_resource_guard() {
  rmtc::Stage2Weights w;w.acquisition.enabled=true;w.collection_mode=0;w.rollout.weight=0;
  w.information_per_cell=0;w.frontier_resource_weight=w.frontier_mission_weight=0;
  auto c=w;c.battle_curve.enabled=true;c.battle_curve.combat_end=3;
  c.battle_curve.hp_risk_end=.25;c.battle_curve.future_risk_end=.25;
  rmtc::Stage2Strategy old(w),curved(c);
  rmtc::PublicPerception f;f.session_id="battle-curve";f.tick=250;f.map={9,9};f.vision_radius=4;
  f.self.team=f.self.robot_id="A";f.self.position={4,4};f.self.alive=true;f.self.hp=100;
  rmtc::VisibleEntity resource;resource.type="resource_normal";resource.id="visible";resource.value=10;resource.position={5,4};
  rmtc::VisibleEntity enemy;enemy.type="robot";enemy.team="B";enemy.id="B";enemy.hp=100;enemy.position={4,2};
  f.visible_entities={resource,enemy};
  const auto a=old.decide(f,250),b=curved.decide(f,250);
  require(a.type==b.type&&a.direction==b.direction&&curved.plan().goal=="resource"&&
          curved.plan().battle_curve.value("alpha",-1.0)==0,
          "resource guarding changed a real late pickup action");
  f.tick=251;f.visible_entities={enemy};old.decide(f,249);curved.decide(f,249);
  require(curved.plan().battle_curve.value("alpha",0.0)>0&&
          curved.plan().combat_weight>old.plan().combat_weight,
          "confirmed resource disappearance did not release late combat preferences");
}

/// 参数错误必须在启动时被拒绝，不把无界值带进每Tick决策。
void check_validation() {
  for(double value:{0.,5.,std::numeric_limits<double>::infinity()}) {
    rmtc::Stage2Weights w;w.battle_curve.hp_risk_end=value;bool rejected=false;
    try{rmtc::Stage2Strategy s(w);}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"invalid battle risk multiplier accepted");
  }
}

/** @brief 开局即使受可见火力威胁也只移动，资源拿取期不让rollout重新插入攻击/防御。 */
void check_opening_move_only() {
  rmtc::Stage2Weights w;w.acquisition.enabled=true;w.opening_collection_ticks=80;
  w.rollout.weight=1;w.collection_mode=0;
  rmtc::Stage2Strategy s(w);
  rmtc::PublicPerception f;f.session_id="opening-move-only";f.tick=20;f.map={9,9};f.vision_radius=4;
  f.self.team=f.self.robot_id="A";f.self.position={4,4};f.self.alive=true;f.self.hp=10;
  rmtc::VisibleEntity resource;resource.type="resource_normal";resource.id="pickup";resource.value=10;resource.position={5,4};
  rmtc::VisibleEntity enemy;enemy.type="robot";enemy.team="B";enemy.id="B";enemy.hp=10;enemy.position={4,2};
  f.visible_entities={resource,enemy};
  const auto a=s.decide(f,480);
  require(s.plan().opening_collection&&s.plan().goal=="resource"&&a.type=="move",
          "opening collection spent a Tick attacking or defending");
  for(const auto& score:s.plan().action_scores)
    require(score.at("action")=="move","forbidden opening action entered rollout roots");
  f.tick=80;s.decide(f,420);
  require(!s.plan().opening_collection,"opening action restriction did not expire");
}

/// 截击/无效追逐预算在采集开局后启用；资源线索回来时立即暂停。
void check_purpose_gate() {
  rmtc::Stage2PurposeWeights w;w.enabled=true;w.start_tick=120;w.protect_resources=true;
  require(!rmtc::purpose_policy_active(w,119,false),"purpose policy started during protected opening");
  require(!rmtc::purpose_policy_active(w,200,true),"purpose pursuit displaced resource opportunity");
  require(rmtc::purpose_policy_active(w,120,false),"purpose policy did not activate after opening");
  w.enabled=false;
  require(!rmtc::purpose_policy_active(w,200,false),"disabled purpose policy became active");
}

/** @brief 三名敌车与一堵墙堵住全部邻格时，提前转战，保留攻击而不是被开局规则锁在WAIT。 */
void check_blocked_opening_enters_battle() {
  rmtc::Stage2Weights w;w.opening_collection_ticks=100;w.rollout.weight=0;
  w.hp_loss_cost=0;w.future_exposure_cost=0;
  rmtc::Stage2Strategy s(w);
  rmtc::PublicPerception f;f.session_id="blocked-opening";f.tick=20;f.map={15,15};f.vision_radius=4;
  f.self.team=f.self.robot_id="A";f.self.position={7,7};f.self.alive=true;f.self.hp=100;
  for(auto p:std::vector<rmtc::Position>{{7,6},{7,8},{6,7}}) {
    rmtc::VisibleEntity e;e.type="robot";e.id=std::to_string(p.x)+":"+std::to_string(p.y);
    e.team=e.id;e.hp=100;e.position=p;f.visible_entities.push_back(e);
  }
  rmtc::VisibleEntity wall;wall.type="obstacle";wall.position={8,7};f.visible_entities.push_back(wall);
  const auto a=s.decide(f,480);
  require(s.plan().opening_blocked_battle&&!s.plan().opening_collection&&a.type=="attack",
          "blocked opening did not reenter battle");
}
} // namespace

/// 运行采集约束、时间曲线、真实策略切换和启动校验场景。
int main() {
  try{check_curve_invariants();check_strategy_resource_guard();check_validation();
      check_opening_move_only();check_purpose_gate();
      check_blocked_opening_enters_battle();
      std::cout<<"Battle curve resource/score/phase invariants passed\n";return 0;}
  catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
