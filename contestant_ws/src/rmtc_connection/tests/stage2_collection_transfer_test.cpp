/**
 * @file stage2_collection_transfer_test.cpp
 * @brief 检查前期采集移植按时退出、战斗参数完整保留、真实交战状态决策一致及非法输入拒绝。
 *
 * @details 模块协作与维护约定
 * 验证对象：stage2_strategy.hpp。
 * 运行约定：由独立测试可执行文件/CTest执行，断言失败用于定位回归，不是正式选手的逐帧逻辑。
 * 结果边界：通过表示这些构造场景满足约束，不证明所有随机地图均最优或稳定达到某个总分。
 */
#include "rmtc_connection/stage2_strategy.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
/// Release构建也执行约束检查；失败消息说明被破坏的行为。
void require(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}

/** @brief 时间曲线单调退出、重复调用不累乘，资源以外的攻防/任务/机关字段始终不变。 */
void check_weights_and_handoff() {
  rmtc::Stage2Weights w;w.acquisition.enabled=true;w.collection_transfer.enabled=true;
  auto& c=w.collection_transfer;c.hold_ticks=20;c.fade_ticks=60;
  require(rmtc::collection_transfer_alpha(c,0,true)==1&&
          rmtc::collection_transfer_alpha(c,20,true)==1&&
          rmtc::collection_transfer_alpha(c,50,true)==.5&&
          rmtc::collection_transfer_alpha(c,80,true)==0&&
          rmtc::collection_transfer_alpha(c,30,false)==0,"transfer window or battle eligibility failed");
  const auto p=rmtc::collection_transfer_weights_at(w,.5);
  require(p.acquisition.resource_weight==(w.acquisition.resource_weight+c.resource_weight)/2,
          "resource blend has wrong midpoint");
  require(p.opening_combat_weight==w.opening_combat_weight&&p.combat_weight_end==w.combat_weight_end&&
          p.hp_loss_cost==w.hp_loss_cost&&p.death_cost==w.death_cost&&p.future_exposure_cost==w.future_exposure_cost&&
          p.route_risk==w.route_risk&&p.kill_reward_estimate==w.kill_reward_estimate&&
          p.intent_chase_weight==w.intent_chase_weight&&p.third_party_attack_bonus==w.third_party_attack_bonus&&
          p.recent_enemy_search_weight==w.recent_enemy_search_weight&&p.patrol_value==w.patrol_value&&
          p.events.fortress_value==w.events.fortress_value&&p.events.unrivaled_risk_multiplier==w.events.unrivaled_risk_multiplier&&
          p.purpose.start_tick==w.purpose.start_tick&&p.purpose.intercept_weight==w.purpose.intercept_weight&&
          p.travel_offset==w.travel_offset&&p.mission_potential==w.mission_potential&&
          p.damage_cap==w.damage_cap&&p.resource_cap==w.resource_cap,"collection transfer modified combat or rules");
  const auto ended=rmtc::collection_transfer_weights_at(w,0.);
  require(ended.acquisition.resource_weight==w.acquisition.resource_weight&&
          ended.contested_resource_factor==w.contested_resource_factor,"handoff did not restore exact source values");
  require(rmtc::collection_transfer_weights_at(w,.5).acquisition.resource_weight==p.acquisition.resource_weight&&
          w.acquisition.resource_weight!=p.acquisition.resource_weight,"transfer compounded or mutated source weights");
}

/** @brief 相同公开交战状态在前期无资源时、退出后有资源时，均与原版选出相同动作。 */
void check_real_strategy_battle_identity() {
  for(int tick:{10,100}) {
    rmtc::Stage2Weights w;w.acquisition.enabled=true;w.rollout.weight=0;
    auto c=w;c.collection_transfer.enabled=true;
    rmtc::Stage2Strategy base(w),transfer(c);
    rmtc::PublicPerception f;f.session_id="collection-transfer";f.tick=tick;f.map={15,15};f.vision_radius=4;
    f.self.team=f.self.robot_id="A";f.self.position={7,7};f.self.alive=true;f.self.hp=80;
    rmtc::VisibleEntity enemy;enemy.type="robot";enemy.team=enemy.id="B";enemy.hp=30;enemy.position={7,5};
    f.visible_entities={enemy};
    if(tick==100) {rmtc::VisibleEntity r;r.type="resource_normal";r.id="pickup";r.value=10;r.position={8,7};f.visible_entities.push_back(r);}
    const auto a=base.decide(f,500-tick),b=transfer.decide(f,500-tick);
    require(a.type==b.type&&a.direction==b.direction&&
            transfer.plan().collection_transfer.value("alpha",-1.)==0,
            "inactive transfer changed a real combat or post-handoff decision");
  }
}

/// 非有限值、负时间及越界门槛必须在策略创建时被拒绝。
void check_bad_parameters() {
  for(double floor:{-1.,2.,std::numeric_limits<double>::infinity()}) {
    rmtc::Stage2Weights w;w.collection_transfer.resource_floor=floor;bool rejected=false;
    try{rmtc::Stage2Strategy s(w);}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"invalid transfer preference accepted");
  }
}
} // namespace

/// 仅执行本地纯C++验证，不创建ROS节点，不调用模型。
int main(){try{check_weights_and_handoff();check_real_strategy_battle_identity();check_bad_parameters();
  std::cout<<"Collection transfer handoff/combat invariants passed\n";return 0;}
  catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
