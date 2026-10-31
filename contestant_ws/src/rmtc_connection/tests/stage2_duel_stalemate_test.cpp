/** @file stage2_duel_stalemate_test.cpp
 * @brief 验证连续互防证据、重复/断帧/新敌人重置及真实策略能试探但保留伤害上界。
 */
#include "rmtc_connection/stage2_strategy.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
/// Release构建中也执行行为断言，不依赖会被NDEBUG关闭的assert。
void require(bool v,const char* message){if(!v)throw std::runtime_error(message);}

/// 创建只有公开状态的普通互防场景，不提供隐藏地形或对手当前指令。
rmtc::PublicPerception frame(int tick){
  rmtc::PublicPerception f;f.session_id="duel";f.tick=tick;f.map={15,15};f.vision_radius=4;
  f.self.team=f.self.robot_id="A";f.self.alive=true;f.self.position={7,7};f.self.hp=20;
  rmtc::VisibleEntity e;e.type="robot";e.team=e.id="B";e.position={7,5};e.hp=100;e.defending=true;
  f.visible_entities={e};return f;
}

/// 连续证据必须真实累积；发生动作/身份/血量/得分变化或漏帧后不再沿用。
void check_evidence(){
  rmtc::DuelStalemateTracker tracker;
  for(int t=80;t<85;++t){auto now=frame(t),old=frame(t-1);
    tracker.observe(now,old,{"defend",""},{&now.visible_entities[0]});}
  require(tracker.ticks()==5,"continuous defense not accumulated");
  auto now=frame(84),old=frame(83);
  tracker.observe(now,old,{"defend",""},{&now.visible_entities[0]});
  require(tracker.ticks()==5,"duplicate Tick accumulated twice");
  now=frame(86);old=frame(84);
  tracker.observe(now,old,{"defend",""},{&now.visible_entities[0]});
  require(tracker.ticks()==0,"missing Tick retained evidence");
  now=frame(87);old=frame(86);now.self.hp=10;
  tracker.observe(now,old,{"defend",""},{&now.visible_entities[0]});
  require(tracker.ticks()==0,"HP loss retained evidence");
  now=frame(88);old=frame(87);now.visible_entities[0].id=now.visible_entities[0].team="C";
  tracker.observe(now,old,{"defend",""},{&now.visible_entities[0]});
  require(tracker.ticks()==0,"new opponent inherited old defense evidence");
  tracker.reset();require(tracker.ticks()==0,"reset retained evidence");
}

/// 在实际策略中让持久互防出现一次试探；原版动作与真实最坏伤害上界仍可对照。
void check_real_strategy(){
  rmtc::Stage2Weights w;w.rollout.weight=0;w.duel_stalemate.enabled=true;
  w.duel_stalemate.ticks=3;w.duel_stalemate.death_risk_factor=0;
  auto disabled=w;disabled.duel_stalemate.enabled=false;
  rmtc::Stage2Strategy strategy(w),baseline(disabled);bool attacked=false;
  for(int t=80;t<94;++t){auto f=frame(t);const auto a=strategy.decide(f,500-t);baseline.decide(f,500-t);
    require(strategy.plan().incoming_damage_bound==baseline.plan().incoming_damage_bound,
        "probe changed confirmed incoming damage bound");
    if(strategy.plan().duel_stalemate.value("active",false)&&a.type=="attack")attacked=true;
  }
  require(attacked,"persistent mutual defense never produced a probe");
  auto f=frame(94);rmtc::VisibleEntity resource;resource.type="resource_normal";resource.id="r";
  resource.position={8,7};resource.value=10;f.visible_entities.push_back(resource);
  strategy.decide(f,406);
  require(!strategy.plan().duel_stalemate.value("active",false),"probe overrode resource protection");
}

/// 非有限倍率、负等待或越界血量应在策略初始化时拒绝。
void check_limits(){
  for(double v:{-1.,2.,std::numeric_limits<double>::infinity()}){
    rmtc::Stage2Weights w;w.duel_stalemate.death_risk_factor=v;bool rejected=false;
    try{rmtc::Stage2Strategy s(w);}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"invalid probe risk accepted");
  }
}
} // namespace

/// 本地纯C++行为验证，不启动ROS、不调用模型。
int main(){try{check_evidence();check_real_strategy();check_limits();std::cout<<"Duel stalemate checks passed\n";return 0;}
  catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
