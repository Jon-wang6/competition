/**
 * @file stage2_purpose_test.cpp
 * @brief 检查多步截击的概率/地形边界、无资源转战与晚期资源重新夺回优先级。
 */
#include "rmtc_connection/stage2_strategy.hpp"
#include <iostream>
#include <stdexcept>

/// 不使用Release中会消失的assert，失败原因直接指向策略行为。
void require(bool ok,const char* message) {if(!ok)throw std::runtime_error(message);}
/// 创建全图可见的公开场景，避免用隐藏地图作为目标信息。
rmtc::PublicPerception scene(int tick) {
  rmtc::PublicPerception p;p.tick=tick;p.session_id="purpose-test";p.map={9,9};
  p.vision_radius=8;p.self.team="A";p.self.robot_id="A";p.self.position={4,4};
  p.self.alive=true;p.self.hp=100;return p;
}
/// 创建具有稳定队伍和ID的敌车，允许连续帧估计移动惯性。
rmtc::VisibleEntity robot(int x,int y) {
  rmtc::VisibleEntity e;e.type="robot";e.id="B";e.team="B";e.position={x,y};
  e.hp=50;e.spawn_protected=false;return e;
}
/// 检查预测多步保持归一、尊重墙且没有丢掉转向可能，不把单一路线当事实。
void forecast_bounds() {
  auto p=scene(100);p.visible_entities={robot(5,4)};
  rmtc::MapMemory memory;memory.update(p);
  rmtc::OpponentIntentModel model;
  model.opponents.push_back({p.visible_entities[0],{{{5,4},.5},{{6,4},.5}},0,0});
  rmtc::Stage2PurposeWeights w;w.intercept_horizon=6;
  auto forecast=rmtc::forecast_intercepts(p,std::nullopt,model,1,w,
    [](rmtc::Position q){return q.x>=0&&q.y>=0&&q.x<9&&q.y<9&&!(q.x==6&&q.y==4);},
    [](rmtc::Position){return true;});
  require(forecast.opponents[0].size()==7,"all forecast layers available");
  for(const auto& layer:forecast.opponents[0]) {
    double total=0;for(const auto& state:layer) {
      total+=state.probability;
      require(!(state.position.x==6&&state.position.y==4),"forecast crossed known wall");
    }
    require(std::abs(total-1)<1e-9,"forecast probability not normalized");
  }
  require(forecast.opponents[0][2].size()>1,"forecast discarded alternate escape directions");
  require(rmtc::intercept_probability({4,4},0,2,forecast,p.visible_entities)>0,
      "reachable future firing chance missing");
}
/// 无资源转战不是永久阶段锁；400Tick新出现的可采资源仍须重新进入采集。
void resource_return() {
  rmtc::Stage2Weights w;w.purpose.enabled=true;w.purpose.resource_quiet_ticks=0;
  w.purpose.resource_floor=0;w.purpose.battle_floor=0;
  w.acquisition.enabled=true;w.acquisition.resource_weight=4;
  w.temporal.enabled=false;w.phase.enabled=false;w.rollout.weight=0;w.collection_mode=0;
  rmtc::Stage2Strategy s(w);
  auto p=scene(399);p.visible_entities={robot(4,8)};
  s.decide(p,101);
  require(s.plan().purpose.at("mode")=="battle","no resource should select purposeful battle");
  require(s.plan().goal=="combat","visible enemy should produce combat target");
  p.tick=400;rmtc::VisibleEntity e;e.type="resource_normal";e.id="late";e.value=10;e.position={5,4};
  p.visible_entities.push_back(e);s.decide(p,100);
  require(s.plan().purpose.at("mode")=="collect"&&s.plan().target_id=="late",
      "late visible resource did not interrupt battle planning");
  s.reset();p=scene(0);p.visible_entities={robot(4,8)};s.decide(p);
  require(s.plan().purpose.at("chase_spent").empty(),"pursuit budget leaked across session");
}
/// 对手沿长直道逃跑时，本车经过新格子也要受追逐预算约束，不能无限跟随。
void unproductive_chase() {
  rmtc::Stage2Weights w;w.purpose.enabled=true;w.purpose.resource_quiet_ticks=0;
  w.purpose.battle_floor=0;w.purpose.intercept_weight=0;w.purpose.chase_progress_ticks=4;
  w.acquisition.enabled=false;w.temporal.enabled=false;w.phase.enabled=false;
  w.rollout.weight=0;w.collection_mode=0;w.information_per_cell=0;
  w.frontier_resource_weight=w.frontier_mission_weight=0;
  w.patrol_value=w.recent_enemy_search_weight=0;
  rmtc::Stage2Strategy s(w);
  for(int tick=0;tick<=4;++tick) {
    auto p=scene(tick);p.map={30,9};p.self.position={1+tick,4};
    p.visible_entities={robot(5+tick,4)};s.decide(p);
    if(tick<4)require(s.plan().goal=="combat","chase fixture failed to pursue visible enemy");
  }
  require(s.plan().pursuit_cooldowns>0&&s.plan().goal!="combat",
      "long unproductive pursuit failed to yield target");
}
/// 已看见可追击敌人时，过期区域的高巡逻奖励不能让车转身找不存在的更好目标。
void visible_enemy_before_patrol() {
  for(double scale:{0.0,1.0}) {
    rmtc::Stage2Weights w;w.purpose.enabled=true;w.purpose.resource_quiet_ticks=0;
    w.purpose.intercept_weight=0;w.purpose.patrol_visible_scale=scale;
    w.acquisition.enabled=false;w.temporal.enabled=false;w.phase.enabled=false;
    w.rollout.weight=0;w.collection_mode=0;w.information_per_cell=0;
    w.frontier_resource_weight=w.frontier_mission_weight=w.recent_enemy_search_weight=0;
    w.patrol_value=100;
    rmtc::Stage2Strategy s(w);
    for(int tick=0;tick<=10;++tick) {
      auto p=scene(tick);p.map={15,15};p.vision_radius=2;
      p.self.position={2+std::min(tick,5),2+std::max(0,tick-5)};s.decide(p);
    }
    auto p=scene(100);p.map={15,15};p.vision_radius=2;p.self.position={7,7};
    p.visible_entities={robot(7,9)};s.decide(p);
    require(s.plan().purpose.at("visible_pursuit")==true,"visible pursuit not detected");
    require(s.plan().goal==(scale==0?"combat":"patrol"),"patrol visibility gate failed");
  }
}
/// 批量搜索中的非法参数应当启动即失败，不能静默截断成另一种策略。
void parameter_bounds() {
  rmtc::Stage2Weights w;w.purpose.intercept_horizon=100;
  bool failed=false;try {rmtc::Stage2Strategy s(w);}catch(const std::invalid_argument&) {failed=true;}
  require(failed,"unbounded pursuit forecast accepted");
}
int main() {try {forecast_bounds();resource_return();unproductive_chase();visible_enemy_before_patrol();parameter_bounds();std::cout<<"purpose scenarios passed\n";}
  catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
