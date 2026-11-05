/**
 * @file stage2_interference_test.cpp
 * @brief 满伤害后只利用低风险收尾，不追远敌、不耽误自己的资源、不把干扰记为得分。
 *
 * @details 模块协作与维护约定
 * 验证对象：stage2_strategy.hpp。
 * 运行约定：由独立测试可执行文件/CTest执行，断言失败用于定位回归，不是正式选手的逐帧逻辑。
 * 结果边界：通过表示这些构造场景满足约束，不证明所有随机地图均最优或稳定达到某个总分。
 */
#include "rmtc_connection/stage2_strategy.hpp"
#include <iostream>
#include <stdexcept>

/** @brief 关闭与测试无关的探索/前瞻，仍使用真实的攻击先结算与掉血风险。 */
rmtc::Stage2Weights weights() {
  rmtc::Stage2Weights w;w.rollout.weight=0;w.collection_mode=0;
  w.acquisition.enabled=false;w.temporal.enabled=false;w.purpose.enabled=false; // 保留本组攻防边界。
  w.information_per_cell=w.frontier_resource_weight=w.frontier_mission_weight=0;
  w.recent_enemy_search_weight=w.patrol_value=0;w.intent_risk_weight=w.intent_chase_weight=0;
  w.opening_resource_weight=w.resource_weight_end=w.visible_resource_weight=1;
  w.opening_combat_weight=w.combat_weight_end=1;w.interference.weight=.35;
  return w;
}

/** @brief 创建当前真实可见的敌车，远处目标也必须保留合法可见坐标。 */
rmtc::VisibleEntity enemy(int x,int y,double hp=10,std::string id="B") {
  rmtc::VisibleEntity e;e.type="robot";e.id=id;e.team=id;e.position={x,y};e.hp=hp;
  return e;
}

/** @brief 伤害刚满分的公开场景；不提供敌方累计得分、内部动作或隐藏坐标。 */
rmtc::PublicPerception frame(std::vector<rmtc::VisibleEntity> enemies) {
  rmtc::PublicPerception f;f.session_id="interference";f.map={9,9};f.vision_radius=9;
  f.self.team="A";f.self.position={4,4};f.self.hp=100;f.self.alive=true;
  f.score.effective_damage=500;f.visible_entities=std::move(enemies);return f;
}

/** @brief 验证战术边界，尤其不能用对方潜在损失掩盖本车的致死风险和计分任务。 */
int main() {
  try {
    auto w=weights();rmtc::Stage2Strategy strategy(w);
    auto f=frame({enemy(5,4)});
    if(strategy.decide(f).type!="attack"||strategy.plan().interference_utility<=0||
        strategy.plan().combat_points!=0)
      throw std::runtime_error("safe finishing opportunity missing or counted as real score");
    strategy.reset();f.self.hp=30;
    if(strategy.decide(f).type=="attack"||strategy.plan().interference_utility!=0)
      throw std::runtime_error("low HP interference consumed own safety reserve");
    strategy.reset();f=frame({enemy(5,4),enemy(4,5,100,"C")});
    if(strategy.decide(f).type=="attack"||strategy.plan().interference_utility!=0)
      throw std::runtime_error("multiple incoming attacks exceeded interference budget");
    strategy.reset();f=frame({enemy(8,4)});
    if(strategy.decide(f).type!="wait"||strategy.plan().goal=="combat")
      throw std::runtime_error("full-score interference generated a distant chase");
    strategy.reset();f=frame({enemy(5,4)});
    rmtc::VisibleEntity resource;resource.type="resource_normal";resource.id="own-resource";
    resource.position={4,3};resource.value=10;f.visible_entities.push_back(resource);
    auto action=strategy.decide(f);
    if(action.type!="move"||action.direction!="up"||strategy.plan().interference_utility!=0)
      throw std::runtime_error("interference delayed own available resource points");
    w.interference.weight=0;strategy=rmtc::Stage2Strategy(w);f=frame({enemy(5,4)});
    if(strategy.decide(f).type=="attack")
      throw std::runtime_error("disabled policy changed the capped-score baseline");
    std::cout<<"Postcap interference checks passed\n";return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
