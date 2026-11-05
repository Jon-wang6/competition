/**
 * @file stage2_enemy_belief_test.cpp
 * @brief 检查空视野证据不能遗忘、完全失去线索不复活、死亡与换局隔离。
 *
 * @details 模块协作与维护约定
 * 验证对象：stage2_enemy_belief.hpp、stage2_enemy_sightings.hpp。
 * 运行约定：由独立测试可执行文件/CTest执行，断言失败用于定位回归，不是正式选手的逐帧逻辑。
 * 结果边界：通过表示这些构造场景满足约束，不证明所有随机地图均最优或稳定达到某个总分。
 */
#include "rmtc_connection/stage2_enemy_belief.hpp"
#include "rmtc_connection/stage2_enemy_sightings.hpp"
#include <iostream>
#include <stdexcept>

/** @brief 构造只包含本车公开视野的帧；观察者每帧最多移动一格。 */
rmtc::PublicPerception perception(int tick,int x,bool target=false) {
  rmtc::PublicPerception f;f.session_id="belief";f.tick=tick;f.map={9,3};f.vision_radius=1;
  f.self.team="A";f.self.alive=true;f.self.hp=100;f.self.position={x,1};
  if(target) {
    rmtc::VisibleEntity enemy;enemy.type="robot";enemy.team="B";enemy.id="B";
    enemy.hp=100;enemy.position={1,1};f.visible_entities.push_back(enemy);
  }
  return f;
}

/** @brief 在实际观察序列上比较条件分布，业务断言不依赖扩散实现的具体系数。 */
int main() {
  try {
    rmtc::MapMemory memory;rmtc::EnemyBeliefTracker beliefs;
    auto update=[&](rmtc::PublicPerception f) {memory.update(f);beliefs.update(f,memory,10,30);};
    update(perception(0,0,true));
    update(perception(1,0));
    if(beliefs.in_view({2,1},0)<=0 || beliefs.in_view({0,1},1)>0)
      throw std::runtime_error("unseen enemy did not leave the observed-empty view");
    update(perception(2,1));update(perception(3,2));
    if(beliefs.in_view({0,1},0)>0 || beliefs.in_view({4,1},0)<=0)
      throw std::runtime_error("previous empty observations were forgotten");
    const rmtc::EnemySightingsField old(*memory.current(),memory,10,30);
    if(old.in_view({0,1},0)<=0)
      throw std::runtime_error("regression scene does not exercise the old revived-location problem");
    if(beliefs.in_view({4,1},9)>std::exp2(-3.0/10)+1e-9)
      throw std::runtime_error("history confidence exceeds its age budget");

    // 一步内所有可能落点都被确认为空：可能死亡或线索失效，不能下一帧凭空重生。
    memory=rmtc::MapMemory{};beliefs.reset();
    update(perception(0,0,true));update(perception(1,1));update(perception(2,0));
    if(beliefs.in_view({4,1},9)>0)
      throw std::runtime_error("fully eliminated track revived without an observation");
    update(perception(3,0,true));update(perception(4,0));
    if(beliefs.in_view({2,1},0)<=0)
      throw std::runtime_error("a new real sighting failed to restore a lost track");
    update(perception(35,0));
    if(beliefs.in_view({4,1},9)>0)
      throw std::runtime_error("expired track retained search probability");
    auto next=perception(0,0);next.session_id="next";update(next);
    if(beliefs.in_view({4,1},9)>0)
      throw std::runtime_error("enemy probability leaked between sessions");
    std::cout<<"Enemy belief checks passed\n";return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
