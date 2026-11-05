/**
 * @file stage2_resource_sweep_test.cpp
 * @brief 联合采集束搜索的重复计分、资源封顶及公开地形执行边界检查。
 *
 * @details 模块协作与维护约定
 * 验证对象：stage2_resource_sweep.hpp。
 * 运行约定：由独立测试可执行文件/CTest执行，断言失败用于定位回归，不是正式选手的逐帧逻辑。
 * 结果边界：通过表示这些构造场景满足约束，不证明所有随机地图均最优或稳定达到某个总分。
 */
#include "rmtc_connection/stage2_resource_sweep.hpp"
#include <iostream>
#include <stdexcept>

/** @brief 在可控公开地图中检查一条路线的业务不变量，不固定搜索内部顺序。 */
int main() {
  try {
    rmtc::PublicPerception frame;
    frame.session_id="sweep-test";frame.map={9,9};frame.vision_radius=9;
    frame.self.position={1,1};frame.self.team="A";frame.self.alive=true;frame.self.hp=100;
    rmtc::MapMemory memory;memory.update(frame);
    rmtc::Stage2Weights w;w.sweep_weight=1;w.sweep_horizon=6;w.sweep_beam=24;w.sweep_discount=1;
    rmtc::JointOpportunity a;a.key="normal";a.position={2,1};a.resource_points=2.5;a.priority=1;
    rmtc::JointOpportunity b;b.key="high";b.position={3,1};b.resource_points=5;b.priority=1;b.mission_points["high-task"]=3;
    const auto free=[&](rmtc::Position p) {
      return p.x>=0&&p.y>=0&&p.x<9&&p.y<9&&memory.cells()[rmtc::MapMemory::index(p,frame.map)]==0;
    };
    const auto no_information=[](int,double){return 0.0;};
    auto plan=rmtc::resource_sweep(frame,memory,{a,b},500,w,1,1,free,no_information);
    if(plan.path.size()<3 || plan.resource_points>7.5+1e-9 || plan.resource_points<7.5-1e-9 ||
       plan.mission_points>3+1e-9 || plan.expanded<=0)
      throw std::runtime_error("known resources not jointly collected or counted repeatedly");
    frame.score.resource_value=119;
    plan=rmtc::resource_sweep(frame,memory,{a,b},500,w,1,1,free,no_information);
    if(plan.resource_points>.25+1e-9) throw std::runtime_error("resource score budget exceeded");
    frame.score.resource_value=0;
    const auto blocked=[&](rmtc::Position p){return free(p)&&!(p.x==2&&p.y==1);};
    plan=rmtc::resource_sweep(frame,memory,{b},500,w,1,1,blocked,no_information);
    for(auto p:plan.path) if(!blocked({p.x,p.y})) throw std::runtime_error("blocked cell in executable route");
    w.sweep_weight=0;
    if(!rmtc::resource_sweep(frame,memory,{a,b},500,w,1,1,free,no_information).path.empty())
      throw std::runtime_error("disabled planner changed baseline");
    w.sweep_weight=1;frame.self.position={4,4};frame.vision_radius=1;
    memory=rmtc::MapMemory{};memory.update(frame);
    plan=rmtc::resource_sweep(frame,memory,{},500,w,1,1,free,[](int cells,double){return .1*cells;});
    if(plan.path.size()<2) throw std::runtime_error("empty-resource scene did not explore");
    for(auto p:plan.path) if(!free({p.x,p.y})) throw std::runtime_error("unknown terrain treated as executable");
    w.sweep_collect_only=true;
    if(!rmtc::resource_sweep(frame,memory,{},500,w,1,1,free,
                           [](int cells,double){return .1*cells;}).path.empty())
      throw std::runtime_error("collection-only search replaced pure exploration");
    // 有资源也不能先领取看见未知格的奖励，再把未到达资源的路线冒充采集计划。
    b.position={8,8};w.sweep_horizon=1;
    if(!rmtc::resource_sweep(frame,memory,{b},500,w,1,1,free,
                           [](int cells,double){return .1*cells;}).path.empty())
      throw std::runtime_error("unreachable-in-budget resource legitimized an exploration-only route");
    std::cout<<"Resource sweep checks passed\n";return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
