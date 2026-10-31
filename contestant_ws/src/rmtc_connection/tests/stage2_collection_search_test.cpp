/** @file stage2_collection_search_test.cpp
 * @brief 检查多段前瞻的后续收益、唯一计分、危险避让与公开地形边界。
 */
#include "rmtc_connection/stage2_collection_search.hpp"
#include <iostream>
#include <stdexcept>

/** @brief 用业务不变量断言搜索结果，避免固定内部束搜索展开顺序。 */
void require_search(bool value,const char* message) {
  if(!value) throw std::runtime_error(message);
}

/** @brief 构造一块资源机会，使用真实规则中普通资源的2.5分边际价值。 */
rmtc::JointOpportunity resource(const std::string& key,int x,int y,int steps) {
  rmtc::JointOpportunity item;item.key=key;item.position={x,y};item.value=10;
  item.resource_points=2.5;item.priority=1;item.direct_steps=steps;return item;
}

/** @brief 在小地图上验证有效多段预测与禁止透视、重复领取、超时限执行。 */
int main() {
  try {
    rmtc::PublicPerception frame;frame.session_id="collection-search";frame.map={9,9};
    frame.self.position={1,1};frame.self.team="A";frame.self.alive=true;frame.self.hp=100;
    frame.vision_radius=9;
    rmtc::MapMemory memory;memory.update(frame);
    rmtc::Stage2Weights w;w.collection_search_weight=1;w.collection_search_budget_ms=150;
    w.collection_search_discount=1;w.collection_search_future=.7;
    w.collection_search_horizon=8;w.collection_search_depth=4;
    const auto free=[&](rmtc::Position p) {
      return p.x>=0&&p.y>=0&&p.x<9&&p.y<9&&
          memory.cells()[rmtc::MapMemory::index(p,frame.map)]==0;
    };
    const auto no_info=[](int,double){return 0.;};
    const auto no_risk=[](rmtc::Position,int){return 0.;};
    const auto a=resource("east",2,1,1),b=resource("south",1,2,1);
    auto plan=rmtc::search_collection_route(frame,memory,{a,b},500,w,1,1,free,no_info,no_risk);
    require_search(plan.route.success()&&plan.route.path.size()==2,"first segment skipped an adjacent resource");
    require_search(plan.forecast_resource>plan.resource_points&&plan.forecast_resource<=5+1e-9,
                   "future resources absent or counted repeatedly");
    require_search(plan.completed_depth>=2&&plan.expanded>0,"multi-target planning never ran");
    frame.score.resource_value=119;
    plan=rmtc::search_collection_route(frame,memory,{a,b},500,w,1,1,free,no_info,no_risk);
    require_search(plan.forecast_resource<=.25+1e-9,"future route exceeded remaining resource score");
    frame.score.resource_value=0;
    auto dangerous=resource("dangerous",3,1,2);dangerous.resource_points=5;
    const auto danger=[](rmtc::Position p,int arrival){return p.x==2&&p.y==1&&arrival>=1?100.:0.;};
    plan=rmtc::search_collection_route(frame,memory,{dangerous,b},500,w,1,1,free,no_info,danger);
    require_search(plan.route.success()&&plan.route.path.back().y==2,"time-indexed danger ignored");
    plan=rmtc::search_collection_route(frame,memory,{dangerous},1,w,1,1,free,no_info,no_risk);
    require_search(plan.route.path.empty(),"resource beyond remaining Tick became executable");
    frame.vision_radius=1;frame.self.position={4,4};memory=rmtc::MapMemory{};memory.update(frame);
    plan=rmtc::search_collection_route(frame,memory,{},500,w,1,1,free,
        [](int cells,double){return .1*cells;},no_risk);
    require_search(plan.route.success(),"no-resource case failed to propose exploration");
    for(const auto p:plan.route.path)
      require_search(free({p.x,p.y}),"unknown terrain used as an executable path");
    w.collection_search_weight=0;
    plan=rmtc::search_collection_route(frame,memory,{},500,w,1,1,free,no_info,no_risk);
    require_search(!plan.ran&&plan.route.path.empty(),"disabled search changed the baseline");
    std::cout<<"Collection multi-target search checks passed\n";return 0;
  } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
