/**
 * @file stage2_resource_race_test.cpp
 * @brief 对照小图穷举最优值，并验证抢夺期限、重复计分、晚期新发现和资源消失。
 *
 * @details 模块协作与维护约定
 * 验证对象：stage2_resource_race.hpp、stage2_strategy.hpp。
 * 运行约定：由独立测试可执行文件/CTest执行，断言失败用于定位回归，不是正式选手的逐帧逻辑。
 * 结果边界：通过表示这些构造场景满足约束，不证明所有随机地图均最优或稳定达到某个总分。
 */
#include "rmtc_connection/stage2_resource_race.hpp"
#include "rmtc_connection/stage2_strategy.hpp"
#include <cmath>
#include <functional>
#include <iostream>
#include <random>
#include <stdexcept>

namespace {
/// Release构建仍执行检查，失败原因直接显示业务不变量。
void check(bool okay, const char* reason) {
  if (!okay) throw std::runtime_error(reason);
}
/// 空白小图使用曼哈顿距离作为独立的最短路真值。
int distance(rmtc::Position a, rmtc::Position b) {
  return std::abs(a.x-b.x) + std::abs(a.y-b.y);
}
/// 穷举不同资源访问顺序与子集；仅测试工具使用，不接入正式控制节点。
double oracle(rmtc::Position start, const std::vector<rmtc::RaceResource>& resources,
              const rmtc::ResourceRaceWeights& w) {
  double best = -1e100;
  std::function<void(rmtc::Position,unsigned,int,double)> visit;
  visit = [&](rmtc::Position at, unsigned mask, int elapsed, double value) {
    if (mask) best = std::max(best, value-w.distance_cost*elapsed);
    for (std::size_t i=0;i<resources.size();++i) {
      if (mask & (1U<<i)) continue;
      const auto& r=resources[i];
      const int when=elapsed+distance(at,r.position);
      if (when>w.horizon || (r.enemy_steps>=0 && when>r.enemy_steps+w.deadline_slack)) continue;
      visit(r.position,mask|(1U<<i),when,value+r.value*r.confidence);
    }
  };
  visit(start,0,0,0);
  return best;
}
/// 生成可见资源，后期新发现与首次开局发现使用完全相同的实体字段。
rmtc::VisibleEntity resource(const char* id,int x,int y) {
  rmtc::VisibleEntity r;r.type="resource_normal";r.id=id;r.position={x,y};r.value=10;return r;
}
/// 所有地形均在本车公开视野内；测试不依赖裁判全图文件。
rmtc::PublicPerception frame(int tick,std::vector<rmtc::VisibleEntity> entities={}) {
  rmtc::PublicPerception f;f.session_id="late-race";f.tick=tick;f.map={9,9};f.vision_radius=9;
  f.self.position={4,4};f.self.team="A";f.self.alive=true;f.self.hp=100;
  f.visible_entities=std::move(entities);return f;
}
} // namespace

/** @brief 资源顺序应与独立穷举一致；整局优先不因开局窗口结束失效。 */
int main() {
  try {
    rmtc::ResourceRaceWeights w;w.enabled=true;w.budget_ms=100;w.distance_cost=.1;
    const auto free=[](rmtc::Position p){return p.x>=0&&p.y>=0&&p.x<9&&p.y<9;};
    const auto no_information=[](rmtc::Position){return 0.0;};
    std::mt19937 rng(20261029);
    for (int trial=0;trial<100;++trial) {
      std::vector<rmtc::RaceResource> rs;
      for (int i=0;i<7;++i) {
        rmtc::Position p;
        do {p={static_cast<int>(rng()%9),static_cast<int>(rng()%9)};}
        while ((p.x==4&&p.y==4) || std::any_of(rs.begin(),rs.end(),[&](const auto& r){return r.position.x==p.x&&r.position.y==p.y;}));
        rs.push_back({std::to_string(i),p,(rng()%2)?10.:20.,.5+.1*(rng()%6),
                      i==0?-1:static_cast<int>(rng()%20+1)});
      }
      const auto plan=rmtc::plan_resource_race({9,9},{4,4},rs,500,w,free,no_information);
      check(plan.complete,"small-map search unexpectedly incomplete");
      check(std::abs(plan.objective-oracle({4,4},rs,w))<1e-8,"subset deadline route differs from exhaustive optimum");
      check(plan.route.success(),"feasible optimum has no executable first leg");
      for(std::size_t i=1;i<plan.route.path.size();++i)
        check(distance({plan.route.path[i-1].x,plan.route.path[i-1].y},
                       {plan.route.path[i].x,plan.route.path[i].y})==1,"route skipped a grid cell");
    }
    // 左一格普通资源与右两格高资源：先向左会错过右侧期限，应先向右再返回。
    w.deadline_slack=0;
    auto plan=rmtc::plan_resource_race({9,9},{4,4},
        {{"near",{3,4},10,1,-1},{"urgent",{6,4},20,1,2}},500,w,free,no_information);
    check(plan.order.size()==2 && plan.order.front()=="urgent","nearest-first lost a resource with an achievable deadline");
    check(std::abs(plan.forecast_value-30)<1e-9,"one-off resource value counted twice");
    const auto blocked=[&](rmtc::Position p){return free(p)&&p.x!=5;};
    plan=rmtc::plan_resource_race({9,9},{4,4},{{"other-side",{6,4},20,1,-1}},500,w,blocked,no_information);
    check(!plan.route.success(),"resource route crossed a known wall");
    rmtc::Stage2Weights strategy_weights;strategy_weights.resource_race=w;
    strategy_weights.resource_race.coverage_fallback=false;
    strategy_weights.resource_race.move_priority=1.5;
    rmtc::Stage2Strategy strategy(strategy_weights);
    strategy.decide(frame(300),200);
    const auto action=strategy.decide(frame(350,{resource("late",5,4)}),150);
    check(strategy.plan().goal=="resource" && action.type=="move" && action.direction=="right",
          "late rediscovered resource did not immediately regain collection priority");
    strategy.decide(frame(351),149);
    check(strategy.plan().resource_race.value("selected",false)==false,
          "confirmed missing resource kept a fake collection goal");
    // 攻击有收益也不能挤掉安全可采的一次性资源；低血时仍应保留防御。
    auto combat_frame=frame(400,{resource("safe-pickup",5,4)});
    rmtc::VisibleEntity enemy;enemy.type="robot";enemy.team="B";enemy.id="B";
    enemy.position={4,6};enemy.hp=100;enemy.spawn_protected=false;
    combat_frame.visible_entities.push_back(enemy);
    rmtc::Stage2Strategy safe_collection(strategy_weights);
    check(safe_collection.decide(combat_frame,100).type=="move",
          "ordinary profitable attack displaced safe late resource collection");
    combat_frame.self.hp=15;
    rmtc::Stage2Strategy low_hp(strategy_weights);
    check(low_hp.decide(combat_frame,100).type=="defend",
          "collection priority ignored attack-before-movement lethal risk");
    auto capped=frame(450,{resource("leftover",5,4)});
    capped.score.resource_value=120;capped.score.effective_damage=500;
    rmtc::Stage2Strategy capped_collection(strategy_weights);
    check(capped_collection.decide(capped,50).type=="move",
          "late leftover was ignored after own resource score saturated");
    std::cout<<"Deadline resource route and late-discovery checks passed\n";return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
