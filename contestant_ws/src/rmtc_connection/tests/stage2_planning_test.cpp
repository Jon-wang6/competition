/**
 * @file stage2_planning_test.cpp
 * @brief 验证同窗口机会成本、封顶期望、任务/观察去重、公开信息边界与预算回退。
 */
#include "rmtc_connection/stage2_strategy.hpp"
#include "rmtc_connection/stage2_collection_neural.hpp"
#include <cmath>
#include <chrono>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <filesystem>
#include <stdexcept>

namespace {
/// Release构建仍执行的业务断言。
void require(bool value,const char* message) {if(!value)throw std::runtime_error(message);}

/// 使用完整公开视野构造基准场景，不向策略传入裁判内部状态。
rmtc::PublicPerception frame() {
  rmtc::PublicPerception f;f.session_id="planning-test";f.map={5,5};f.vision_radius=5;
  f.self.team="A";f.self.robot_id="A";f.self.position={2,2};f.self.alive=true;f.self.hp=100;
  return f;
}

/// 构造规则规定的普通资源实体。
rmtc::VisibleEntity resource(int x,int y,const char* id) {
  rmtc::VisibleEntity e;e.type="resource_normal";e.id=id;e.position={x,y};e.value=10;return e;
}

/// 仅查询本队地图记忆，隔离风险与探索权重，便于检验真实资源分的数学不变量。
rmtc::PlanningResult run(const rmtc::PublicPerception& f,const rmtc::MapMemory& memory,
                         const rmtc::Stage2Weights& w,int normal=0,bool exploration=false) {
  const auto free=[&](rmtc::Position p,int) {
    return p.x>=0&&p.x<f.map.width&&p.y>=0&&p.y<f.map.height&&
        memory.cells()[rmtc::MapMemory::index(p,f.map)]==0;
  };
  return rmtc::plan_collection_actions(f,memory,w,500,normal,0,1,1,
      std::vector<int>(25,-1),free,[&](double cells,double){return exploration?.05*cells:0.;},
      [](rmtc::Position,int){return 0.;},{});
}
} // namespace

/// 执行独立规划器与策略接线的回归，不启动ROS、裁判或外部模型。
int main(int argc,char**) {
  try {
    using namespace rmtc;
    // 脱困目标跨Tick保持；到达或出现真正收益才结束，不能刚移动一步就改向原格。
    Stage2Weights nav_weights;nav_weights.navigation_cycle_window=8;
    nav_weights.navigation_cycle_start_tick=0;nav_weights.navigation_cycle_cooldown=12;
    NavigationProgress nav;auto nav_frame=frame();
    for(int tick=0;tick<8;++tick) {
      nav_frame.tick=tick;nav_frame.self.position={1+tick%2,2};nav.observe(nav_frame,25,nav_weights);
    }
    require(nav.active(),"two-cell no-progress cycle not detected");
    require(nav.cycle_distance({3,2})==1&&nav.cycle_distance({4,2})==2,"escape margin wrong");
    nav.commit_exit({4,2},7,2,20);
    nav_frame.tick=8;nav_frame.self.position={3,2};nav.observe(nav_frame,25,nav_weights);
    require(nav.exit()&&nav.exit()->x==4,"moving toward exit retargeted escape");
    nav_frame.tick=9;nav_frame.self.position={4,2};nav.observe(nav_frame,25,nav_weights);
    require(!nav.active()&&!nav.exit(),"arrival did not finish escape");
    require(std::abs(planning_damage_loss(500,500,15)-1.8)<1e-9,"500 damage loss wrong");
    require(std::abs(planning_damage_loss(540,500,15)-.6)<1e-9,"damage buffer ignored");
    require(planning_damage_loss(560,500,15)==0,"full damage buffer still loses points");
    Stage2Weights w;w.planning.enabled=true;w.planning.horizon=4;w.planning.beam_width=8;
    w.planning.max_nodes=4000;w.planning.budget_ms=100;w.planning.terminal_weight=0;
    w.planning.discount=.9;w.competition_weight=w.competition_weight_end=0;
    w.visible_resource_weight=1;
    require(planning_resource_presence(0,1,w.planning)==1,"visible resource discounted as missing");
    require(planning_resource_presence(80,-1,w.planning)>
        planning_resource_presence(80,1,w.planning),"enemy evidence did not affect existence estimate");
    auto f=frame();f.visible_entities={resource(3,2,"near")};MapMemory memory;memory.update(f);
    auto p=run(f,memory,w);
    require(p.completed_depth==4&&p.roots[0].valid&&p.roots[4].valid,"roots have unequal horizon");
    require(p.roots[4].value>p.roots[0].value,"delaying collection had no opportunity cost");
    for(const auto& r:p.roots)if(r.valid)
      require(r.resource_points<=2.5+1e-9,"one resource collected repeatedly");
    f.score.resource_value=119;p=run(f,memory,w);
    require(std::abs(p.roots[4].resource_points-.25)<1e-9,"resource score exceeded cap");
    // 旧资源存在性为0.5：必须先对每条结果封顶，不能先把10点资源均值化成5点。
    f.tick=80;f.vision_radius=0;f.visible_entities.clear();memory.update(f);
    p=run(f,memory,w);
    require(std::abs(p.roots[4].resource_points-.125)<1e-9,"uncertain reward capped after expectation");
    f.score.resource_value=120;p=run(f,memory,w);
    require(!p.useful&&p.roots[4].resource_points==0,"saturated resources created fictitious points");
    f=frame();f.visible_entities={resource(3,2,"first"),resource(4,2,"second")};
    MissionInfo m;m.id="collect-two";m.type="normal_resources";m.target=2;f.missions={m};
    memory=MapMemory{};memory.update(f);p=run(f,memory,w);
    require(std::abs(p.roots[4].mission_points-3)<1e-9,"completion opportunity missing or counted repeatedly");
    f.score.mission_completed_ids={m.id};p=run(f,memory,w);
    require(p.roots[4].mission_points==0,"completed task rewarded again");
    w.planning.max_nodes=5;p=run(f,memory,w);
    require(p.budget_hit&&p.completed_depth==1,"partial search layer committed");
    for(const auto& r:p.roots)if(r.valid)require(r.path.size()==2,"budget roots have unequal depth");
    w.planning.max_nodes=4000;
    f=frame();f.vision_radius=1;memory=MapMemory{};memory.update(f);p=run(f,memory,w,0,true);
    require(p.useful,"no-resource state failed to propose exploration");
    for(const auto& r:p.roots)if(r.valid) {
      for(const auto at:r.path)require(memory.cells()[MapMemory::index(at,f.map)]==0,
          "forecast crossed unobserved terrain");
      require(r.information<=.05*16+1e-9,"unknown observations counted repeatedly");
    }
    w.planning.enabled=false;p=run(f,memory,w);
    require(!p.ran,"disabled planner changed baseline");
    // 最后一层验证新模式已经进入真实Stage2Strategy决策和诊断入口。
    w.planning.enabled=true;w.opening_collection_ticks=40;w.collection_mode=0;
    w.frontier_resource_weight=w.frontier_mission_weight=w.information_per_cell=0;
    f=frame();f.visible_entities={resource(3,2,"right")};
    Stage2Strategy strategy(w);const auto a=strategy.decide(f);
    require(a.type=="move"&&a.direction=="right","strategy did not apply collection critic");
    require(strategy.plan().planning.value("applied",false),"new mode diagnostics missing");
    require(strategy.plan().target_id=="right","display target differs from selected collection route");
    const auto again=strategy.decide(f);
    require(a.type==again.type&&a.direction==again.direction,"duplicate Tick changed cached action");
    // 重新看到空格必须撤销旧资源，不允许新的存在性模型让已消失资源复活。
    f.tick=1;f.visible_entities.clear();f.self.position={3,2};f.score.resource_value=10;
    strategy.decide(f);
    require(strategy.plan().resource_races.empty(),"confirmed empty resource survived in planning");
    // 相同未来墙体对所有根动作使用相同时间线，首步禁止走入即将生效的墙。
    f=frame();f.visible_entities={resource(3,2,"blocked")};memory=MapMemory{};memory.update(f);
    const auto walls=[](Position p,int) {return p.x>=0&&p.x<5&&p.y>=0&&p.y<5&&!(p.x==3&&p.y==2);};
    p=plan_collection_actions(f,memory,w,500,0,0,1,1,std::vector<int>(25,-1),walls,
        [](double,double){return 0.;},[](Position,int){return 0.;},{});
    require(!p.roots[4].valid,"first move ignored dynamic wall callback");
    // 同一敌人先吃左边后必须再走两步才能吃右边，不能两个资源都按距离1判定丢失。
    f=frame();f.self.position={2,1};f.visible_entities={resource(1,2,"left"),resource(3,2,"right")};
    VisibleEntity rival;rival.type="robot";rival.id="rival";rival.team="B";rival.position={2,2};rival.hp=100;
    f.visible_entities.push_back(rival);memory=MapMemory{};memory.update(f);
    const auto race=forecast_collection_race(f,memory,nullptr,20);
    require(race.scenarios.size()==4,"joint enemy scenarios missing");
    for(const auto& scene:race.scenarios) {
      require(scene.arrival.size()==2,"enemy route omitted reachable resource");
      auto it=scene.arrival.begin();int t=it->second;++it;
      require(std::abs(t-it->second)>=2,"one enemy collected two separate resources simultaneously");
    }
    auto scenario_weights=w;scenario_weights.planning.race_scenarios=true;
    scenario_weights.planning.horizon=2;scenario_weights.planning.terminal_weight=0;
    const auto avoid_body=[](Position q,int){return q.x>=0&&q.y>=0&&q.x<5&&q.y<5&&!(q.x==2&&q.y==2);};
    p=plan_collection_actions(f,memory,scenario_weights,500,0,0,1,1,std::vector<int>(25,1),avoid_body,
        [](double,double){return 0.;},[](Position,int){return 0.;},{},std::nullopt,{},&race);
    require(p.roots[4].value>p.roots[3].value,"joint enemy route did not leave a second resource opportunity");
    // 分区先验仅调整方向，未知格总机会不增加；没有证据也不能把某区判成必无资源。
    f=frame();f.map={12,12};f.vision_radius=1;memory=MapMemory{};memory.update(f);
    const auto supply=regional_collection_supply(f,memory,.5);double sum=0;int unseen=0;
    for(int y=0;y<12;++y)for(int x=0;x<12;++x)if(memory.cells()[MapMemory::index({x,y},f.map)]==-1) {
      sum+=supply.factors[y*12+x];++unseen;
      require(supply.factors[y*12+x]>0,"regional prior erased an unknown area");
    }
    require(std::abs(sum-unseen)<1e-8,"regional prior inflated total exploration reward");
    // 末端扩展读取两三个资源，仍遵守积分封顶与统一根动作提交。
    f=frame();f.self.position={0,0};f.visible_entities={resource(4,3,"cluster1"),resource(4,4,"cluster2")};
    memory=MapMemory{};memory.update(f);auto chain=w;
    chain.planning.horizon=2;chain.planning.terminal_weight=.55;chain.planning.terminal_stops=2;
    p=run(f,memory,chain);
    require(p.terminal_extended,"bounded multi-resource terminal extension not applied");
    for(std::size_t i=0;i<p.roots.size();++i)if(p.roots[i].valid) {
      const auto features=collection_route_features(f,memory,p,i,0,0,std::vector<int>(25,-1),{});
      for(float v:features)require(std::isfinite(v)&&v>=-1&&v<=1,"collection feature normalization invalid");
    }
    for(const auto& root:p.roots)if(root.valid)
      require(root.value<=5.0+1e-9,"terminal extension invented extra collection reward");
    // 零输出采集网络必须与原规划动作完全相同；配置尺度改变时必须拒绝旧模型。
    const auto model_path=std::filesystem::temp_directory_path()/"rmtc-zero-collector-test.json";
    const Action base_action=Stage2Strategy(chain).decide(f);
    chain.planning.collection_neural.enabled=true;chain.planning.collection_neural.model_file=model_path.string();
    TinyNeuralNetwork zero(1,32,16,64,"collection");
    zero.save(model_path.string(),{{"schema","planning-v3-collection"},{"collection_baseline",collection_baseline_signature(chain)}});
    Stage2Strategy zero_strategy(chain);const Action zero_action=zero_strategy.decide(f);
    require(base_action.type==zero_action.type&&base_action.direction==zero_action.direction,"zero collection network changed policy");
    chain.planning.discount=.85;bool mismatch_rejected=false;
    try{Stage2Strategy incompatible(chain);}catch(const std::exception&){mismatch_rejected=true;}
    std::filesystem::remove(model_path);
    require(mismatch_rejected,"incompatible collection baseline model accepted");
    // 真实持续互防证据只能触发有限侧移；不能再等待两百Tick而从不重选方向。
    auto stuck=frame();stuck.map={24,24};stuck.vision_radius=24;stuck.tick=300;
    stuck.self.position={2,1};stuck.self.hp=30;stuck.score.effective_damage=500;
    MissionInfo reach;reach.id="reach";reach.type="reach";reach.position=Position{12,12};stuck.missions={reach};
    for(auto at:{Position{4,1},Position{2,3}}) {
      VisibleEntity e;e.type="robot";e.team=at.x==4?"B":"C";e.position=at;e.hp=100;e.defending=false;
      stuck.visible_entities.push_back(e);
    }
    Stage2Weights recover=w;recover.opening_collection_ticks=0;recover.reach_finish_ticks=200;
    recover.planning.reach_recovery=true;recover.planning.recovery_ticks=8;
    recover.rollout.weight=0;recover.interference.weight=0;
    Stage2Strategy recovering(recover);bool escaped=false;
    for(int t=300;t<325;++t) {
      stuck.tick=t;recovering.decide(stuck,500-t);
      if(recovering.plan().reach_finish.value("recovery_selected",false)){escaped=true;break;}
    }
    require(escaped,"prolonged defensive task deadlock never considered a safe destination");
    // 可选性能探针只执行本队公开场景，不运行比赛、不训练、不调用AI。
    if(argc>1) {
      std::vector<double> timings;int min_depth=32,max_nodes=0;
      for(int scene=0;scene<20;++scene) {
        auto probe=frame();probe.session_id="planning-benchmark";probe.map={24,24};probe.vision_radius=24;
        probe.self.position={8+scene%4,8+scene/4};probe.score.resource_value=scene%3*40;
        for(int i=0;i<16;++i)probe.visible_entities.push_back(resource(6+i%4,6+i/4,("r"+std::to_string(i)).c_str()));
        MapMemory observed;observed.update(probe);
        if(scene%2) {probe.tick=80;probe.vision_radius=0;probe.visible_entities.clear();observed.update(probe);}
        Stage2Weights params;params.planning.enabled=true;params.opening_collection_ticks=40;
        Stage2Strategy controller(params);
        // 先给策略与测试相同的完整记忆，再通过连续公开观察构造旧资源场景。
        auto initial=probe;initial.tick=0;initial.vision_radius=24;initial.visible_entities.clear();
        for(int i=0;i<16;++i)initial.visible_entities.push_back(resource(6+i%4,6+i/4,("r"+std::to_string(i)).c_str()));
        const auto began=std::chrono::steady_clock::now();
        controller.decide(initial);
        timings.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count());
        if(scene%2) {
          const auto old_began=std::chrono::steady_clock::now();controller.decide(probe,420);
          timings.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-old_began).count());
        }
        const auto& detail=controller.plan().planning;
        min_depth=std::min(min_depth,detail.value("completed_depth",0));
        max_nodes=std::max(max_nodes,detail.value("expanded",0));
      }
      std::sort(timings.begin(),timings.end());
      std::cout<<Json{{"probe_decisions",timings.size()},{"decision_ms",{
          {"mean",std::accumulate(timings.begin(),timings.end(),0.0)/timings.size()},
          {"p95",timings[static_cast<std::size_t>(.95*(timings.size()-1))]},
          {"max",timings.back()}}},{"minimum_completed_depth",min_depth},{"maximum_expanded_nodes",max_nodes}}.dump()<<'\n';
    }
    std::cout<<"Planning score, uncertainty, coverage, budget and integration checks passed\n";
    return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
