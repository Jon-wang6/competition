/**
 * @file stage2_planning.cpp
 * @brief 固定节点预算的滚动采集规划；正式封顶积分、任务完成与探索机会分别记账。
 *
 * @details 模块协作与维护约定
 * 同一帧比较WAIT/UP/DOWN/LEFT/RIGHT五个根动作，按共同完成的搜索深度提交结果。
 * 资源计分与任务阈值在各采集结果分支中计算后再求期望，避免先平均再封顶造成高估。
 * 时间预算使用毫秒，路线到达时间使用相对Tick；预算不足时返回有效的已完成结果供上层回退。
 */
#include "rmtc_connection/stage2_planning.hpp"
#include "rmtc_connection/stage2_config.hpp"
#include "rmtc_connection/stage2_collection.hpp"
#include <algorithm>
#include <bitset>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <stdexcept>
#include <tuple>

namespace rmtc {
/**
 * @brief 检查前瞻深度、宽度、节点/时间预算和概率尺度，防止非法配置进入搜索。
 * @throws std::invalid_argument 非有限、负值或超出实现支持范围。
 * @note 调用不修改参数，也不将非法值静默截断成另一个实验配置。
 */
void PlanningWeights::validate() const {
  for (double x : {budget_ms,discount,terminal_weight,memory_half_life,enemy_pressure,
                   exploration_floor,route_tie_margin,future_risk_weight,regional_strength,race_scenario_weight})
    if (!std::isfinite(x) || x < 0) throw std::invalid_argument("invalid planning weight");
  if (horizon<1 || horizon>32 || beam_width<1 || beam_width>64 || max_nodes<5 ||
      max_nodes>100000 || budget_ms<=0 || budget_ms>100 || discount<=0 || discount>1 ||
      terminal_weight>1 || memory_half_life<=0 || exploration_floor>1 ||
      recovery_ticks<4 || recovery_ticks>100 || memory_model<0 || memory_model>1 ||
      escape_ticks<4 || escape_ticks>100 || terminal_stops<1 || terminal_stops>3 ||
      race_memory_ticks<0 || race_memory_ticks>100 || regional_strength>1 || race_scenario_weight>1)
    throw std::invalid_argument("invalid planning budget or probability scale");
}

/**
 * @brief 先将扣除前后累计伤害分别映射到封顶积分，再求真正损失的积分差。
 * @param damage 当前累计伤害，可能已超过计分上限。
 * @param cap 伤害计分的正上限，调用方必须提供有效正数。
 * @param points 该项满分。
 * @param deduction 预计被扣除的原始伤害量，不能直接当成积分扣除。
 * @return 在封顶规则下的积分减少量；超出上限的冗余伤害可能抵消一部分损失。
 */
double planning_damage_loss(double damage,double cap,double points,double deduction) {
  const auto score=[&](double d) {return points*std::clamp(d/cap,0.0,1.0);};
  return score(damage)-score(std::max(0.0,damage-deduction));
}

/**
 * @brief 用最后目击年龄和敌方接近程度计算旧资源仍存在的软估计。
 * @param age 距最后目击的Tick数；负值按0处理。
 * @param enemy_steps 敌方距离；负值表示没有有效距离证据，不施加敌方压力。
 * @param w 已验证的规划参数，memory_half_life必须大于0。
 * @return 指数衰减估计，亲眼刚看到时为1；不声称是经过统计校准的真实概率。
 */
double planning_resource_presence(int age,int enemy_steps,const PlanningWeights& w) {
  const double pressure=enemy_steps<0?0.0:w.enemy_pressure/(1.0+enemy_steps);
  return std::exp2(-std::max(0,age)*(1.0+pressure)/w.memory_half_life);
}

/**
 * @brief 将搜索结果导出为解释日志，记录有效根、完成深度、耗时和预算触限。
 * @return JSON快照；路线是预测路径，不是机器人已执行的轨迹。
 * @note confidence_is_calibrated_probability固定为false，避免误解启发式置信度。
 */
Json PlanningResult::describe() const {
  Json rows=Json::array();
  constexpr std::array<const char*,5> names{{"wait","up","down","left","right"}};
  for(std::size_t i=0;i<roots.size();++i) if(roots[i].valid) {
    const auto& r=roots[i];Json path=Json::array();
    for(const auto p:r.path) path.push_back({p.x,p.y});
    rows.push_back({{"root",names[i]},{"value",r.value},{"resource_points",r.resource_points},
        {"mission_points",r.mission_points},{"information",r.information},
        {"terminal",r.terminal},{"path",std::move(path)}});
  }
  return {{"ran",ran},{"useful",useful},{"completed_depth",completed_depth},
      {"expanded",expanded},{"budget_hit",budget_hit},{"elapsed_ms",elapsed_ms},
      {"resources",resources},{"roots",std::move(rows)},{"competition",competition},
      {"terminal_extended",terminal_extended},
      {"confidence_is_calibrated_probability",false}};
}

/**
 * @brief 在已知地形上同时评估五种首步，保留采集结果分布与唯一观察覆盖。
 * @details 参数契约见stage2_planning.hpp。先准备资源/距离/视野集合，再逐层扩展候选；
 * 只提交可公平比较的完整层，末端机会估值不等于已兑现收益。
 * @return 可包含ran=false或useful=false的结果，上层必须检查标志并保留回退决策。
 * @note passable与future_risk接收相对到达步数，调用方负责换算为事件绝对Tick。
 */
PlanningResult plan_collection_actions(
    const PublicPerception& f,const MapMemory& memory,const Stage2Weights& w,
    int remaining,int normal_collected,int high_collected,double resource_weight,
    double mission_weight,const std::vector<int>& enemy_steps,
    const std::function<bool(Position,int)>& passable,
    const std::function<double(double,double)>& information,
    const std::function<double(Position,int)>& future_risk,
    const std::vector<double>& supply,std::optional<Position> exploration_target,
    const std::vector<Position>& original_path,const CollectionRaceForecast* competition) {
  PlanningResult result;
  if(!w.planning.enabled || remaining<=0) return result;
  const auto began=std::chrono::steady_clock::now();
  const auto elapsed=[&] {return std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-began).count();};
  const auto finish=[&] {result.elapsed_ms=elapsed();return result;};
  const int width=f.map.width,height=f.map.height,count=width*height;
  if(count<=0 || count>1024 || enemy_steps.size()!=static_cast<std::size_t>(count) ||
      memory.cells().size()!=static_cast<std::size_t>(count) ||
      (!supply.empty()&&supply.size()!=static_cast<std::size_t>(count))) return finish();
  result.ran=true;
  const bool shared_competition=w.planning.race_scenarios&&competition&&!competition->scenarios.empty();
  if(shared_competition)result.competition=competition->describe();
  const int horizon=std::min(remaining,w.planning.horizon);
  constexpr std::array<Position,5> deltas{{{0,0},{0,-1},{0,1},{-1,0},{1,0}}};
  const auto inside=[&](Position p) {return p.x>=0&&p.x<width&&p.y>=0&&p.y<height;};
  if(!inside(f.self.position))return finish();
  const auto index=[&](Position p) {return p.y*width+p.x;};
  const auto position=[&](int i) {return Position{i%width,i/width};};
  /// 静态距离只用于窗口外估计；真实展开的每一步仍检查该Tick的临时墙体。
  const auto field=[&](Position goal) {
    std::vector<int> distances(count,-1);std::queue<Position> pending;
    if(!inside(goal)) return distances;
    distances[index(goal)]=0;pending.push(goal);
    while(!pending.empty()) {
      const auto p=pending.front();pending.pop();
      for(std::size_t i=1;i<deltas.size();++i) {
        const Position q{p.x+deltas[i].x,p.y+deltas[i].y};
        if(!inside(q)||distances[index(q)]>=0||memory.cells()[MapMemory::index(q,f.map)]!=0) continue;
        distances[index(q)]=distances[index(p)]+1;pending.push(q);
      }
    }
    return distances;
  };
  struct Resource {Position at;double value,presence,preference;bool high;int enemy;std::vector<int> distances;std::string key;};
  std::vector<Resource> resources;
  for(const auto& [key,m]:memory.entities()) {
    const auto& e=m.entity;
    if(resources.size()>=16) break;
    if(m.first_missing_tick || !e.value || *e.value<=0 || !inside(e.position) ||
       (e.type!="resource_normal"&&e.type!="resource_high")) continue;
    const int enemy=enemy_steps[index(e.position)];
    const int age=static_cast<int>(std::max<std::int64_t>(0,f.tick-m.last_seen_tick));
    const double presence=w.planning.memory_model==0?1.0/(1.0+age/w.resource_age_ticks):
        planning_resource_presence(age,enemy,w.planning);
    resources.push_back({e.position,*e.value,presence,age==0?w.visible_resource_weight:1.0,
        e.type=="resource_high",enemy,field(e.position),key});
    result.resources.push_back({{"id",e.id.value_or(key)},{"age_ticks",age},
        {"presence_estimate",presence},{"enemy_steps",enemy}});
  }
  const auto exploration_distances=exploration_target?field(*exploration_target):std::vector<int>{};
  std::vector<std::bitset<1024>> views(count);
  std::vector<double> edges(count),supplies(count);
  for(int i=0;i<count;++i) {
    if((i&63)==0&&elapsed()>=w.planning.budget_ms) {result.budget_hit=true;return finish();}
    const auto p=position(i);
    edges[i]=1+w.edge_information_bonus/(1.0+std::min({p.x,p.y,width-1-p.x,height-1-p.y}));
    supplies[i]=supply.empty()?1.0:supply[i];
    if(memory.cells()[MapMemory::index(p,f.map)]!=0) continue;
    for(int y=std::max(0,p.y-f.vision_radius);y<=std::min(height-1,p.y+f.vision_radius);++y)
      for(int x=std::max(0,p.x-f.vision_radius);x<=std::min(width-1,p.x+f.vision_radius);++x)
        if(memory.cells()[MapMemory::index({x,y},f.map)]==-1) views[i].set(index({x,y}));
  }
  /// 不确定采集分支保留联合分布，先按每个分支封顶，再求期望；避免平均后封顶高估。
  struct Outcome {double added{0};int normal{0},high{0};double mass{1};int scenario{0};};
  struct Node {
    Position at;std::uint32_t consumed{0};std::bitset<1024> revealed;
    std::vector<Outcome> outcomes{{}};std::vector<Position> path;
    double reward{0},resource{0},mission{0},info{0},effective{0},edge{0},cost{0},value{0},terminal{0};
  };
  std::vector<const MissionInfo*> missions;
  for(const auto& m:f.missions)
    if(m.target&&*m.target>0&&std::find(f.score.mission_completed_ids.begin(),
        f.score.mission_completed_ids.end(),m.id)==f.score.mission_completed_ids.end()&&
        (m.type=="normal_resources"||m.type=="high_resources"||m.type=="resource_value"))
      missions.push_back(&m);
  /// 任务只在预测达到阈值时计入机会价值，不把每一步的相同任务收益重复累加。
  const auto scores=[&](const Outcome& o) {
    const double resource=w.resource_points/w.resource_cap*
        std::max(0.0,std::min(o.added,w.resource_cap-f.score.resource_value));
    double task=0;
    for(const auto* m:missions) {
      const double before=m->type=="normal_resources"?normal_collected:
          (m->type=="high_resources"?high_collected:f.score.resource_value);
      const double after=before+(m->type=="normal_resources"?o.normal:
          (m->type=="high_resources"?o.high:o.added));
      if(before<*m->target&&after>=*m->target) task+=w.mission_points*w.mission_potential;
    }
    return std::pair<double,double>{resource,task};
  };
  const auto capture=[&](const Resource& r,int arrival,int scenario) {
    if(!shared_competition||scenario<0)return r.presence*collection_race_factor(arrival,r.enemy,f.tick,w);
    const auto& deadlines=competition->scenarios.at(scenario).arrival;
    const auto it=deadlines.find(r.key);
    const double race=it==deadlines.end()||arrival<it->second?1.0:(arrival==it->second?.5:0.0);
    return r.presence*race;
  };
  /// 对尚未访问的资源求额外正式积分/任务机会，不重复估算已消耗资源。
  const auto increment=[&](const Node& n,const Resource& r,int arrival) {
    double resource=0,mission=0;
    for(auto o:n.outcomes) {
      const auto before=scores(o);o.added+=r.value;o.normal+=!r.high;o.high+=r.high;
      const auto after=scores(o);
      const double probability=capture(r,arrival,o.scenario);
      resource+=o.mass*probability*(after.first-before.first);mission+=o.mass*probability*(after.second-before.second);
    }
    return std::pair<double,double>{resource,mission};
  };
  /// 窗口外的路线同样使用预计到达时间检查事件墙和软危险；未知格仍不能当成空地。
  const auto terminal_cost=[&](Position from,const std::vector<int>& distances,int depth) {
    double cost=0;int step=0;
    while(distances[index(from)]>0) {
      std::optional<Position> next;double danger=std::numeric_limits<double>::infinity();
      for(std::size_t i=1;i<deltas.size();++i) {
        const Position p{from.x+deltas[i].x,from.y+deltas[i].y};
        if(!inside(p)||distances[index(p)]!=distances[index(from)]-1||!passable(p,depth+step+1))continue;
        const double risk=std::max(0.0,future_risk(p,depth+step+1));
        if(risk<danger){danger=risk;next=p;}
      }
      if(!next)return std::numeric_limits<double>::infinity();
      cost+=w.planning.future_risk_weight*std::pow(w.planning.discount,step)*danger;
      from=*next;++step;
    }
    return cost;
  };
  /// 当前窗口外只保留一个最有希望的后续机会；此估值不是已获得积分。
  const auto terminal=[&](const Node& n,int depth) {
    double best=0;
    for(std::size_t i=0;i<resources.size();++i) {
      if(n.consumed&(std::uint32_t{1}<<i)) continue;
      const auto& r=resources[i];const int d=r.distances[index(n.at)];
      if(d<=0||depth+d>remaining) continue;
      const auto gain=increment(n,r,depth+d);
      const double cost=w.planning.safe_terminal?terminal_cost(n.at,r.distances,depth):0;
      best=std::max(best,std::pow(w.planning.discount,d)*r.preference*
          (resource_weight*gain.first+mission_weight*gain.second)-cost);
    }
    if(exploration_target) {
      const int d=exploration_distances[index(n.at)];
      if(d>0&&depth+d<=remaining) {
        const auto fresh=views[index(*exploration_target)]&~n.revealed;
        double cells=n.effective,edge=n.edge;
        for(int i=0;i<count;++i)if(fresh[i]) {cells+=supplies[i];edge+=edges[i];}
        const double cost=w.planning.safe_terminal?terminal_cost(n.at,exploration_distances,depth):0;
        best=std::max(best,std::pow(w.planning.discount,d)*std::max(0.0,information(cells,edge)-n.info)-cost);
      }
    }
    return w.planning.terminal_weight*std::pow(w.planning.discount,depth)*best;
  };
  /// 按同一情景更新采集结果；供逐格搜索和末端多目标延伸共同使用。
  const auto pickup=[&](Node& n,std::size_t i,int depth) {
      const auto& r=resources[i];n.consumed|=std::uint32_t{1}<<i;
      const auto gain=increment(n,r,depth);
      n.resource+=gain.first;n.mission+=gain.second;
      n.reward+=std::pow(w.planning.discount,depth-1)*r.preference*
          (resource_weight*gain.first+mission_weight*gain.second);
      std::map<std::tuple<double,int,int,int>,double> distribution;
      for(const auto& o:n.outcomes) {
        const double probability=capture(r,depth,o.scenario);
        distribution[{o.added,o.normal,o.high,o.scenario}]+=o.mass*(1-probability);
        distribution[{o.added+r.value,o.normal+!r.high,o.high+r.high,o.scenario}]+=o.mass*probability;
      }
      n.outcomes.clear();
      for(const auto& [state,mass]:distribution)if(mass>1e-12)
        n.outcomes.push_back({std::get<0>(state),std::get<1>(state),std::get<2>(state),mass,std::get<3>(state)});
  };
  /// 推进一步，资源联合分布、唯一观察集合与时空风险均属于该路线自己的状态。
  const auto advance=[&](const Node& old,Position p,int depth) {
    Node n=old;n.at=p;n.path.push_back(p);
    for(std::size_t i=0;i<resources.size();++i) {
      const auto& r=resources[i];
      if((n.consumed&(std::uint32_t{1}<<i))||r.at.x!=p.x||r.at.y!=p.y)continue;
      pickup(n,i,depth);
    }
    const auto fresh=views[index(p)]&~n.revealed;n.revealed|=views[index(p)];
    for(int i=0;i<count;++i)if(fresh[i]) {n.effective+=supplies[i];n.edge+=edges[i];}
    const double info=information(n.effective,n.edge);
    n.reward+=std::pow(w.planning.discount,depth-1)*(info-n.info);n.info=info;
    if(depth>1)n.cost+=std::pow(w.planning.discount,depth-1)*w.planning.future_risk_weight*
        std::max(0.0,future_risk(p,depth));
    n.terminal=terminal(n,depth);n.value=n.reward-n.cost+n.terminal;
    return n;
  };
  std::array<std::vector<Node>,5> beams;
  Node start;start.at=f.self.position;start.path.push_back(start.at);
  if(shared_competition) {
    start.outcomes.clear();
    if(w.planning.race_scenario_weight<1)
      start.outcomes.push_back({0,0,0,1.0-w.planning.race_scenario_weight,-1});
    for(std::size_t i=0;i<competition->scenarios.size();++i)
      if(w.planning.race_scenario_weight>0)
        start.outcomes.push_back({0,0,0,w.planning.race_scenario_weight/competition->scenarios.size(),static_cast<int>(i)});
  }
  /// 已经看过的出生视野不允许在WAIT预测里再获一次探索价值。
  start.revealed=views[index(start.at)];
  for(std::size_t root=0;root<deltas.size();++root) {
    const Position p{start.at.x+deltas[root].x,start.at.y+deltas[root].y};
    if(!inside(p)||(root>0&&!passable(p,1)))continue;
    beams[root].push_back(advance(start,p,1));++result.expanded;
  }
  std::optional<Node> baseline;int baseline_root=-1;
  if(original_path.size()>1&&original_path[0].x==start.at.x&&original_path[0].y==start.at.y)
    for(std::size_t root=1;root<deltas.size();++root)
      if(!beams[root].empty()&&original_path[1].x==start.at.x+deltas[root].x&&
          original_path[1].y==start.at.y+deltas[root].y) {
        baseline=beams[root].front();baseline_root=static_cast<int>(root);break;
      }
  const auto commit=[&](int depth) {
    result.useful=false;
    for(std::size_t root=0;root<beams.size();++root)if(!beams[root].empty()) {
      const auto& n=beams[root].front();auto& out=result.roots[root];out.valid=true;
      out.value=n.value;out.resource_points=n.resource;out.mission_points=n.mission;
      out.information=n.info;out.terminal=n.terminal;out.path=n.path;
      result.useful=result.useful||n.resource>0||n.mission>0||n.info>0||n.terminal>0;
    }
    result.completed_depth=depth;
  };
  commit(1);
  for(int depth=2;depth<=horizon;++depth) {
    std::array<std::vector<Node>,5> next;bool interrupted=false;
    for(std::size_t root=0;root<beams.size()&&!interrupted;++root) {
      for(const auto& n:beams[root]) {
        for(const auto delta:deltas) {
          if(result.expanded>=w.planning.max_nodes ||
              ((result.expanded&31)==0&&elapsed()>=w.planning.budget_ms)) {interrupted=true;break;}
          const Position p{n.at.x+delta.x,n.at.y+delta.y};
          if(!inside(p)||!passable(p,depth))continue;
          next[root].push_back(advance(n,p,depth));++result.expanded;
        }
        if(interrupted)break;
      }
      std::stable_sort(next[root].begin(),next[root].end(),[](const auto& a,const auto& b){return a.value>b.value;});
      // 保留不同落点/采集集合的路线，防止等价走法挤满束宽。
      std::vector<Node> unique;
      for(auto& n:next[root]) {
        if(std::any_of(unique.begin(),unique.end(),[&](const auto& old) {
          if(old.at.x!=n.at.x||old.at.y!=n.at.y||old.consumed!=n.consumed||old.revealed!=n.revealed)return false;
          if(!w.planning.distribution_state)return true;
          if(old.outcomes.size()!=n.outcomes.size())return false;
          for(std::size_t j=0;j<n.outcomes.size();++j) {
            const auto& a=old.outcomes[j];const auto& b=n.outcomes[j];
            if(a.added!=b.added||a.normal!=b.normal||a.high!=b.high||a.scenario!=b.scenario||std::abs(a.mass-b.mass)>1e-10)return false;
          }
          return true;
        }))continue;
        unique.push_back(std::move(n));
        if(unique.size()>=static_cast<std::size_t>(w.planning.beam_width))break;
      }
      next[root]=std::move(unique);
    }
    if(interrupted) {result.budget_hit=true;break;}
    // 保留原路线作为额外提案；开启继续规划时，到点后寻找下一资源或可新增视野的邻格。
    std::optional<Node> next_baseline;
    if(baseline) {
      auto p=original_path[std::min<std::size_t>(depth,original_path.size()-1)];
      if(w.planning.continue_baseline&&static_cast<std::size_t>(depth)>=original_path.size()) {
        p=baseline->at;
        const std::vector<int>* next_field=nullptr;double best_value=0;
        for(std::size_t i=0;i<resources.size();++i) {
          if(baseline->consumed&(std::uint32_t{1}<<i))continue;
          const auto& r=resources[i];const int d=r.distances[index(baseline->at)];
          if(d<=0||depth+d>remaining)continue;
          const auto gain=increment(*baseline,r,depth+d);
          const double value=(resource_weight*gain.first+mission_weight*gain.second)/(d+w.travel_offset);
          if(value>best_value){best_value=value;next_field=&r.distances;}
        }
        if(next_field) {
          for(std::size_t i=1;i<deltas.size();++i) {
            const Position q{baseline->at.x+deltas[i].x,baseline->at.y+deltas[i].y};
            if(inside(q)&&passable(q,depth)&&(*next_field)[index(q)]==(*next_field)[index(baseline->at)]-1){p=q;break;}
          }
        } else {
          p=baseline->at;
          for(std::size_t i=1;i<deltas.size();++i) {
            const Position q{baseline->at.x+deltas[i].x,baseline->at.y+deltas[i].y};
            if(!inside(q)||!passable(q,depth))continue;
            const auto fresh=views[index(q)]&~baseline->revealed;
            const double value=static_cast<double>(fresh.count());
            if(value>best_value){best_value=value;p=q;}
          }
        }
      }
      if(passable(p,depth)) {
        if(result.expanded>=w.planning.max_nodes||elapsed()>=w.planning.budget_ms)
          {result.budget_hit=true;break;}
        next_baseline=advance(*baseline,p,depth);++result.expanded;
        auto& options=next[baseline_root];
        const bool retained=std::any_of(options.begin(),options.end(),[&](const auto& n) {
          return n.at.x==p.x&&n.at.y==p.y&&n.consumed==next_baseline->consumed&&
              n.revealed==next_baseline->revealed&&n.value>=next_baseline->value;
        });
        if(!retained) {
          options.push_back(*next_baseline);
          std::stable_sort(options.begin(),options.end(),[](const auto& a,const auto& b){return a.value>b.value;});
          if(options.size()>static_cast<std::size_t>(w.planning.beam_width))options.resize(w.planning.beam_width);
        }
      }
    }
    // 未来墙体可能让整组路线失效；不能拿较短窗口冒充相同深度的有效后续。
    for(std::size_t root=0;root<beams.size();++root)
      if(!beams[root].empty()&&next[root].empty()) {interrupted=true;break;}
    if(interrupted) {result.budget_hit=true;break;}
    beams=std::move(next);baseline=std::move(next_baseline);commit(depth);
  }
  // 只对完整束搜索保留下来的路线做有界末端扩展，避免每个中间节点都重跑多目标搜索。
  // 整组完成前不覆盖旧值；超时保留所有根动作原来的同窗口比较结果。
  if(w.planning.terminal_stops>1&&resources.size()>1&&result.completed_depth>=2) {
    bool interrupted=false;
    std::function<double(const Node&,int,int)> continuation;
    continuation=[&](const Node& n,int depth,int stops) {
      if(stops==0)return 0.0;
      if(elapsed()>=w.planning.budget_ms){interrupted=true;return 0.0;}
      struct Choice {std::size_t resource;int distance;double immediate;};
      std::vector<Choice> choices;
      for(std::size_t i=0;i<resources.size();++i) {
        if(n.consumed&(std::uint32_t{1}<<i))continue;
        const auto& r=resources[i];const int d=r.distances[index(n.at)];
        if(d<=0||depth+d>remaining)continue;
        const auto gain=increment(n,r,depth+d);
        const double immediate=std::pow(w.planning.discount,d)*r.preference*
            (resource_weight*gain.first+mission_weight*gain.second);
        if(immediate>0)choices.push_back({i,d,immediate});
      }
      std::stable_sort(choices.begin(),choices.end(),[](const auto& a,const auto& b){return a.immediate>b.immediate;});
      if(choices.size()>3)choices.resize(3);
      double best=0;
      for(const auto& c:choices) {
        const auto& r=resources[c.resource];
        const double cost=w.planning.safe_terminal?terminal_cost(n.at,r.distances,depth):0;
        if(!std::isfinite(cost))continue;
        Node next=n;double segment=0;bool valid=true;
        for(int step=1;step<=c.distance;++step) {
          std::optional<Position> chosen;double risk=std::numeric_limits<double>::infinity();
          for(std::size_t k=1;k<deltas.size();++k) {
            Position p{next.at.x+deltas[k].x,next.at.y+deltas[k].y};
            if(!inside(p)||r.distances[index(p)]!=r.distances[index(next.at)]-1||!passable(p,depth+step))continue;
            const double danger=std::max(0.0,future_risk(p,depth+step));
            if(danger<risk){chosen=p;risk=danger;}
          }
          if(!chosen){valid=false;break;}
          next.at=*chosen;
          // 经过的资源也必须即时结算，不能随后折回把同一块资源再估一次。
          for(std::size_t j=0;j<resources.size();++j)if(!(next.consumed&(std::uint32_t{1}<<j))&&
              resources[j].at.x==next.at.x&&resources[j].at.y==next.at.y) {
            const auto gain=increment(next,resources[j],depth+step);
            segment+=std::pow(w.planning.discount,step)*resources[j].preference*
                (resource_weight*gain.first+mission_weight*gain.second);
            pickup(next,j,depth+step);
          }
        }
        if(!valid)continue;
        const double after=continuation(next,depth+c.distance,stops-1);
        best=std::max(best,segment-cost+std::pow(w.planning.discount,c.distance)*after);
        if(interrupted)return 0.0;
      }
      return best;
    };
    auto extended=beams;
    for(auto& options:extended) {
      for(auto& n:options) {
        const double extra=w.planning.terminal_weight*std::pow(w.planning.discount,result.completed_depth)*
            continuation(n,result.completed_depth,w.planning.terminal_stops);
        // 探索末端机会依旧保留；多资源估计与原单资源估计取较大值，不重复相加。
        n.terminal=std::max(n.terminal,extra);n.value=n.reward-n.cost+n.terminal;
        if(interrupted)break;
      }
      if(interrupted)break;
      std::stable_sort(options.begin(),options.end(),[](const auto& a,const auto& b){return a.value>b.value;});
    }
    if(!interrupted){beams=std::move(extended);commit(result.completed_depth);result.terminal_extended=true;}
    else result.budget_hit=true;
  }
  return finish();
}
}  // namespace rmtc
