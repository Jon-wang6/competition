/** @file stage2_collection_search.cpp
 * @brief 用已知地形距离、唯一覆盖集合与到达时间，搜索有限多段采集路线。
 *
 * @details 模块协作与维护约定
 * 状态同时记录资源访问、观察覆盖和累计到达时间；只对新增收益计分。
 * 搜索深度/候选宽度限制计算量，结果是局部滚动建议，不保证竞争环境中的全局最优。
 */
#include "rmtc_connection/stage2_collection_search.hpp"
#include "rmtc_connection/stage2_collection.hpp"
#include <array>
#include <bitset>
#include <chrono>
#include <limits>
#include <queue>

namespace rmtc {
CollectionSearchPlan search_collection_route(
    const PublicPerception& frame, const MapMemory& memory,
    std::vector<JointOpportunity> resources, int remaining, const Stage2Weights& w,
    double resource_weight, double mission_weight,
    const std::function<bool(Position)>& passable,
    const std::function<double(double,double)>& information,
    const std::function<double(Position,int)>& risk,const std::vector<double>& unknown_weights) {
  CollectionSearchPlan result;
  if (w.collection_search_weight<=0 || frame.tick>=w.collection_search_ticks || remaining<=0)
    return result;
  const auto began=std::chrono::steady_clock::now();
  result.ran=true;
  const auto elapsed=[&] {return std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-began).count();};
  const auto expired=[&] {return elapsed()>=w.collection_search_budget_ms;};
  const auto finish=[&] {result.elapsed_ms=elapsed();return result;};
  const int width=frame.map.width,height=frame.map.height,count=width*height;
  if(count<=0 || count>1024 || (!unknown_weights.empty()&&unknown_weights.size()!=static_cast<std::size_t>(count))) return finish();
  const auto index=[&](Position p) {return p.y*width+p.x;};
  const auto position=[&](int i) {return Position{i%width,i/width};};
  const auto free=[&](Position p) {return p.x>=0&&p.y>=0&&p.x<width&&p.y<height&&passable(p);};
  const int horizon=std::min(remaining,w.collection_search_horizon);
  constexpr std::array<Position,4> steps{{{0,-1},{0,1},{-1,0},{1,0}}};
  struct Field {std::vector<int> distance,parent;};
  /// 同一目标的距离场在本Tick内复用；所有可执行路径只通过当前确认可走的格子。
  const auto bfs=[&](Position start) {
    Field field;field.distance.assign(count,-1);field.parent.assign(count,-1);
    std::queue<int> pending;field.distance[index(start)]=0;pending.push(index(start));
    while(!pending.empty()) {
      const int here=pending.front();pending.pop();const auto at=position(here);
      for(const auto d:steps) {
        const Position next{at.x+d.x,at.y+d.y};
        if(!free(next)||field.distance[index(next)]>=0) continue;
        field.distance[index(next)]=field.distance[here]+1;field.parent[index(next)]=here;
        pending.push(index(next));
      }
    }
    return field;
  };
  std::vector<Position> targets{frame.self.position};
  std::vector<Field> fields{bfs(frame.self.position)};
  std::stable_sort(resources.begin(),resources.end(),[](const auto& a,const auto& b){return a.priority>b.priority;});
  resources.erase(std::remove_if(resources.begin(),resources.end(),[&](const auto& r) {
    return !free(r.position)||fields[0].distance[index(r.position)]<=0||
           fields[0].distance[index(r.position)]>horizon;
  }),resources.end());
  if(resources.size()>static_cast<std::size_t>(w.collection_search_resources))
    resources.resize(w.collection_search_resources);
  /// 加入唯一目标；资源先占候选名额，再给分散的观察点保留搜索空间。
  const auto add_target=[&](Position p) {
    if(targets.size()>static_cast<std::size_t>(w.collection_search_targets)) return false;
    if(std::any_of(targets.begin(),targets.end(),[&](Position q){return p.x==q.x&&p.y==q.y;})) return false;
    targets.push_back(p);return true;
  };
  for(const auto& r:resources) add_target(r.position);
  struct Reach {Position at;double value,preference;};
  std::vector<Reach> reach;
  for(const auto& task:frame.missions) {
    if(task.type!="reach"||!task.position||!free(*task.position)||reach.size()>=16||
       std::find(frame.score.mission_completed_ids.begin(),frame.score.mission_completed_ids.end(),task.id)!=frame.score.mission_completed_ids.end()) continue;
    const int d=fields[0].distance[index(*task.position)];
    if(d<=0||d>horizon) continue;
    reach.push_back({*task.position,w.mission_points*w.mission_potential,
        reach_priority_at(w,frame.tick,remaining,d,true)});
    add_target(*task.position);
  }
  std::vector<std::bitset<1024>> views(count);
  std::vector<double> edge_weight(count,0);
  for(int i=0;i<count;++i) {
    const auto p=position(i);
    const int edge=std::min({p.x,p.y,width-1-p.x,height-1-p.y});
    edge_weight[i]=1+w.edge_information_bonus/(1.0+edge);
    if(fields[0].distance[i]<0) continue;
    for(int y=std::max(0,p.y-frame.vision_radius);y<=std::min(height-1,p.y+frame.vision_radius);++y)
      for(int x=std::max(0,p.x-frame.vision_radius);x<=std::min(width-1,p.x+frame.vision_radius);++x)
        if(memory.cells()[MapMemory::index({x,y},frame.map)]==-1) views[i].set(index({x,y}));
  }
  struct Observation {Position at;double value;};
  std::vector<Observation> observations;
  for(int i=0;i<count;++i) {
    const int d=fields[0].distance[i];if(d<=0||d>horizon||views[i].none()) continue;
    double edge=0,effective=0;for(int j=0;j<count;++j) if(views[i][j]) {
      edge+=edge_weight[j];effective+=unknown_weights.empty()?1.:unknown_weights[j];
    }
    const double value=information(effective,edge)/(d+w.travel_offset);
    if(value>0) observations.push_back({position(i),value});
  }
  std::stable_sort(observations.begin(),observations.end(),[](const auto& a,const auto& b){return a.value>b.value;});
  std::vector<Position> observation_points;
  for(const auto& observation:observations) {
    if(std::any_of(observation_points.begin(),observation_points.end(),[&](Position p) {
      return std::abs(p.x-observation.at.x)+std::abs(p.y-observation.at.y)<std::max(2,frame.vision_radius/2);
    })) continue;
    if(add_target(observation.at)) observation_points.push_back(observation.at);
    if(targets.size()>static_cast<std::size_t>(w.collection_search_targets)) break;
  }
  result.targets=static_cast<int>(targets.size())-1;
  for(std::size_t i=1;i<targets.size();++i) {
    if(expired()) {result.budget_hit=true;return finish();}
    fields.push_back(bfs(targets[i]));
  }
  std::vector<std::vector<int>> resource_at(count),reach_at(count);
  for(std::size_t i=0;i<resources.size();++i) resource_at[index(resources[i].position)].push_back(static_cast<int>(i));
  for(std::size_t i=0;i<reach.size();++i) reach_at[index(reach[i].at)].push_back(static_cast<int>(i));
  std::vector<double> risk_cache(static_cast<std::size_t>(count)*(horizon+1),-1);
  struct Node {
    int target{0},ticks{0},first_ticks{0},first_direction{-1};
    std::uint32_t consumed{0},reached{0};std::uint64_t visited{1};
    std::bitset<1024> revealed;
    double edge{0},effective_unknown{0},information{0},resource{0},mission{0},reward{0},cost{0};
    double first_resource_points{0},first_mission_points{0},first_information{0};
    double first_reward{0},first_cost{0},utility{0},ordering{0},budget{0};
    std::map<std::string,double> mission_budget;
    std::vector<planning::GridIndex> first_path;
    std::string first_resource;
  };
  Node root;root.budget=w.resource_points/w.resource_cap*std::max(0.0,w.resource_cap-frame.score.resource_value);
  std::vector<Node> beam{root};
  for(int depth=1;depth<=w.collection_search_depth;++depth) {
    std::vector<Node> next;
    bool interrupted=false;
    for(const auto& old:beam) {
      if(interrupted) break;
      for(std::size_t target=1;target<targets.size();++target) {
        if((result.expanded&31)==0&&expired()) {interrupted=true;break;}
        if(old.visited&(std::uint64_t{1}<<target)) continue;
        const auto& field=fields[old.target];const int destination=index(targets[target]);
        const int distance=field.distance[destination];
        if(distance<=0||old.ticks+distance>horizon) continue;
        ++result.expanded;
        std::vector<int> leg;
        for(int at=destination;at!=index(targets[old.target]);at=field.parent[at]) leg.push_back(at);
        std::reverse(leg.begin(),leg.end());
        Node node=old;node.target=static_cast<int>(target);node.visited|=std::uint64_t{1}<<target;
        const auto before_revealed=node.revealed;
        for(const int cell:leg) {
          ++node.ticks;const auto at=position(cell);
          node.revealed|=views[cell];
          auto& danger=risk_cache[static_cast<std::size_t>(cell)*(horizon+1)+node.ticks];
          if(danger<0) danger=std::max(0.0,risk(at,node.ticks));
          node.cost+=w.collection_search_risk*danger;
          const double discount=std::pow(w.collection_search_discount,node.ticks-1);
          for(const int r:resource_at[cell]) {
            if(node.consumed&(std::uint32_t{1}<<r)) continue;
            node.consumed|=std::uint32_t{1}<<r;const auto& item=resources[r];
            const double direct=collection_race_factor(item.direct_steps,item.nearest_enemy_steps,frame.tick,w);
            const double delayed=collection_race_factor(node.ticks,item.nearest_enemy_steps,frame.tick,w);
            const double reliability=std::min(1.0,delayed/std::max(1e-9,direct));
            const double earned=std::min(node.budget,item.resource_points*reliability);
            node.budget-=earned;node.resource+=earned;
            double mission=0;
            for(const auto& [id,value]:item.mission_points) {
              auto [it,inserted]=node.mission_budget.emplace(id,w.mission_points*w.mission_potential);(void)inserted;
              const double amount=std::min(it->second,value*reliability);it->second-=amount;mission+=amount;
            }
            node.mission+=mission;
            node.reward+=discount*item.preference*(earned*resource_weight+mission*mission_weight);
            if(depth==1&&node.first_resource.empty()) node.first_resource=item.key;
          }
          for(const int r:reach_at[cell]) if(!(node.reached&(std::uint32_t{1}<<r))) {
            node.reached|=std::uint32_t{1}<<r;node.mission+=reach[r].value;
            node.reward+=discount*reach[r].value*reach[r].preference;
          }
        }
        const auto added=node.revealed&~before_revealed;
        for(int i=0;i<count;++i) if(added[i]) {
          node.edge+=edge_weight[i];node.effective_unknown+=unknown_weights.empty()?1.:unknown_weights[i];
        }
        const double info=information(node.effective_unknown,node.edge);
        node.reward+=std::pow(w.collection_search_discount,node.ticks-1)*std::max(0.0,info-old.information);
        node.information=info;
        if(node.reward<=old.reward+1e-12) continue;
        if(depth==1) {
          node.first_ticks=node.ticks;node.first_reward=node.reward;node.first_cost=node.cost;
          node.first_resource_points=node.resource;node.first_mission_points=node.mission;node.first_information=info;
          node.first_path.push_back({frame.self.position.x,frame.self.position.y});
          for(const int i:leg) {const auto p=position(i);node.first_path.push_back({p.x,p.y});}
          const auto first=position(leg.front());
          node.first_direction=first.y<frame.self.position.y?0:first.y>frame.self.position.y?1:first.x<frame.self.position.x?2:3;
        }
        node.utility=w.collection_search_weight*(
            node.first_reward/(node.first_ticks+w.travel_offset)-node.first_cost+
            w.collection_search_future*((node.reward-node.first_reward)/(node.ticks+w.travel_offset)-(node.cost-node.first_cost)));
        double potential=0;
        for(std::size_t i=0;i<resources.size();++i) if(!(node.consumed&(std::uint32_t{1}<<i))) {
          const int d=fields[target].distance[index(resources[i].position)];
          if(d<=0||node.ticks+d>horizon) continue;
          potential=std::max(potential,std::min(node.budget,resources[i].resource_points)*resource_weight/(node.ticks+d+w.travel_offset));
        }
        node.ordering=node.utility+w.collection_search_future*.25*potential;
        next.push_back(std::move(node));
      }
    }
    if(interrupted) {result.budget_hit=true;break;}
    result.completed_depth=depth;
    for(const auto& node:next) if(node.utility>result.utility+1e-9) {
      result.utility=node.utility;result.route.status=planning::SearchStatus::Found;
      result.route.path=node.first_path;result.route.cost=node.first_ticks;
      result.first_resource=node.first_resource;result.resource_points=node.first_resource_points;
      result.mission_points=node.first_mission_points;result.information_points=node.first_information;
      result.forecast_resource=node.resource;result.forecast_mission=node.mission;
    }
    std::stable_sort(next.begin(),next.end(),[](const auto& a,const auto& b){return a.ordering>b.ordering;});
    beam.clear();std::array<int,4> directions_kept{};
    for(auto& node:next) {
      if(directions_kept[node.first_direction]>=(w.collection_search_beam+3)/4) continue;
      const bool duplicate=std::any_of(beam.begin(),beam.end(),[&](const auto& q) {
        return node.target==q.target&&node.first_direction==q.first_direction&&node.consumed==q.consumed&&node.reached==q.reached&&node.revealed==q.revealed;
      });
      if(duplicate) continue;
      ++directions_kept[node.first_direction];beam.push_back(std::move(node));
      if(beam.size()>=static_cast<std::size_t>(w.collection_search_beam)) break;
    }
    if(beam.empty()) break;
  }
  return finish();
}
}  // namespace rmtc
