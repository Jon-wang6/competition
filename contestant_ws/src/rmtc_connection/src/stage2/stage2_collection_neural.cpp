/**
 * @file stage2_collection_neural.cpp
 * @brief 采集路线评分器的固定具名输入；模型架构与训练标签均与战斗评分器分开。
 */
#include "rmtc_connection/stage2_collection_neural.hpp"
#include "rmtc_connection/stage2_config.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <queue>

namespace rmtc {
Json collection_baseline_signature(const Stage2Weights& w) {
  const auto& p=w.planning;
  return {{"horizon",p.horizon},{"beam_width",p.beam_width},{"max_nodes",p.max_nodes},
      {"discount",p.discount},{"terminal_weight",p.terminal_weight},{"terminal_stops",p.terminal_stops},
      {"race_scenarios",p.race_scenarios},{"race_memory_ticks",p.race_memory_ticks},
      {"race_scenario_weight",p.race_scenario_weight},
      {"memory_model",p.memory_model},{"memory_half_life",p.memory_half_life},{"enemy_pressure",p.enemy_pressure},
      {"regional_exploration",p.regional_exploration},{"regional_strength",p.regional_strength},
      {"resource_weight",w.acquisition.resource_weight},{"visible_weight",w.visible_resource_weight}};
}

const std::array<const char*,kNeuralInputs>& collection_neural_feature_names() {
  static const std::array<const char*,kNeuralInputs> names{{
    "tick_fraction","own_hp","resource_fraction","damage_fraction","map_coverage",
    "visible_resources","remembered_resources","visible_enemies","known_high_resources",
    "normal_task_need","high_task_need","reach_task_need","completed_depth","budget_hit",
    "wait_root","up_root","down_root","left_root","right_root","root_value",
    "root_resource_points","root_mission_points","root_information","root_terminal","root_advantage",
    "route_length","end_dx","end_dy","first_resource_steps","first_resource_value",
    "first_resource_age","first_enemy_steps","first_race_margin","unique_route_cells","route_unknown_views",
    "route_normal_count","route_high_count","end_wall_neighbors","end_free_neighbors","end_unknown_neighbors",
    "resource_value_0","resource_value_1","resource_value_2","resource_value_3",
    "resource_value_4","resource_value_5","resource_value_6","resource_value_7",
    "resource_age_0","resource_age_1","resource_age_2","resource_age_3",
    "resource_age_4","resource_age_5","resource_age_6","resource_age_7",
    "regional_supply_mean","spawn_protected","nearest_visible_enemy","normal_collected","high_collected",
    "resource_score_remaining","competition_scenarios","terminal_extended"}};
  return names;
}

NeuralFeatures collection_route_features(const PublicPerception& f,const MapMemory& memory,
    const PlanningResult& plan,std::size_t root,int normal,int high,const std::vector<int>& enemy,
    const std::vector<double>& supply) {
  NeuralFeatures x{};if(root>=plan.roots.size()||!plan.roots[root].valid)return x;
  const auto put=[&](int i,double v){x[i]=static_cast<float>(std::clamp(v,-1.0,1.0));};
  const auto& r=plan.roots[root];const int count=f.map.width*f.map.height;
  const auto inside=[&](Position p){return p.x>=0&&p.y>=0&&p.x<f.map.width&&p.y<f.map.height;};
  const auto at=[&](Position p){return p.y*f.map.width+p.x;};
  put(0,f.tick/500.0);put(1,f.self.hp/100);put(2,f.score.resource_value/120);put(3,f.score.effective_damage/500);
  put(4,count?double(std::count_if(memory.cells().begin(),memory.cells().end(),[](auto c){return c!=-1;}))/count:0);
  int visible_resources=0,visible_enemies=0,nearest=100;
  for(const auto& e:f.visible_entities) {
    if(e.type=="resource_normal"||e.type=="resource_high")++visible_resources;
    if(e.type=="robot"&&e.team&&*e.team!=f.self.team) {
      ++visible_enemies;nearest=std::min(nearest,std::abs(e.position.x-f.self.position.x)+std::abs(e.position.y-f.self.position.y));
    }
  }
  put(5,visible_resources/16.0);put(7,visible_enemies/3.0);put(58,nearest/48.0);
  int normal_need=0,high_need=0,reach_need=0;
  for(const auto& m:f.missions)if(std::find(f.score.mission_completed_ids.begin(),f.score.mission_completed_ids.end(),m.id)==f.score.mission_completed_ids.end()) {
    if(m.type=="normal_resources"&&m.target)normal_need=std::max(0,int(*m.target)-normal);
    if(m.type=="high_resources"&&m.target)high_need=std::max(0,int(*m.target)-high);
    if(m.type=="reach")reach_need=1;
  }
  put(9,normal_need/4.0);put(10,high_need/4.0);put(11,reach_need);
  put(12,plan.completed_depth/32.0);put(13,plan.budget_hit);put(14+static_cast<int>(root),1);
  put(19,r.value/kCollectionNeuralScale);put(20,r.resource_points/30);put(21,r.mission_points/15);
  put(22,r.information/16);put(23,r.terminal/kCollectionNeuralScale);
  put(24,(r.value-plan.roots[0].value)/kCollectionNeuralScale);put(25,r.path.size()/32.0);
  const Position end=r.path.empty()?f.self.position:r.path.back();
  // 窗口末端到资源的特征也走已知地形，不能用曼哈顿距离冒充能穿墙的路线。
  std::vector<int> end_distance(std::max(0,count),-1);std::queue<Position> pending;
  if(inside(end)){end_distance[at(end)]=0;pending.push(end);}
  constexpr std::array<Position,4> delta{{{0,-1},{0,1},{-1,0},{1,0}}};
  while(!pending.empty()) {
    const auto p=pending.front();pending.pop();
    for(auto d:delta) {
      Position q{p.x+d.x,p.y+d.y};
      if(!inside(q)||end_distance[at(q)]>=0||memory.cells()[MapMemory::index(q,f.map)]!=0)continue;
      end_distance[at(q)]=end_distance[at(p)]+1;pending.push(q);
    }
  }
  put(26,double(end.x-f.self.position.x)/std::max(1,f.map.width));put(27,double(end.y-f.self.position.y)/std::max(1,f.map.height));
  std::set<std::pair<int,int>> unique,unknown;
  for(auto p:r.path) {
    unique.emplace(p.x,p.y);
    for(int y=std::max(0,p.y-f.vision_radius);y<=std::min(f.map.height-1,p.y+f.vision_radius);++y)
      for(int z=std::max(0,p.x-f.vision_radius);z<=std::min(f.map.width-1,p.x+f.vision_radius);++z)
        if(memory.cells()[MapMemory::index({z,y},f.map)]==-1)unknown.emplace(z,y);
  }
  put(33,unique.size()/32.0);put(34,unknown.size()/100.0);
  double region=0;for(auto p:unknown)region+=supply.empty()?1:supply[at({p.first,p.second})];
  put(56,unknown.empty()?.5:region/unknown.size()/2);
  struct Item {int steps;double value,age;int enemy;bool high,on_route;};std::vector<Item> resources;
  for(const auto& [key,m]:memory.entities()) {
    (void)key;const auto& e=m.entity;
    if(m.first_missing_tick||!e.value||!inside(e.position)||(e.type!="resource_normal"&&e.type!="resource_high"))continue;
    int steps=end_distance[at(e.position)]<0?100:static_cast<int>(r.path.size())-1+end_distance[at(e.position)];bool seen=false;
    for(std::size_t i=1;i<r.path.size();++i)if(r.path[i].x==e.position.x&&r.path[i].y==e.position.y){steps=static_cast<int>(i);seen=true;break;}
    resources.push_back({steps,*e.value,double(f.tick-m.last_seen_tick),enemy.empty()?-1:enemy[at(e.position)],e.type=="resource_high",seen});
  }
  std::stable_sort(resources.begin(),resources.end(),[](const auto& a,const auto& b){return a.steps<b.steps;});
  put(6,resources.size()/16.0);int route_normal=0,route_high=0,known_high=0;
  for(const auto& e:resources){known_high+=e.high;route_normal+=e.on_route&&!e.high;route_high+=e.on_route&&e.high;}
  put(8,known_high/4.0);put(35,route_normal/12.0);put(36,route_high/4.0);
  if(!resources.empty()) {
    const auto& e=resources.front();put(28,e.steps/48.0);put(29,e.value/20);put(30,e.age/100);
    put(31,e.enemy<0?1:e.enemy/48.0);put(32,e.enemy<0?0:(e.enemy-e.steps)/24.0);
  }
  for(std::size_t i=0;i<std::min<std::size_t>(8,resources.size());++i){put(40+i,resources[i].value/20);put(48+i,resources[i].age/100);}
  for(auto d:delta){Position p{end.x+d.x,end.y+d.y};const int v=inside(p)?memory.cells()[MapMemory::index(p,f.map)]:100;put(v==100?37:v==0?38:39,x[v==100?37:v==0?38:39]+.25);}
  put(57,f.self.spawn_protected);put(59,normal/12.0);put(60,high/4.0);put(61,std::max(0.0,120-f.score.resource_value)/120);
  put(62,plan.competition.value("scenarios",Json::array()).size()/4.0);put(63,plan.terminal_extended);
  return x;
}
} // namespace rmtc
