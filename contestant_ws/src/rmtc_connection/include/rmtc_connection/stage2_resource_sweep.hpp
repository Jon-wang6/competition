/**
 * @file stage2_resource_sweep.hpp
 * @brief 第二阶段有限束搜索：联合估计沿路采集、新视野及争抢延误，不读取真实全图。
 */
#pragma once
#include "rmtc_connection/stage2_collection.hpp"
#include "rmtc_connection/stage2_joint_planner.hpp"
#include <array>
#include <bitset>
#include <cmath>
#include <map>
#include <vector>

namespace rmtc {
/// 预测收益不是已取得分数；路径只穿过本帧已知可通行格，下帧重新规划。
struct ResourceSweepPlan {
  std::vector<planning::GridIndex> path;
  double utility{0}, resource_points{0}, mission_points{0}, information_points{0};
  std::string first_resource;
  int expanded{0};
};

/**
 * @brief 枚举有限移动序列，资源和未知格仅计首次到达/观察，分别扣减分数预算。
 * @param information 由上层根据新观察格总数、边缘加权总数估计机会价值。
 * @param passable 本帧已知空地，包含当前车辆/事件阻挡；不假定未知格可移动。
 * 已知敌人的资源争抢按整条路线预计到达步数重新折扣；旧位置不作为真实敌人。
 * 复杂度由horizon<=24、beam<=64及最多16块已知资源限制，只输出可审计的建议。
 */
template<class Passable, class Information>
ResourceSweepPlan resource_sweep(const PublicPerception& frame, const MapMemory& memory,
    std::vector<JointOpportunity> resources, int remaining, const Stage2Weights& w,
    double resource_weight, double mission_weight, Passable passable, Information information) {
  ResourceSweepPlan best;
  const auto size=frame.map;
  if (w.sweep_weight<=0 || frame.tick>=w.sweep_ticks || size.width*size.height>1024)
    return best;
  if (w.sweep_collect_only && resources.empty())
    return best;
  std::stable_sort(resources.begin(),resources.end(),[](const auto& a,const auto& b){return a.priority>b.priority;});
  if(resources.size()>16) resources.resize(16);
  struct Node {
    Position at;
    std::vector<planning::GridIndex> path;
    std::bitset<1024> revealed;
    std::uint32_t consumed{0};
    int first{-1}, new_cells{0};
    double edge_cells{0}, resource{0}, mission{0}, reward{0}, info{0}, budget{0};
    std::map<std::string,double> mission_budget;
    std::string first_resource;
    double value{0}, ordering{0};
  };
  Node root;root.at=frame.self.position;root.path.push_back({root.at.x,root.at.y});
  root.budget=w.resource_points/w.resource_cap*std::max(0.0,w.resource_cap-frame.score.resource_value);
  std::vector<Node> beam{root};
  constexpr std::array<Position,4> steps{{{0,-1},{0,1},{-1,0},{1,0}}};
  const int horizon=std::min(remaining,w.sweep_horizon);
  for(int depth=1;depth<=horizon;++depth) {
    std::vector<Node> next;
    for(const auto& old:beam) for(int direction=0;direction<4;++direction) {
      const Position at{old.at.x+steps[direction].x,old.at.y+steps[direction].y};
      if(!passable(at)) continue;
      Node n=old;n.at=at;n.first=depth==1?direction:old.first;n.path.push_back({at.x,at.y});++best.expanded;
      const double discount=std::pow(w.sweep_discount,depth-1);
      for(std::size_t i=0;i<resources.size();++i) {
        const auto& r=resources[i];
        if((n.consumed&(1u<<i)) || r.position.x!=at.x || r.position.y!=at.y) continue;
        n.consumed|=1u<<i;
        if(n.first_resource.empty()) n.first_resource=r.key;
        const double direct=collection_race_factor(r.direct_steps,r.nearest_enemy_steps,frame.tick,w);
        const double delayed=collection_race_factor(depth,r.nearest_enemy_steps,frame.tick,w);
        const double reliability=std::min(1.0,delayed/std::max(1e-9,direct));
        const double gain=std::min(n.budget,r.resource_points*reliability);n.budget-=gain;n.resource+=gain;
        double missions=0;
        for(const auto& [id,value]:r.mission_points) {
          auto [it,inserted]=n.mission_budget.emplace(id,w.mission_points*w.mission_potential);(void)inserted;
          const double earned=std::min(it->second,value*reliability);it->second-=earned;missions+=earned;
        }
        n.mission+=missions;n.reward+=discount*r.preference*(gain*resource_weight+missions*mission_weight);
      }
      for(int y=std::max(0,at.y-frame.vision_radius);y<=std::min(size.height-1,at.y+frame.vision_radius);++y)
        for(int x=std::max(0,at.x-frame.vision_radius);x<=std::min(size.width-1,at.x+frame.vision_radius);++x) {
          const int index=y*size.width+x;
          if(n.revealed[index] || memory.cells()[MapMemory::index({x,y},size)]!=-1) continue;
          n.revealed.set(index);++n.new_cells;
          const int edge=std::min({x,y,size.width-1-x,size.height-1-y});
          n.edge_cells+=1+w.edge_information_bonus/(1.0+edge);
        }
      const double info=information(n.new_cells,n.edge_cells);
      n.reward+=discount*std::max(0.0,info-old.info);n.info=info;
      n.value=n.reward/(depth+w.travel_offset);
      // 此潜力只用于保留尚在接近资源的分支，不计入最终收益。
      double potential=0;
      for(std::size_t i=0;i<resources.size();++i) if(!(n.consumed&(1u<<i))) {
        const auto& r=resources[i];const int d=std::abs(at.x-r.position.x)+std::abs(at.y-r.position.y);
        if(depth+d>horizon) continue;
        double mission=0;for(const auto& item:r.mission_points) mission+=item.second;
        potential=std::max(potential,r.preference*(r.resource_points*resource_weight+mission*mission_weight)/(depth+d+w.travel_offset));
      }
      n.ordering=n.value+.6*potential;
      // 采集专用模式不让纯开图分支覆盖已经验证的普通探索策略。
      if((!w.sweep_collect_only || n.consumed!=0) && n.value>best.utility+1e-9) {
        best.utility=n.value;best.path=n.path;best.resource_points=n.resource;best.mission_points=n.mission;
        best.information_points=n.info;best.first_resource=n.first_resource;
      }
      next.push_back(std::move(n));
    }
    std::stable_sort(next.begin(),next.end(),[](const auto& a,const auto& b){return a.ordering>b.ordering;});
    beam.clear();std::array<int,4> kept{};
    for(auto& n:next) {
      if(kept[n.first]>=(w.sweep_beam+3)/4) continue;
      bool duplicate=false;
      for(const auto& q:beam) if(n.first==q.first && n.at.x==q.at.x && n.at.y==q.at.y && n.consumed==q.consumed && n.revealed==q.revealed) {duplicate=true;break;}
      if(duplicate) continue;
      ++kept[n.first];beam.push_back(std::move(n));
      if(beam.size()>=static_cast<std::size_t>(w.sweep_beam)) break;
    }
    if(beam.empty()) break;
  }
  best.utility*=w.sweep_weight;
  return best;
}
} // namespace rmtc
