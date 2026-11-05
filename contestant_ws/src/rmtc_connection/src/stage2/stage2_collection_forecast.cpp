/**
 * @file stage2_collection_forecast.cpp
 * @brief 公开记忆上的有界敌方采集路线与分区发现机会估计。
 *
 * @details 模块协作与维护约定
 * 共同情景内每名对手只有一条采集路线，避免独立估计每个资源时把同一个对手当成能分身。
 * 输入为公开记忆和可通行地形，输出估计到达期限与区域供给先验；历史敌人位置随时可能失真。
 */
#include "rmtc_connection/stage2_collection_forecast.hpp"
#include <array>
#include <cmath>
#include <limits>
#include <queue>

namespace rmtc {
/**
 * @brief 导出敌人来源计数和每个共同情景中的资源到达期限。
 * @return 可解释JSON，明确标注历史位置仅为假设，不暴露或推断成裁判真值。
 */
Json CollectionRaceForecast::describe() const {
  Json rows=Json::array();
  for(const auto& s:scenarios)rows.push_back({{"name",s.name},{"arrival_steps",s.arrival}});
  return {{"current_enemies",current_enemies},{"historical_enemies",historical_enemies},
      {"historical_positions_are_hypotheses",true},{"scenarios",std::move(rows)}};
}

/**
 * @brief 基于公开资源与敌人记忆推演有界采集路线，共同情景内保持每个敌人只有一个位置。
 * @param f 当前公开感知。
 * @param memory 本队已观察地图与实体记忆。
 * @param previous 可空的上一公开帧，仅连续且同场时用于方向估计。
 * @param memory_ticks 接纳历史敌人的最大目击年龄，单位Tick。
 * @return 各情景下的资源预计到达时间；输入尺寸无效或无有效参与者时可为空。
 * @note 最近、高价值、朝向、交火停顿都是行为假设，不等于识别出了对手源代码。
 */
CollectionRaceForecast forecast_collection_race(const PublicPerception& f,const MapMemory& memory,
    const PublicPerception* previous,int memory_ticks) {
  CollectionRaceForecast result;
  const int width=f.map.width,height=f.map.height,count=width*height;
  if(count<=0||count>1024||memory.cells().size()!=static_cast<std::size_t>(count))return result;
  const auto inside=[&](Position p){return p.x>=0&&p.y>=0&&p.x<width&&p.y<height;};
  const auto index=[&](Position p){return p.y*width+p.x;};
  constexpr std::array<Position,4> steps{{{0,-1},{0,1},{-1,0},{1,0}}};
  struct Resource {std::string key;Position at;double value;std::vector<int> distance;};
  std::vector<Resource> resources;
  struct Actor {Position at,heading;int age,pause,goal{-1};};
  std::vector<Actor> actors;
  for(const auto& [key,m]:memory.entities()) {
    const auto& e=m.entity;if(!inside(e.position))continue;
    if((e.type=="resource_normal"||e.type=="resource_high")&&!m.first_missing_tick&&e.value&&*e.value>0) {
      Resource r{key,e.position,*e.value,std::vector<int>(count,-1)};
      std::queue<Position> q;q.push(e.position);r.distance[index(e.position)]=0;
      while(!q.empty()) {
        const auto p=q.front();q.pop();
        for(auto step:steps) {
          Position n{p.x+step.x,p.y+step.y};
          if(!inside(n)||r.distance[index(n)]>=0||memory.cells()[MapMemory::index(n,f.map)]!=0)continue;
          r.distance[index(n)]=r.distance[index(p)]+1;q.push(n);
        }
      }
      resources.push_back(std::move(r));
    } else if(e.type=="robot"&&e.team&&*e.team!=f.self.team&&(!e.hp||*e.hp>0)) {
      const int age=static_cast<int>(f.tick-m.last_seen_tick);if(age<0||age>memory_ticks)continue;
      Actor a{e.position,{0,0},age,e.defending.value_or(false)?3:0,-1};
      if(age==0&&previous&&previous->tick+1==f.tick&&previous->session_id==f.session_id)
        for(const auto& old:previous->visible_entities)if(old.type=="robot"&&old.team==e.team&&
            std::abs(old.position.x-e.position.x)+std::abs(old.position.y-e.position.y)==1)
          a.heading={e.position.x-old.position.x,e.position.y-old.position.y};
      if(age==0)++result.current_enemies;else ++result.historical_enemies;
      actors.push_back(a);
    }
  }
  if(resources.empty()||actors.empty())return result;
  constexpr std::array<const char*,4> names{{"nearest","high_value","heading","engaged"}};
  for(int mode=0;mode<4;++mode) {
    CollectionRaceScenario scenario;scenario.name=names[mode];auto state=actors;
    std::vector<bool> claimed(resources.size(),false);
    for(int time=0;time<=80;++time)for(auto& a:state) {
      if(mode==3&&time<a.pause+std::min(a.age,4))continue;
      for(std::size_t j=0;j<resources.size();++j)
        if(!claimed[j]&&a.at.x==resources[j].at.x&&a.at.y==resources[j].at.y) {
          claimed[j]=true;scenario.arrival[resources[j].key]=time;a.goal=-1;
        }
      if(time==80)continue;
      if(a.goal<0||claimed[a.goal]) {
        a.goal=-1;double best=std::numeric_limits<double>::infinity();
        for(std::size_t j=0;j<resources.size();++j) {
          const auto& r=resources[j];const int d=r.distance[index(a.at)];
          if(claimed[j]||d<=0)continue;
          double cost=d;
          if(mode==1)cost=(d+2)/(r.value/10.0);
          if(mode==2) {
            const int projection=(r.at.x-a.at.x)*a.heading.x+(r.at.y-a.at.y)*a.heading.y;
            cost+=projection<0?2.0:0.0;
          }
          if(cost<best){best=cost;a.goal=static_cast<int>(j);}
        }
      }
      if(a.goal<0)continue;
      const auto& field=resources[a.goal].distance;
      // 一个敌人每Tick最多移动一次；场景间的路径不能互相拼接成全知最快敌人。
      for(auto step:steps) {
        Position n{a.at.x+step.x,a.at.y+step.y};
        if(inside(n)&&field[index(n)]==field[index(a.at)]-1) {a.at=n;break;}
      }
    }
    result.scenarios.push_back(std::move(scenario));
  }
  return result;
}

RegionalResourceSupply regional_collection_supply(const PublicPerception& f,const MapMemory& memory,
    double strength,double initial_resource_value) {
  RegionalResourceSupply result;
  const int width=f.map.width,height=f.map.height,count=width*height;
  result.factors.assign(std::max(0,count),1.0);
  if(count<=0||count>1024||strength<=0)return result;
  const int span=6,cols=(width+span-1)/span,rows=(height+span-1)/span;
  struct Region {int unknown{0},seen{0},normal{0},high{0};double enemy{0},weight{1};};
  std::vector<Region> regions(cols*rows);
  const auto region=[&](Position p){return p.y/span*cols+p.x/span;};
  const auto inside=[&](Position p){return p.x>=0&&p.y>=0&&p.x<width&&p.y<height;};
  for(int y=0;y<height;++y)for(int x=0;x<width;++x) {
    auto& r=regions[region({x,y})];
    if(memory.cells()[MapMemory::index({x,y},f.map)]==-1)++r.unknown;else ++r.seen;
  }
  int seen_resources=0,seen_high=0;
  for(const auto& [key,m]:memory.entities()) {
    (void)key;const auto& e=m.entity;if(!inside(e.position))continue;
    auto& r=regions[region(e.position)];
    if(e.type=="resource_normal"){++r.normal;++seen_resources;}
    if(e.type=="resource_high"){++r.high;++seen_resources;++seen_high;}
    if(e.type=="robot"&&e.team&&*e.team!=f.self.team&&(!e.hp||*e.hp>0)) {
      const double age=std::max<std::int64_t>(0,f.tick-m.last_seen_tick);
      if(age>40)continue;
      for(int j=0;j<rows;++j)for(int i=0;i<cols;++i) {
        const int dx=std::max(0,std::abs(e.position.x-(i*span+span/2))-span/2);
        const int dy=std::max(0,std::abs(e.position.y-(j*span+span/2))-span/2);
        regions[j*cols+i].enemy+=std::exp2(-age/16.0)*std::exp(-double(dx+dy)/5.0);
      }
    }
  }
  bool need_high=false;
  for(const auto& m:f.missions)if(m.type=="high_resources"&&
      std::find(f.score.mission_completed_ids.begin(),f.score.mission_completed_ids.end(),m.id)==f.score.mission_completed_ids.end())need_high=true;
  // 初始总价值来自原规则参数，类型比例只是弱先验，不宣称知道真实隐藏资源数量。
  const double resource_prior=std::max(1.0,initial_resource_value/12.5);
  const double density=(resource_prior+seen_resources)/(count+std::count_if(memory.cells().begin(),memory.cells().end(),[](auto c){return c!=-1;}));
  const double high_density=(.25*resource_prior+seen_high)/(count+std::count_if(memory.cells().begin(),memory.cells().end(),[](auto c){return c!=-1;}));
  double total=0;int unknown=0;
  for(auto& r:regions) {
    // 64格伪观察提供强收缩；局部一个资源不会被当成确定的资源簇。
    const double local=(64*density+r.normal+r.high)/(64+r.seen)/density;
    const double local_high=(64*high_density+r.high)/(64+r.seen)/high_density;
    const double evidence=(need_high?.65*local+.35*local_high:local)*std::exp(-.35*r.enemy);
    r.weight=std::clamp(1+strength*(evidence-1),.35,1.8);
    total+=r.weight*r.unknown;unknown+=r.unknown;
  }
  const double mean=unknown?total/unknown:1;
  for(int y=0;y<height;++y)for(int x=0;x<width;++x)
    result.factors[y*width+x]=regions[region({x,y})].weight/mean;
  for(int j=0;j<rows;++j)for(int i=0;i<cols;++i) {
    const auto& r=regions[j*cols+i];
    result.regions.push_back({{"region",{i,j}},{"unknown",r.unknown},{"weight",r.weight/mean},{"enemy_evidence",r.enemy}});
  }
  return result;
}
} // namespace rmtc
