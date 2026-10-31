/**
 * @file train_resource_presence.cpp
 * @brief C++资源存在性校准：只用本队公开感知，按地图分组验证记忆衰减参数。
 *
 * 只接纳旧资源后来重新进入视野时的确认标签；从未重新观察的资源被删失，不当负例。
 * 此数据受本队路线选择影响，验证误差只代表重新观察样本，不宣称全图概率已校准。
 */
#include "rmtc_connection/stage2_planning.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <queue>
#include <set>
#include <stdexcept>
#include <map>
#include <cmath>

namespace {
using Json=rmtc::Json;
using Path=std::filesystem::path;
struct Sample {int group,age,enemy;bool present;double reference_age;};
/// 只比较公开的绝对格坐标，不读取裁判全图。
bool same(rmtc::Position a,rmtc::Position b){return a.x==b.x&&a.y==b.y;}

/// 在预测当时的已知地形上，计算当前可见敌车到各格的最近距离。
std::vector<int> enemies(const rmtc::PublicPerception& f,const rmtc::MapMemory& memory) {
  const int width=f.map.width,height=f.map.height;
  std::vector<int> distances(width*height,-1);std::queue<rmtc::Position> pending;
  for(const auto& e:f.visible_entities)if(e.type=="robot"&&e.team!=f.self.team&&(!e.hp||*e.hp>0)) {
    distances[e.position.y*width+e.position.x]=0;pending.push(e.position);
  }
  constexpr rmtc::Position directions[]{{0,-1},{0,1},{-1,0},{1,0}};
  while(!pending.empty()) {
    const auto at=pending.front();pending.pop();
    for(auto d:directions) {
      const rmtc::Position p{at.x+d.x,at.y+d.y};
      if(p.x<0||p.y<0||p.x>=width||p.y>=height||distances[p.y*width+p.x]>=0||
          memory.cells()[rmtc::MapMemory::index(p,f.map)]!=0)continue;
      distances[p.y*width+p.x]=distances[at.y*width+at.x]+1;pending.push(p);
    }
  }
  return distances;
}

/// 逐帧读取A席位感知，预测特征来自更新前的记忆，更新后的公开观察只用于标签。
void extract(const Path& path,int group,const Json& parameters,std::vector<Sample>& samples) {
  rmtc::MapMemory memory;std::ifstream input(path);std::string line;
  while(std::getline(input,line)) {
    const auto row=Json::parse(line);
    if(row.value("kind","")!="perception"||row.value("seat","")!="A")continue;
    const auto f=rmtc::parse_public_perception(row.at("data").dump(),"A");
    const auto old=memory.current();
    if(old&&old->session_id==f.session_id&&old->tick+1==f.tick&&old->self.alive&&f.self.alive) {
      const auto nearest=enemies(*old,memory);
      for(const auto& [key,m]:memory.entities()) {
        (void)key;const auto& e=m.entity;
        const auto age=old->tick-m.last_seen_tick;
        if(m.first_missing_tick||age<=0||age>200||
            (e.type!="resource_normal"&&e.type!="resource_high"))continue;
        if(std::abs(e.position.x-f.self.position.x)>f.vision_radius||
            std::abs(e.position.y-f.self.position.y)>f.vision_radius)continue;
        bool wall=false,present=false;
        for(const auto& visible:f.visible_entities)if(same(visible.position,e.position)) {
          wall=wall||visible.type=="obstacle";
          present=present||visible.type==e.type;
        }
        if(wall)continue; // 临时墙可能遮蔽资源，不能据此宣称已消失。
        const bool own_pickup=e.value&&same(f.self.position,e.position)&&
            std::abs(f.score.resource_value-old->score.resource_value-*e.value)<1e-6;
        double reference_age=parameters.value("resource_age_ticks",35.0);
        if(parameters.value("temporal_policy_enabled",false)) {
          const double alpha=std::pow(std::clamp(double(old->tick)/parameters.value("temporal_transition_ticks",100),0.0,1.0),
              parameters.value("temporal_shape",1.0));
          reference_age+=(parameters.value("temporal_resource_age_end",8.0)-reference_age)*alpha;
        }
        samples.push_back({group,static_cast<int>(age),nearest[e.position.y*f.map.width+e.position.x],present||own_pickup,reference_age});
      }
    }
    memory.update(f);
  }
}

/// 以每张地图等权计算Brier误差，避免长轨迹或重复出生多的地图支配验证分数。
Json evaluate(const std::vector<Sample>& samples,const rmtc::PlanningWeights& w,bool validation,bool legacy=false) {
  std::map<int,std::pair<double,int>> maps;int total=0,positive=0;
  for(const auto& s:samples)if((s.group%4==0)==validation) {
    const double p=legacy?1.0/(1+s.age/s.reference_age):rmtc::planning_resource_presence(s.age,s.enemy,w);
    maps[s.group].first+=(p-s.present)*(p-s.present);++maps[s.group].second;
    ++total;positive+=s.present;
  }
  double brier=0;
  for(const auto& [group,stats]:maps){(void)group;brier+=stats.first/stats.second;}
  return {{"samples",total},{"maps",maps.size()},{"positive",positive},
      {"map_mean_brier",maps.empty()?1.0:brier/maps.size()}};
}
} // namespace

/// 从完整对局目录提取校准样本，训练/开发按地图种子模4分开；保存所有候选的真实误差。
int main(int argc,char** argv) {
  try {
    if(argc!=3)throw std::runtime_error("usage: train_resource_presence MATCH_BATCH OUTPUT.json");
    std::vector<Sample> samples;
    for(const auto& directory:std::filesystem::directory_iterator(argv[1])) {
      const auto result=directory.path()/"result.json";
      if(!std::filesystem::is_regular_file(result))continue;
      std::ifstream file(result);Json data;file>>data;
      for(const auto& entry:std::filesystem::recursive_directory_iterator(directory.path()/"referee"))
        if(entry.path().filename()=="communication.jsonl")extract(entry.path(),data.at("seed"),data.at("parameters"),samples);
    }
    if(samples.size()<20)throw std::runtime_error("too few confirmed reobservations for calibration");
    rmtc::PlanningWeights best;double loss=1e9;Json candidates=Json::array();
    for(double half:{4.,8.,12.,20.,35.,50.,80.,120.})for(double pressure:{0.,.5,1.,2.,4.}) {
      auto w=best;w.memory_half_life=half;w.enemy_pressure=pressure;
      const auto train=evaluate(samples,w,false),validation=evaluate(samples,w,true);
      candidates.push_back({{"half_life",half},{"enemy_pressure",pressure},{"train",train},{"validation",validation}});
      if(train.at("map_mean_brier").get<double>()<loss){loss=train.at("map_mean_brier");best=w;}
    }
    const auto validation=evaluate(samples,best,true),legacy=evaluate(samples,best,true,true);
    const bool supported=validation.at("samples").get<int>()>=20&&validation.at("maps").get<int>()>=2&&
        validation.at("map_mean_brier").get<double>()<legacy.at("map_mean_brier").get<double>();
    const Json result{{"schema","resource-presence-v1"},{"source",Path(argv[1]).string()},
        {"split","map seed modulo 4; remainder zero validation"},
        {"sample_selection","confirmed reobservations only; unseen outcomes censored; route selection bias remains"},
        {"planning_memory_half_life",best.memory_half_life},{"planning_enemy_pressure",best.enemy_pressure},
        {"validation",validation},{"legacy_validation",legacy},{"validation_supports_candidate",supported},
        {"candidates",candidates},{"paid_api_calls",0}};
    std::ofstream out(argv[2]);if(!out)throw std::runtime_error("cannot open calibration output");
    out<<result.dump(2)<<'\n';
    std::cout<<Json{{"parameters",{best.memory_half_life,best.enemy_pressure}},
        {"validation",validation},{"legacy_validation",legacy},{"supported",supported}}.dump()<<'\n';
    return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
