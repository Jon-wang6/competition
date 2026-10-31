/**
 * @file stage2_enemy_belief.hpp
 * @brief 连续更新敌人可能位置，保留每一帧空视野证据，避免旧线索反复复活。
 *
 * 仅使用公开观察：每Tick允许停留或移动一格，已知墙不可穿过。未知地形仅参与
 * 敌人位置假设，不成为本车可执行路径。质量随最后目击年龄衰减，不表示真实坐标。
 */
#pragma once
#include "rmtc_connection/map_memory.hpp"
#include <array>
#include <cmath>
#include <map>
#include <numeric>
#include <string>
#include <vector>

namespace rmtc {
class EnemyBeliefTracker {
 public:
  Json details = Json::array();

  /** @brief 换局清除全部条件分布，不能把上一局敌人带入本局。 */
  void reset() { tracks_.clear(); prefix_.clear(); session_.clear(); tick_=-1; details=Json::array(); }

  /**
   * @brief 先扩散上一帧分布，再排除本帧已确认空置的视野；看到敌人时直接校正。
   * 已消失的概率不从旧目击位置重新生成。完全失去线索时交给巡逻策略，
   * 不猜测死亡、复活点或瞬移目的地。漏帧只扩散，不编造中间的空视野。
   */
  void update(const PublicPerception& frame, const MapMemory& memory,
              double half_life, int max_age) {
    if (frame.session_id!=session_ || frame.map.width!=size_.width || frame.map.height!=size_.height) {
      reset();session_=frame.session_id;size_=frame.map;
    }
    if (frame.tick<=tick_) return;
    const int count=size_.width*size_.height;
    const auto passable=[&](Position p) {
      return p.x>=0&&p.y>=0&&p.x<size_.width&&p.y<size_.height&&
          memory.cells()[MapMemory::index(p,size_)]!=100;
    };
    const auto seen=[&](Position p) {
      return frame.self.alive&&std::abs(p.x-frame.self.position.x)<=frame.vision_radius&&
          std::abs(p.y-frame.self.position.y)<=frame.vision_radius;
    };
    constexpr std::array<Position,5> steps{{{0,0},{0,-1},{0,1},{-1,0},{1,0}}};
    std::vector<double> total(count,0);
    details=Json::array();
    for (const auto& [key,remembered]:memory.entities()) {
      const auto& enemy=remembered.entity;
      if (enemy.type!="robot"||enemy.team==frame.self.team) continue;
      const auto age=frame.tick-remembered.last_seen_tick;
      if ((enemy.hp&&*enemy.hp<=0)||age>max_age) {tracks_.erase(key);continue;}
      auto [it,inserted]=tracks_.try_emplace(key);
      auto& track=it->second;
      if (inserted) {
        track.mass.assign(count,0);
        if (passable(enemy.position)) track.mass[index(enemy.position)]=1;
        track.tick=remembered.last_seen_tick;
      }
      if (age==0) {
        std::fill(track.mass.begin(),track.mass.end(),0);
        track.mass[index(enemy.position)]=1;
      } else {
        const int elapsed=static_cast<int>(std::min<std::int64_t>(max_age,frame.tick-track.tick));
        for (int dt=0;dt<elapsed;++dt) {
          std::vector<double> next(count,0);
          for (int y=0;y<size_.height;++y) for (int x=0;x<size_.width;++x) {
            const double mass=track.mass[index({x,y})];
            if (mass==0 || !passable({x,y})) continue;
            int options=0;
            for (const auto step:steps) options+=passable({x+step.x,y+step.y});
            if (options==0) continue;
            for (const auto step:steps) {
              Position p{x+step.x,y+step.y};
              if (passable(p)) next[index(p)]+=mass/options;
            }
          }
          track.mass=std::move(next);
        }
        for (int y=0;y<size_.height;++y) for (int x=0;x<size_.width;++x)
          if (seen({x,y})) track.mass[index({x,y})]=0;
        const double mass=std::accumulate(track.mass.begin(),track.mass.end(),0.0);
        if (mass>0) for (auto& value:track.mass) value/=mass;
      }
      track.tick=frame.tick;
      if (age<=0) continue;
      const double confidence=std::exp2(-static_cast<double>(age)/half_life);
      int possible=0;
      for (int i=0;i<count;++i) {total[i]+=confidence*track.mass[i];possible+=track.mass[i]>0;}
      if (possible>0) details.push_back({{"id",enemy.id.value_or(enemy.team.value_or("enemy"))},
          {"last_seen_tick",remembered.last_seen_tick},{"age",age},{"possible_cells",possible},
          {"confidence",confidence},{"old_position_confirmed_empty",remembered.first_missing_tick.has_value()},
          {"continuous_empty_view_memory",true}});
    }
    tick_=frame.tick;
    const int stride=size_.width+1;
    prefix_.assign(stride*(size_.height+1),0);
    for (int y=0;y<size_.height;++y) for (int x=0;x<size_.width;++x)
      prefix_[(y+1)*stride+x+1]=total[index({x,y})]+prefix_[y*stride+x+1]+
          prefix_[(y+1)*stride+x]-prefix_[y*stride+x];
  }

  /** @brief 查询观察位置可覆盖的历史敌人质量；只作为搜索机会，不能用于直接攻击。 */
  double in_view(Position p,int radius) const {
    if (prefix_.empty()) return 0;
    const int x0=std::max(0,p.x-radius),y0=std::max(0,p.y-radius);
    const int x1=std::min(size_.width,p.x+radius+1),y1=std::min(size_.height,p.y+radius+1);
    const int stride=size_.width+1;
    const double mass=prefix_[y1*stride+x1]-prefix_[y0*stride+x1]-
        prefix_[y1*stride+x0]+prefix_[y0*stride+x0];
    return mass<1e-12?0.0:std::min(1.0,mass);
  }

 private:
  struct Track {std::vector<double> mass;std::int64_t tick{0};};
  /** @brief 私有概率数组采用上到下行序，调用方已校验地图边界。 */
  int index(Position p) const {return p.y*size_.width+p.x;}
  MapSize size_{};
  std::string session_;
  std::int64_t tick_{-1};
  std::map<std::string,Track> tracks_;
  std::vector<double> prefix_;
};
}  // namespace rmtc
