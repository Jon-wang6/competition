/**
 * @file stage2_battle_evidence.hpp
 * @brief 公开掉血/死亡形成的交火线索，以及开局敌人来路的资源消耗先验。
 *
 * 线索不等于敌人实际位置或数量，不据此直接开火。排除可能由本车和已解释事件
 * 造成的掉血；单纯消失不算死亡。敌人新实例重现时清除旧尸体关联的线索。
 *
 * @details 模块协作与维护约定
 * 掉血/死亡是战斗线索，需排除本车与已知事件造成的变化，单纯离开视野不是死亡证据。
 * 输出是带时效的软证据；敌人重新出现时旧尸体关联线索应失效，不能出现两个确定敌人。
 */
#pragma once
#include "rmtc_connection/stage2_events.hpp"
#include "rmtc_connection/stage2_opponent_intent.hpp"
#include "rmtc_connection/stage2_config.hpp"
#include <map>
#include <set>

namespace rmtc {
class BattleEvidence {
 public:
  Json details=Json::array(),origins=Json::array();
  /** @brief 新场次清空旧交火和来路，绝不把上一场的概率带入本场。 */
  void reset() {clues_.clear();origins_.clear();seen_.clear();mass_.clear();exposure_.clear();
    details=Json::array();origins=Json::array();session_.clear();tick_=-1;}

  /** @brief 席位优先关联敌人；只有缺少席位时才使用实例ID。 */
  static std::string identity(const VisibleEntity& e) {
    return e.team.value_or(e.id.value_or(e.robot_id.value_or("")));
  }

  /**
   * @brief 从连续公开帧更新证据，last_own为本车上一Tick期望的物理动作。
   * 掉帧不猜中间发生了什么；环境事件可解释伤害时保守放弃该次交火推断。
   */
  void update(const PublicPerception& frame,const std::optional<PublicPerception>& previous,
      const Stage2Events& events,const Action& last_own,const MapMemory& memory,
      const Stage2Weights& w) {
    if(frame.session_id!=session_) {reset();session_=frame.session_id;}
    if(frame.tick<=tick_) return;
    size_=frame.map;tick_=frame.tick;details=Json::array();origins=Json::array();
    const bool continuous=previous&&previous->tick+1==frame.tick&&previous->session_id==frame.session_id;
    const auto lookup=[&](const VisibleEntity& enemy)->const VisibleEntity* {
      if(!continuous) return nullptr;
      for(const auto& e:previous->visible_entities)
        if(e.type=="robot"&&identity(e)==identity(enemy)) return &e;
      return nullptr;
    };
    std::vector<VisibleEntity> old_bodies;
    if(continuous) {
      for(const auto& e:previous->visible_entities)
        if(e.type=="robot"&&(!e.hp||*e.hp>0)) old_bodies.push_back(e);
      VisibleEntity self;self.type="robot";self.team=frame.self.team;self.position=previous->self.position;
      old_bodies.push_back(self);
    }
    for(const auto& enemy:frame.visible_entities) {
      if(enemy.type!="robot"||enemy.team==frame.self.team||identity(enemy).empty()) continue;
      const auto key=identity(enemy);const auto* before=lookup(enemy);
      const bool reappeared=!before&&seen_.count(key)>0;
      const bool new_instance=before&&before->id&&enemy.id&&before->id!=enemy.id;
      const bool respawn=before&&before->hp&&*before->hp<=0&&enemy.hp&&*enemy.hp>0;
      if(new_instance||respawn||reappeared) {
        clues_.erase(key);origins_.erase(key);
        for(auto& [victim,clue]:clues_) {
          (void)victim;
          clue.attackers.erase(std::remove(clue.attackers.begin(),clue.attackers.end(),key),clue.attackers.end());
        }
      }
      seen_.insert(key);
      if(enemy.hp&&*enemy.hp<=0) origins_.erase(key);
      if((!enemy.hp||*enemy.hp>0)&&frame.tick<=w.opening_depletion_ticks) {
        auto found=origins_.find(key);
        Position direction{enemy.position.x-frame.self.position.x,enemy.position.y-frame.self.position.y};
        double confidence=.35;
        if(before&&!new_instance&&!respawn&&intent_distance(before->position,enemy.position)==1) {
          direction={before->position.x-enemy.position.x,before->position.y-enemy.position.y};
          confidence=.55;
        }
        if(found==origins_.end()||confidence>found->second.confidence) {
          if(std::abs(direction.x)>=std::abs(direction.y)) direction={direction.x>=0?1:-1,0};
          else direction={0,direction.y>=0?1:-1};
          origins_[key]={enemy.position,direction,frame.tick,confidence};
        }
      }
      if(!before||!before->hp||!enemy.hp||*before->hp<=0||new_instance||respawn) continue;
      const double loss=*before->hp-*enemy.hp;
      if(loss<=0||frame.event_text!=previous->event_text) continue;
      bool environmental=false;
      for(const auto& [id,e]:events.effects()) {
        (void)id;
        if(e.begin_tick>frame.tick||e.end_tick<=previous->tick||!e.actor.empty()) continue;
        if(e.kind=="purity") environmental=true;
        if(e.kind=="explosion"&&Stage2Events::contains(e.cells,before->position)) environmental=true;
        if(e.kind=="pepper") for(const auto p:e.points)
          if((p.x==before->position.x||p.y==before->position.y)&&
             (p.x!=before->position.x||p.y!=before->position.y)) environmental=true;
      }
      if(environmental) continue;
      double own_bound=0;
      if(last_own.type=="attack"&&!previous->self.spawn_protected) {
        const auto buff=events.buff(frame.self.team,"unrivaled",previous->tick);
        bool can_hit=false;
        if(buff) can_hit=std::max(std::abs(previous->self.position.x-before->position.x),
            std::abs(previous->self.position.y-before->position.y))<=buff->effect.attack_radius;
        else if(intent_clear_shot(previous->self.position,before->position,old_bodies)) {
          const auto a=previous->self.position,b=before->position;
          const auto direction=b.y<a.y?"up":b.y>a.y?"down":b.x<a.x?"left":"right";
          can_hit=last_own.direction==direction;
        }
        if(can_hit) own_bound=buff?buff->effect.attack_damage:20.;
        const auto pan=events.buff(enemy.team.value_or(""),"pan",previous->tick);
        if(pan) own_bound*=pan->effect.damage_multiplier;
      }
      if(loss<=own_bound+1e-9) continue;
      Clue clue;clue.at=before->position;clue.tick=frame.tick;
      clue.loss=loss-own_bound;clue.dead=*enemy.hp<=0;clue.confidence=clue.dead?.6:.5;
      double known_damage_bound=0;
      for(const auto& attacker:frame.visible_entities) {
        if(attacker.type!="robot"||identity(attacker)==key||attacker.team==frame.self.team||
           (attacker.hp&&*attacker.hp<=0)||attacker.defending.value_or(false)) continue;
        const auto* earlier=lookup(attacker);
        const auto fortress=events.buff(attacker.team.value_or(""),"unrivaled",previous->tick);
        const bool can_hit=earlier&&(fortress
            ? std::max(std::abs(earlier->position.x-before->position.x),
                       std::abs(earlier->position.y-before->position.y))<=fortress->effect.attack_radius
            : intent_clear_shot(earlier->position,before->position,old_bodies));
        if(earlier&&(!earlier->hp||*earlier->hp>0)&&!earlier->spawn_protected.value_or(false)&&
           intent_distance(earlier->position,attacker.position)==0&&can_hit) {
          clue.attackers.push_back(identity(attacker));
          const auto vulnerable=events.buff(enemy.team.value_or(""),"pan",previous->tick);
          known_damage_bound+=(fortress?fortress->effect.attack_damage:20.)*
              (vulnerable?vulnerable->effect.damage_multiplier:1.);
        }
      }
      clue.unattributed=clue.attackers.empty()||clue.loss>known_damage_bound+1e-9;
      clues_[key]=std::move(clue);
    }
    const int count=size_.width*size_.height;mass_.assign(count,0);exposure_.assign(count,0);
    for(auto it=clues_.begin();it!=clues_.end();) {
      const auto& clue=it->second;const int age=static_cast<int>(frame.tick-clue.tick);
      // 明确死亡后同席位重新活着出现，旧死亡现场不再引导本车寻找该敌人。
      bool seen_alive=false;
      for(const auto& e:frame.visible_entities)
        if(e.type=="robot"&&identity(e)==it->first&&e.hp&&*e.hp>0&&age>0&&clue.dead) seen_alive=true;
      if(age>w.combat_clue_max_age||seen_alive) {it=clues_.erase(it);continue;}
      const double confidence=clue.confidence*std::exp2(-age/w.combat_clue_half_life);
      std::vector<int> possible;
      if(clue.unattributed) for(int y=0;y<size_.height;++y) for(int x=0;x<size_.width;++x) {
        const Position p{x,y};
        if(memory.cells()[MapMemory::index(p,size_)]==100||
           (frame.self.alive&&std::abs(x-frame.self.position.x)<=frame.vision_radius&&
            std::abs(y-frame.self.position.y)<=frame.vision_radius)) continue;
        int d=100000;
        for(const Position delta:std::vector<Position>{{0,-1},{0,1},{-1,0},{1,0}})
          for(int range=1;range<=2;++range)
            d=std::min(d,intent_distance(p,{clue.at.x+delta.x*range,clue.at.y+delta.y*range}));
        if(d<=age+1) possible.push_back(y*size_.width+x);
      }
      for(const int cell:possible) mass_[cell]+=confidence/possible.size();
      details.push_back({{"victim",it->first},{"position",{clue.at.x,clue.at.y}},
          {"age",age},{"hp_loss",clue.loss},{"explicit_death",clue.dead},
          {"confidence",confidence},{"known_attackers",clue.attackers},
          {"unattributed",clue.unattributed},{"possible_cells",possible.size()}});
      ++it;
    }
    for(int y=0;y<size_.height;++y) for(int x=0;x<size_.width;++x) {
      const auto probability=std::min(1.,mass_[y*size_.width+x]);
      for(const Position delta:std::vector<Position>{{0,-1},{0,1},{-1,0},{1,0}})
        for(int range=1;range<=2;++range) {
          const int a=x+delta.x*range,b=y+delta.y*range;
          if(a>=0&&b>=0&&a<size_.width&&b<size_.height) exposure_[b*size_.width+a]+=probability;
        }
    }
    for(const auto& [key,origin]:origins_) origins.push_back({{"team",key},
        {"position",{origin.at.x,origin.at.y}},{"came_from",{origin.direction.x,origin.direction.y}},
        {"age",frame.tick-origin.tick},{"confidence",origin.confidence}});
  }

  /** @brief 未知资源预期的软保留比例；只看敌人来路，不改变已确认存在的资源价值。 */
  double supply_factor(Position p,std::int64_t tick,const Stage2Weights& w) const {
    double strongest=0;
    for(const auto& [key,origin]:origins_) {
      (void)key;const int dx=p.x-origin.at.x,dy=p.y-origin.at.y;
      const int along=dx*origin.direction.x+dy*origin.direction.y;
      const int across=std::abs(dx*origin.direction.y-dy*origin.direction.x);
      if(along<0||across>2+along/2) continue;
      strongest=std::max(strongest,w.opening_depletion_weight*origin.confidence*
          std::exp2(-static_cast<double>(tick-origin.tick)/w.opening_depletion_half_life)*
          std::exp(-static_cast<double>(std::abs(dx)+std::abs(dy))/8.));
    }
    return std::clamp(1-strongest,w.opening_depletion_floor,1.);
  }

  /** @brief 仅作软危险，不把推测出的攻击者当成确定存在的第四辆车。 */
  double exposure(Position p) const {
    if(exposure_.empty()||p.x<0||p.y<0||p.x>=size_.width||p.y>=size_.height) return 0;
    return std::min(1.,exposure_[p.y*size_.width+p.x]);
  }

  /** @brief 预计在观察位置可再找到未知攻击者的质量；达到伤害上限后由上层停止奖励。 */
  double in_view(Position p,int radius) const {
    if(mass_.empty()) return 0;
    double value=0;
    for(int y=std::max(0,p.y-radius);y<=std::min(size_.height-1,p.y+radius);++y)
      for(int x=std::max(0,p.x-radius);x<=std::min(size_.width-1,p.x+radius);++x)
        value+=mass_[y*size_.width+x];
    return std::min(1.,value);
  }

  /** @brief 最近几Tick可解释的第三方射击机会；低置信度不能抹掉转火的可能。 */
  double engagement(const std::string& team,std::int64_t tick,int memory_ticks) const {
    double confidence=0;
    for(const auto& [victim,clue]:clues_) {
      (void)victim;const auto age=tick-clue.tick;
      if(age<0||age>memory_ticks) continue;
      if(std::find(clue.attackers.begin(),clue.attackers.end(),team)!=clue.attackers.end())
        confidence=std::max(confidence,.6/std::max<std::size_t>(1,clue.attackers.size())*std::exp2(-age/2.));
    }
    return confidence;
  }

 private:
  struct Clue {Position at;std::int64_t tick{0};double loss{0},confidence{0};
    bool dead{false},unattributed{false};std::vector<std::string> attackers;};
  struct Origin {Position at,direction;std::int64_t tick;double confidence;};
  std::map<std::string,Clue> clues_;
  std::map<std::string,Origin> origins_;
  std::set<std::string> seen_;
  MapSize size_{};std::string session_;std::int64_t tick_{-1};
  std::vector<double> mass_,exposure_;
};
}  // namespace rmtc
