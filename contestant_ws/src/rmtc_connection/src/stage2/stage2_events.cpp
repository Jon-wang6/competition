/**
 * @file stage2_events.cpp
 * @brief 用公开帧推进事件状态，分离基础地图和临时墙，按绝对Tick恢复/到期。
 *
 * @details 模块协作与维护约定
 * 事件解释先经结构校验，再由这里维护Tick有效期、区域和临时状态，不能直接执行模型原文。
 * 临时墙与基础地形分开，事件到期需尊重最新观察；原来未知的格子不能凭恢复操作变成已知空地。
 */
#include "rmtc_connection/stage2_events.hpp"
#include <algorithm>
#include <cmath>

namespace rmtc {
namespace {
struct Body {
  std::string team;
  Position position;
  bool alive;
};
/// 仅提取真正可见的敌人和本车，不用上帝视角补齐其他队伍。
std::vector<Body> bodies(const PublicPerception& frame) {
  std::vector<Body> out{{frame.self.team, frame.self.position, frame.self.alive}};
  for (const auto& entity : frame.visible_entities)
    if (entity.type == "robot" && entity.team && *entity.team != frame.self.team)
      out.push_back({*entity.team, entity.position, !entity.hp || *entity.hp > 0});
  return out;
}
/// 区域/时刻作为稳定事件键，避免LLM给同一公告换ID导致重复授予。
std::string key(const Stage2EventEffect& event) {
  std::string id = event.kind + ":" + event.actor + ":" + std::to_string(event.begin_tick);
  std::vector<std::pair<int, int>> geometry;
  for (const auto p : event.cells)
    geometry.emplace_back(p.x, p.y);
  for (const auto p : event.points)
    geometry.emplace_back(p.x, p.y);
  std::sort(geometry.begin(), geometry.end());
  for (const auto& [x, y] : geometry)
    id += ":" + std::to_string(x) + "," + std::to_string(y);
  return id;
}
}  // namespace

bool Stage2Events::contains(const std::vector<Position>& cells, Position p) {
  return std::any_of(cells.begin(), cells.end(), [&](auto q) { return p.x == q.x && p.y == q.y; });
}

void Stage2Events::reset() {
  history_.clear();
  effects_.clear();
  revisions_.clear();
  buffs_.clear();
  progress_.clear();
  wall_underlay_.clear();
  inferred_walls_.clear();
}

void Stage2Events::observe(const PublicPerception& frame, MapMemory& memory) {
  if (!history_.empty() && history_.back().frame.session_id != frame.session_id)
    reset();
  if (!history_.empty() && frame.tick <= history_.back().frame.tick)
    return;
  const auto* previous = history_.empty() ? nullptr : &history_.back().frame;
  observe_terrain_changes(frame);
  step(frame, previous);
  history_.push_back({frame, memory.cells()});
  if (history_.size() > 1001)
    history_.pop_front();
  terrain(frame, memory);
  history_.back().terrain = memory.cells();
  for (const auto& [position, base] : wall_underlay_)
    history_.back().terrain[MapMemory::index({position.first, position.second}, frame.map)] = base;
  for(const auto& [position,wall]:inferred_walls_)
    history_.back().terrain[MapMemory::index({position.first,position.second},frame.map)]=wall.base;
}

/** @brief 固定底图中确认过的空地新增障碍，先以疑似分裂墙覆盖，保留底图而非写成永久墙。 */
void Stage2Events::observe_terrain_changes(const PublicPerception& frame) {
  if(!frame.self.alive||history_.empty())return;
  for(const auto& entity:frame.visible_entities) {
    if(entity.type!="obstacle")continue;
    const auto p=entity.position;const auto at=std::make_pair(p.x,p.y);
    if(p.x<0||p.y<0||p.x>=frame.map.width||p.y>=frame.map.height||wall_underlay_.count(at))continue;
    const bool confirmed=std::any_of(effects_.begin(),effects_.end(),[&](const auto& item){
      const auto& e=item.second;return e.kind=="world_split"&&e.actor.empty()&&
          e.begin_tick<=frame.tick&&frame.tick<e.end_tick&&contains(e.cells,p);});
    if(confirmed)continue;
    const auto base=history_.back().terrain[MapMemory::index(p,frame.map)];
    if(base==0) {
      auto [it,inserted]=inferred_walls_.try_emplace(at,InferredWall{0,frame.tick,frame.tick});
      (void)inserted;it->second.last_seen=frame.tick;
    }
  }
}

void Stage2Events::merge(const Stage2EventConstraints& incoming, const PublicPerception& frame,
                         MapMemory& memory) {
  bool changed = false;
  for (auto event : incoming.events) {
    const auto id = key(event);
    if (revisions_.count(id) && revisions_.at(id) > incoming.source_tick)
      continue;
    event.id = id;
    effects_[id] = event;
    revisions_[id] = incoming.source_tick;
    changed = true;
  }
  if (changed) {
    buffs_.clear();
    progress_.clear();
    const PublicPerception* previous = nullptr;
    for (const auto& observation : history_) {
      step(observation.frame, previous);
      previous = &observation.frame;
    }
  }
  terrain(frame, memory);
}

void Stage2Events::step(const PublicPerception& frame, const PublicPerception* previous) {
  const auto current = bodies(frame);
  const bool continuous =
      previous && previous->session_id == frame.session_id && previous->tick + 1 == frame.tick;
  const auto prior = continuous ? bodies(*previous) : std::vector<Body>{};
  const auto grant = [&](const Body& body, const Stage2EventEffect& effect, std::int64_t until) {
    auto& state = buffs_[body.team + ":" + effect.kind];
    if (state.until <= until)
      state = {effect, until};
  };
  for (const auto& [id, event] : effects_) {
    if (!event.actor.empty()) {
      if (event.begin_tick <= frame.tick && frame.tick < event.end_tick)
        grant({event.actor, {}, true}, event, event.end_tick);
      continue;
    }
    for (const auto& body : current) {
      const auto old = std::find_if(prior.begin(), prior.end(),
                                    [&](const auto& b) { return b.team == body.team; });
      const bool observed = continuous && old != prior.end();
      const int delta = observed ? std::abs(body.position.x - old->position.x) +
                                       std::abs(body.position.y - old->position.y)
                                 : 0;
      bool moved = observed && old->alive && body.alive && delta == 1;
      if (body.team == frame.self.team)
        moved = moved && frame.self.last_action && frame.self.last_action->tick == frame.tick - 1 &&
                frame.self.last_action->action == "move" && frame.self.last_action->moved &&
                frame.self.position_reason != "respawn";
      // 幻术在激活Tick移动前授予；复活不会清除状态，绝对到期时间也不后移。
      if (event.kind == "illusion" && body.alive && frame.tick == event.begin_tick &&
          contains(event.cells, body.position))
        grant(body, event, event.begin_tick + event.effect_ticks);
      const auto settled = frame.tick - 1;
      if (event.kind == "angel" && moved && event.begin_tick <= settled &&
          settled < event.end_tick && contains(event.points, body.position))
        grant(body, event, settled + event.effect_ticks);
      if (event.kind == "pan" && moved && event.begin_tick <= settled &&
          settled < event.begin_tick + event.movement_window_ticks)
        grant(body, event, event.end_tick);
      if (event.kind == "unrivaled") {
        auto& count = progress_[id + ":" + body.team];
        if (!body.alive || !observed || !old->alive || settled < event.begin_tick ||
            settled >= event.end_tick || !contains(event.cells, body.position))
          count = 0;
        else
          ++count;
        if (count >= event.stay_ticks)
          grant(body, event, event.end_tick);
      }
    }
  }
}

void Stage2Events::terrain(const PublicPerception& frame, MapMemory& memory) {
  std::set<std::pair<int, int>> active_walls;
  for (const auto& [id, event] : effects_) {
    (void)id;
    if (event.kind != "world_split" || !event.actor.empty() || frame.tick < event.begin_tick ||
        frame.tick >= event.end_tick)
      continue;
    for (const auto p : event.cells) {
      const auto at = std::make_pair(p.x, p.y);
      active_walls.insert(at);
      if (!wall_underlay_.count(at)) {
        std::int8_t base = -1;
        // 公告解释可能迟到；取激活前的最后一帧，不把已出现的临时墙误当永久墙。
        for (auto it = history_.rbegin(); it != history_.rend(); ++it)
          if (it->frame.tick < event.begin_tick) {
            base = it->terrain[MapMemory::index(p, frame.map)];
            break;
          }
        wall_underlay_[at] = base;
      }
    }
  }
  for (auto it = wall_underlay_.begin(); it != wall_underlay_.end();) {
    const Position p{it->first.first, it->first.second};
    if (active_walls.count(it->first)) {
      memory.restore_event_terrain(p, 100);
      ++it;
      continue;
    }
    const bool currently_seen = frame.self.alive &&
                                std::abs(p.x - frame.self.position.x) <= frame.vision_radius &&
                                std::abs(p.y - frame.self.position.y) <= frame.vision_radius;
    // 当前视野的真实观测优先；视野外只恢复原值，未知仍是未知。
    if (!currently_seen)
      memory.restore_event_terrain(p, it->second);
    it = wall_underlay_.erase(it);
  }
  for(auto it=inferred_walls_.begin();it!=inferred_walls_.end();) {
    const Position p{it->first.first,it->first.second};
    // 已校验的区域接管此格，不重复计数；底图已保存在掩去临时层的历史中。
    if(active_walls.count(it->first)) {it=inferred_walls_.erase(it);continue;}
    const bool seen=frame.self.alive&&std::abs(p.x-frame.self.position.x)<=frame.vision_radius&&
        std::abs(p.y-frame.self.position.y)<=frame.vision_radius;
    const bool obstacle=seen&&std::any_of(frame.visible_entities.begin(),frame.visible_entities.end(),
        [&](const auto& e){return e.type=="obstacle"&&e.position.x==p.x&&e.position.y==p.y;});
    const bool explained_end=std::any_of(effects_.begin(),effects_.end(),[&](const auto& item){
      const auto& e=item.second;return e.kind=="world_split"&&e.actor.empty()&&
          e.begin_tick<=it->second.first_seen&&e.end_tick<=frame.tick&&contains(e.cells,p);});
    if((seen&&!obstacle)||(!seen&&explained_end)) {
      memory.restore_event_terrain(p,it->second.base);it=inferred_walls_.erase(it);
    } else {
      // 不知道结束Tick就不编造期限，也不把视野外仍未确认恢复的格子直接当空地走。
      memory.restore_event_terrain(p,100);++it;
    }
  }
}

const Stage2EventBuff* Stage2Events::buff(const std::string& team, const std::string& kind,
                                          std::int64_t tick) const {
  const auto it = buffs_.find(team + ":" + kind);
  return it != buffs_.end() && it->second.until > tick ? &it->second : nullptr;
}

const Stage2EventEffect* Stage2Events::active(const std::string& kind, std::int64_t tick) const {
  for (const auto& [id, event] : effects_) {
    (void)id;
    if (event.actor.empty() && event.kind == kind && event.begin_tick <= tick &&
        tick < event.end_tick)
      return &event;
  }
  return nullptr;
}

int Stage2Events::progress(const Stage2EventEffect& event, const std::string& team) const {
  const auto it = progress_.find(event.id + ":" + team);
  return it == progress_.end() ? 0 : it->second;
}

std::int8_t Stage2Events::terrain_at(Position p, std::int64_t tick, const MapMemory& memory) const {
  if (!memory.current() || p.x < 0 || p.y < 0 || p.x >= memory.current()->map.width ||
      p.y >= memory.current()->map.height)
    return 100;
  const auto it = wall_underlay_.find({p.x, p.y});
  std::int8_t base = it == wall_underlay_.end()
                         ? memory.cells()[MapMemory::index(p, memory.current()->map)]
                         : it->second;
  for (const auto& [id, e] : effects_) {
    (void)id;
    if (e.kind == "world_split" && e.actor.empty() && e.begin_tick <= tick && tick < e.end_tick &&
        contains(e.cells, p))
      return 100;
  }
  if(inferred_walls_.count({p.x,p.y}))return 100;
  return base;
}

/// 为RViz返回疑似临时墙的位置快照，不伪造模型已解释的事件范围或倒计时。
std::vector<Position> Stage2Events::inferred_wall_cells() const {
  std::vector<Position> cells;
  for(const auto& [p,wall]:inferred_walls_) {(void)wall;cells.push_back({p.first,p.second});}
  return cells;
}

Json Stage2Events::describe(const PublicPerception& frame) const {
  Json result{{"effects", Json::array()},
              {"buffs", Json::array()},
              {"temporary_wall_cells", wall_underlay_.size()+inferred_walls_.size()},
              {"inferred_world_split_walls",Json::array()}};
  for(const auto& [p,w]:inferred_walls_)
    result["inferred_world_split_walls"].push_back({{"position",{p.first,p.second}},
        {"base_terrain",w.base},{"first_seen_tick",w.first_seen},{"last_seen_tick",w.last_seen},
        {"confirmed_by_ai",false}});
  for (const auto& [id, event] : effects_)
    if (event.end_tick > frame.tick ||
        (event.kind == "illusion" && event.begin_tick + event.effect_ticks > frame.tick))
      result["effects"].push_back(
          {{"id", id},
           {"kind", event.kind},
           {"actor", event.actor},
           {"begin_tick", event.begin_tick},
           {"end_tick", event.end_tick},
           {"remaining_ticks", std::max<std::int64_t>(0, event.end_tick - frame.tick)},
           {"own_stay_progress", progress(event, frame.self.team)}});
  for (const auto& [id, buff] : buffs_)
    if (buff.until > frame.tick)
      result["buffs"].push_back(
          {{"id", id}, {"until", buff.until}, {"remaining_ticks", buff.until - frame.tick}});
  return result;
}
}  // namespace rmtc
