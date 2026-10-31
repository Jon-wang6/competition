/**
 * @file stage2_opponent_intent.hpp
 * @brief 仅从连续公开帧构造三名对手的可能活动范围和第三方交火置信度。
 *
 * 敌方没有公开攻击动作；掉血和站位只是线索。活动范围保留多个落点，
 * 短暂离开视野的敌人仅按衰减记忆参与软风险，不成为确定障碍或确定攻击目标。
 */
#pragma once
#include "rmtc_connection/map_memory.hpp"
#include <algorithm>
#include <cmath>
#include <queue>

namespace rmtc {
struct IntentPosition {
  Position position;
  double probability{0};
};
struct OpponentIntent {
  VisibleEntity entity;
  std::vector<IntentPosition> next;
  double third_party_confidence{0};
  int age{0};
};
struct OpponentIntentModel {
  std::vector<OpponentIntent> opponents;
  std::vector<double> exposure;  ///< 世界坐标从上到下排列；单位为预计可射击本格的敌人数。
  Json details = Json::array();
};
/// 四方向距离，只用于射程、运动推断，不代替有障碍的实际寻路。
inline int intent_distance(Position a, Position b) {
  return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}
/// 关联同一可见机器人；必须有稳定ID或队伍信息，不能凭位置相同认作同一辆。
inline bool same_opponent(const VisibleEntity& a, const VisibleEntity& b) {
  return a.type == "robot" && b.type == "robot" && a.team == b.team && a.id == b.id &&
         (a.id || a.team);
}
/// 标准子弹不被墙阻挡，但最近机器人会挡住更远目标；ignore用于排除被预测目标自身旧坐标。
inline bool intent_clear_shot(Position from, Position target,
                              const std::vector<VisibleEntity>& bodies, int ignore = -1) {
  const int d = intent_distance(from, target);
  if (d < 1 || d > 2 || (from.x != target.x && from.y != target.y))
    return false;
  for (std::size_t i = 0; i < bodies.size(); ++i) {
    if (static_cast<int>(i) == ignore)
      continue;
    const auto p = bodies[i].position;
    const int near = intent_distance(from, p);
    if (near > 0 && near < d && near + intent_distance(p, target) == d)
      return false;
  }
  return true;
}
/**
 * @brief 构造未来一步的分布与近时记忆；只在已知墙上禁行，未知区域表示敌方可能通过。
 * 与上上帧方向一致才加大惯性；掉帧/跳跃/转向削弱外推。第三方交火最多给0.6置信度，
 * 不把未公开动作当事实，不读取裁判私有地图，不把边际得分当已获得的分数。
 */
inline OpponentIntentModel assess_opponent_intents(const PublicPerception& frame,
                                                   const std::optional<PublicPerception>& previous,
                                                   const std::optional<PublicPerception>& older,
                                                   const MapMemory& memory,
                                                   const std::vector<const VisibleEntity*>& visible,
                                                   int memory_ticks, double heading_weight) {
  OpponentIntentModel model;
  const int width = frame.map.width, height = frame.map.height;
  model.exposure.assign(static_cast<std::size_t>(width * height), 0);
  const auto valid = [&](Position p) {
    return p.x >= 0 && p.y >= 0 && p.x < width && p.y < height &&
           memory.cells()[MapMemory::index(p, frame.map)] != 100;
  };
  const auto lookup = [](const PublicPerception& f,
                         const VisibleEntity& e) -> const VisibleEntity* {
    for (const auto& x : f.visible_entities)
      if (same_opponent(x, e))
        return &x;
    return nullptr;
  };
  std::vector<VisibleEntity> bodies;
  for (const auto* e : visible)
    bodies.push_back(*e);
  const bool continuous =
      previous && previous->session_id == frame.session_id && previous->tick + 1 == frame.tick;
  std::vector<VisibleEntity> old_bodies;
  if (continuous) {
    for (const auto& e : previous->visible_entities)
      if (e.type == "robot")
        old_bodies.push_back(e);
    VisibleEntity self;
    self.type = "robot";
    self.team = previous->self.team;
    self.position = previous->self.position;
    old_bodies.push_back(self);
  }
  for (std::size_t index = 0; index < visible.size(); ++index) {
    const auto& e = *visible[index];
    OpponentIntent intent;
    intent.entity = e;
    const auto* old = continuous ? lookup(*previous, e) : nullptr;
    Position velocity{0, 0};
    double persistence = 0;
    if (old && (!old->hp || *old->hp > 0)) {
      velocity = {e.position.x - old->position.x, e.position.y - old->position.y};
      if (intent_distance(e.position, old->position) == 1) {
        persistence = 0.25;
        if (older && older->session_id == frame.session_id && older->tick + 1 == previous->tick) {
          const auto* last = lookup(*older, e);
          if (last && old->position.x - last->position.x == velocity.x &&
              old->position.y - last->position.y == velocity.y)
            persistence = 0.75;
        }
      }
      // 公告是累计历史，只有新增/变化帧暂停推断；标准伤害增量仍只是交火线索。
      // 只在候选攻击者未移动、未防御且此前不受保护时分配交火线索。
      if (frame.event_text == previous->event_text &&
          intent_distance(e.position, old->position) == 0 && !e.defending.value_or(false) &&
          !old->spawn_protected.value_or(false)) {
        for (const auto* victim : visible) {
          if (victim == visible[index] || !victim->hp || *victim->hp <= 0)
            continue;
          const auto* before = lookup(*previous, *victim);
          if (!before || !before->hp)
            continue;
          const double loss = *before->hp - *victim->hp;
          if (loss <= 0 || loss > 40 || std::abs(loss / 10.0 - std::round(loss / 10.0)) > 1e-6 ||
              !intent_clear_shot(old->position, before->position, old_bodies))
            continue;
          // 本车上一Tick也可能打了它：无法排除时不把掉血认作第三方攻击证据。
          if (frame.self.last_action && frame.self.last_action->action == "attack" &&
              intent_clear_shot(previous->self.position, before->position, old_bodies) &&
              loss <= 20)
            continue;  // 大于20的标准掉血，无法仅由本车一次普通攻击解释。
          int suspects = 0;
          for (const auto* other : visible) {
            if (other == victim)
              continue;
            const auto* before_other = lookup(*previous, *other);
            if (before_other && intent_distance(other->position, before_other->position) == 0 &&
                !other->defending.value_or(false) &&
                !before_other->spawn_protected.value_or(false) &&
                intent_clear_shot(before_other->position, before->position, old_bodies))
              ++suspects;
          }
          intent.third_party_confidence =
              std::max(intent.third_party_confidence, 0.6 / std::max(1, suspects));
        }
      }
    }
    double total = 0;
    for (const Position delta : std::vector<Position>{{0, 0}, {0, -1}, {0, 1}, {-1, 0}, {1, 0}}) {
      Position next{e.position.x + delta.x, e.position.y + delta.y};
      if (!valid(next))
        continue;
      double mass = (delta.x == 0 && delta.y == 0) ? 2.0 : 1.0;
      if (persistence > 0 && delta.x == velocity.x && delta.y == velocity.y)
        mass += heading_weight * persistence;
      intent.next.push_back({next, mass});
      total += mass;
    }
    for (auto& p : intent.next)
      p.probability /= total;
    model.opponents.push_back(std::move(intent));
  }
  // 视野外近时记忆扩散为多个可能落点，不将其旧位置硬封死。
  for (const auto& [key, remembered] : memory.entities()) {
    (void)key;
    const auto& e = remembered.entity;
    const int age = static_cast<int>(frame.tick - remembered.last_seen_tick);
    if (e.type != "robot" || !e.team || *e.team == frame.self.team || (e.hp && *e.hp <= 0) ||
        age <= 0 || age > memory_ticks ||
        std::any_of(visible.begin(), visible.end(),
                    [&](const auto* x) { return same_opponent(*x, e); }))
      continue;
    OpponentIntent intent;
    intent.entity = e;
    intent.age = age;
    std::queue<std::pair<Position, int>> pending;
    std::vector<bool> seen(static_cast<std::size_t>(width * height), false);
    if (!valid(e.position))
      continue;
    pending.push({e.position, 0});
    seen[e.position.y * width + e.position.x] = true;
    while (!pending.empty()) {
      const auto [at, d] = pending.front();
      pending.pop();
      intent.next.push_back({at, 0});
      if (d >= age + 1)
        continue;
      for (const Position delta : std::vector<Position>{{0, -1}, {0, 1}, {-1, 0}, {1, 0}}) {
        Position next{at.x + delta.x, at.y + delta.y};
        if (!valid(next) || seen[next.y * width + next.x])
          continue;
        seen[next.y * width + next.x] = true;
        pending.push({next, d + 1});
      }
    }
    for (auto& p : intent.next)
      p.probability = std::pow(0.5, age) / intent.next.size();
    model.opponents.push_back(std::move(intent));
  }
  for (const auto& intent : model.opponents) {
    Json cells = Json::array();
    for (const auto& p : intent.next) {
      cells.push_back({{"position", {p.position.x, p.position.y}}, {"mass", p.probability}});
      // 下一步身体遮挡不确定：软范围采用保守射线，当前实际攻击另按精确挡弹判定。
      for (const Position delta : std::vector<Position>{{0, -1}, {0, 1}, {-1, 0}, {1, 0}})
        for (int d = 1; d <= 2; ++d) {
          Position at{p.position.x + delta.x * d, p.position.y + delta.y * d};
          if (at.x >= 0 && at.y >= 0 && at.x < width && at.y < height)
            model.exposure[at.y * width + at.x] += p.probability;
        }
    }
    model.details.push_back(
        {{"id", intent.entity.id.value_or(intent.entity.team.value_or("enemy"))},
         {"age", intent.age},
         {"third_party_confidence", intent.third_party_confidence},
         {"next_positions", cells}});
  }
  return model;
}
/// 对某个可见敌人计算下一Tick进入射程的质量；其他车按当前身体作保守挡弹检查。
inline double intent_hit_probability(Position from, std::size_t target,
                                     const OpponentIntentModel& model,
                                     const std::vector<VisibleEntity>& bodies) {
  double p = 0;
  for (const auto& next : model.opponents[target].next)
    if (intent_clear_shot(from, next.position, bodies, static_cast<int>(target)))
      p += next.probability;
  return p;
}
}  // namespace rmtc
