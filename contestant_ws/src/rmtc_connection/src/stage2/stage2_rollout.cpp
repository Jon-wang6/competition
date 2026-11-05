/**
 * @file stage2_rollout.cpp
 * @brief 多情景、共同动作序列的局部束搜索；按真实结算顺序推演并按预算逐层完成。
 *
 * @details 模块协作与维护约定
 * 推演输入是当前公开局面和受校验事件，按结算顺序构造多个可能未来，而非读取真实未来。
 * 根动作必须在可比较的预算/深度下评估；扩大搜索前先检查预测误差与决策耗时尾部。
 */
#include "rmtc_connection/stage2_rollout.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>

namespace rmtc {
void RolloutWeights::validate() const {
  for (double v :
       {weight, budget_ms, discount, worst_fraction, goal_progress, override_margin, risk_scale})
    if (!std::isfinite(v) || v < 0)
      throw std::invalid_argument("invalid rollout weights");
  if (weight > 3 || horizon < 2 || horizon > 7 || beam_width < 2 || beam_width > 16 ||
      scenarios < 2 || scenarios > 16 || budget_ms < 1 || budget_ms > 150 || discount > 1 ||
      worst_fraction > 1 || activation_range < 1 || activation_range > 12 || start_tick < 0 ||
      start_tick > 5000 || risk_scale > 2 || max_transitions<0 || max_transitions>500000)
    throw std::invalid_argument("rollout budget/depth/probability outside bounds");
}
namespace {
constexpr std::array<Position, 4> steps{{{0, -1}, {0, 1}, {-1, 0}, {1, 0}}};
constexpr std::array<const char*, 4> directions{{"up", "down", "left", "right"}};
using Clock = std::chrono::steady_clock;
/// 小整数动作避免在搜索树内反复构造字符串；移动方向代表已补偿幻术后的实际方向。
int code(const Action& a) {
  if (a.type == "defend")
    return 1;
  for (int i = 0; i < 4; ++i)
    if (a.direction == directions[i]) {
      if (a.type == "move")
        return 2 + i;
      if (a.type == "attack")
        return 6 + i;
    }
  return 0;
}
/// 仅在输出最终根动作时还原公开动作类型，不自行发布ROS。
Action action(int a) {
  return a == 0   ? Action::wait()
         : a == 1 ? Action{"defend", ""}
                  : Action{a < 6 ? "move" : "attack", directions[(a - 2) % 4]};
}
int manhattan(Position a, Position b) {
  return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}
bool same(Position a, Position b) {
  return a.x == b.x && a.y == b.y;
}
struct Robot {
  Position p, heading;
  double hp{100};
  bool alive{true}, last_defending{false};
  int protection{0};
  std::int64_t aoe_until{0}, heal_until{0}, pan_until{0};
  double aoe_damage{0}, aoe_defended{0}, heal{0}, hp_cap{100}, pan_multiplier{1};
  int radius{0};
  int fortress_progress{0};
};
struct Resource {
  Position p;
  double value, confidence;
  bool high;
};
struct State {
  std::array<Robot, 4> robots;
  std::uint64_t taken{0}, missions{0};
  double damage{0}, resource{0}, future{0};
  int normal{0}, high{0}, anchor{-1};
};
struct Node {
  std::array<State, 16> states;
  double value{0};
};

/// 一次决策的只读上下文；地图与事件均由合法感知/模型结果注入。
struct World {
  const PublicPerception& frame;
  const MapMemory& memory;
  const Stage2Weights& w;
  const RolloutWeights& config;
  const std::vector<int>& goal_distances;
  const Stage2Events& events;
  double goal_utility;
  std::vector<Resource> resources;
  std::vector<const Stage2EventEffect*> effects;
  int robot_count{1};
  bool inside(Position p) const {
    return p.x >= 0 && p.y >= 0 && p.x < frame.map.width && p.y < frame.map.height;
  }
  int distance_to_goal(Position p) const {
    return inside(p) && goal_distances.size() == memory.cells().size()
               ? goal_distances[p.y * frame.map.width + p.x]
               : -1;
  }
  /// 规划不会穿越本车未知格；敌方进入未知地形只是一种情景，不能用于本车路线。
  bool terrain(Position p, std::int64_t tick, bool own) const {
    if (!inside(p))
      return false;
    const int cell = events.terrain_at(p, tick, memory);
    if (cell == 100 || (own && cell != 0))
      return false;
    for (const auto* e : effects)
      if (e->kind == "world_split" && e->actor.empty() && e->begin_tick <= tick &&
          tick < e->end_tick && Stage2Events::contains(e->cells, p))
        return false;
    return true;
  }
  /// 普通射击受机器人挡弹影响但不受墙遮挡；无双是方形群攻。
  bool hits(const State& s, int from, int to, std::int64_t tick, int direction = -1) const {
    const auto& a = s.robots[from];
    const auto& b = s.robots[to];
    if (!a.alive || !b.alive || from == to || same(a.p, b.p))
      return false;
    if (tick < a.aoe_until)
      return std::max(std::abs(a.p.x - b.p.x), std::abs(a.p.y - b.p.y)) <= a.radius;
    const int d = manhattan(a.p, b.p);
    if (d < 1 || d > 2 || (a.p.x != b.p.x && a.p.y != b.p.y))
      return false;
    if (direction >= 0 &&
        (b.p.x != a.p.x + steps[direction].x * d || b.p.y != a.p.y + steps[direction].y * d))
      return false;
    for (int i = 0; i < robot_count; ++i)
      if (i != from && i != to && s.robots[i].alive && manhattan(a.p, s.robots[i].p) < d &&
          manhattan(a.p, s.robots[i].p) + manhattan(s.robots[i].p, b.p) == d)
        return false;
    return true;
  }
  /// 多种公开可解释的对手倾向：抢资源、追击、延续移动、防御、第三方交战、原地交火。
  int opponent(const State& s, int who, int scenario, std::int64_t tick) const {
    const auto& r = s.robots[who];
    if (!r.alive)
      return 0;
    const int mode = config.independent_opponents ? (scenario + who*2) % 6 :
        (scenario < 6 ? scenario : (scenario + who * 2) % 6);
    int victim = -1;
    double best = -1e9;
    int threats = 0;
    for (int i = 0; i < robot_count; ++i)
      if (i != who && s.robots[i].alive) {
        threats += hits(s, i, who, tick) && s.robots[i].protection == 0;
        if (r.protection == 0 && s.robots[i].protection == 0 && hits(s, who, i, tick)) {
          const double value =
              100 - s.robots[i].hp - manhattan(r.p, s.robots[i].p) + (mode == 4 && i != 0 ? 50 : 0);
          if (value > best) {
            best = value;
            victim = i;
          }
        }
      }
    if (threats > 0 &&
        ((mode == 2 && (r.hp <= 40 || r.last_defending)) || (mode == 4 && threats >= 2)))
      return 1;
    Position target = r.p;
    int nearest = 100000;
    if (mode == 1)
      for (std::size_t i = 0; i < resources.size(); ++i)
        if (!(s.taken & (std::uint64_t{1} << i))) {
          int d = manhattan(r.p, resources[i].p);
          if (d < nearest) {
            nearest = d;
            target = resources[i].p;
          }
        }
    if (victim >= 0 && !(mode == 1 && nearest <= 2) &&
        !(mode == 3 && (r.heading.x || r.heading.y))) {
      if (tick < r.aoe_until)
        return 6;
      const auto q = s.robots[victim].p;
      return 6 + (q.y < r.p.y ? 0 : q.y > r.p.y ? 1 : q.x < r.p.x ? 2 : 3);
    }
    if (mode == 5)
      return r.last_defending && threats > 0 ? 1 : 0;
    if (nearest == 100000 || nearest > 4) {
      nearest = 100000;
      for (int i = 0; i < robot_count; ++i)
        if (i != who && s.robots[i].alive && !(mode == 4 && i == 0 && robot_count > 2)) {
          const int d = manhattan(r.p, s.robots[i].p);
          if (d < nearest) {
            nearest = d;
            target = s.robots[i].p;
          }
        }
    }
    int choice = 0;
    double gain = -1e9;
    for (int i = 0; i < 4; ++i) {
      Position p{r.p.x + steps[i].x, r.p.y + steps[i].y};
      if (!terrain(p, tick, false))
        continue;
      bool occupied = false;
      for (int j = 0; j < robot_count; ++j)
        if (j != who && s.robots[j].alive && same(p, s.robots[j].p))
          occupied = true;
      if (occupied)
        continue;
      double value = -manhattan(p, target);
      if (mode == 3 && steps[i].x == r.heading.x && steps[i].y == r.heading.y)
        value += 4;
      if (value > gain) {
        gain = value;
        choice = 2 + i;
      }
    }
    return choice;
  }
  /// 一次真实顺序的局部结算；所有动作先固定，战斗同时扣血，争格统一失败。
  double advance(State& s, int own_action, int scenario, std::int64_t tick, bool& own_moved,
                 const std::array<int, 4>* fixed_commands = nullptr) const {
    const double old_hp = s.robots[0].hp, old_damage = s.damage, old_resource = s.resource;
    const bool old_alive = s.robots[0].alive;
    const auto old_missions = s.missions;
    std::array<int, 4> commands{};
    std::array<Position, 4> origins{};
    // 激活事件先于本Tick动作，未来世界分裂后的复活位置不属于已知信息，按离场保守处理。
    for (const auto* e : effects)
      if (e->actor.empty() && e->begin_tick == tick &&
          (e->kind == "explosion" || e->kind == "world_split")) {
        for (int i = 0; i < robot_count; ++i) {
          auto& r = s.robots[i];
          if (!r.alive || !Stage2Events::contains(e->cells, r.p))
            continue;
          if (e->kind == "world_split")
            r.hp = 0;
          else if (r.protection == 0)
            r.hp = std::max(0.0, r.hp - e->damage);
          r.alive = r.hp > 0;
        }
      }
    commands[0] = s.robots[0].alive ? own_action : 0;
    for (int i = 1; i < robot_count; ++i)
      commands[i] = opponent(s, i, scenario, tick);
    if (fixed_commands)
      commands = *fixed_commands;
    const Stage2EventEffect* purity = nullptr;
    for (const auto* e : effects)
      if (e->actor.empty() && e->kind == "purity" && e->begin_tick <= tick && tick < e->end_tick)
        purity = e;
    std::array<std::array<double, 4>, 4> damage{};
    std::array<double, 4> reflected{};
    for (int a = 0; a < robot_count; ++a) {
      if (commands[a] < 6 || !s.robots[a].alive || s.robots[a].protection > 0)
        continue;
      for (int b = 0; b < robot_count; ++b)
        if (hits(s, a, b, tick, commands[a] - 6) && s.robots[b].protection == 0) {
          if (purity && commands[b] == 1) {
            reflected[a] += purity->reflection_damage;
            continue;
          }
          const auto& r = s.robots[a];
          const auto& target = s.robots[b];
          double hit = tick < r.aoe_until ? (commands[b] == 1 ? r.aoe_defended : r.aoe_damage)
                                          : (commands[b] == 1 ? 10 : 20);
          damage[a][b] = hit * (tick < target.pan_until ? target.pan_multiplier : 1.0);
        }
    }
    double credited = 0;
    for (int b = 0; b < robot_count; ++b) {
      auto& r = s.robots[b];
      if (!r.alive)
        continue;
      double total = 0;
      for (int a = 0; a < robot_count; ++a)
        total += damage[a][b];
      if (b != 0 && total > 0 && damage[0][b] > 0) {
        credited += std::min(r.hp, total) * damage[0][b] / total;
        if (r.hp <= total + reflected[b])
          credited += w.kill_reward_estimate * damage[0][b] / total;
      }
      r.hp = std::max(0.0, r.hp - total - reflected[b]);
      r.alive = r.hp > 0;
    }
    s.damage += credited;
    std::array<Position, 4> destinations{};
    std::array<bool, 4> move{};
    for (int i = 0; i < robot_count; ++i) {
      auto& r = s.robots[i];
      origins[i] = r.p;
      destinations[i] = r.p;
      if (!r.alive || commands[i] < 2 || commands[i] >= 6)
        continue;
      const auto step = steps[commands[i] - 2];
      Position next{r.p.x + step.x, r.p.y + step.y};
      if (!terrain(next, tick, i == 0))
        continue;
      bool occupied = false;
      for (int j = 0; j < robot_count; ++j)
        if (j != i && s.robots[j].alive && same(next, s.robots[j].p))
          occupied = true;
      if (!occupied) {
        move[i] = true;
        destinations[i] = next;
      }
    }
    for (int i = 0; i < robot_count; ++i)
      for (int j = i + 1; j < robot_count; ++j)
        if (move[i] && move[j] && same(destinations[i], destinations[j])) {
          move[i] = false;
          move[j] = false;
        }
    // 三车以上争同一点时也必须全失败，不能由前两个失败后让第三个通过。
    for (int i = 0; i < robot_count; ++i)
      if (commands[i] >= 2 && commands[i] < 6) {
        int count = 0;
        for (int j = 0; j < robot_count; ++j)
          if (s.robots[j].alive && !same(destinations[j], origins[j]) &&
              same(destinations[i], destinations[j]))
            ++count;
        if (count > 1)
          move[i] = false;
      }
    for (int i = 0; i < robot_count; ++i) {
      auto& r = s.robots[i];
      if (move[i]) {
        r.heading = {destinations[i].x - r.p.x, destinations[i].y - r.p.y};
        r.p = destinations[i];
        r.protection = 0;
      } else
        r.heading = {0, 0};
      r.last_defending = commands[i] == 1;
      for (const auto* e : effects)
        if (e->actor.empty() && e->begin_tick <= tick && tick < e->end_tick && r.alive) {
          if (move[i] && e->kind == "angel" && Stage2Events::contains(e->points, r.p)) {
            r.heal = e->heal_per_tick;
            r.heal_until = tick + e->effect_ticks;
            r.hp_cap = e->hp_cap;
          }
          if (move[i] && e->kind == "pan" && tick < e->begin_tick + e->movement_window_ticks) {
            r.pan_until = e->end_tick;
            r.pan_multiplier = e->damage_multiplier;
          }
          if (e->kind == "unrivaled") {
            r.fortress_progress =
                Stage2Events::contains(e->cells, r.p) ? r.fortress_progress + 1 : 0;
            if (r.fortress_progress >= e->stay_ticks) {
              r.aoe_until = e->end_tick;
              r.radius = e->attack_radius;
              r.aoe_damage = e->attack_damage;
              r.aoe_defended = e->defended_damage;
            }
          }
        }
    }
    std::array<double, 4> environmental{};
    for (const auto* e : effects)
      if (e->actor.empty() && e->kind == "pepper" && e->begin_tick <= tick && tick < e->end_tick)
        for (int i = 0; i < robot_count; ++i)
          if (move[i] && s.robots[i].alive && Stage2Events::contains(e->points, s.robots[i].p))
            for (int j = 0; j < robot_count; ++j)
              if (s.robots[j].alive && s.robots[j].protection == 0 &&
                  !same(s.robots[j].p, s.robots[i].p) &&
                  (s.robots[j].p.x == s.robots[i].p.x || s.robots[j].p.y == s.robots[i].p.y))
                environmental[j] += e->damage;
    for (int i = 0; i < robot_count; ++i) {
      auto& r = s.robots[i];
      r.hp = std::max(0.0, r.hp - environmental[i]);
      r.alive = r.hp > 0;
      if (r.alive && tick < r.heal_until)
        r.hp = std::min(r.hp_cap, r.hp + r.heal);
      if (r.protection > 0)
        --r.protection;
    }
    for (std::size_t j = 0; j < resources.size(); ++j)
      if (!(s.taken & (std::uint64_t{1} << j)))
        for (int i = 0; i < robot_count; ++i)
          if (s.robots[i].alive && same(s.robots[i].p, resources[j].p)) {
            s.taken |= std::uint64_t{1} << j;
            if (i == 0) {
              s.resource += resources[j].value;
              if (resources[j].high)
                ++s.high;
              else
                ++s.normal;
            }
            break;
          }
    double mission_reward = 0;
    for (std::size_t j = 0; j < frame.missions.size() && j < 64; ++j)
      if (!(s.missions & (std::uint64_t{1} << j)) && s.robots[0].alive) {
        const auto& m = frame.missions[j];
        bool complete = false;
        if (m.type == "reach" && m.position)
          complete = same(s.robots[0].p, *m.position);
        if (m.target) {
          if (m.type == "resource_value")
            complete = s.resource >= *m.target;
          if (m.type == "normal_resources")
            complete = s.normal >= *m.target;
          if (m.type == "high_resources")
            complete = s.high >= *m.target;
          if (m.type == "effective_damage")
            complete = s.damage >= *m.target;
        }
        if (complete) {
          s.missions |= std::uint64_t{1} << j;
          if(config.collection_rewards || (m.type!="resource_value"&&m.type!="normal_resources"&&m.type!="high_resources"))
            mission_reward += w.mission_points * w.mission_potential;
        }
      }
    const bool died = old_alive && !s.robots[0].alive;
    if (died)
      s.damage = std::max(0.0, s.damage - 60.0);
    const auto capped = [](double x, double cap) { return std::clamp(x, 0.0, cap); };
    own_moved = move[0] && s.robots[0].alive;
    (void)old_missions;
    return (capped(s.damage, w.damage_cap) - capped(old_damage, w.damage_cap)) * w.damage_points /
               w.damage_cap +
           (config.collection_rewards?1.0:0.0)*(capped(s.resource, w.resource_cap) - capped(old_resource, w.resource_cap)) *
               w.resource_points / w.resource_cap +
           mission_reward -
           config.risk_scale * w.hp_loss_cost * std::max(0.0, old_hp - s.robots[0].hp) +
           (s.robots[0].hp > old_hp ? config.risk_scale * w.hp_loss_cost * (s.robots[0].hp - old_hp)
                                    : 0) -
           (died ? config.risk_scale * w.death_cost : 0);
  }
  /// 所有情景共用同一未来动作序列；不会为不同敌人未来偷偷选择不同的本车动作。
  double value(const Node& node) const {
    std::array<double, 16> values{};
    double sum = 0;
    for (int i = 0; i < config.scenarios; ++i) {
      const auto& s = node.states[i];
      double v = s.future;
      const int d = distance_to_goal(s.robots[0].p);
      if (s.robots[0].alive && d >= 0 && s.anchor >= 0)
        v += config.goal_progress * std::max(0.0, goal_utility) * (s.anchor - d);
      values[i] = v;
      sum += v;
    }
    std::sort(values.begin(), values.begin() + config.scenarios);
    const int count = std::max(1, config.scenarios / 4);
    double tail = 0;
    for (int i = 0; i < count; ++i)
      tail += values[i];
    return (1 - config.worst_fraction) * sum / config.scenarios +
           config.worst_fraction * tail / count;
  }
};
}  // namespace

RolloutResult tactical_rollout(const PublicPerception& frame, const MapMemory& memory,
                               const Stage2Events& events,
                               const std::optional<PublicPerception>& previous,
                               const Stage2Weights& weights, const RolloutWeights& config,
                               const std::vector<RolloutRoot>& raw_roots, Action original,
                               const std::vector<int>& goal_distances, double goal_utility,
                               int normal_count, int high_count, int remaining_ticks) {
  RolloutResult result;
  result.action = original;
  if (config.weight <= 0 || frame.tick < config.start_tick || !frame.self.alive ||
      remaining_ticks < 2)
    return result;
  const auto started = Clock::now();
  World world{frame, memory, weights, config, goal_distances, events, goal_utility, {}, {}, 1};
  State initial;
  initial.robots[0].p = frame.self.position;
  initial.robots[0].hp = frame.self.hp;
  initial.robots[0].protection = frame.self.spawn_protected ? 1 : 0;
  initial.damage = frame.score.effective_damage;
  initial.resource = frame.score.resource_value;
  initial.normal = normal_count;
  initial.high = high_count;
  std::array<std::string, 4> teams{{frame.self.team, "", "", ""}};
  int nearest = 1000;
  for (const auto& e : frame.visible_entities)
    if (e.type == "robot" && e.team && *e.team != frame.self.team && (!e.hp || *e.hp > 0) &&
        world.robot_count < 4) {
      const int i = world.robot_count++;
      auto& r = initial.robots[i];
      teams[i] = *e.team;
      r.p = e.position;
      r.hp = e.hp.value_or(100);
      r.protection = e.spawn_protected.value_or(false) ? 1 : 0;
      r.last_defending = e.defending.value_or(false);
      nearest = std::min(nearest, manhattan(frame.self.position, e.position));
      if (previous && previous->tick + 1 == frame.tick && previous->session_id == frame.session_id)
        for (const auto& old : previous->visible_entities)
          if (old.type == "robot" && old.team == e.team && manhattan(old.position, e.position) <= 1)
            r.heading = {e.position.x - old.position.x, e.position.y - old.position.y};
    }
  if (nearest > config.activation_range)
    return result;
  for (const auto& [id, e] : events.effects()) {
    (void)id;
    if (e.end_tick > frame.tick ||
        (e.kind == "illusion" && e.begin_tick + e.effect_ticks > frame.tick))
      world.effects.push_back(&e);
  }
  for (int i = 0; i < world.robot_count; ++i) {
    auto& r = initial.robots[i];
    if (const auto b = events.buff(teams[i], "unrivaled", frame.tick)) {
      r.aoe_until = b->until;
      r.radius = b->effect.attack_radius;
      r.aoe_damage = b->effect.attack_damage;
      r.aoe_defended = b->effect.defended_damage;
    }
    if (const auto b = events.buff(teams[i], "angel", frame.tick)) {
      r.heal_until = b->until;
      r.heal = b->effect.heal_per_tick;
      r.hp_cap = b->effect.hp_cap;
    }
    if (const auto b = events.buff(teams[i], "pan", frame.tick)) {
      r.pan_until = b->until;
      r.pan_multiplier = b->effect.damage_multiplier;
    }
    for (const auto* e : world.effects)
      if (e->kind == "unrivaled" && e->actor.empty())
        r.fortress_progress = events.progress(*e, teams[i]);
  }
  for (std::size_t i = 0; i < frame.missions.size() && i < 64; ++i)
    if (std::find(frame.score.mission_completed_ids.begin(),
                  frame.score.mission_completed_ids.end(),
                  frame.missions[i].id) != frame.score.mission_completed_ids.end())
      initial.missions |= std::uint64_t{1} << i;
  for (const auto& [id, m] : memory.entities()) {
    (void)id;
    const auto& e = m.entity;
    if (world.resources.size() < 64 && !m.first_missing_tick && e.value && *e.value > 0 &&
        (e.type == "resource_normal" || e.type == "resource_high"))
      world.resources.push_back(
          {e.position, *e.value,
           1.0 / (1.0 + (frame.tick - m.last_seen_tick) / weights.resource_age_ticks),
           e.type == "resource_high"});
  }
  std::map<int, double> roots;
  for (const auto& r : raw_roots) {
    const int c = code(r.action);
    auto it = roots.find(c);
    if (it == roots.end() || r.utility > it->second)
      roots[c] = r.utility;
  }
  if (roots.size() < 2 || !roots.count(code(original)))
    return result;
  result.ran = true;
  std::map<int, std::vector<Node>> beams;
  std::map<int, double> completed;
  for (const auto& [c, base] : roots) {
    Node node;
    for (int scenario = 0; scenario < config.scenarios; ++scenario) {
      auto& state = node.states[scenario];
      state = initial;
      for (std::size_t j = 0; j < world.resources.size(); ++j) {
        const std::uint32_t hash =
            static_cast<std::uint32_t>((j + 11) * 2654435761u + (scenario + 3) * 2246822519u);
        if ((hash % 65536) / 65536.0 > world.resources[j].confidence)
          state.taken |= std::uint64_t{1} << j;
      }
      bool moved = false;
      world.advance(state, c, scenario, frame.tick, moved);
      ++result.transitions;
      if (c >= 2 && c < 6 && !moved)
        state.future = -std::max(0.0, base);
      state.anchor = world.distance_to_goal(state.robots[0].p);
    }
    node.value = world.value(node);
    beams[c].push_back(std::move(node));
    completed[c] = beams[c][0].value;
  }
  result.completed_depth = 1;
  const int horizon = std::min(config.horizon, remaining_ticks);
  for (int depth = 1; depth < horizon; ++depth) {
    std::map<int, std::vector<Node>> next_beams;
    bool timeout = false;
    for (const auto& [root, beam] : beams) {
      std::vector<Node> candidates;
      candidates.reserve(beam.size() * 10);
      for (const auto& parent : beam) {
        if (timeout)
          break;
        for (int c = 0; c < 10; ++c) {
          // 不对全部情景均无合法落点的移动展开，减少撞墙和未知格产生的无意义分支。
          if (c >= 2 && c < 6) {
            bool possible = false;
            for (int scenario = 0; scenario < config.scenarios; ++scenario) {
              const auto& r = parent.states[scenario].robots[0];
              const auto d = steps[c - 2];
              possible = possible || (r.alive && world.terrain({r.p.x + d.x, r.p.y + d.y},
                                                               frame.tick + depth, true));
            }
            if (!possible)
              continue;
          }
          if (c >= 6) {
            bool target = false;
            for (int scenario = 0; scenario < config.scenarios; ++scenario) {
              const auto& state = parent.states[scenario];
              for (int enemy = 1; enemy < world.robot_count; ++enemy)
                target = target ||
                         (state.robots[0].protection == 0 && state.robots[enemy].protection == 0 &&
                          world.hits(state, 0, enemy, frame.tick + depth, c - 6) &&
                          (c == 6 || frame.tick + depth >= state.robots[0].aoe_until));
            }
            if (!target)
              continue;
          }
          Node next = parent;
          for (int scenario = 0; scenario < config.scenarios; ++scenario) {
            bool moved = false;
            const double reward =
                world.advance(next.states[scenario], c, scenario, frame.tick + depth, moved);
            next.states[scenario].future += std::pow(config.discount, depth) * reward;
            ++result.transitions;
          }
          next.value = world.value(next);
          candidates.push_back(std::move(next));
          if ((config.max_transitions>0&&result.transitions>=static_cast<std::size_t>(config.max_transitions)) ||
              ((result.transitions & 63) < static_cast<std::size_t>(config.scenarios) &&
              std::chrono::duration<double, std::milli>(Clock::now() - started).count() >
                  config.budget_ms)) {
            timeout = true;
            break;
          }
        }
      }
      if (timeout)
        break;
      std::stable_sort(candidates.begin(), candidates.end(),
                       [](const Node& a, const Node& b) { return a.value > b.value; });
      if (candidates.size() > static_cast<std::size_t>(config.beam_width))
        candidates.resize(config.beam_width);
      next_beams[root] = std::move(candidates);
    }
    if (timeout) {
      result.budget_hit = true;
      break;
    }
    beams = std::move(next_beams);
    ++result.completed_depth;
    for (const auto& [root, beam] : beams)
      completed[root] = beam.front().value;
  }
  int selected = code(original);
  double best = roots[selected] + config.weight * completed[selected] + config.override_margin;
  result.utility = best - config.override_margin;
  if (result.completed_depth >= 2)
    for (const auto& [root, base] : roots) {
      const double utility = base + config.weight * completed[root];
      result.roots.push_back({{"action", action(root).type},
                              {"direction", action(root).direction},
                              {"base", base},
                              {"future", completed[root]},
                              {"combined", utility}});
      if (utility > best + 1e-9) {
        best = utility;
        selected = root;
        result.utility = utility;
      }
    }
  result.action = action(selected);
  result.changed = selected != code(original);
  result.elapsed_ms = std::chrono::duration<double, std::milli>(Clock::now() - started).count();
  return result;
}

#ifdef RMTC_ROLLOUT_TESTING
// 仅测试目标编译此入口，以固定双方动作验证物理顺序；正式库不提供/安装该测试接口。
Json rollout_test_step(const PublicPerception& frame,
                       const std::vector<Stage2EventEffect>& specifications,
                       const std::vector<Action>& actions,bool collection_rewards) {
  MapMemory memory;
  memory.update(frame);
  Stage2Events events;
  events.observe(frame, memory);
  Stage2EventConstraints c;
  c.session_id = frame.session_id;
  c.source_text = frame.event_text;
  c.source_tick = frame.tick;
  c.begin_tick = 0;
  c.end_tick = 10000;
  c.timeline = true;
  c.events = specifications;
  events.merge(c, frame, memory);
  Stage2Weights weights;
  RolloutWeights config;config.collection_rewards=collection_rewards;
  std::vector<int> distances;
  World world{frame, memory, weights, config, distances, events, 0, {}, {}, 1};
  State state;
  state.robots[0].p = frame.self.position;
  state.robots[0].hp = frame.self.hp;
  state.robots[0].protection = frame.self.spawn_protected ? 1 : 0;
  state.damage = frame.score.effective_damage;
  state.resource = frame.score.resource_value;
  std::array<std::string, 4> teams{{frame.self.team, "", "", ""}};
  for (const auto& e : frame.visible_entities) {
    if (e.type == "robot" && world.robot_count < 4) {
      const int i = world.robot_count++;
      auto& r = state.robots[i];
      r.p = e.position;
      r.hp = e.hp.value_or(100);
      r.protection = e.spawn_protected.value_or(false) ? 1 : 0;
      teams[i] = e.team.value_or("");
    } else if ((e.type == "resource_normal" || e.type == "resource_high") && e.value)
      world.resources.push_back({e.position, *e.value, 1, e.type == "resource_high"});
  }
  for (const auto& [id, e] : events.effects()) {
    (void)id;
    world.effects.push_back(&e);
  }
  for (int i = 0; i < world.robot_count; ++i) {
    auto& r = state.robots[i];
    if (const auto b = events.buff(teams[i], "unrivaled", frame.tick)) {
      r.aoe_until = b->until;
      r.radius = b->effect.attack_radius;
      r.aoe_damage = b->effect.attack_damage;
      r.aoe_defended = b->effect.defended_damage;
    }
    if (const auto b = events.buff(teams[i], "pan", frame.tick)) {
      r.pan_until = b->until;
      r.pan_multiplier = b->effect.damage_multiplier;
    }
  }
  std::array<int, 4> commands{};
  for (std::size_t i = 0; i < actions.size() && i < 4; ++i)
    commands[i] = code(actions[i]);
  bool moved = false;
  double reward = world.advance(state, commands[0], 0, frame.tick, moved, &commands);
  Json out{{"robots", Json::array()},
           {"damage", state.damage},
           {"resource", state.resource},
           {"moved", moved},
           {"reward", reward}};
  for (int i = 0; i < world.robot_count; ++i) {
    const auto& r = state.robots[i];
    out["robots"].push_back({{"position", {r.p.x, r.p.y}},
                             {"hp", r.hp},
                             {"alive", r.alive},
                             {"pan_until", r.pan_until},
                             {"aoe_until", r.aoe_until}});
  }
  return out;
}
#endif
}  // namespace rmtc
