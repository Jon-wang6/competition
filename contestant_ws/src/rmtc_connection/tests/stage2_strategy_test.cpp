/**
 * @file stage2_strategy_test.cpp
 * @brief 第二阶段规则边界与场次回归，不依赖裁判内部信息，不连接实际比赛ROS域。
 *
 * 覆盖目标收益、动态任务、资源/敌人记忆、攻击先于移动、保护和事件解释边界。
 * 使用真实公开数据结构构造视野，不以神经网络或已知敌方脚本作为测试前提。
 *
 * @details 模块协作与维护约定
 * 验证对象：stage2_strategy.hpp、stage2_resource_lookahead.hpp、stage2_enemy_sightings.hpp、stage2_tactical_navigation.hpp、stage2_observation_age.hpp、stage2_collection.hpp、stage2_reach_finish.hpp、stage2_opponent_motion.hpp、stage2_opponent_intent.hpp。
 * 运行约定：由独立测试可执行文件/CTest执行，断言失败用于定位回归，不是正式选手的逐帧逻辑。
 * 结果边界：通过表示这些构造场景满足约束，不证明所有随机地图均最优或稳定达到某个总分。
 */
#include "rmtc_connection/stage2_strategy.hpp"
#include "rmtc_connection/stage2_resource_lookahead.hpp"
#include "rmtc_connection/stage2_enemy_sightings.hpp"
#include "rmtc_connection/stage2_tactical_navigation.hpp"
#include "rmtc_connection/stage2_observation_age.hpp"
#include "rmtc_connection/stage2_collection.hpp"
#include "rmtc_connection/stage2_reach_finish.hpp"
#include "rmtc_connection/stage2_opponent_motion.hpp"
#include "rmtc_connection/stage2_opponent_intent.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

/// 失败时给出业务原因；不使用Release构建会被删除的assert。
void require(bool condition, const std::string& reason) {
  if (!condition)
    throw std::runtime_error(reason);
}

/// 创建普通或高价值资源，保留稳定ID以检查跨帧关联。
rmtc::VisibleEntity resource(std::string id, int x, int y, double value = 10) {
  rmtc::VisibleEntity e;
  e.type = value >= 20 ? "resource_high" : "resource_normal";
  e.id = std::move(id);
  e.position = {x, y};
  e.value = value;
  return e;
}

/// 创建当前可见敌车；protected和defending字段分别测试保护与历史动作的区别。
rmtc::VisibleEntity enemy(int x, int y, double hp = 100, bool protected_spawn = false) {
  rmtc::VisibleEntity e;
  e.type = "robot";
  e.id = std::to_string(x) + "," + std::to_string(y);
  e.team = "B";
  e.position = {x, y};
  e.hp = hp;
  e.spawn_protected = protected_spawn;
  e.defending = false;
  return e;
}

/// 创建纯地形障碍；攻击测试故意把墙放到两车之间。
rmtc::VisibleEntity wall(int x, int y) {
  rmtc::VisibleEntity e;
  e.type = "obstacle";
  e.position = {x, y};
  return e;
}

/// 从场景裁剪方形视野；地图外和视野外的实体不会交给策略。
rmtc::PublicPerception frame(std::int64_t tick, rmtc::Position position,
                             std::vector<rmtc::VisibleEntity> entities = {},
                             rmtc::MapSize map = {9, 9}, int radius = 4) {
  rmtc::PublicPerception p;
  p.tick = tick;
  p.session_id = "stage2-test";
  p.map = map;
  p.vision_radius = radius;
  p.self.team = "A";
  p.self.robot_id = "A";
  p.self.position = position;
  p.self.alive = true;
  p.self.hp = 100;
  for (auto& e : entities) {
    if (std::abs(e.position.x - position.x) <= radius &&
        std::abs(e.position.y - position.y) <= radius)
      p.visible_entities.push_back(std::move(e));
  }
  return p;
}

/// 关闭探索奖励以隔离资源/任务/攻防行为，不改变被测规则常量。
rmtc::Stage2Weights without_exploration() {
  rmtc::Stage2Weights w;
  w.acquisition.enabled=false;w.temporal.enabled=false; // 单项旧规则与学习到的时间线隔离。
  w.purpose.enabled=false; // 规则边界不受可选的目标切换/截击偏好影响。
  w.separate_goal_switch_bonus = false;
  w.phase.enabled = false; // 单项规则不受默认分期策略影响；分期另有独立场景。
  w.interference.weight=0;w.enemy_belief_tracking=false;  // 单项规则不混入额外干扰/搜索估值。
  w.frontier_resource_weight=0;w.frontier_mission_weight=0;w.rollout.weight=0;
  w.collection_mode = 0;  // 单项规则测试关闭联合候选；联合规划另有专门场景。
  w.intent_chase_weight = w.intent_risk_weight = 0.0;
  w.third_party_risk_discount = w.third_party_attack_bonus = 0.0;
  w.chase_stall_ticks = 0;
  w.chase_shared_progress = false;
  w.route_alternatives_weight = w.observation_age_weight = 0.0;
  w.resource_lookahead_weight = w.route_information_weight = 0.0;
  w.blocked_retry_ticks = 0;
  w.recent_enemy_search_weight = 0;
  w.opening_resource_weight = w.resource_weight_end = 1.0;
  w.opening_combat_weight = w.combat_weight_end = 1.0;
  w.combat_ramp_ticks = 0;
  w.opening_reach_weight = w.reach_weight_end = 1.0;
  w.reach_ramp_ticks = 0;
  w.visible_resource_weight = 1.0;
  w.opponent_motion_weight = w.endgame_preserve_weight = w.low_hp_risk_weight = 0.0;
  w.astar_resource_competition = false;
  w.information_per_cell = 0;
  w.adaptive_resources = false;
  w.opening_ticks = 0;  // 既有规则场景不受开局战术默认值影响。
  // 单项规则使用基准设置；阶段偏好另有专门用例。
  w.recent_enemy_half_life = 6.0;
  w.recent_enemy_max_age = 18;
  w.contested_resource_factor = 0.5;
  return w;
}

/// 路程取可达路径，资源收益按真实价值并受120饱和限制。
void check_resource_and_caps() {
  rmtc::Stage2Strategy s(without_exploration());
  auto p = frame(0, {4, 4}, {resource("normal", 5, 4), resource("high", 4, 5, 20)});
  require(s.decide(p).direction == "down" && s.plan().target_id == "high",
          "high resource should win at equal distance");
  require(std::abs(s.plan().resource_points - 5) < 1e-9, "high resource score conversion");
  s.reset();
  p.score.resource_value = 115;
  s.decide(p);
  require(std::abs(s.plan().resource_points - 1.25) < 1e-9, "resource cap marginal gain");
  s.reset();
  p.score.resource_value = 120;
  require(s.decide(p).type == "wait", "capped resource must not create fake points");

  s.reset();
  p = frame(0, {0, 0},
            {resource("behind-wall", 2, 0, 20), resource("near", 0, 2), wall(1, 0), wall(1, 1),
             wall(1, 2), wall(1, 3)});
  require(s.decide(p).direction == "down" && s.plan().target_id == "near",
          "obstacle detour must enter goal cost");
  s.reset();
  p = frame(0, {4, 4}, {resource("too-far", 4, 6)});
  require(s.decide(p, 1).type == "wait", "unreachable within remaining budget");
}

/// 未知地图不能因眼前没资源停住；资源确认消失后立即取消追踪。
void check_exploration_and_memory() {
  rmtc::Stage2Strategy s;
  auto p = frame(0, {1, 1}, {}, {24, 24});
  require(s.decide(p).type == "move" && s.plan().goal == "explore",
          "empty visible resources must explore unknown map");
  s = rmtc::Stage2Strategy(without_exploration());
  s.decide(frame(0, {1, 1}, {resource("remember", 4, 1)}));
  s.decide(frame(1, {1, 3}, {}, {9, 9}, 1));
  require(s.plan().target_id == "remember" && s.plan().resource_points < 2.5,
          "unseen resource should persist with age discount");
  require(s.decide(frame(2, {1, 1})).type == "wait" && !s.plan().target,
          "observed missing resource must no longer be a target");
}

/// 任务位置/ID来自每帧，已完成任务不重复追；未知类型只报告不自行编造语义。
void check_dynamic_missions() {
  rmtc::Stage2Strategy s(without_exploration());
  auto p = frame(0, {4, 4});
  rmtc::MissionInfo m;
  m.id = "dynamic-17";
  m.type = "reach";
  m.position = rmtc::Position{6, 4};
  p.missions = {m};
  require(s.decide(p).direction == "right" && s.plan().goal == "mission",
          "dynamic reach mission should guide route");
  p.tick = 1;
  p.missions[0].position = rmtc::Position{4, 2};
  require(s.decide(p).direction == "up", "mission changed coordinate not applied");
  p.tick = 2;
  p.score.mission_completed_ids = {m.id};
  require(s.decide(p).type == "wait", "completed mission was pursued again");
  p.tick = 3;
  p.score.mission_completed_ids.clear();
  p.missions[0].type = "future-unknown-type";
  require(s.decide(p).type == "wait" && s.plan().unsupported_missions == 1,
          "unknown mission must not silently invent semantics");

  s.reset();
  p = frame(0, {4, 4}, {resource("normal", 5, 4), resource("high", 4, 5, 20)});
  m.id = "one-normal";
  m.type = "normal_resources";
  m.target = 1;
  m.position.reset();
  p.missions = {m};
  require(s.decide(p).direction == "right" && s.plan().mission_points > 0,
          "collection mission potential not included");
  auto next = frame(1, {5, 4}, {resource("second", 6, 4)});
  next.missions = {m};
  next.score.resource_value = 10;
  s.decide(next);
  require(s.plan().mission_points == 0, "confirmed own pickup not tracked conservatively");
}

/// 攻击穿墙，两格内最近目标受保护时不能越过；上帧defending不保证本帧继续防御。
void check_attack_rules() {
  rmtc::Stage2Strategy s(without_exploration());
  auto p = frame(0, {4, 4}, {wall(5, 4), enemy(6, 4)});
  auto a = s.decide(p);
  require(a.type == "attack" && a.direction == "right", "wall must not block standard attack");
  p.tick = 1;
  p.visible_entities.back().defending = true;
  require(s.decide(p).type == "attack", "previous defend cannot guarantee future invulnerability");
  s.reset();
  p = frame(0, {0, 0}, {enemy(1, 0, 100, true), enemy(2, 0)}, {3, 1});
  require(s.decide(p).type != "attack", "cannot shoot past nearest protected robot");
  s.reset();
  p = frame(0, {4, 4}, {enemy(6, 4)});
  p.self.spawn_protected = true;
  require(s.decide(p).type != "attack", "spawn-protected self cannot attack");
  s.reset();
  p.self.spawn_protected = false;
  p.score.effective_damage = 500;
  require(s.decide(p).type != "attack", "capped damage should not create fake combat points");
}

/// 当前攻击在移动前结算：15血面对20伤害应防御，不能以侧移假装躲掉本Tick攻击。
void check_defense_and_occupancy() {
  rmtc::Stage2Strategy s(without_exploration());
  auto p = frame(0, {4, 4}, {enemy(6, 4), resource("tempting", 4, 5, 20)});
  p.self.hp = 15;
  require(s.decide(p).type == "defend" && s.plan().incoming_damage_bound == 20,
          "moving cannot avoid attack-before-movement damage");
  for (const auto& score : s.plan().action_scores) {
    if (score.at("action") == "move")
      require(score.at("incoming_bound") == 20, "move candidate skipped current incoming attack");
  }
  s.reset();
  p = frame(0, {0, 0}, {enemy(1, 0, 100, true), resource("blocked", 2, 0)}, {3, 1});
  p.self.spawn_protected = true;
  require(s.decide(p).type == "wait", "visible robot must block corridor movement");
  p.tick = 1;
  p.visible_entities.erase(p.visible_entities.begin());
  require(s.decide(p).direction == "right", "old enemy must not permanently block path");
}

/// 原文不走关键词分支；未来LLM结构化结果必须匹配本局原公告且到期后不再封格。
void check_event_boundary() {
  rmtc::Stage2Strategy s(without_exploration());
  auto p = frame(0, {4, 4}, {resource("target", 6, 4)});
  p.event_text = "禁止向右，测试原文不能被关键词程序解析";
  require(s.decide(p).direction == "right" && s.plan().event_pending,
          "raw event text must remain explicitly uninterpreted");
  rmtc::Stage2EventConstraints e;
  e.session_id = "other-session";
  e.source_text = p.event_text;
  e.end_tick = 3;
  e.dangerous_cells = {{5, 4}};
  require(!s.set_event_constraints(e), "cross-session event accepted");
  e.session_id = p.session_id;
  e.dangerous_cells = {{99, 99}};
  require(!s.set_event_constraints(e), "out-of-map event accepted");
  e.dangerous_cells = {{5, 4}};
  e.source_tick = 5;
  require(!s.set_event_constraints(e), "future-source event accepted");
  e.source_tick = 0;
  require(s.set_event_constraints(e), "valid structured event rejected");
  p.tick = 1;
  require(s.decide(p).direction != "right" && !s.plan().event_pending,
          "validated dangerous cell not avoided");
  p.tick = 3;
  require(s.decide(p).direction == "right", "expired event still blocked path");
}

/// 地形已全部探明后，战斗未满分应巡查旧区域；满分后停止追逐无收益的目标。
void check_late_game_patrol() {
  rmtc::Stage2Strategy s;
  for (int i = 0; i < 4; ++i)
    s.decide(frame(i, {1 + 3 * i, 2}, {}, {12, 5}, 2));
  auto p = frame(20, {1, 2}, {}, {12, 5}, 2);
  p.score.effective_damage = 455;
  require(s.decide(p).type == "move" && s.plan().goal == "patrol" && s.plan().patrol_potential > 0,
          "fully explored terrain must not disable searching for remaining combat points");
  p.tick = 21;
  p.score.effective_damage = 500;
  require(s.decide(p).type == "wait" && s.plan().goal == "hold",
          "combat cap must stop unnecessary patrol and goal inertia");
  p.tick = 22;
  p.score.effective_damage = 440;
  require(s.decide(p).type == "move", "damage loss must reactivate patrol opportunity");
  p.tick = 23;
  p.self.alive = false;
  require(s.decide(p).type == "wait", "dead robot cannot patrol");
  p.tick = 26;
  p.self.alive = true;
  p.self.spawn_protected = true;
  require(s.decide(p).type == "move" && s.plan().goal == "patrol",
          "respawn must not turn complete terrain memory into permanent spawn waiting");
  p.tick = 27;
  require(s.decide(p, 1).type == "wait", "patrol must leave time to use discovered opportunities");
  p.tick = 28;
  p.score.effective_damage = 500;
  rmtc::MissionInfo mission;
  mission.id = "damage-beyond-score-cap";
  mission.type = "effective_damage";
  mission.target = 600;
  p.missions = {mission};
  require(s.decide(p).type == "move",
          "unfinished damage mission can justify patrol past score cap");
  p.tick = 29;
  p.score.mission_completed_ids = {mission.id};
  require(s.decide(p).type == "wait", "completed damage mission must not prolong patrol");

  s.reset();
  auto fresh = frame(0, {2, 2}, {}, {5, 5});
  s.decide(fresh);
  fresh.tick = 100;
  require(s.decide(fresh).type == "wait", "fully visible empty arena needs no fake patrol");
}

/// 开局加权随Tick恢复：能暂缓交战采集，低血仍防御；资源耗尽不再采集，仍可还击。
void check_opening_resource_priority() {
  auto w = without_exploration();
  w.opening_ticks = 80;
  w.opening_resource_weight = 2.0;
  w.opening_combat_weight = 0.7;
  w.combat_ramp_ticks = 80;
  auto p = frame(0, {4, 4}, {resource("opening", 0, 0), enemy(6, 4)});
  rmtc::Stage2Strategy s(w);
  require(s.decide(p).type == "move" && s.plan().target_id == "opening",
          "opening should favor collecting a reachable resource over optional attack");
  require(std::abs(s.plan().resource_points - 2.5) < 1e-9,
          "opening preference must not change actual resource scoring");
  const double initial = s.plan().goal_utility;
  p.tick = 40;
  s.decide(p);
  const double middle = s.plan().goal_utility;
  p.tick = 80;
  require(s.decide(p).type == "attack", "normal combat preference must return after opening");
  require(initial > middle && middle > s.plan().goal_utility,
          "resource bonus must decay smoothly instead of remaining constant until cutoff");

  s.reset();
  p.tick = 0;
  p.self.hp = 15;
  require(s.decide(p).type == "defend", "opening bonus must retain lethal-risk defense");
  s.reset();
  auto tempting = frame(0, {4, 4}, {resource("high", 4, 5, 20), enemy(6, 4)});
  tempting.self.hp = 15;
  rmtc::MissionInfo collection;
  collection.id = "last-high";
  collection.type = "high_resources";
  collection.target = 1;
  tempting.missions = {collection};
  require(s.decide(tempting).type == "defend",
          "opening bonus must not override defense even beside a high-value mission resource");
  s.reset();
  p.self.hp = 100;
  s.decide(p);
  p.tick = 1;
  p.visible_entities = {enemy(6, 4)};
  require(s.decide(p).type == "attack" && s.plan().goal != "resource",
          "fully observed resource depletion must cancel collection without disabling attack");
  const double depleted_utility = s.plan().action_utility;
  auto normal = without_exploration();
  rmtc::Stage2Strategy baseline(normal);
  baseline.decide(p);
  require(depleted_utility < baseline.plan().action_utility &&
              std::abs(s.plan().combat_weight - (0.7 + 0.3 / 80)) < 1e-9,
          "resource depletion must not jump the independent combat ramp to full strength");
}

/// 资源经济只使用公开记忆：采集和缺失不重复扣除，开图不会抹去已知资源/任务收益。
void check_adaptive_resource_economy() {
  rmtc::Stage2Weights w;
  w.adaptive_resources = true;
  rmtc::MapMemory memory;
  auto p = frame(0, {1, 1}, {resource("seen", 2, 1)}, {24, 24});
  memory.update(p);
  const auto first = rmtc::assess_resource_economy(memory, p, w);
  require(first.coverage > 0 && first.coverage < 1 && first.known_available_value == 10,
          "economy must use public observed terrain and unique resource memories");
  p.tick = 1;
  p.self.position = {2, 1};
  p.visible_entities.clear();
  p.score.resource_value = 10;
  memory.update(p);
  const auto collected = rmtc::assess_resource_economy(memory, p, w);
  require(collected.confirmed_removed_value == 10 && collected.known_available_value == 0,
          "own collected value and the same missing resource must not be added twice");
  require(collected.collection_factor < first.collection_factor &&
              collected.exploration_factor >= w.exploration_floor,
          "collection should reduce extra appetite without zeroing remaining corners");
  p.tick = 2;
  memory.update(p);
  require(rmtc::assess_resource_economy(memory, p, w).confirmed_removed_value == 10,
          "repeated observation must not consume the same missing resource again");

  rmtc::Stage2Strategy s(w);
  p = frame(0, {4, 4}, {resource("known-high", 5, 4, 20)});
  require(
      s.decide(p).type == "move" && s.plan().economy.coverage == 1 && s.plan().resource_points == 5,
      "complete map coverage must not remove a known resource's actual value");
  s.reset();
  p.score.resource_value = 120;
  rmtc::MissionInfo m;
  m.id = "still-need-high";
  m.type = "high_resources";
  m.target = 1;
  p.missions = {m};
  require(
      s.decide(p).type == "move" && s.plan().resource_points == 0 && s.plan().mission_points > 0,
      "resource score saturation must retain unfinished collection mission value");
  s.reset();
  p.session_id = "fresh-economy";
  p.score = {};
  p.missions.clear();
  s.decide(p);
  require(s.plan().economy.confirmed_removed_value == 0,
          "new scene must not inherit previous resource depletion");
  w.exploration_floor = 1.1;
  bool rejected = false;
  try {
    rmtc::Stage2Strategy invalid(w);
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  require(rejected, "exploration floor above one must fail early");
}

/// 连续小范围循环追击才触发暂停；立即攻击、远行探索和新场次不被误封禁。
void check_unproductive_pursuit() {
  auto w = without_exploration();
  w.chase_stall_ticks = 12;
  w.chase_cooldown_ticks = 16;
  rmtc::Stage2Strategy s(w);
  const std::array<rmtc::Position, 4> loop{{{4, 8}, {5, 8}, {4, 8}, {3, 8}}};
  for (int tick = 0; tick <= 12; ++tick)
    s.decide(frame(tick, loop[tick % 4], {enemy(4, 4)}, {14, 14}));
  require(s.plan().pursuit_cooldowns == 1 && s.plan().goal != "combat",
          "repeated local pursuit without damage must temporarily yield to other goals");
  auto nearby = enemy(6, 8);
  nearby.id = "4,4";
  require(s.decide(frame(13, {4, 8}, {nearby}, {14, 14})).type == "attack",
          "pursuit cooldown must not forbid an immediate legal attack on that enemy");
  require(s.decide(frame(30, {4, 8}, {enemy(4, 4)}, {14, 14})).type == "move" &&
              s.plan().pursuit_cooldowns == 0,
          "pursuit cooldown must expire instead of permanently abandoning an enemy");
  auto fresh = frame(0, {4, 8}, {enemy(4, 5)}, {14, 14});
  fresh.session_id = "new-pursuit-session";
  s.decide(fresh);
  require(s.plan().pursuit_cooldowns == 0, "new session inherited pursuit cooldown");

  s.reset();
  for (int tick = 0; tick <= 12; ++tick) {
    auto moving = enemy(4, 12 - tick);
    moving.id = "moving-enemy";
    s.decide(frame(tick, {4, 16 - tick}, {moving}, {24, 24}));
    require(s.plan().pursuit_cooldowns == 0,
            "long pursuit through new cells must not count as a local movement loop");
  }
}

/// 切换两名追击对象不能绕过循环检测；真正打出伤害或漏帧则重新观察。
void check_shared_pursuit_progress() {
  auto w = without_exploration();
  w.chase_shared_progress = true;
  w.chase_stall_ticks = 12;
  w.chase_cooldown_ticks = 16;
  rmtc::Stage2Strategy shared(w);
  w.chase_shared_progress = false;
  rmtc::Stage2Strategy legacy(w);
  const std::array<rmtc::Position, 4> loop{{{4, 8}, {5, 8}, {4, 8}, {3, 8}}};
  for (int tick = 0; tick <= 12; ++tick) {
    auto target = enemy(4, 4);
    target.id = tick % 2 ? "B" : "C";
    auto p = frame(tick, loop[tick % 4], {target}, {14, 14});
    shared.decide(p);
    legacy.decide(p);
  }
  require(shared.plan().pursuit_cooldowns == 2,
          "alternating enemies must not erase the shared movement loop");
  require(legacy.plan().pursuit_cooldowns == 0,
          "legacy per-target detector is required as an ablation control");
  auto nearby = enemy(6, 8);
  nearby.id = "B";
  require(shared.decide(frame(13, {4, 8}, {nearby}, {14, 14})).type == "attack",
          "shared cooldown must keep immediate valid shots available");
  shared.reset();
  for (int tick = 0; tick <= 24; ++tick) {
    auto target = enemy(4, 4);
    target.id = tick % 2 ? "B" : "C";
    auto p = frame(tick, loop[tick % 4], {target}, {14, 14});
    p.score.effective_damage = 20 * (tick / 8);
    shared.decide(p);
    require(shared.plan().pursuit_cooldowns == 0,
            "damage progress must reset the no-progress window");
  }
}

/// 距离场必须绕障碍、排除未知/动态占位，并给相同长度的首步相同进展。
void check_goal_distance_field() {
  const rmtc::MapSize size{7, 7};
  const auto passable = [](rmtc::Position p) {
    return !(p.x == 3 && p.y <= 4) && !(p.x == 6 && p.y == 6);
  };
  const auto field = rmtc::goal_distance_field(size, {5, 2}, passable);
  require(field[2 * 7 + 2] == 9 && field[3 * 7 + 2] == 8 && field[1 * 7 + 2] == 10,
          "goal distance must follow the real detour rather than Manhattan progress");
  require(field[2 * 7 + 3] == -1 && field[6 * 7 + 6] == -1,
          "walls and caller-excluded cells must stay unreachable");
  const auto open = rmtc::goal_distance_field(size, {5, 5}, [](rmtc::Position) { return true; });
  require(open[2 * 7 + 3] == open[3 * 7 + 2],
          "equally short first steps must receive equal progress");
  const auto invalid = rmtc::goal_distance_field(size, {-1, 0}, passable);
  require(std::all_of(invalid.begin(), invalid.end(), [](int d) { return d == -1; }),
          "out-of-bounds target must not index the grid");
  auto w = without_exploration();
  w.route_alternatives_weight = 1.01;
  bool rejected = false;
  try {
    rmtc::Stage2Strategy bad(w);
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  require(rejected, "route mixture outside zero to one must fail before the match");
}

/// 观测年龄按最近一次实际看到计算；未知/墙不重复计奖，视野边缘查询必须正确。
void check_observation_age() {
  const rmtc::MapSize size{3, 3};
  std::vector<std::int8_t> cells(9, 0);
  std::vector<std::int64_t> last(9, 0);
  last[0] = 100;
  cells[rmtc::MapMemory::index({1, 0}, size)] = -1;
  last[1] = -1;
  cells[rmtc::MapMemory::index({2, 0}, size)] = 100;
  rmtc::ObservationAgeField age(size, cells, last, 100, 8, 60);
  require(std::abs(age.mean_in_view({1, 1}, 1) - 6.0 / 7.0) < 1e-9,
          "mean age must exclude unknown and obstacle cells and refresh visible cells");
  require(age.mean_in_view({0, 0}, 0) == 0 && age.mean_in_view({2, 2}, 0) == 1,
          "fresh cell zero, old cell capped at one");
  require(std::abs(age.mean_in_view({0, 0}, 1) - 2.0 / 3.0) < 1e-9,
          "clipped view prefix sum must match actual map cells");
  last[0] = 66;
  rmtc::ObservationAgeField middle(size, cells, last, 100, 8, 60);
  require(std::abs(middle.mean_in_view({0, 0}, 0) - 0.5) < 1e-9,
          "age must increase smoothly between fresh and stale thresholds");
  auto w = without_exploration();
  w.observation_age_weight = 0.75;
  w.patrol_value = 0;
  rmtc::Stage2Strategy strategy(w);
  strategy.decide(frame(0, {4, 4}, {}, {9, 9}, 10));
  strategy.decide(frame(100, {1, 1}, {}, {9, 9}, 1));
  require(strategy.plan().goal == "explore" && strategy.plan().observation_age_points > 0,
          "old known free area must guide searching when combat score remains available");
  auto saturated = frame(101, {1, 1}, {}, {9, 9}, 1);
  saturated.score.effective_damage = 500;
  require(strategy.decide(saturated).type == "wait" && strategy.plan().observation_age_points == 0,
          "age alone must not cause endless searching after damage rewards are saturated");
  w.observation_stale_ticks = w.observation_fresh_ticks;
  bool rejected = false;
  try {
    rmtc::Stage2Strategy invalid(w);
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  require(rejected, "equal age thresholds must fail before division by zero");
}

/// 早期可延后非消耗型任务，后期补齐；采集任务偏好独立，不能伪造任务真实收益。
// 验证分期的外在行为：机会优先级、顺路约束、任务截止与攻击先结算的生存边界。
void check_explicit_goal_phases() {
  auto w = without_exploration();
  w.phase.enabled = true;
  w.phase.resource_ticks = 80;
  w.phase.opportunity_floor = 0;
  w.phase.combat_opportunity_floor = 0;
  w.phase.task_detour = -1;
  w.route_risk = 0;
  rmtc::MissionInfo goal;
  goal.id = "reach"; goal.type = "reach"; goal.position = rmtc::Position{4, 5};
  auto opening = frame(0, {4, 4}, {resource("scarce", 4, 1)});
  opening.missions = {goal};
  rmtc::Stage2Strategy collecting(w);
  require(collecting.decide(opening).direction == "up" &&
          collecting.plan().phase == "collect" && collecting.plan().goal == "resource",
          "opening phase abandoned a collectible resource for a private reach task");

  auto combat = frame(100, {4, 4}, {resource("leftover", 4, 1), enemy(6, 4)});
  combat.missions = {goal};
  rmtc::Stage2Strategy fighting(w);
  require(fighting.decide(combat).type == "attack" && fighting.plan().phase == "fight" &&
          fighting.plan().goal == "combat", "middle phase did not focus a legal damage opportunity");
  rmtc::Stage2Strategy deadline(w);
  deadline.decide(combat, 3);
  require(deadline.plan().goal == "mission", "phase focus ignored an expiring reach task");

  combat.score.effective_damage = 500;
  rmtc::Stage2Strategy finishing(w);
  finishing.decide(combat);
  require(finishing.plan().phase == "finish" && finishing.plan().goal == "mission",
          "damage cap did not release the controller to unfinished tasks");

  goal.position = rmtc::Position{4, 3}; opening.missions = {goal};
  w.phase.task_detour = 0;
  rmtc::Stage2Strategy waypoint(w);
  waypoint.decide(opening);
  require(waypoint.plan().task_waypoint && waypoint.plan().target &&
          waypoint.plan().target->x == 4 && waypoint.plan().target->y == 3,
          "an exactly on-route task was not used as a collection waypoint");
  opening.missions.front().position = rmtc::Position{5, 3};
  rmtc::Stage2Strategy detour(w);
  detour.decide(opening);
  require(!detour.plan().task_waypoint && detour.plan().goal == "resource",
          "zero-detour policy allowed an extra two-step task detour");

  // 近邻资源优先仍受真实步数约束，不能用“视野内”把远方高价值点冒充近邻。
  w.phase.visible_resource_steps = 1;
  auto nearby = frame(0, {4, 4}, {resource("near", 5, 4), resource("far", 4, 1, 20)});
  rmtc::Stage2Strategy near_resource(w);
  near_resource.decide(nearby);
  require(near_resource.plan().target_id == "near", "near visible resource was overlooked");
  w.phase.visible_resource_steps = 0;
  rmtc::Stage2Strategy no_near_override(w);
  no_near_override.decide(nearby);
  require(no_near_override.plan().target_id == "far", "disabled near-resource override changed ranking");

  // 两个等长射击接近点中，后续任务距离场应选能顺路完成任务的一侧。
  auto approach = frame(100, {1, 1}, {enemy(4, 4)});
  goal.position = rmtc::Position{1, 4}; approach.missions = {goal};
  rmtc::Stage2Strategy direct_fight(w);
  direct_fight.decide(approach);
  w.phase.task_lookahead_weight = 1.0;
  rmtc::Stage2Strategy task_aware_fight(w);
  task_aware_fight.decide(approach);
  require(!direct_fight.plan().task_waypoint && task_aware_fight.plan().task_waypoint,
          "combat lookahead failed to prefer an equally short on-task approach");
  w.phase.task_lookahead_weight = 0;

  // 资源暂时看完后可以提前转战，新资源出现又能采集；不能在重开一局时沿用旧计时。
  w.phase.resource_silence_ticks = 4;
  rmtc::Stage2Strategy adaptive_phase(w);
  auto seen = frame(0, {4, 4}, {resource("first", 4, 1)}, {24, 24});
  adaptive_phase.decide(seen);
  require(adaptive_phase.plan().phase == "collect", "visible resource failed to retain collection phase");
  auto silence = frame(5, {4, 4}, {enemy(6, 4)}, {24, 24});
  adaptive_phase.decide(silence);
  require(adaptive_phase.plan().phase == "fight" && adaptive_phase.plan().phase_resource_age == 5,
          "resource silence did not release early collection focus");
  auto rediscovered = frame(6, {4, 4}, {resource("new", 4, 1)}, {24, 24});
  adaptive_phase.decide(rediscovered);
  require(adaptive_phase.plan().phase == "collect" && adaptive_phase.plan().phase_resource_age == 0,
          "new resource failed to restore a collection opportunity");
  adaptive_phase.reset();
  auto restarted = frame(0, {4, 4}, {}, {24, 24});
  adaptive_phase.decide(restarted);
  require(adaptive_phase.plan().phase == "collect" && adaptive_phase.plan().phase_resource_age == 0,
          "new match inherited an old resource-silence clock");
  w.phase.resource_silence_ticks = 0;

  auto lethal = frame(0, {4, 4}, {resource("scarce", 4, 1), enemy(6, 4)});
  lethal.self.hp = 15;
  rmtc::Stage2Strategy protection(w);
  require(protection.decide(lethal).type == "defend",
          "phase collection overrode attack-before-movement survival");
  w.phase.opportunity_floor = 2;
  bool rejected = false;
  try { rmtc::Stage2Strategy bad(w); } catch (const std::invalid_argument&) { rejected = true; }
  require(rejected, "out-of-range phase opportunity floor accepted");
}

/// 保目标的选择奖励不能冒充移动收益；在射程内仍应比较真实攻击，而不是原地绕着走。
void check_goal_switch_bonus_separation() {
  auto w = without_exploration();
  w.switch_margin = 0.5;
  w.route_risk = 0;
  auto p = frame(0, {4, 4}, {enemy(6, 4)});
  rmtc::Stage2Strategy coupled(w);
  coupled.decide(p);
  p.tick = 1;
  require(coupled.decide(p).type == "move", "fixture did not reproduce target bonus overriding a shot");
  w.separate_goal_switch_bonus = true;
  rmtc::Stage2Strategy separated(w);
  p.tick = 0;
  separated.decide(p);
  p.tick = 1;
  require(separated.decide(p).type == "attack" && separated.plan().goal == "combat",
          "goal persistence bonus still overrode an in-range attack");

  // 接近500伤害上限时，默认0.03惯性也能大于剩余一发伤害的真实边际价值。
  // 此处关闭局部掉血/后续暴露代价，隔离惯性耦合；真实危险场景仍允许优先避险。
  w.switch_margin = 0.03;
  w.future_exposure_cost = w.hp_loss_cost = 0;
  w.separate_goal_switch_bonus = false;
  p.tick = 0; p.score.effective_damage = 499;
  rmtc::Stage2Strategy near_cap_coupled(w);
  near_cap_coupled.decide(p); p.tick = 1;
  require(near_cap_coupled.decide(p).type == "move", "near-cap fixture did not expose default bonus coupling");
  w.separate_goal_switch_bonus = true;
  rmtc::Stage2Strategy near_cap_separated(w);
  p.tick = 0; near_cap_separated.decide(p); p.tick = 1;
  require(near_cap_separated.decide(p).type == "attack",
          "default persistence bonus still blocked finishing the last damage point");
}

/// 旧线性任务偏好仍按原参数工作，不被关闭的显式分期改变。
void check_phase_mission_preferences() {
  auto w = without_exploration();
  w.opening_ticks = 80;
  w.opening_reach_weight = 0;
  w.reach_ramp_ticks = 80;
  w.reach_weight_end = 2;
  rmtc::Stage2Strategy strategy(w);
  auto p = frame(0, {4, 4}, {resource("normal", 5, 4)});
  rmtc::MissionInfo reach;
  reach.id = "reach";
  reach.type = "reach";
  reach.position = rmtc::Position{4, 5};
  p.missions = {reach};
  strategy.decide(p);
  require(strategy.plan().goal == "resource",
          "opening reach delay must leave scarce resources first");
  p.tick = 80;
  strategy.decide(p);
  require(strategy.plan().goal == "mission", "reach end weight must recover the delayed mission");

  w = without_exploration();
  w.resource_mission_weight = 0;
  rmtc::Stage2Strategy no_collection_bonus(w);
  w.resource_mission_weight = 2;
  rmtc::Stage2Strategy collection_bonus(w);
  p = frame(0, {4, 4}, {resource("high", 5, 4, 20)});
  p.score.resource_value = 120;
  rmtc::MissionInfo high;
  high.id = "high";
  high.type = "high_resources";
  high.target = 1;
  p.missions = {reach, high};
  no_collection_bonus.decide(p);
  collection_bonus.decide(p);
  require(
      no_collection_bonus.plan().goal == "mission" && collection_bonus.plan().goal == "resource",
      "collection mission preference must be independently adjustable");
  require(std::abs(collection_bonus.plan().mission_points - 3.0) < 1e-9,
          "preference multiplier must not inflate raw mission potential points");
}

/** @brief 开局采集、晚期到达、截止兜底和攻防渐变必须各自按独立时间线生效。 */
void check_resource_first_schedule() {
  auto w = without_exploration();
  w.opening_resource_weight = w.resource_weight_end = 0.95;
  w.opening_reach_weight = 0.15;
  w.reach_weight_end = 1.0;
  w.reach_ramp_ticks = 240;
  w.visible_resource_weight = 1.25;
  w.switch_margin = 0;
  rmtc::Stage2Strategy strategy(w);
  auto p = frame(0, {4, 4}, {resource("scarce", 5, 4)});
  rmtc::MissionInfo goal;
  goal.id = "goal"; goal.type = "reach"; goal.position = rmtc::Position{4, 5};
  p.missions = {goal};
  for (int tick : {0, 80, 120, 240}) {
    p.tick = tick;
    strategy.decide(p, 500 - tick);
    require(std::abs(strategy.plan().reach_weight - (0.15 + 0.85 * tick / 240.0)) < 1e-9,
            "reach preference did not use the independent 240-tick ramp");
    require(strategy.plan().goal == (tick == 240 ? "mission" : "resource"),
            "early collection / later reach schedule chose the wrong objective");
    require(strategy.plan().resource_points == (tick == 240 ? 0.0 : 2.5),
            "visible resource preference inflated raw scoring points");
  }
  strategy.reset(); p.tick = 0;
  strategy.decide(p, 5);
  require(strategy.plan().goal == "mission", "reach deadline ignored travel plus safety buffer");
  require(rmtc::reach_priority_at(w, 0, 500, -1, false) == 1.0,
          "confirmed resource exhaustion failed to release deferred reach task");

  w.opening_combat_weight = 0.6; w.combat_weight_end = 1.15; w.combat_ramp_ticks = 200;
  rmtc::Stage2Strategy combat(w);
  for (int tick : {0, 100, 200, 400}) {
    auto f = frame(tick, {4, 4}, {enemy(6, 4)});
    combat.decide(f, 500 - tick);
    const double expected = 0.6 + 0.55 * std::min(tick / 200.0, 1.0);
    require(std::abs(combat.plan().combat_weight - expected) < 1e-9,
            "combat ramp changed with map coverage or did not reach its endpoint");
    require(combat.plan().combat_points > 0, "soft combat preference disabled a valid attack opportunity");
  }
  auto invalid = w; invalid.reach_ramp_ticks = -1;
  bool rejected = false;
  try { rmtc::Stage2Strategy bad(invalid); } catch (const std::invalid_argument&) { rejected = true; }
  require(rejected, "negative reach ramp was accepted");
}

/** @brief 末200Tick补任务不受采集/分期覆盖；途中允许攻击，停留预算用完才推进。 */
void check_reach_finish_window() {
  auto w = without_exploration();
  w.resource_race.enabled = true;
  w.phase.enabled = true;
  w.purpose.enabled = true;
  w.phase.resource_ticks = 500;
  w.opening_combat_weight = w.combat_weight_end = 3;
  w.future_exposure_cost = w.route_risk = 0;
  w.switch_margin = 0;
  auto p = frame(299, {4,4}, {resource("near",3,4)}, {10,10}, 10);
  rmtc::MissionInfo mission;
  mission.id = "late-reach"; mission.type = "reach"; mission.position = rmtc::Position{8,8};
  p.missions = {mission};
  rmtc::Stage2Strategy s(w);
  s.decide(p, 201);
  require(s.plan().goal == "resource" && !s.plan().reach_finish.value("active", false),
          "late reach policy took scarce resources before the final 200 ticks");
  auto disabled=w;disabled.reach_finish_ticks=0;
  rmtc::Stage2Strategy legacy(disabled);p.tick=300;legacy.decide(p,200);
  require(legacy.plan().goal=="resource"&&!legacy.plan().reach_finish.value("active",false),
          "disabling the late reach layer did not restore ordinary resource comparison");
  p.visible_entities.push_back(enemy(4,2));
  p.tick = 300;
  require(s.decide(p,200).type == "attack" && s.plan().goal == "mission" &&
          s.plan().reach_finish.value("active", false),
          "late mission must become the destination while retaining a good roadside attack");
  for (int tick=301;tick<=303;++tick) {
    p.tick=tick;
    const auto action=s.decide(p,500-tick);
    require(action.type == (tick==303?"move":"attack"),
            "reach progress allowance either blocked roadside attacks or permitted endless fighting");
  }
  p.tick=304; p.score.mission_completed_ids={mission.id};
  s.decide(p,196);
  require(!s.plan().reach_finish.value("active",false) && s.plan().goal!="mission",
          "completed reach mission kept attracting the controller");
  s.reset(); p.score.mission_completed_ids.clear();p.tick=300;p.self.hp=15;
  s.decide(p,200);p.tick=303;
  require(s.decide(p,197).type=="defend" &&
          !s.plan().reach_finish.value("progress_selected",false),
          "reach deadline forced a move that dies before movement settlement");
  // 双方互防不能永远冻住：连续12帧未受伤后，仅允许一步脱离射线的试探。
  s.reset();p.tick=300;
  for(auto& e:p.visible_entities)if(e.type=="robot")e.defending=true;
  for(int tick=300;tick<=312;++tick) {
    p.tick=tick;const auto action=s.decide(p,500-tick);
    require(action.type==(tick<312?"defend":"move"),
            "mutual defense escape ignored its observation window");
    if(tick==312)require(action.direction=="down"||action.direction=="right",
                        "mutual defense escape did not leave the enemy shot line");
  }
  require(s.plan().reach_finish.value("stalemate_escape",false),
          "mutual defense experiment was not recorded in the plan");
  require(rmtc::reach_attack_allowance(200,20,24,3)==3 &&
          rmtc::reach_attack_allowance(28,20,24,3)==0,
          "roadside combat allowance ignored the remaining path and reserve");

  // 同一份参数在短赛程也按剩余Tick生效，不把300写死成开始时间。
  p.self.hp=100;p.tick=0;p.visible_entities.clear();s.reset();s.decide(p,100);
  require(s.plan().reach_finish.value("active",false),"reach finish ignored a custom match horizon");
  auto invalid=w;invalid.reach_finish_ticks=501;
  bool rejected=false;
  try {rmtc::Stage2Strategy bad(invalid);}catch(const std::invalid_argument&){rejected=true;}
  require(rejected,"invalid reach finish window was accepted");
}

/** @brief 未连通任务经乐观路径逐步开图，可绕已知墙，不会把未知格当成可执行落点。 */
void check_reach_finish_unknown_route() {
  auto w=without_exploration();w.switch_margin=0;
  rmtc::Stage2Strategy s(w);
  // 竖墙挡住直线，开口在y=1；先向上离开同一横线，不能靠曼哈顿贪心硬撞。
  std::vector<rmtc::VisibleEntity> walls;
  for(int y=2;y<9;++y)walls.push_back(wall(4,y));
  rmtc::MissionInfo mission;
  mission.id="fog-reach";mission.type="reach";mission.position=rmtc::Position{8,5};
  rmtc::Position at{2,5};bool explored=false,reached=false;
  for(int tick=300;tick<400;++tick) {
    auto p=frame(tick,at,walls,{10,10},2);p.missions={mission};
    const auto action=s.decide(p,500-tick);
    explored=explored||s.plan().goal=="mission_explore";
    require(action.type=="move","reachable fog mission did not make forward progress");
    if(action.direction=="up")--at.y;else if(action.direction=="down")++at.y;
    else if(action.direction=="left")--at.x;else if(action.direction=="right")++at.x;
    require(at.x>=0&&at.y>=0&&at.x<10&&at.y<10&&!(at.x==4&&at.y>=2&&at.y<9),
            "reach frontier navigation stepped outside or through a wall");
    if(at.x==8&&at.y==5){reached=true;break;}
  }
  require(explored&&reached,"late reach navigation could not complete a partially observed detour");
}

/** @brief 补任务不能和另一辆车永远争同一空格；旧参数关闭普通避让时仍按真实回执绕行。 */
void check_reach_finish_collision_backoff() {
  auto w=without_exploration();w.blocked_retry_ticks=0;
  rmtc::Stage2Strategy s(w);
  auto p=frame(300,{4,4});
  rmtc::MissionInfo mission;mission.id="reach";mission.type="reach";mission.position=rmtc::Position{8,4};
  p.missions={mission};
  auto action=s.decide(p,200);
  require(action.type=="move"&&action.direction=="right","collision fixture did not take the direct route");
  p.tick=301;
  rmtc::ActionReceipt receipt;receipt.tick=300;receipt.action="move";receipt.moved=false;receipt.error="E_BLOCKED";
  p.self.last_action=receipt;
  action=s.decide(p,199);
  require(action.type=="move"&&action.direction!="right"&&!s.plan().blocked_retries.empty(),
          "late reach retried the same contested cell instead of taking an available detour");
}

/** @brief 真实可见资源的额外偏好不施加到旧记忆；最快敌车路程确实进入争抢比较。 */
void check_visible_collection_and_race_diagnostics() {
  auto w = without_exploration();
  w.switch_margin = 0;
  rmtc::Stage2Strategy baseline(w);
  w.visible_resource_weight = 2.1;
  rmtc::Stage2Strategy visible_first(w);
  const auto old = frame(0, {4, 4}, {resource("old-high", 4, 5, 20)});
  baseline.decide(old); visible_first.decide(old);
  const auto now = frame(1, {4, 3}, {resource("visible", 5, 4)}, {9, 9}, 1);
  baseline.decide(now); visible_first.decide(now);
  require(baseline.plan().target_id == "old-high" && visible_first.plan().target_id == "visible",
          "visible resource multiplier did not distinguish current observations from memory");
  w = without_exploration();
  w.astar_resource_competition = true; w.switch_margin = 0;
  w.contested_resource_factor = 0.2;
  rmtc::Stage2Strategy race(w);
  race.decide(frame(0, {4, 4}, {resource("contested", 6, 4), resource("safer", 4, 6),
                               enemy(7, 4, 100, true)}));
  require(race.plan().target_id == "safer", "resource race ignored the faster visible opponent");
  for (const auto& item : race.plan().resource_races)
    if (item.at("id") == "contested")
      require(item.at("own_steps") == 2 && item.at("nearest_visible_enemy_steps") == 1 &&
                  item.at("race_factor").get<double>() < 0.6 && item.at("visible").get<bool>(),
              "race diagnostics do not match the actual known-terrain shortest paths");
}

/// 时效搜索在设定Tick前不干扰采集，启用边界与线性渐增必须连续且可复现。
void check_observation_schedule() {
  auto w = without_exploration();
  w.observation_age_weight = 0.75;
  w.observation_start_tick = 120;
  w.observation_ramp_ticks = 40;
  w.switch_margin = 0;
  w.patrol_value = 0;
  rmtc::Stage2Strategy ramp(w);
  w.observation_ramp_ticks = 0;
  rmtc::Stage2Strategy full(w);
  for (int tick : {0, 100, 120, 140, 160}) {
    auto p = tick == 0 ? frame(tick, {4, 4}, {}, {9, 9}, 10) : frame(tick, {1, 1}, {}, {9, 9}, 1);
    ramp.decide(p);
    full.decide(p);
    if (tick <= 120)
      require(ramp.plan().observation_age_points == 0,
              "age reward must be zero before start and at the start of a positive ramp");
    if (tick == 140)
      require(full.plan().observation_age_points > 0 &&
                  std::abs(ramp.plan().observation_age_points * 2 -
                           full.plan().observation_age_points) < 1e-9,
              "mid-ramp age reward must be exactly half under identical observations");
    if (tick == 160)
      require(
          std::abs(ramp.plan().observation_age_points - full.plan().observation_age_points) < 1e-9,
          "age reward must reach the configured strength at ramp completion");
  }
  w.observation_start_tick = -1;
  bool rejected = false;
  try {
    rmtc::Stage2Strategy invalid(w);
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  require(rejected, "negative observation schedule must fail before running a match");
}

/// 联合路线复用第一阶段库，但不能绕过任务饱和、敌车占位、已知首步和防御规则。
void check_joint_collection() {
  auto w = without_exploration();
  w.collection_mode = 1;
  w.joint_ticks = 80;
  rmtc::Stage2Strategy s(w);
  auto p =
      frame(0, {4, 4}, {resource("one", 5, 4), resource("two", 6, 4), resource("three", 7, 5)});
  require(s.decide(p).type == "move" && s.plan().joint_used && s.plan().joint_future_utility > 0,
          "joint route must expose its immediate leg and discounted future collection value");
  require(s.plan().resource_points == 2.5,
          "joint forecast must not inflate immediate resource score");
  p.tick = 1;
  p.visible_entities.erase(p.visible_entities.begin());
  s.decide(p);
  for (const auto& key : s.plan().joint_visit_order)
    require(key != "resource:id:one", "joint route retained an observed missing resource");
  s.reset();
  auto one = frame(0, {4, 4}, {resource("single", 5, 4)});
  require(s.decide(one).type == "move" && !s.plan().joint_planned,
          "single-resource case should use the cheap existing planner without a joint call");
  p.tick = 80;
  s.decide(p);
  require(!s.plan().joint_used, "joint opening window did not hand control back to baseline");
  s.reset();
  p = frame(0, {4, 4}, {resource("high", 4, 5, 20), enemy(6, 4)});
  p.self.hp = 15;
  require(s.decide(p).type == "defend",
          "joint route cannot override attack-before-movement survival");
  s.reset();
  p = frame(0, {0, 0}, {enemy(1, 0, 100, true), resource("blocked", 2, 0)}, {3, 1});
  require(s.decide(p).type == "wait", "joint estimate cannot move through a visible occupied cell");
  s.reset();
  p = frame(0, {1, 1}, {resource("near", 4, 4)}, {24, 24});
  s.decide(p);
  for (auto q : s.plan().route.path)
    require(std::abs(q.x - 1) <= 4 && std::abs(q.y - 1) <= 4,
            "joint execution leg must not include unknown cells beyond current memory");
  s.reset();
  p = frame(0, {4, 4}, {resource("capped", 5, 4)});
  p.score.resource_value = 120;
  require(s.decide(p).type == "wait", "joint route must stop creating saturated resource rewards");
  s.reset();
  p = frame(0, {4, 4}, {resource("late", 6, 4)});
  require(s.decide(p, 1).type == "wait",
          "joint forecast must respect actual remaining tick budget");
}

/// 时间插值有明确端点；争抢估计连续、可关闭，敌方绕墙距离不能用直线冒充。
void check_linear_collection_and_race() {
  auto w = without_exploration();
  w.opening_ticks = 120;
  w.opening_resource_weight = 1.3;
  w.resource_weight_end = 0.7;
  rmtc::Stage2Strategy desire(w);
  auto simple = frame(0, {4, 4}, {resource("desire", 5, 4)});
  for (const auto tick : {0, 60, 120, 200}) {
    simple.tick = tick;
    desire.decide(simple);
    const double expected = tick == 0 ? 1.3 : tick == 60 ? 1.0 : 0.7;
    require(std::abs(desire.plan().resource_weight - expected) < 1e-9 &&
                desire.plan().resource_points == 2.5,
            "resource desire schedule changed score or endpoint");
  }
  w.opening_resource_weight = w.resource_weight_end = 1;
  w.joint_linear_weights = true;
  w.joint_future_weight = 0.6;
  w.joint_future_weight_end = 0.1;
  w.joint_observation_offset = 8;
  w.joint_observation_offset_end = 20;
  require(std::abs(rmtc::collection_weights_at(w, 0).joint_future_weight - 0.6) < 1e-9,
          "linear start weight");
  require(std::abs(rmtc::collection_weights_at(w, 40).joint_future_weight - 0.35) < 1e-9 &&
              rmtc::collection_weights_at(w, 40).joint_observation_offset == 14,
          "linear midpoint weights");
  require(std::abs(rmtc::collection_weights_at(w, 100).joint_future_weight - 0.1) < 1e-9,
          "linear endpoint clamping");
  w.joint_linear_weights = false;
  require(rmtc::collection_weights_at(w, 40).joint_future_weight == 0.6,
          "disabled linear schedule changed fixed weight");
  require(rmtc::collection_linear(1, 2, 0, 0) == 2, "zero duration must not divide by zero");
  w.resource_mission_weight=1.2;w.resource_mission_weight_end=.6;
  require(rmtc::resource_mission_weight_at(w,100)==1.2,
          "disabled mission ramp must preserve original fixed weight");
  w.resource_mission_ramp_ticks=80;
  require(std::abs(rmtc::resource_mission_weight_at(w,0)-1.2)<1e-9&&
          std::abs(rmtc::resource_mission_weight_at(w,40)-.9)<1e-9&&
          std::abs(rmtc::resource_mission_weight_at(w,100)-.6)<1e-9,
          "collection mission preference must interpolate and saturate independently");
  w.resource_mission_weight=1.;w.resource_mission_weight_end=1.;w.resource_mission_ramp_ticks=0;
  require(rmtc::collection_race_factor(2, 6, 0, w) > rmtc::collection_race_factor(2, 2, 0, w) &&
              rmtc::collection_race_factor(2, 2, 0, w) > rmtc::collection_race_factor(6, 2, 0, w),
          "race ordering");
  require(rmtc::collection_race_factor(2, -1, 0, w) == 1,
          "unknown enemy route must not imply certain loss");
  w.competition_weight = w.competition_weight_end = 0;
  require(rmtc::collection_race_factor(8, 1, 0, w) == 1, "race strength zero");
  w.competition_weight = w.competition_weight_end = 1;
  w.astar_resource_competition = true;
  w.patrol_value = 0;
  w.contested_resource_factor = 0;
  rmtc::Stage2Strategy strategy(w);
  // 敌车看似只差2步，实际需绕墙6步；我们只需2步。保护车用于隔离战斗收益。
  auto p = frame(
      0, {1, 3},
      {resource("around-wall", 3, 3), enemy(5, 3, 100, true), wall(4, 2), wall(4, 3), wall(4, 4)},
      {9, 9});
  strategy.decide(p);
  require(strategy.plan().target_id == "around-wall" && strategy.plan().resource_points > 2,
          "enemy known-terrain detour was ignored");
}

/// 敌方预测仅使用连续可见帧；末局撤离提前于当前射程风险，低血留更多余量。
void check_motion_and_endgame_preservation() {
  auto w = without_exploration();
  w.endgame_preserve_weight = 1;
  w.opponent_motion_weight = 0.5;
  rmtc::Stage2Strategy strategy(w);
  auto p = frame(490, {4, 4}, {enemy(7, 4)});
  p.score.effective_damage = 500;
  require(strategy.decide(p, 8).type == "move" && strategy.plan().preserving_score &&
              strategy.plan().goal == "evade" && strategy.plan().escape_steps == 1 &&
              strategy.plan().retreat_lead_ticks == 9,
          "full damage must start leaving before enemy enters actual shooting range");
  strategy.reset();
  require(strategy.decide(p, 20).type == "wait" && !strategy.plan().preserving_score,
          "endgame rule must not claim its short retreat window lasts all match");
  strategy.reset();
  p.self.hp = 15;
  strategy.decide(p, 12);
  require(strategy.plan().preserving_score && strategy.plan().retreat_lead_ticks == 13,
          "low HP requires earlier reserve");
  strategy.reset();
  p.self.hp = 100;
  p.score.effective_damage = 100;
  strategy.decide(p, 8);
  require(!strategy.plan().preserving_score,
          "low damage progress must not masquerade as full score");
  {
    auto blocker_weights = without_exploration();
    blocker_weights.opponent_motion_weight = 1;
    blocker_weights.patrol_value = 0;
    rmtc::Stage2Strategy blocked_shot(blocker_weights);
    auto near = enemy(3, 1, 100, true);
    near.team = "C";
    auto scene = frame(0, {1, 1}, {near, enemy(4, 1)}, {7, 3});
    blocked_shot.decide(scene);
    const auto target = blocked_shot.plan().target;
    require(target && !(target->x == 2 && target->y == 1),
            "predicted firing goal ignored a nearer protected blocking robot");
  }
  strategy.reset();
  p = frame(0, {4, 4}, {enemy(6, 5)});
  p.visible_entities[0].id = "motion";
  strategy.decide(p);
  p.tick = 1;
  p.visible_entities = {enemy(6, 4)};
  p.visible_entities[0].id = "motion";
  require(strategy.decide(p).type == "attack" &&
              strategy.plan().opponent_motion[0]["extrapolated"] == true &&
              strategy.plan().opponent_motion[0]["tendency"] == "approaching",
          "motion extrapolation must not change current attack-before-movement aim");
  p.tick = 3;
  strategy.decide(p);
  require(strategy.plan().opponent_motion[0]["continuous_observation"] == false,
          "missed perception must reset motion confidence");
  strategy.reset();
  p = frame(0, {4, 4}, {enemy(6, 5)});
  p.visible_entities[0].id = "motion";
  strategy.decide(p);
  p.tick = 1;
  p.visible_entities = {enemy(6, 4), wall(6, 3)};
  p.visible_entities[0].id = "motion";
  strategy.decide(p);
  require(strategy.plan().opponent_motion[0]["extrapolated"] == false,
          "opponent prediction must not pass through known obstacle");
}

/// 第三方交火是推断；保留转火致死上界，多落点范围只取公开连续帧。
void check_opponent_intents() {
  auto b = enemy(4, 4);
  b.id = "B";
  auto c = enemy(6, 4);
  c.id = "C";
  c.team = "C";
  auto first = frame(0, {2, 4}, {b, c});
  first.event_text = "old event remains in accumulated bulletin";
  auto second = first;
  second.tick = 1;
  second.visible_entities[1].hp = 80;
  rmtc::MapMemory memory;
  memory.update(first);
  memory.update(second);
  std::vector<const rmtc::VisibleEntity*> visible{&second.visible_entities[0],
                                                  &second.visible_entities[1]};
  auto model = rmtc::assess_opponent_intents(second, first, std::nullopt, memory, visible, 2, 3);
  require(model.opponents[0].third_party_confidence > 0.5 &&
              model.opponents[0].third_party_confidence < 1,
          "third-party damage should create uncertainty rather than guaranteed free attack");
  for (const auto& e : model.opponents) {
    double mass = 0;
    for (const auto& p : e.next)
      mass += p.probability;
    require(std::abs(mass - 1) < 1e-9 && e.next.size() > 1,
            "live opponent range must be a normalized distribution");
  }
  auto announcement = second;
  announcement.event_text += "new event";
  auto event_model =
      rmtc::assess_opponent_intents(announcement, first, std::nullopt, memory, visible, 2, 3);
  require(event_model.opponents[0].third_party_confidence == 0,
          "changed bulletin must suspend damage attribution");
  auto blast = second;
  blast.visible_entities[1].hp = 65;
  std::vector<const rmtc::VisibleEntity*> blast_visible{&blast.visible_entities[0],
                                                        &blast.visible_entities[1]};
  auto blast_model =
      rmtc::assess_opponent_intents(blast, first, std::nullopt, memory, blast_visible, 2, 3);
  require(blast_model.opponents[0].third_party_confidence == 0,
          "35HP blast must not become normal-shot evidence");
  // 本车自己也可能造成掉血时，不能将伤害归给另一名敌人。
  auto ambiguous = second;
  ambiguous.self.position = {6, 2};
  auto before = first;
  before.self.position = {6, 2};
  rmtc::ActionReceipt receipt;
  receipt.tick = 0;
  receipt.action = "attack";
  ambiguous.self.last_action = receipt;
  auto uncertain =
      rmtc::assess_opponent_intents(ambiguous, before, std::nullopt, memory, visible, 2, 3);
  require(uncertain.opponents[0].third_party_confidence == 0,
          "own shot must not masquerade as third-party engagement");
  ambiguous.visible_entities[1].hp = 60;
  std::vector<const rmtc::VisibleEntity*> residual_visible{&ambiguous.visible_entities[0],
                                                           &ambiguous.visible_entities[1]};
  auto residual = rmtc::assess_opponent_intents(ambiguous, before, std::nullopt, memory,
                                                residual_visible, 2, 3);
  require(residual.opponents[0].third_party_confidence > 0,
          "40HP loss must retain another-shooter hypothesis");
  auto weights = without_exploration();
  weights.third_party_risk_discount = 1;
  weights.third_party_attack_bonus = 1;
  rmtc::Stage2Strategy strategy(weights);
  strategy.decide(first);
  second.self.hp = 15;
  require(strategy.decide(second).type == "defend" && strategy.plan().incoming_damage_bound == 20,
          "suspected third-party attack must not erase attack-before-movement death bound");
  // 已知墙不进入移动分布；墙后仍可能被标准子弹打到。
  second.visible_entities.push_back(wall(5, 4));
  memory.update(frame(2, {2, 4}, second.visible_entities));
  second.tick = 3;
  memory.update(second);
  visible = {&second.visible_entities[0], &second.visible_entities[1]};
  model = rmtc::assess_opponent_intents(second, std::nullopt, std::nullopt, memory, visible, 2, 3);
  for (const auto& p : model.opponents[0].next)
    require(!(p.position.x == 5 && p.position.y == 4), "forecast moved through known wall");
  require(model.exposure[4 * second.map.width + 6] > 0,
          "attack risk must not treat wall as bullet cover");
  // 两次同向才增强惯性；仍保留等待和转向。
  auto x = enemy(4, 4);
  x.id = "track";
  auto f0 = frame(0, {3, 6}, {x}, {13, 13});
  x.position = {5, 4};
  auto f1 = frame(1, {3, 6}, {x}, {13, 13});
  x.position = {6, 4};
  auto f2 = frame(2, {3, 6}, {x}, {13, 13});
  rmtc::MapMemory track;
  track.update(f0);
  track.update(f1);
  track.update(f2);
  visible = {&f2.visible_entities[0]};
  model = rmtc::assess_opponent_intents(f2, f1, f0, track, visible, 2, 3);
  double forward = 0, still = 0;
  for (const auto& p : model.opponents[0].next) {
    if (p.position.x == 7 && p.position.y == 4)
      forward = p.probability;
    if (p.position.x == 6 && p.position.y == 4)
      still = p.probability;
  }
  require(forward > still && still > 0, "consistent motion must retain alternative actions");
}

/// 断线重接不读取上一局目标；重复Tick返回缓存、旧Tick和旧场次不推进状态。
void check_lifecycle() {
  rmtc::Stage2Strategy s(without_exploration());
  auto p = frame(0, {4, 4}, {resource("first", 5, 4)});
  require(s.decide(p).direction == "right", "initial action");
  p.visible_entities = {resource("changed", 4, 5)};
  require(s.decide(p).direction == "right", "duplicate frame recomputed action");
  p.tick = 1;
  p.self.alive = false;
  require(s.decide(p).type == "wait", "dead robot must wait");
  require(s.decide(frame(0, {4, 4})).type == "wait", "old tick must not produce move");
  p.tick = 0;
  p.session_id = "new-session";
  p.self.alive = true;
  p.visible_entities.clear();
  require(s.decide(p).type == "wait" && !s.plan().target, "new session retained old resource");
  require(s.decide(frame(2, {4, 4}, {resource("late", 5, 4)})).type == "wait",
          "retired session accepted");
  s.reset();
  require(s.decide(frame(0, {4, 4}, {resource("no-budget", 5, 4)}), 0).type == "wait",
          "terminal state emitted action");
  auto w = without_exploration();
  w.travel_offset = 0;
  bool rejected = false;
  try {
    rmtc::Stage2Strategy invalid(w);
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  require(rejected, "invalid stage2 configuration must fail early");
}

}  // namespace

/// 短程采集前瞻不重复算首资源、不穿过未知或障碍、不突破真实分数和时限预算。
void check_resource_lookahead() {
  rmtc::Stage2Weights w;
  w.resource_lookahead_weight = 1;
  w.resource_lookahead_depth = 3;
  const rmtc::MapSize size{9, 9};
  const auto passable = [](rmtc::Position p) { return p.x < 7; };
  const auto distances = rmtc::goal_distance_field(size, {0, 0}, passable);
  rmtc::JointOpportunity a, b, c;
  a.key = "a"; a.position = {1, 0}; a.resource_points = 2.5; a.priority = 1;
  b.key = "b"; b.position = {2, 0}; b.resource_points = 2.5; b.priority = 0.9;
  c.key = "c"; c.position = {3, 0}; c.resource_points = 2.5; c.priority = 0.8;
  auto result = rmtc::resource_lookahead(size, {a,b,c}, distances, 30, 0, 1, 1, 500, w, passable);
  require(std::abs(result.at("a") - (2.5 / 5 + 0.7 * 2.5 / 6)) < 1e-9,
          "lookahead must count each future resource once at cumulative arrival distance");
  result = rmtc::resource_lookahead(size, {a,b,c}, distances, 2.5, 0, 1, 1, 500, w, passable);
  require(result.at("a") == 0, "first pickup exhausted score cap but future points remain");
  result = rmtc::resource_lookahead(size, {a,b,c}, distances, 30, 0, 1, 1, 1, w, passable);
  require(result.empty(), "lookahead promised resources after final tick");
  a.mission_points["collect"] = b.mission_points["collect"] = c.mission_points["collect"] = 3;
  result = rmtc::resource_lookahead(size, {a,b,c}, distances, 0, 3, 1, 1, 500, w, passable);
  require(result.at("a") == 0, "same mission credited again after its potential budget was exhausted");
  // 一整列障碍隔开两个资源；有坐标但没有已知路线时，后续收益必须为零。
  b.position = {8, 0}; a.mission_points.clear(); b.mission_points.clear();
  result = rmtc::resource_lookahead(size, {a,b}, distances, 30, 0, 1, 1, 500, w, passable);
  require(result.at("a") == 0, "lookahead crossed impassable/unknown terrain");
}

/// 同长度去资源的两条路线，优先选择首步能看到新区域的路线；不增加绕行步数。
void check_route_information() {
  auto w = without_exploration();
  w.route_information_weight = 1;
  w.information_per_cell = 0.1;
  rmtc::Stage2Strategy strategy(w);
  strategy.decide(frame(0, {1, 3}, {resource("target", 3, 3)}, {12, 12}, 2));
  const auto action = strategy.decide(frame(1, {1, 1}, {resource("target", 3, 3)}, {12, 12}, 2));
  require(strategy.plan().target_id == "target" && strategy.plan().route.path.size() == 5 &&
              action.type == "move" && action.direction == "right",
          "equal shortest path should observe new columns rather than revisit known rows");
}

/// 已确认冲突的落点暂时绕开；到期恢复可走，不污染地形，不把复活当作冲突。
void check_blocked_retry() {
  auto w = without_exploration();
  w.blocked_retry_ticks = 2;
  w.blocked_retry_min_failures = 1;
  rmtc::Stage2Strategy strategy(w);
  auto p = frame(0, {1, 1}, {resource("target", 3, 1)});
  require(strategy.decide(p).direction == "right", "blocked retry setup route");
  p.tick = 1;
  rmtc::ActionReceipt receipt;
  receipt.tick = 0; receipt.action = "move"; receipt.error = "E_BLOCKED";
  p.self.last_action = receipt;
  const auto action = strategy.decide(p);
  require(action.type == "move" && action.direction != "right",
          "repeated same failed first step instead of a temporary detour");
  require(strategy.plan().blocked_retries.size() == 1 &&
              strategy.plan().blocked_retries[0]["until_tick"] == 3,
          "temporary collision avoidance was not logged with its expiry Tick");
  p.tick = 3; p.self.last_action.reset();
  require(strategy.decide(p).direction == "right", "temporary block did not expire");
  strategy.reset(); p.tick = 0;
  strategy.decide(p);
  p.tick = 1; p.self.last_action = receipt; p.self.position_reason = "respawn";
  require(strategy.decide(p).direction == "right", "respawn normalized receipt created false wall");
}

/// 相同冲突时A与D错开重试；关闭开关时保持旧行为，避免对正常通行凭空引入等待。
void check_collision_retry_desynchronization() {
  auto w = without_exploration();
  w.blocked_retry_ticks = 2;
  w.blocked_retry_min_failures = 1;
  for (const std::string team : {"A", "D"}) {
    rmtc::Stage2Strategy strategy(w);
    auto p = frame(0, {1, 1}, {resource("target", 3, 1)});
    p.self.team = p.self.robot_id = team;
    require(strategy.decide(p).direction == "right", "collision setup must approach target");
    p.tick = 1;
    rmtc::ActionReceipt receipt;
    receipt.tick = 0; receipt.action = "move"; receipt.error = "E_BLOCKED";
    p.self.last_action = receipt;
    strategy.decide(p);
    p.tick = 3; p.self.last_action.reset();
    const auto retry = strategy.decide(p);
    require((retry.direction == "right") == (team == "A"),
            "different seats retried the same collision on the same Tick");
    p.tick = 6;
    require(strategy.decide(p).direction == "right" && strategy.plan().blocked_retries.empty(),
            "seat-staggered avoidance never expired");
  }
  w.blocked_retry_ticks = 0;
  rmtc::Stage2Strategy disabled(w);
  auto p = frame(0, {1, 1}, {resource("target", 3, 1)});
  disabled.decide(p);
  p.tick = 1;
  rmtc::ActionReceipt receipt;
  receipt.tick = 0; receipt.action = "move"; receipt.error = "E_BLOCKED";
  p.self.last_action = receipt;
  require(disabled.decide(p).direction == "right" && disabled.plan().blocked_retries.empty(),
          "disabled collision feedback changed baseline behaviour");
}

/// 连续三次失败才绕行；一两次正常争抢、成功移动及漏帧不能累计成死锁。
void check_persistent_collision_threshold() {
  auto w = without_exploration();
  w.blocked_retry_ticks = 2;
  w.blocked_retry_min_failures = 3;
  rmtc::Stage2Strategy strategy(w);
  auto p = frame(0, {1, 1}, {resource("target", 3, 1)});
  strategy.decide(p);
  for (int tick = 1; tick <= 3; ++tick) {
    p.tick = tick;
    rmtc::ActionReceipt receipt;
    receipt.tick = tick - 1; receipt.action = "move"; receipt.error = "E_BLOCKED";
    p.self.last_action = receipt;
    const auto action = strategy.decide(p);
    require((action.direction == "right") == (tick < 3),
            "short collision was abandoned or persistent collision was retried forever");
    require(strategy.plan().blocked_retry_streak == tick, "collision streak not consecutive");
  }
  p.tick = 5; p.self.last_action.reset();
  require(strategy.decide(p).direction == "right" && strategy.plan().blocked_retries.empty(),
          "persistent collision avoidance became a permanent wall");
  strategy.reset();
  p = frame(0, {1, 1}, {resource("target", 3, 1)});
  strategy.decide(p);
  p.tick = 1;
  rmtc::ActionReceipt receipt;
  receipt.tick = 0; receipt.action = "move"; receipt.error = "E_BLOCKED";
  p.self.last_action = receipt;
  strategy.decide(p);
  p.tick = 3; receipt.tick = 2; p.self.last_action = receipt;
  require(strategy.decide(p).direction == "right" && strategy.plan().blocked_retry_streak == 0,
          "missing observation falsely extended a collision streak");
}

/// 最后看见Tick决定扩散和可信度；确认旧格空了仍保留邻域线索，但不在可见空地放幽灵。
void check_enemy_sightings() {
  rmtc::MapMemory memory;
  auto p = frame(0, {4, 4}, {enemy(6, 4)}, {12, 12}, 2);
  memory.update(p);
  p = frame(1, {4, 4}, {}, {12, 12}, 2);
  memory.update(p);
  require(memory.entities().begin()->second.first_missing_tick.has_value(),
          "enemy sightings test must confirm old position empty");
  rmtc::EnemySightingsField field(p, memory, 6, 18);
  require(field.in_view({4, 4}, 2) == 0 && field.in_view({7, 4}, 0) > 0,
          "missing enemy should spread outside current view, not disappear or occupy old cell");
  require(std::abs(field.in_view({6, 6}, 20) - std::exp2(-1.0 / 6)) < 1e-9,
          "enemy search mass must decay according to last-seen tick");
  p.tick = 6;
  memory.update(p);
  rmtc::EnemySightingsField older(p, memory, 6, 18);
  require(std::abs(older.in_view({6, 6}, 20) - 0.5) < 1e-9,
          "half life should halve total search confidence");
  p.tick = 19;
  memory.update(p);
  rmtc::EnemySightingsField expired(p, memory, 6, 18);
  require(expired.in_view({6, 6}, 20) == 0, "expired enemy history still attracts search");
  memory = rmtc::MapMemory{};
  p = frame(0, {5, 4}, {enemy(6, 4), wall(7, 4)}, {12, 12}, 2);
  memory.update(p);
  p = frame(1, {4, 4}, {}, {12, 12}, 2);
  memory.update(p);
  rmtc::EnemySightingsField blocked(p, memory, 6, 18);
  require(blocked.in_view({7, 4}, 0) == 0, "enemy forecast treated a known wall as occupied");
  auto w = without_exploration();
  w.recent_enemy_search_weight = 1;
  rmtc::Stage2Strategy strategy(w);
  strategy.decide(frame(0, {4, 4}, {enemy(6, 4)}, {12, 12}, 2));
  p = frame(1, {4, 4}, {}, {12, 12}, 2);
  strategy.decide(p);
  require(strategy.plan().goal == "search_enemy", "recent enemy did not create a search goal");
  p.tick = 2; p.score.effective_damage = 500;
  require(strategy.decide(p).type == "wait", "saturated damage score still chases old enemies");
}

/// 执行独立场景；任意规则回归返回非零退出码供CTest发现。
int main() {
  try {
    check_resource_and_caps();
    check_exploration_and_memory();
    check_dynamic_missions();
    check_attack_rules();
    check_defense_and_occupancy();
    check_event_boundary();
    check_late_game_patrol();
    check_opening_resource_priority();
    check_adaptive_resource_economy();
    check_unproductive_pursuit();
    check_shared_pursuit_progress();
    check_goal_distance_field();
    check_observation_age();
    check_explicit_goal_phases();
    check_goal_switch_bonus_separation();
    check_phase_mission_preferences();
    check_resource_first_schedule();
    check_reach_finish_window();
    check_reach_finish_unknown_route();
    check_reach_finish_collision_backoff();
    check_visible_collection_and_race_diagnostics();
    check_observation_schedule();
    check_joint_collection();
    check_linear_collection_and_race();
    check_motion_and_endgame_preservation();
    check_opponent_intents();
    check_lifecycle();
    check_resource_lookahead();
    check_route_information();
    check_blocked_retry();
    check_collision_retry_desynchronization();
    check_persistent_collision_threshold();
    check_enemy_sightings();
    std::cout << "Stage2 goal/combat/memory/event/lifecycle checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
