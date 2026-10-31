/**
 * @file stage2_events_test.cpp
 * @brief 八类事件的结算边界、地图恢复、方向转换与战术回归；不调用API、不连接ROS。
 */
#include "rmtc_connection/ai_client.hpp"
#include "rmtc_connection/stage2_event_route.hpp"
#include "rmtc_connection/stage2_event_escape.hpp"
#include "rmtc_connection/stage2_fortress_policy.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
using namespace rmtc;
void require(bool condition, const char* reason) {
  if (!condition)
    throw std::runtime_error(reason);
}

/// 默认全可见小地图；裁剪半径可测试视野外恢复，真实位置永远取自当前公开帧。
PublicPerception frame(std::int64_t tick, Position p, int radius = 10) {
  PublicPerception f;
  f.session_id = "event-tests";
  f.tick = tick;
  f.map = {12, 12};
  f.vision_radius = radius;
  f.self.team = f.self.robot_id = "A";
  f.self.position = p;
  f.self.alive = true;
  f.self.hp = 100;
  f.event_text = "opaque public data interpreted by a model";
  return f;
}
VisibleEntity resource(Position p) {
  VisibleEntity e;
  e.id = "resource";
  e.type = "resource_normal";
  e.position = p;
  e.value = 10;
  return e;
}
VisibleEntity enemy(Position p, std::string team = "B") {
  VisibleEntity e;
  e.id = "robot_" + team;
  e.team = team;
  e.type = "robot";
  e.position = p;
  e.hp = 100;
  e.spawn_protected = false;
  e.defending = false;
  return e;
}
Stage2Weights isolated() {
  Stage2Weights w;
  w.acquisition.enabled=false;w.temporal.enabled=false;w.purpose.enabled=false; // 本组只校验事件结算。
  w.collection_mode = 0;
  w.information_per_cell = 0;
  w.recent_enemy_search_weight = 0;
  w.intent_chase_weight = w.intent_risk_weight = w.third_party_risk_discount =
      w.third_party_attack_bonus = 0;
  w.opening_resource_weight = w.resource_weight_end = 1;
  w.chase_stall_ticks = 0;
  return w;
}
Stage2EventConstraints packet(const PublicPerception& f, std::vector<Stage2EventEffect> effects) {
  Stage2EventConstraints p;
  p.session_id = f.session_id;
  p.source_text = f.event_text;
  p.source_tick = f.tick;
  p.begin_tick = 0;
  p.end_tick = 1000;
  p.events = std::move(effects);
  p.timeline = true;
  return p;
}
Stage2EventEffect event(std::string kind, int begin, int end) {
  Stage2EventEffect e;
  e.kind = std::move(kind);
  e.begin_tick = begin;
  e.end_tick = end;
  return e;
}
/// 真实成功移动的回执，复活和失败移动不允许借此获得Buff。
void moved(PublicPerception& f, std::int64_t tick) {
  ActionReceipt receipt;
  receipt.tick = tick;
  receipt.action = "move";
  receipt.moved = true;
  f.self.last_action = receipt;
}

void check_world_split() {
  MapMemory memory;
  Stage2Events events;
  auto f = frame(0, {3, 3}, 2);
  VisibleEntity wall;
  wall.type = "obstacle";
  wall.position = {4, 3};
  f.visible_entities = {wall};
  memory.update(f);
  events.observe(f, memory);
  auto e = event("world_split", 2, 5);
  e.cells = {{3, 3}, {4, 3}, {10, 3}};
  events.merge(packet(f, {e}), f, memory);
  require(memory.cells()[MapMemory::index({3, 3}, f.map)] == 0,
          "temporary wall started before activation");
  f = frame(2, {1, 1}, 1);
  memory.update(f);
  events.observe(f, memory);
  require(memory.cells()[MapMemory::index({3, 3}, f.map)] == 100, "temporary wall not added");
  require(events.terrain_at({3, 3}, 5, memory) == 0 && events.terrain_at({4, 3}, 5, memory) == 100,
          "future wall expiry erased permanent wall or retained a known temporary wall");
  f = frame(5, {1, 1}, 1);
  memory.update(f);
  events.observe(f, memory);
  require(memory.cells()[MapMemory::index({3, 3}, f.map)] == 0,
          "known free terrain not restored outside view");
  require(memory.cells()[MapMemory::index({4, 3}, f.map)] == 100,
          "original wall erased at event end");
  require(memory.cells()[MapMemory::index({10, 3}, f.map)] == -1,
          "unknown terrain falsely restored to free");

  // 激活时已看到的临时墙不能被记成永久墙；底图应取严格早于激活的观察。
  events.reset();
  memory = MapMemory{};
  f = frame(0, {3, 3}, 2);
  memory.update(f);
  events.observe(f, memory);
  e = event("world_split", 2, 5);
  e.cells = {{4, 3}};
  events.merge(packet(f, {e}), f, memory);
  f.tick = 2;
  f.visible_entities = {wall};
  memory.update(f);
  events.observe(f, memory);
  f = frame(5, {0, 0}, 0);
  memory.update(f);
  events.observe(f, memory);
  require(memory.cells()[MapMemory::index({4, 3}, f.map)] == 0,
          "activation-frame temporary wall became a permanent wall");
}

void check_illusion_and_late_notice() {
  Stage2Strategy s(isolated());
  auto f = frame(0, {3, 3});
  f.visible_entities = {resource({4, 3})};
  auto e = event("illusion", 0, 1);
  e.cells = {{3, 3}};
  e.effect_ticks = 4;
  auto p = packet(f, {e});
  auto action = s.decide(f, 100, &p);
  require(action.type == "move" && action.direction == "left" && s.plan().movement_inverted,
          "illusion must reverse MOVE command, not physical route");
  f.tick = 1;
  f.self.alive = false;
  s.decide(f);
  f.tick = 2;
  f.self.alive = true;
  f.self.position_reason = "respawn";
  action = s.decide(f);
  require(action.direction == "left", "respawn incorrectly removed illusion");
  f.tick = 4;
  action = s.decide(f);
  require(action.direction == "right" && !s.plan().movement_inverted, "illusion expiry not exact");
  Stage2Strategy late(isolated());
  f = frame(0, {3, 3});
  f.visible_entities = {resource({4, 3})};
  late.decide(f);
  f.tick = 2;
  p = packet(f, {e});
  action = late.decide(f, 100, &p);
  require(action.direction == "left",
          "late model result failed to recover activation from own history");
  Stage2Strategy attack(isolated());
  f = frame(0, {3, 3});
  f.visible_entities = {enemy({4, 3})};
  p = packet(f, {e});
  action = attack.decide(f, 100, &p);
  require(action.type == "attack" && action.direction == "right",
          "illusion must not reverse ATTACK");
}

void check_blast_and_angel() {
  Stage2Strategy s(isolated());
  auto f = frame(0, {3, 3});
  f.visible_entities = {resource({4, 3})};
  auto blast = event("explosion", 2, 3);
  blast.damage = 80;
  blast.cells = {{3, 3}, {4, 3}};
  auto p = packet(f, {blast});
  auto a = s.decide(f, 100, &p);
  require(a.type == "move" && s.plan().goal == "evade" && a.direction != "right",
          "blast must evacuate before move-first deadline");
  auto heal = event("angel", 0, 20);
  heal.points = {{4, 3}};
  heal.effect_ticks = 8;
  heal.heal_per_tick = 5;
  heal.hp_cap = 100;
  Stage2Strategy low(isolated());
  f = frame(0, {3, 3});
  f.self.hp = 30;
  p = packet(f, {heal});
  a = low.decide(f, 100, &p);
  require(a.type == "move" && a.direction == "right" && low.plan().goal == "event_angel",
          "low HP must value accessible healing");
  Stage2Strategy full(isolated());
  f.self.hp = 100;
  p = packet(f, {heal});
  require(full.decide(f, 100, &p).type == "wait", "full HP fabricated healing reward");
  auto opening_weights=isolated();opening_weights.opening_collection_ticks=100;
  opening_weights.events.heal_value=.1; // 此场景让近处可用回血的收益明确高于远处普通资源。
  Stage2Strategy opening(opening_weights);f.self.hp=30;f.visible_entities={resource({3,6})};
  p=packet(f,{heal});a=opening.decide(f,500,&p);
  require(opening.plan().opening_collection&&opening.plan().goal=="event_angel"&&
          a.type=="move"&&a.direction=="right","opening collection ignored a valuable refreshed mechanism");
  MapMemory memory;
  Stage2Events events;
  f = frame(0, {4, 3});
  memory.update(f);
  events.observe(f, memory);
  events.merge(packet(f, {heal}), f, memory);
  require(!events.buff("A", "angel", 0),
          "standing on newly created heal point incorrectly grants buff");
  f = frame(1, {3, 3});
  moved(f, 0);
  memory.update(f);
  events.observe(f, memory);
  f = frame(2, {4, 3});
  moved(f, 1);
  memory.update(f);
  events.observe(f, memory);
  require(events.buff("A", "angel", 2) && events.buff("A", "angel", 2)->until == 9,
          "successful entry did not grant absolute heal expiry");
}

/** @brief 无LLM时保留固定底图，新增墙只当临时覆盖；再观察恢复或晚到解释后正确撤销。 */
void check_observed_split_fallback() {
  const auto obstacle=[](Position p){VisibleEntity e;e.type="obstacle";e.position=p;return e;};
  MapMemory memory;Stage2Events events;
  auto f=frame(0,{2,2},2);f.visible_entities={obstacle({1,2})};
  memory.update(f);events.observe(f,memory);
  require(events.inferred_wall_cells().empty(),"initial static wall was called world split");
  f.tick=1;f.visible_entities.push_back(obstacle({3,2}));memory.update(f);events.observe(f,memory);
  require(events.inferred_wall_cells().size()==1&&events.terrain_at({3,2},1,memory)==100,
          "new wall without AI was not treated as temporary blocking evidence");
  require(events.describe(f).at("inferred_world_split_walls")[0].at("base_terrain")==0,
          "temporary wall destroyed fixed free terrain");
  f.tick=2;f.self.position={9,9};f.vision_radius=1;f.visible_entities.clear();
  memory.update(f);events.observe(f,memory);
  require(events.inferred_wall_cells().size()==1&&events.terrain_at({3,2},100,memory)==100,
          "unconfirmed wall was assigned an invented expiration");
  f.tick=3;f.self.position={2,2};f.vision_radius=2;f.visible_entities={obstacle({1,2})};
  memory.update(f);events.observe(f,memory);
  require(events.inferred_wall_cells().empty()&&events.terrain_at({3,2},3,memory)==0&&
          events.terrain_at({1,2},3,memory)==100,"observed recovery changed original wall or kept phantom wall");
  f.tick=4;f.visible_entities.push_back(obstacle({3,2}));memory.update(f);events.observe(f,memory);
  f.tick=5;f.self.position={9,9};f.vision_radius=1;f.visible_entities.clear();
  memory.update(f);events.observe(f,memory);
  auto split=event("world_split",4,8);split.cells={{3,2}};
  events.merge(packet(f,{split}),f,memory);
  require(events.inferred_wall_cells().empty()&&events.terrain_at({3,2},7,memory)==100,
          "late AI failed to take over inferred wall");
  f.tick=8;memory.update(f);events.observe(f,memory);
  require(events.terrain_at({3,2},8,memory)==0,"late explained event did not restore offscreen fixed terrain");
  f.session_id="new-static-map";f.tick=0;memory.update(f);events.observe(f,memory);
  require(events.inferred_wall_cells().empty(),"inferred walls leaked across sessions");
}

void check_fortress_and_range() {
  auto w = isolated();
  Stage2Strategy s(w);
  auto f = frame(0, {3, 3});
  auto fortress = event("unrivaled", 0, 20);
  fortress.cells = {{3, 3}, {4, 3}};
  fortress.stay_ticks = 2;
  fortress.attack_radius = 2;
  fortress.attack_damage = 35;
  fortress.defended_damage = 7;
  auto p = packet(f, {fortress});
  require(s.decide(f, 100, &p).type == "wait",
          "fortress stay should not require needless movement");
  f.tick = 1;
  s.decide(f);
  f.tick = 2;
  f.visible_entities = {enemy({5, 5}), enemy({1, 5}, "C")};
  auto action = s.decide(f);
  require(action.type == "attack", "unrivaled must attack diagonal targets in square range");
  double gain = 0;
  for (const auto& score : s.plan().action_scores)
    if (score["action"] == "attack")
      gain = score["utility"];
  require(gain > 0.5, "unrivaled attack did not aggregate both targets");
  Stage2Strategy seeking(w);
  f = frame(0, {1, 3});
  p = packet(f, {fortress});
  require(seeking.decide(f, 100, &p).type == "move" && seeking.plan().goal == "event_unrivaled",
          "fortress did not attract deliberate capture");
}

/** @brief 补到达任务仍让路于必须提前完成的爆炸撤离，并按幻术还原实际移动方向。 */
void check_reach_finish_event_safety() {
  auto w=isolated();w.reach_attack_pause_ticks=0;
  auto f=frame(300,{3,3});
  MissionInfo mission;mission.id="reach";mission.type="reach";mission.position=Position{8,3};
  f.missions={mission};
  auto blast=event("explosion",301,302);blast.damage=100;
  blast.cells={{3,3},{4,3},{5,3}};
  auto p=packet(f,{blast});
  Stage2Strategy escape(w);
  auto action=escape.decide(f,200,&p);
  require(escape.plan().goal=="evade" && action.type=="move" && action.direction!="right" &&
          !escape.plan().reach_finish.value("progress_selected",false),
          "late reach progress overrode imminent explosion evacuation");
  auto illusion=event("illusion",300,301);illusion.cells={{3,3}};illusion.effect_ticks=5;
  p=packet(f,{illusion});
  Stage2Strategy inverted(w);
  action=inverted.decide(f,200,&p);
  require(inverted.plan().goal=="mission" && action.type=="move" && action.direction=="left" &&
          inverted.plan().reach_finish.value("progress_selected",false),
          "late reach navigation failed to invert the submitted move under illusion");
}

/** @brief 落后的堡垒争夺及时放弃，原地驻留的奖励不能把车继续留在对手强化射程内。 */
void check_fortress_contest_and_withdraw() {
  auto w=isolated();w.rollout.weight=0;
  Stage2Strategy s(w);
  auto f=frame(0,{2,3});f.visible_entities={enemy({4,3})};
  auto fortress=event("unrivaled",0,40);fortress.stay_ticks=10;
  fortress.attack_radius=2;fortress.attack_damage=50;fortress.defended_damage=10;
  fortress.cells={{3,3},{3,4},{4,3},{4,4}};
  auto p=packet(f,{fortress});s.decide(f,500,&p);
  require(s.plan().fortress["contests"].empty(),
          "one glimpse of a passer-by was treated as committed fortress occupation");
  for(int tick=1;tick<=3;++tick){f.tick=tick;s.decide(f,500-tick);}
  require(!s.plan().fortress["contests"].empty() && s.plan().goal!="event_unrivaled",
          "a clearly losing fortress approach remained an attractive capture goal");
  require(!s.plan().fortress["pending_threats"].empty() &&
          s.plan().fortress["buff_threats"].empty() && s.plan().incoming_damage_bound==20,
          "projected fortress fire was confused with a granted buff or physical damage");
  f.tick=4;f.self.position={3,4};
  const auto action=s.decide(f,496);
  require(s.plan().goal=="evade" && s.plan().target_id.rfind("fortress-withdraw:",0)==0 &&
          action.type=="move" && s.plan().target &&
          !Stage2Events::contains(fortress.cells,*s.plan().target),
          "losing fortress dwell did not withdraw to a safe external destination");
  f.tick=5;f.visible_entities={enemy({8,8})};s.decide(f,495);
  require(!s.plan().fortress.contains("withdraw"),
          "enemy departure left a permanent fictional owner on the fortress");

  // 对手已经强化但离开很远，不能把仍可使用的堡垒视为名额被抢光。
  MapMemory memory;Stage2Events events;
  memory.update(f);events.observe(f,memory);
  auto holder=fortress;holder.actor="B";holder.begin_tick=5;holder.end_tick=20;
  events.merge(packet(f,{fortress,holder}),f,memory);
  const auto& global=events.effects().at(std::find_if(events.effects().begin(),events.effects().end(),
      [](const auto& item){return item.second.actor.empty();})->first);
  require(!fortress_contest(events,global,f,{3,4},0,w.events).losing,
          "a distant buff holder incorrectly excluded all later captures");
  f.tick=6;f.visible_entities={enemy({5,4})};memory.update(f);events.observe(f,memory);
  require(fortress_contest(events,global,f,{3,4},0,w.events).losing,
          "a nearby buff holder lost its danger merely by stepping outside the fortress");
  f.tick=20;memory.update(f);events.observe(f,memory);
  require(!fortress_contest(events,global,f,{3,4},0,w.events).losing,
          "a departed holder kept its special danger after buff expiration");
}

/** @brief 堡垒按每车分别驻留累计；确认无双后危险加权只影响软决策，真实伤害仍为50。 */
void check_multi_car_fortress_and_buff_danger() {
  auto fortress=event("unrivaled",0,40);fortress.cells={{3,3},{4,4}};
  fortress.stay_ticks=10;fortress.attack_radius=2;fortress.attack_damage=50;fortress.defended_damage=10;
  MapMemory memory;Stage2Events events;
  auto f=frame(0,{3,3});f.visible_entities={enemy({4,4})};
  memory.update(f);events.observe(f,memory);events.merge(packet(f,{fortress}),f,memory);
  for(int tick=1;tick<=10;++tick){f.tick=tick;memory.update(f);events.observe(f,memory);}
  require(events.buff("A","unrivaled",10)&&events.buff("B","unrivaled",10),
          "fortress incorrectly became exclusive to the first vehicle");

  auto w=isolated();w.rollout.weight=0;w.events.fortress_contest_enabled=false;
  w.events.unrivaled_risk_multiplier=1;
  Stage2Strategy ordinary(w);w.events.unrivaled_risk_multiplier=4;Stage2Strategy cautious(w);
  f=frame(0,{3,3});f.visible_entities={enemy({4,4})};
  auto holder=fortress;holder.actor="B";holder.begin_tick=0;holder.end_tick=3;
  auto p=packet(f,{holder});ordinary.decide(f,500,&p);cautious.decide(f,500,&p);
  const auto wait_utility=[](const Stage2Plan& plan){
    for(const auto& score:plan.action_scores)if(score["action"]=="wait")return score["utility"].get<double>();
    throw std::runtime_error("missing WAIT action");
  };
  require(cautious.plan().incoming_damage_bound==50 && ordinary.plan().incoming_damage_bound==50 &&
          wait_utility(cautious.plan())<wait_utility(ordinary.plan())-0.1,
          "buff risk multiplier inflated physical damage or failed to increase danger");
  f.tick=3;cautious.decide(f,497);
  require(cautious.plan().incoming_damage_bound==0 && cautious.plan().fortress["buff_threats"].empty(),
          "expired unrivaled buff kept its enhanced diagonal threat");
}

void check_pepper_purity_pan() {
  auto w = isolated();
  w.events.pepper_cost = 0.02;
  Stage2Strategy pepper(w);
  auto f = frame(0, {3, 3});
  f.visible_entities = {enemy({4, 6}), enemy({7, 3}, "C")};
  auto e = event("pepper", 0, 20);
  e.points = {{4, 3}};
  e.damage = 50;
  auto p = packet(f, {e});
  auto action = pepper.decide(f, 100, &p);
  require(action.type == "move" && action.direction == "right",
          "valuable pepper trigger treated as an impassable wall");
  require(pepper.plan().combat_points == 0,
          "environmental pepper damage incorrectly counted as personal combat score");
  auto pur = event("purity", 0, 20);
  pur.reflection_damage = 20;
  Stage2Strategy purity(w);
  f = frame(0, {3, 3});
  auto opponent = enemy({4, 3});
  opponent.defending = true;
  f.visible_entities = {opponent};
  p = packet(f, {pur});
  require(purity.decide(f, 100, &p).type != "attack",
          "purity blindly attacked a likely defending opponent");
  auto pan = event("pan", 0, 20);
  pan.movement_window_ticks = 10;
  pan.damage_multiplier = 2;
  MapMemory memory;
  Stage2Events events;
  f = frame(0, {3, 3});
  memory.update(f);
  events.observe(f, memory);
  events.merge(packet(f, {pan}), f, memory);
  require(!events.buff("A", "pan", 0), "pan vulnerability applied before a successful move");
  f = frame(1, {4, 3});
  moved(f, 0);
  memory.update(f);
  events.observe(f, memory);
  require(events.buff("A", "pan", 1) && events.buff("A", "pan", 1)->until == 20,
          "pan successful move not tracked");
  Stage2Strategy vulnerable(w);
  auto state = pan;
  state.actor = "A";
  f = frame(1, {3, 3});
  f.visible_entities = {enemy({4, 3})};
  p = packet(f, {state});
  vulnerable.decide(f, 100, &p);
  require(vulnerable.plan().incoming_damage_bound == 40,
          "vulnerability not reflected in current damage bound");
  f.tick = 20;
  vulnerable.decide(f);
  require(vulnerable.plan().incoming_damage_bound == 20,
          "vulnerability survived its absolute expiry");
}

/// 验证软风险允许绕路，也允许在唯一通路中通过；它不是新增墙体。
void check_soft_route_and_distinct_regions() {
  const MapSize size{5, 3};
  std::vector<double> risk(15, 0);
  risk[7] = 100;
  auto passable = [&](Position p) { return p.x >= 0 && p.x < 5 && p.y >= 0 && p.y < 3; };
  auto route = event_route(size, {0, 1}, {4, 1}, risk, passable);
  require(route.success() && route.cost == 6, "weighted event route did not detour high risk");
  auto corridor = [&](Position p) { return passable(p) && p.y == 1; };
  route = event_route(size, {0, 1}, {4, 1}, risk, corridor);
  require(route.success() && route.cost == 4, "soft event risk incorrectly blocked unique route");
  MapMemory memory;
  Stage2Events events;
  auto f = frame(0, {3, 3});
  memory.update(f);
  events.observe(f, memory);
  auto a = event("world_split", 2, 5);
  a.cells = {{1, 1}};
  auto b = a;
  b.cells = {{2, 2}};
  events.merge(packet(f, {a, b}), f, memory);
  require(events.effects().size() == 2, "same-time distinct event areas overwrote each other");
  f.tick = 2;
  memory.update(f);
  events.observe(f, memory);
  require(memory.cells()[MapMemory::index({1, 1}, f.map)] == 100 &&
              memory.cells()[MapMemory::index({2, 2}, f.map)] == 100,
          "concurrent temporary areas not both applied");
}

/// 失败移动不触发入点/易伤，敌方无双会扩大当前伤害上界；重叠墙逐一到期。
void check_failed_moves_enemy_buff_and_overlap() {
  auto f = frame(0, {3, 3});
  MapMemory memory;
  Stage2Events events;
  auto pan = event("pan", 0, 20);
  pan.movement_window_ticks = 10;
  pan.damage_multiplier = 2;
  auto angel = event("angel", 0, 20);
  angel.points = {{3, 3}};
  angel.effect_ticks = 8;
  angel.heal_per_tick = 5;
  memory.update(f);
  events.observe(f, memory);
  events.merge(packet(f, {pan, angel}), f, memory);
  f.tick = 1;
  moved(f, 0);
  f.self.last_action->moved = false;
  memory.update(f);
  events.observe(f, memory);
  require(!events.buff("A", "pan", 1) && !events.buff("A", "angel", 1),
          "failed MOVE fabricated entry buffs");
  auto fortress = event("unrivaled", 0, 20);
  fortress.actor = "B";
  fortress.attack_radius = 2;
  fortress.attack_damage = 35;
  fortress.defended_damage = 7;
  Stage2Strategy strategy(isolated());
  f = frame(0, {3, 3});
  f.visible_entities = {enemy({5, 5})};
  auto p = packet(f, {fortress});
  strategy.decide(f, 100, &p);
  require(strategy.plan().incoming_damage_bound == 35,
          "enemy square-range buff missing from damage bound");
  MapMemory terrain;
  Stage2Events walls;
  f = frame(0, {3, 3}, 2);
  terrain.update(f);
  walls.observe(f, terrain);
  auto a = event("world_split", 2, 5);
  a.cells = {{3, 3}};
  auto b = a;
  b.begin_tick = 3;
  b.end_tick = 6;
  walls.merge(packet(f, {a, b}), f, terrain);
  for (int tick = 1; tick <= 6; ++tick) {
    f = frame(tick, {1, 1}, 1);
    terrain.update(f);
    walls.observe(f, terrain);
    if (tick == 5)
      require(terrain.cells()[MapMemory::index({3, 3}, f.map)] == 100,
              "one ending event erased another active wall");
  }
  require(terrain.cells()[MapMemory::index({3, 3}, f.map)] == 0,
          "overlapping temporary walls failed to restore terrain");
}

void check_typed_protocol() {
  auto f = frame(10, {3, 3});
  Json raw{{"events", Json::array({{{"kind", "explosion"},
                                    {"begin_tick", 15},
                                    {"end_tick", 16},
                                    {"rectangles", {{1, 1, 2, 2}}},
                                    {"damage", 35}},
                                   {{"kind", "world_split"},
                                    {"begin_tick", 20},
                                    {"end_tick", 30},
                                    {"cross_point", {4, 5}}},
                                   {{"kind", "angel"},
                                    {"begin_tick", 20},
                                    {"end_tick", 30},
                                    {"points", {{3, 4}, {5, 6}}},
                                    {"effect_ticks", 8},
                                    {"heal_per_tick", 5}}})}};
  auto p = decode_ai_event_timeline(raw, f, 10, 100);
  require(p.timeline && p.events.size() == 3 && p.events[1].cells.size() == 23,
          "typed event/cross geometry validation");
  auto roundtrip = decode_ai_event_timeline(encode_ai_event_timeline(p), f, 10, 100);
  require(roundtrip.events[2].points.size() == 2, "discrete heal points became rectangle");
  raw["events"][0]["damage"] = -10;
  bool rejected = false;
  try {
    decode_ai_event_timeline(raw, f, 10, 100);
  } catch (const std::exception&) {
    rejected = true;
  }
  require(rejected, "negative environmental damage accepted");
}

/// 目标在安全区也不能把首步危险退款；狭窄通道在墙生成前必须等待或撤回。
void check_route_hazard_and_escape_distance() {
  auto w=isolated();w.rollout.weight=0;
  w.frontier_resource_weight=w.frontier_mission_weight=0;
  auto f=frame(9,{3,3});f.visible_entities.push_back(resource({5,3}));
  for(int x=0;x<12;++x)for(int y:{2,4}){
    VisibleEntity wall;wall.type="obstacle";wall.position={x,y};f.visible_entities.push_back(wall);
  }
  auto hazard=event("world_split",10,20);hazard.cells={{4,3}};
  auto plan=packet(f,{hazard});Stage2Strategy strategy(w);
  auto action=strategy.decide(f,100,&plan);
  require(action.type!="move"||action.direction!="right","goal route refunded imminent wall risk");
  std::vector<Position> square;
  for(int y=2;y<=6;++y)for(int x=2;x<=6;++x)square.push_back({x,y});
  EventEscapeField field({9,9},square,[](Position){return true;});
  require(field.distance({4,4})==3&&field.distance({2,4})==1&&field.distance({1,4})==0,
          "escape BFS did not measure distance to actual safe boundary");
  EventEscapeField unknown({9,9},square,[](Position p){return p.x>=2&&p.x<=6&&p.y>=2&&p.y<=6;});
  require(unknown.distance({4,4})==-1,"unknown terrain became a fictional escape route");
}
}  // namespace

/// 纯离线事件规则检查；任何失败返回非零，不会触发真实模型调用。
int main() {
  try {
    check_world_split();
    check_observed_split_fallback();
    check_illusion_and_late_notice();
    check_blast_and_angel();
    check_fortress_and_range();
    check_reach_finish_event_safety();
    check_fortress_contest_and_withdraw();
    check_multi_car_fortress_and_buff_danger();
    check_pepper_purity_pan();
    check_soft_route_and_distinct_regions();
    check_failed_moves_enemy_buff_and_overlap();
    check_typed_protocol();
    check_route_hazard_and_escape_distance();
    std::cout << "Eight-event terrain/buff/tactical/protocol checks passed\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
