/**
 * @file stage2_rollout_test.cpp
 * @brief 以固定联合动作检查推演的真实结算顺序，并检查搜索预算、根动作公平性和关闭行为。
 */
#include "rmtc_connection/stage2_rollout.hpp"
#include <iostream>
#include <stdexcept>
namespace rmtc {
Json rollout_test_step(const PublicPerception&, const std::vector<Stage2EventEffect>&,
                       const std::vector<Action>&,bool collection_rewards=true);
}
namespace {
using namespace rmtc;
void require(bool v, const char* reason) {
  if (!v)
    throw std::runtime_error(reason);
}
PublicPerception frame(Position p = {3, 3}) {
  PublicPerception f;
  f.session_id = "rollout-tests";
  f.event_text = "model validated data";
  f.map = {12, 12};
  f.vision_radius = 10;
  f.self.team = "A";
  f.self.robot_id = "A";
  f.self.position = p;
  f.self.hp = 100;
  f.self.alive = true;
  return f;
}
VisibleEntity enemy(Position p, std::string team = "B", double hp = 100) {
  VisibleEntity e;
  e.type = "robot";
  e.id = team;
  e.team = team;
  e.position = p;
  e.hp = hp;
  return e;
}
Stage2EventEffect event(std::string kind) {
  Stage2EventEffect e;
  e.kind = kind;
  e.begin_tick = 0;
  e.end_tick = 20;
  return e;
}
void physics() {
  auto collected=frame();VisibleEntity pickup;pickup.type="resource_normal";pickup.value=10;pickup.position={4,3};
  collected.visible_entities={pickup};
  const auto full=rollout_test_step(collected,{},{{"move","right"}},true);
  const auto auxiliary=rollout_test_step(collected,{},{{"move","right"}},false);
  require(full.at("resource")==10&&auxiliary.at("resource")==10,"bridge changed resource physics");
  require(full.at("reward").get<double>()>2&&auxiliary.at("reward").get<double>()==0,
          "bridge counted collection reward a second time");
  auto f = frame();
  f.visible_entities = {enemy({3, 1})};
  auto r = rollout_test_step(f, {}, {{"move", "right"}, {"attack", "down"}});
  require(r["robots"][0]["hp"] == 80 && r["robots"][0]["position"] == Json({4, 3}),
          "MOVE incorrectly evaded current-Tick attack");
  VisibleEntity wall;
  wall.type = "obstacle";
  wall.position = {3, 2};
  f.visible_entities.push_back(wall);
  r = rollout_test_step(f, {}, {{"attack", "up"}, {"wait", ""}});
  require(r["robots"][1]["hp"] == 80, "wall incorrectly blocked a bullet");
  r = rollout_test_step(f, {}, {{"move", "up"}, {"wait", ""}});
  require(!r["moved"].get<bool>(), "MOVE passed through known wall");
  f = frame({1, 1});
  f.visible_entities = {enemy({3, 1}), enemy({2, 2}, "C")};
  r = rollout_test_step(f, {}, {{"move", "right"}, {"move", "left"}, {"move", "up"}});
  require(r["robots"][0]["position"] == Json({1, 1}) &&
              r["robots"][1]["position"] == Json({3, 1}) &&
              r["robots"][2]["position"] == Json({2, 2}),
          "three-body contested movement did not all fail");
  f = frame({1, 1});
  f.visible_entities = {enemy({2, 1})};
  r = rollout_test_step(f, {}, {{"move", "right"}, {"move", "right"}});
  require(r["robots"][0]["position"] == Json({1, 1}) && r["robots"][1]["position"] == Json({3, 1}),
          "followed a robot leaving an occupied cell");
  f = frame();
  f.self.hp = 20;
  f.visible_entities = {enemy({3, 1}, "B", 20)};
  r = rollout_test_step(f, {}, {{"attack", "up"}, {"attack", "down"}});
  require(!r["robots"][0]["alive"].get<bool>() && !r["robots"][1]["alive"].get<bool>(),
          "simultaneous attacks were applied sequentially");
}
void events() {
  auto f = frame();
  f.visible_entities = {enemy({3, 1})};
  auto pur = event("purity");
  pur.reflection_damage = 12;
  auto r = rollout_test_step(f, {pur}, {{"attack", "up"}, {"defend", ""}});
  require(r["robots"][0]["hp"] == 88 && r["robots"][1]["hp"] == 100 && r["damage"] == 0,
          "purity reflected damage became enemy score or normal hit");
  auto aoe = event("unrivaled");
  aoe.actor = "B";
  aoe.attack_radius = 2;
  aoe.attack_damage = 35;
  aoe.defended_damage = 7;
  f.visible_entities = {enemy({5, 5})};
  r = rollout_test_step(f, {aoe}, {{"defend", ""}, {"attack", "up"}});
  require(r["robots"][0]["hp"] == 93, "AOE diagonal or dedicated defended damage incorrect");
  auto pan = event("pan");
  pan.movement_window_ticks = 10;
  pan.damage_multiplier = 2;
  f.visible_entities = {enemy({3, 1})};
  r = rollout_test_step(f, {pan}, {{"move", "right"}, {"attack", "down"}});
  require(r["robots"][0]["hp"] == 80 && r["robots"][0]["pan_until"] == 20,
          "new movement vulnerability applied retroactively to current battle");
  pan.actor = "A";
  r = rollout_test_step(f, {pan}, {{"defend", ""}, {"attack", "down"}});
  require(r["robots"][0]["hp"] == 80, "existing vulnerability did not multiply defended damage");
  auto heal = event("angel");
  heal.points = {{4, 3}};
  heal.effect_ticks = 8;
  heal.heal_per_tick = 5;
  f.self.hp = 30;
  r = rollout_test_step(f, {heal}, {{"move", "right"}, {"attack", "down"}});
  require(r["robots"][0]["hp"] == 15, "entry healing did not occur after surviving battle");
  f.self.hp = 20;
  r = rollout_test_step(f, {heal}, {{"move", "right"}, {"attack", "down"}});
  require(!r["robots"][0]["alive"].get<bool>() && !r["moved"].get<bool>(),
          "healing revived a robot killed before movement");
  auto pepper = event("pepper");
  pepper.points = {{4, 3}};
  pepper.damage = 30;
  f = frame();
  f.visible_entities = {enemy({4, 6})};
  r = rollout_test_step(f, {pepper}, {{"move", "right"}, {"wait", ""}});
  require(r["robots"][0]["hp"] == 100 && r["robots"][1]["hp"] == 70 && r["damage"] == 0,
          "pepper intersection immunity or environmental scoring incorrect");
  auto blast = event("explosion");
  blast.cells = {{3, 3}};
  blast.damage = 35;
  f = frame();
  r = rollout_test_step(f, {blast}, {{"move", "right"}});
  require(r["robots"][0]["hp"] == 65 && r["moved"].get<bool>(),
          "pre-movement explosion incorrectly tested destination");
}
void search() {
  auto f = frame();
  f.visible_entities = {enemy({5, 3}), enemy({3, 5}, "C")};
  MapMemory memory;
  memory.update(f);
  Stage2Events events;
  events.observe(f, memory);
  Stage2Weights w;
  RolloutWeights c;
  std::vector<RolloutRoot> roots{{Action::wait(), 0},
                                 {{"defend", ""}, -0.1},
                                 {{"attack", "right"}, 0.2},
                                 {Action::move("left"), 0.05}};
  auto r = tactical_rollout(f, memory, events, {}, w, c, roots, Action::wait(), {}, 0, 0, 0, 500);
  require(!r.ran && r.action.type == "wait", "disabled rollout modified old decisions");
  c.weight = 0.5;
  c.start_tick = 0;
  c.horizon = 3;
  c.beam_width = 4;
  c.scenarios = 6;
  c.budget_ms = 100;
  c.validate();
  r = tactical_rollout(f, memory, events, {}, w, c, roots, Action::wait(), {}, 0, 0, 0, 500);
  require(r.ran && r.completed_depth == 3 && r.roots.size() == 4 && r.transitions > 100,
          "full-depth search did not evaluate every root action");
  auto reverse = roots;
  std::reverse(reverse.begin(), reverse.end());
  const auto again =
      tactical_rollout(f, memory, events, {}, w, c, reverse, Action::wait(), {}, 0, 0, 0, 500);
  require(r.action.type == again.action.type && r.action.direction == again.action.direction &&
              r.roots == again.roots,
          "root enumeration order changed the completed comparison");
  c.budget_ms = 1;
  c.horizon = 7;
  c.beam_width = 16;
  c.scenarios = 16;
  r = tactical_rollout(f, memory, events, {}, w, c, roots, Action::wait(), {}, 0, 0, 0, 500);
  require(r.completed_depth >= 1 && (!r.changed || r.completed_depth >= 2),
          "partial layer was used as a complete decision");
  c.weight = -1;
  bool rejected = false;
  try {
    c.validate();
  } catch (const std::exception&) {
    rejected = true;
  }
  require(rejected, "negative rollout weight accepted");
}
}  // namespace
int main() {
  try {
    physics();
    events();
    search();
    std::cout << "Rollout order/events/fair roots/budget checks passed\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
