/**
 * @file stage2_frontier_value_test.cpp
 * @brief 检查未知资源估计的时间折扣、任务缺口、计分封顶与已消失资源去重。
 */
#include "rmtc_connection/stage2_frontier_value.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
namespace {
using namespace rmtc;
void require(bool v, const char* why) {
  if (!v)
    throw std::runtime_error(why);
}
PublicPerception frame(int tick = 0, int radius = 1) {
  PublicPerception f;
  f.session_id = "frontier-tests";
  f.tick = tick;
  f.map = {12, 12};
  f.vision_radius = radius;
  f.self.team = "A";
  f.self.position = {0, 0};
  f.self.hp = 100;
  f.self.alive = true;
  return f;
}
void check() {
  auto f = frame();
  MapMemory memory;
  memory.update(f);
  Stage2Weights w;
  w.frontier_resource_weight=0;w.frontier_mission_weight=0;w.frontier_resource_half_life=80;
  require(FrontierOpportunityValue(f, memory, w, 0, 0).extra(20) == 0,
          "off switch changed original exploration");
  w.frontier_resource_weight = 1;
  const double initial = FrontierOpportunityValue(f, memory, w, 0, 0).extra(10);
  f.tick = 80;
  const double later = FrontierOpportunityValue(f, memory, w, 0, 0).extra(10);
  require(initial > 0 && std::abs(later * 2 - initial) < 1e-9,
          "unseen resource prior did not halve at configured half-life");
  f.score.resource_value = 120;
  require(FrontierOpportunityValue(f, memory, w, 0, 0).extra(20) == 0,
          "resource score saturation still produced fake points");
  w.frontier_resource_weight = 0;
  w.frontier_mission_weight = 1;
  MissionInfo m;
  m.id = "normal-task";
  m.type = "normal_resources";
  m.target = 5;
  f.missions = {m};
  const double near = FrontierOpportunityValue(f, memory, w, 4, 0).extra(1),
               far = FrontierOpportunityValue(f, memory, w, 0, 0).extra(1);
  require(near > far && far > 0,
          "collection task close to completion did not increase useful frontier value");
  f.score.mission_completed_ids = {m.id};
  require(FrontierOpportunityValue(f, memory, w, 4, 0).extra(20) == 0,
          "completed task rewarded again");
  auto all = frame(0, 12);
  MapMemory known;
  known.update(all);
  require(FrontierOpportunityValue(all, known, w, 0, 0).extra(20) == 0,
          "known terrain manufactured resource discovery");
  f = frame();
  VisibleEntity r;
  r.type = "resource_normal";
  r.id = "seen-once";
  r.position = {1, 0};
  r.value = 10;
  f.visible_entities = {r};
  memory = MapMemory{};
  memory.update(f);
  f.tick = 1;
  f.visible_entities.clear();
  memory.update(f);
  const auto detail = FrontierOpportunityValue(f, memory, w, 0, 0).describe();
  require(detail["unseen_value_upper_bound"] == 190,
          "consumed known resource was counted as unseen again");
  require(FrontierOpportunityValue(f, memory, w, 0, 0).extra(-1) == 0,
          "negative observation count yielded exploration reward");
}
}  // namespace
int main() {
  try {
    check();
    std::cout << "Frontier opportunity checks passed\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
