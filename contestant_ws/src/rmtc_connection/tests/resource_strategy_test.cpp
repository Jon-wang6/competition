/**
 * @file resource_strategy_test.cpp
 * @brief 验证多资源整体顺序、旧最近策略、绕障碍、动态重规划和连续采集。
 *
 * 场景只向策略提供当前视野内实体，防止测试无意中依赖全图资源信息。
 * 不初始化ROS或控制真实比赛；独立小场景执行器核对每次移动是否合法。
 */
#include "rmtc_connection/resource_strategy.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

/** @brief 检查条件，失败时抛出说明；Release构建也保留检查。 */
void require(bool condition, const std::string& message) {
  if (!condition)
    throw std::runtime_error(message);
}

/** @brief 构造测试资源；仅位置与类型/价值用于规划，ID用于跨帧关联。 */
rmtc::VisibleEntity resource(const std::string& id, int x, int y, double value = 10) {
  rmtc::VisibleEntity entity;
  entity.id = id;
  entity.type = value >= 20 ? "resource_high" : "resource_normal";
  entity.position = {x, y};
  entity.value = value;
  return entity;
}

/** @brief 构造测试障碍或敌车；敌车带稳定身份，便于验证历史敌人不永久封路。 */
rmtc::VisibleEntity blocker(int x, int y, bool robot = false) {
  rmtc::VisibleEntity entity;
  entity.type = robot ? "robot" : "obstacle";
  entity.position = {x, y};
  if (robot) {
    entity.id = "enemy-B";
    entity.team = "B";
    entity.robot_id = "B";
  }
  return entity;
}

/** @brief 从测试场景生成一帧视野裁剪后的公开数据，场景其余部分不交给策略。 */
rmtc::PublicPerception observe(std::int64_t tick, rmtc::Position position,
                               const std::vector<rmtc::VisibleEntity>& scene, int radius = 4,
                               rmtc::MapSize map = {9, 9}) {
  rmtc::PublicPerception frame;
  frame.session_id = "resource-test";
  frame.tick = tick;
  frame.map = map;
  frame.vision_radius = radius;
  frame.self.team = "A";
  frame.self.robot_id = "A";
  frame.self.alive = true;
  frame.self.hp = 100;
  frame.self.position = position;
  for (const auto& entity : scene) {
    if (std::abs(entity.position.x - position.x) <= radius &&
        std::abs(entity.position.y - position.y) <= radius)
      frame.visible_entities.push_back(entity);
  }
  return frame;
}

/** @brief 检查几何更近但绕墙更远的资源不会错误压过真正的最短可达目标。 */
void check_distance_choice() {
  rmtc::ResourceStrategy strategy(rmtc::ResourceOrdering::Tour);
  std::vector<rmtc::VisibleEntity> scene{resource("near-behind-wall", 2, 0, 20),
                                         resource("short-route", 0, 3)};
  for (int y = 0; y < 4; ++y)
    scene.push_back(blocker(1, y));
  const auto action = strategy.decide(observe(0, {0, 0}, scene));
  require(strategy.plan().target && strategy.plan().target->id == "short-route" &&
              strategy.plan().route.cost == 3 && action.direction == "down",
          "selected geometric distance instead of A* distance");

  strategy.reset();
  scene = {resource("normal", 2, 1), resource("high", 1, 2, 20)};
  require(strategy.decide(observe(0, {1, 1}, scene)).direction == "down" &&
              strategy.plan().target->id == "high",
          "equal-distance value preference failed");
  scene[0].value = 30;
  strategy.decide(observe(1, {1, 1}, scene));
  require(strategy.plan().target->id == "high", "equal-distance target oscillated");
}

/** @brief 新障碍出现或移动失败后按实际坐标重新规划，不盲目前进旧路径下标。 */
void check_replanning() {
  rmtc::ResourceStrategy strategy(rmtc::ResourceOrdering::Tour);
  std::vector<rmtc::VisibleEntity> scene{resource("target", 3, 1)};
  require(strategy.decide(observe(0, {1, 1}, scene)).direction == "right",
          "initial resource route incorrect");
  scene.push_back(blocker(2, 1));
  const auto action = strategy.decide(observe(1, {1, 1}, scene));
  require(action.type == "move" && action.direction != "right" && strategy.plan().route.cost == 4,
          "new wall did not trigger detour");
  auto frame = observe(2, {1, 1}, scene);
  rmtc::ActionReceipt failed;
  failed.tick = 1;
  failed.action = "move";
  failed.moved = false;
  frame.self.last_action = failed;
  const auto retry = strategy.decide(frame);
  require(retry.direction == action.direction &&
              strategy.plan().route.path.front() == rmtc::planning::GridIndex{1, 1},
          "failed movement incorrectly advanced path");

  strategy.reset();
  scene = {resource("corridor-target", 2, 0), blocker(1, 0, true)};
  require(strategy.decide(observe(0, {0, 0}, scene, 4, {3, 1})).type == "wait",
          "walked through visible enemy");
  scene.pop_back();
  require(strategy.decide(observe(1, {0, 0}, scene, 4, {3, 1})).direction == "right",
          "historical enemy remained a permanent obstacle");
}

/** @brief 追踪视野外资源，确认消失后换目标；重复/旧Tick和旧场次不污染计划。 */
void check_memory_and_sessions() {
  rmtc::ResourceStrategy strategy(rmtc::ResourceOrdering::Tour);
  const std::vector<rmtc::VisibleEntity> scene{resource("remember-me", 4, 1)};
  strategy.decide(observe(0, {1, 1}, scene));
  strategy.decide(observe(1, {1, 5}, scene, 1));
  require(strategy.plan().target && strategy.plan().target->id == "remember-me" &&
              strategy.plan().last_seen_tick == 0 && strategy.plan().route.cost == 7,
          "out-of-view resource forgotten");
  require(strategy.decide(observe(2, {1, 1}, {})).type == "move" && !strategy.plan().target &&
              strategy.plan().state == rmtc::ResourceState::Exploring,
          "confirmed missing resource remained a target");
  auto frame = observe(3, {1, 1}, {resource("new-target", 2, 1)});
  const auto action = strategy.decide(frame);
  frame.visible_entities.clear();
  require(strategy.decide(frame).direction == action.direction &&
              strategy.plan().target->id == "new-target",
          "duplicate Tick replaced resource plan");
  frame.tick = 2;
  require(strategy.decide(frame).type == "wait" && strategy.plan().target->id == "new-target",
          "old Tick corrupted latest plan");
  const auto old = observe(4, {1, 1}, scene);
  frame.tick = 0;
  frame.session_id = "new-match";
  require(strategy.decide(frame).type == "move" && !strategy.plan().target &&
              strategy.memory().entities().empty(),
          "new session retained previous resource");
  require(strategy.decide(old).type == "wait" && !strategy.plan().target,
          "retired session restored a resource");
}

/** @brief 本车阵亡时等待且保留未证伪记忆；资源就在脚下时等待裁判自动采集。 */
void check_wait_states() {
  rmtc::ResourceStrategy strategy(rmtc::ResourceOrdering::Tour);
  auto frame = observe(0, {1, 1}, {resource("same-cell", 1, 1)});
  require(strategy.decide(frame).type == "wait" &&
              strategy.plan().state == rmtc::ResourceState::AtTarget &&
              strategy.plan().route.cost == 0,
          "same-cell resource was not handled safely");
  frame.tick = 1;
  frame.self.alive = false;
  frame.visible_entities.clear();
  require(
      strategy.decide(frame).type == "wait" && strategy.plan().state == rmtc::ResourceState::Dead,
      "dead robot attempted resource movement");
  for (const auto& [key, record] : strategy.memory().entities()) {
    (void)key;
    require(!record.first_missing_tick, "death incorrectly marked resource missing");
  }
  frame.tick = 2;
  frame.self.alive = true;
  require(strategy.decide(frame).type == "move" &&
              strategy.plan().state == rmtc::ResourceState::Exploring,
          "empty partial map did not explore");
  strategy.reset();
  require(strategy.decide(observe(0, {1, 1}, {}, 4, {5, 5})).type == "wait" &&
              strategy.plan().state == rmtc::ResourceState::NoReachableResource,
          "fully known empty map did not wait");
}

/** @brief 执行小型独立采集场景，验证逐格绕墙、到达目标、排除已采集资源并连续换目标。 */
void check_collection_loop() {
  rmtc::ResourceStrategy strategy(rmtc::ResourceOrdering::Tour);
  std::vector<rmtc::VisibleEntity> scene{resource("r1", 2, 1), resource("r2", 4, 2),
                                         resource("h1", 5, 5, 20), blocker(3, 1)};
  rmtc::Position position{1, 1};
  double value = 0;
  int collected = 0;
  for (int tick = 0; tick < 30 && collected < 3; ++tick) {
    const auto action = strategy.decide(observe(tick, position, scene));
    if (action.type == "move") {
      if (action.direction == "right")
        ++position.x;
      else if (action.direction == "left")
        --position.x;
      else if (action.direction == "down")
        ++position.y;
      else if (action.direction == "up")
        --position.y;
      else
        throw std::runtime_error("invalid movement direction");
    }
    require(position.x >= 0 && position.y >= 0 && position.x < 9 && position.y < 9,
            "collection route left map");
    for (const auto& entity : scene) {
      require(entity.type != "obstacle" || entity.position.x != position.x ||
                  entity.position.y != position.y,
              "collection route collided with wall");
    }
    scene.erase(std::remove_if(scene.begin(), scene.end(),
                               [&](const auto& entity) {
                                 if (entity.type == "obstacle" || entity.position.x != position.x ||
                                     entity.position.y != position.y)
                                   return false;
                                 value += entity.value.value_or(0);
                                 ++collected;
                                 return true;
                               }),
                scene.end());
  }
  require(collected == 3 && value == 40,
          "strategy failed to consecutively collect the three resources");
  strategy.decide(observe(31, position, scene));
  require(!strategy.plan().target, "collected resources caused a repeated visit");
}

/**
 * @brief 用同一条完全可见的走廊对比两种策略，直接核对采完全部资源的移动步数。
 * 起点4，资源在2、5、8：最近策略先去5，需要10步；整体策略先去2，只需8步。
 * 测试不向策略传递视野外资源，移动后模拟裁判的自动采集与下一帧资源消失。
 */
void check_tour_vs_nearest() {
  std::vector<int> steps;
  for (const auto ordering : {rmtc::ResourceOrdering::Tour, rmtc::ResourceOrdering::Nearest}) {
    rmtc::ResourceStrategy strategy(ordering);
    std::vector<rmtc::VisibleEntity> scene{resource("left", 2, 0), resource("near", 5, 0),
                                           resource("right", 8, 0)};
    rmtc::Position position{4, 0};
    int moves = 0;
    for (int tick = 0; tick < 25 && !scene.empty(); ++tick) {
      const auto action = strategy.decide(observe(tick, position, scene, 4, {9, 1}));
      if (tick == 0) {
        require(strategy.plan().target->id ==
                    (ordering == rmtc::ResourceOrdering::Tour ? "left" : "near"),
                "strategy did not call the selected resource ordering algorithm");
        if (ordering == rmtc::ResourceOrdering::Tour)
          require(strategy.plan().total_cost == 8 && strategy.plan().visit_order.size() == 3,
                  "multi-resource order not exposed in plan snapshot");
      }
      if (action.type == "move") {
        require(action.direction == "left" || action.direction == "right",
                "corridor route moved vertically");
        position.x += action.direction == "right" ? 1 : -1;
        ++moves;
      }
      require(position.x >= 0 && position.x < 9, "comparison route left map");
      scene.erase(
          std::remove_if(scene.begin(), scene.end(),
                         [&](const auto& entity) { return entity.position.x == position.x; }),
          scene.end());
    }
    require(scene.empty(), "comparison strategy failed to collect all resources");
    steps.push_back(moves);
  }
  require(steps == std::vector<int>{8, 10},
          "whole-route optimizer did not remove the known greedy detour");
  std::cout << "same 3 resources: tour=8 moves, nearest=10 moves\n";
}

/**
 * @brief 按公开练习模板构造18×18离线场景，对比两种策略在局部视野下的完整采集。
 * 这些位置仅属于测试执行器，策略每帧只拿到9×9视野；不读取裁判配置文件。
 * 检查整局路线不越界、不穿墙、100 Tick内收齐16资源，并报告各自移动步数。
 * 在线新信息可能改变后续最优顺序，因此不声称任意隐藏地图上必定优于最近策略。
 */
void check_practice_scenario() {
  for (const auto ordering : {rmtc::ResourceOrdering::Tour, rmtc::ResourceOrdering::Nearest}) {
    std::vector<rmtc::VisibleEntity> scene;
    const std::vector<rmtc::Position> normal{{2, 1},  {15, 1},  {1, 15}, {16, 15}, {4, 2}, {13, 2},
                                             {2, 13}, {15, 13}, {4, 8},  {13, 9},  {8, 4}, {9, 13}};
    const std::vector<rmtc::Position> high{{5, 5}, {12, 5}, {5, 12}, {12, 12}};
    int id = 0;
    for (const auto p : normal)
      scene.push_back(resource("practice_" + std::to_string(id++), p.x, p.y));
    for (const auto p : high)
      scene.push_back(resource("practice_" + std::to_string(id++), p.x, p.y, 20));
    for (const auto p :
         std::vector<rmtc::Position>{{6, 3}, {6, 4}, {11, 13}, {11, 14}, {3, 11}, {14, 6}})
      scene.push_back(blocker(p.x, p.y));
    rmtc::ResourceStrategy strategy(ordering);
    rmtc::Position position{1, 1};
    int collected = 0, moves = 0;
    double value = 0;
    for (int tick = 0; tick < 100 && collected < 16; ++tick) {
      const auto action = strategy.decide(observe(tick, position, scene, 4, {18, 18}));
      if (action.type == "move") {
        if (action.direction == "left")
          --position.x;
        else if (action.direction == "right")
          ++position.x;
        else if (action.direction == "up")
          --position.y;
        else if (action.direction == "down")
          ++position.y;
        else
          throw std::runtime_error("invalid practice movement");
        ++moves;
      }
      require(position.x >= 0 && position.y >= 0 && position.x < 18 && position.y < 18,
              "practice route left map");
      for (const auto& entity : scene)
        require(entity.type != "obstacle" || entity.position.x != position.x ||
                    entity.position.y != position.y,
                "practice route crossed obstacle");
      scene.erase(std::remove_if(scene.begin(), scene.end(),
                                 [&](const auto& entity) {
                                   if (entity.type == "obstacle" ||
                                       entity.position.x != position.x ||
                                       entity.position.y != position.y)
                                     return false;
                                   ++collected;
                                   value += entity.value.value_or(0);
                                   return true;
                                 }),
                  scene.end());
    }
    require(collected == 16 && value == 200,
            "practice scenario did not collect all resources within 100 Tick");
    std::cout << "partial-vision practice "
              << (ordering == rmtc::ResourceOrdering::Tour ? "tour" : "nearest") << ": " << moves
              << " moves, " << value << " resource value\n";
  }
}

/** @brief 运行所有离线策略验证；失败输出原因并返回1，不创建任何ROS对象。 */
int main() {
  try {
    check_distance_choice();
    check_replanning();
    check_memory_and_sessions();
    check_wait_states();
    check_collection_loop();
    check_tour_vs_nearest();
    check_practice_scenario();
    std::cout
        << "resource tracking: A* distance, replanning, memory and consecutive collection passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
