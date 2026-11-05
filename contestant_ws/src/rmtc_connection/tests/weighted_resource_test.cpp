/**
 * @file weighted_resource_test.cpp
 * @brief 验证加权目标选择、剩余Tick限制及三种策略在相同局部视野下的表现。
 *
 * 场景由离线测试执行器持有；策略只收到当帧视野内实体，不读取裁判配置。
 * 可用--csv 路径导出固定场景/权重网格的完整结果，不启动ROS或比赛程序。
 *
 * @details 模块协作与维护约定
 * 验证对象：resource_strategy.hpp。
 * 运行约定：由独立测试可执行文件/CTest执行，断言失败用于定位回归，不是正式选手的逐帧逻辑。
 * 结果边界：通过表示这些构造场景满足约束，不证明所有随机地图均最优或稳定达到某个总分。
 */
#include "rmtc_connection/resource_strategy.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

using rmtc::ResourceOrdering;
using rmtc::ResourceStrategy;
using rmtc::planning::ResourceWeights;

/** @brief Release构建同样检查条件，失败时抛出可读说明。 */
void require(bool condition, const std::string& message) {
  if (!condition)
    throw std::runtime_error(message);
}

/** @brief 断言非法输入被拒绝，避免NaN、负权重等使候选比较失效。 */
void rejects(const std::function<void()>& operation) {
  bool failed = false;
  try {
    operation();
  } catch (const std::invalid_argument&) {
    failed = true;
  }
  require(failed, "invalid weighted resource input accepted");
}

/** @brief 构造测试资源，正常/高价值类型和单个value保持一致。 */
rmtc::VisibleEntity resource(const std::string& id, int x, int y, double value = 10) {
  rmtc::VisibleEntity entity;
  entity.id = id;
  entity.type = value >= 20 ? "resource_high" : "resource_normal";
  entity.position = {x, y};
  entity.value = value;
  return entity;
}

/** @brief 构造障碍或当前可见敌车，二者都不能被当作自由通道。 */
rmtc::VisibleEntity blocker(int x, int y, bool enemy = false) {
  rmtc::VisibleEntity entity;
  entity.type = enemy ? "robot" : "obstacle";
  entity.position = {x, y};
  if (enemy) {
    entity.id = "B";
    entity.team = "B";
    entity.robot_id = "B";
  }
  return entity;
}

/** @brief 只把以当前位置为中心的方形视野交给策略，地图其余内容留在测试执行器。 */
rmtc::PublicPerception observe(std::int64_t tick, rmtc::Position position,
                               const std::vector<rmtc::VisibleEntity>& scene,
                               rmtc::MapSize map = {18, 18}, int radius = 4) {
  rmtc::PublicPerception frame;
  frame.session_id = "weighted-offline";
  frame.tick = tick;
  frame.map = map;
  frame.vision_radius = radius;
  frame.self.team = "A";
  frame.self.robot_id = "A";
  frame.self.alive = true;
  frame.self.hp = 100;
  frame.self.position = position;
  for (const auto& entity : scene)
    if (std::abs(entity.position.x - position.x) <= radius &&
        std::abs(entity.position.y - position.y) <= radius)
      frame.visible_entities.push_back(entity);
  return frame;
}

/** @brief 检查边缘/价值权重能分别改变真实选出的目标，而不仅是公式数值。 */
void check_weight_effects() {
  const auto frame = observe(0, {3, 7}, {resource("near", 5, 7), resource("edge", 1, 5)});
  ResourceStrategy nearest(ResourceOrdering::Weighted, {0, 0});
  nearest.decide(frame, 100);
  require(nearest.plan().target->id == "near", "zero edge/value weights did not select nearest");
  ResourceStrategy edge(ResourceOrdering::Weighted, {1, 0});
  edge.decide(frame, 100);
  require(edge.plan().target->id == "edge" && edge.plan().weighted_cost->edge_distance == 1 &&
              edge.plan().weighted_cost->total == 5,
          "edge coefficient did not favor edge resource");
  auto value_frame = observe(0, {1, 1}, {resource("normal", 2, 1), resource("high", 4, 1, 20)});
  ResourceStrategy no_bonus(ResourceOrdering::Weighted, {0, 0});
  no_bonus.decide(value_frame, 100);
  require(no_bonus.plan().target->id == "normal", "distance-only comparison incorrect");
  ResourceStrategy value(ResourceOrdering::Weighted, {0, 3, 1, 0});
  value.decide(value_frame, 100);
  require(value.plan().target->id == "high" && value.plan().weighted_cost->value_units == 2 &&
              value.plan().weighted_cost->total == -3,
          "value bonus did not select higher-value resource");
  ResourceStrategy limited(ResourceOrdering::Weighted, {0, 3});
  limited.decide(value_frame, 2);
  require(limited.plan().target->id == "normal" && limited.plan().budget_filtered == 1,
          "high-value bonus bypassed remaining-Tick constraint");
  ResourceStrategy too_late(ResourceOrdering::Weighted, {0, 3});
  require(too_late.decide(observe(0, {1, 1}, {resource("high", 4, 1, 20)}), 1).type == "wait" &&
              too_late.plan().state == rmtc::ResourceState::NoResourceWithinBudget,
          "unreachable-in-time target did not wait with explicit budget state");
  ResourceStrategy explore_instead(ResourceOrdering::Weighted, {0, 3});
  require(
      explore_instead.decide(observe(0, {1, 1}, {resource("high", 4, 1, 20)}), 2).type == "move" &&
          !explore_instead.plan().target && explore_instead.plan().budget_filtered == 1 &&
          explore_instead.plan().state == rmtc::ResourceState::Exploring,
      "late known resource prevented nearby exploration for new opportunities");
  const auto score = rmtc::planning::score_resource_candidate(2, {5, 7}, 18, 18, 10, {1, 2});
  require(score.edge_distance == 5 && score.edge_cost == 5 && score.value_reward == 2 &&
              score.total == 5,
          "weighted breakdown incorrect");
  require(rmtc::planning::score_resource_candidate(1, {4, 3}, 5, 10, 10, {1, 0}).edge_distance == 0,
          "non-square map boundary distance incorrect");
}

/** @brief 核对A*绕路、脚下资源、死亡、旧记忆和可见敌人等实际控制边界。 */
void check_safety_and_memory() {
  std::vector<rmtc::VisibleEntity> scene{resource("behind-wall", 2, 0, 20),
                                         resource("reachable", 0, 3)};
  for (int y = 0; y < 4; ++y)
    scene.push_back(blocker(1, y));
  ResourceStrategy wall(ResourceOrdering::Weighted, {0, 0});
  wall.decide(observe(0, {0, 0}, scene, {9, 9}), 4);
  require(wall.plan().target->id == "reachable" && wall.plan().route.cost == 3 &&
              wall.plan().budget_filtered == 1,
          "deadline filter used geometric distance instead of A* detour");
  ResourceStrategy foot(ResourceOrdering::Weighted, {0, 100});
  require(
      foot.decide(observe(0, {0, 0}, {resource("foot", 0, 0), resource("far-high", 2, 0, 20)}), 10)
                  .type == "wait" &&
          foot.plan().target->id == "foot",
      "weights caused departure from uncollected resource under robot");
  ResourceStrategy no_window(ResourceOrdering::Weighted);
  require(no_window.decide(observe(0, {0, 0}, {resource("foot", 0, 0)}), 0).type == "wait" &&
              !no_window.plan().target,
          "same-cell collection ignored need for one action window");
  ResourceStrategy enemy(ResourceOrdering::Weighted);
  require(enemy.decide(observe(0, {0, 0}, {resource("r", 2, 0), blocker(1, 0, true)}, {3, 1}), 10)
                  .type == "wait",
          "weighted route crossed visible enemy");
  require(enemy.decide(observe(1, {0, 0}, {resource("r", 2, 0)}, {3, 1}), 9).direction == "right",
          "departed enemy stayed blocked");
  ResourceStrategy history(ResourceOrdering::Weighted, {0, 2, 1, 0});
  history.decide(observe(0, {1, 1}, {resource("remember", 4, 1)}, {9, 9}), 100);
  const auto old = observe(1, {1, 5}, {}, {9, 9}, 1);
  history.decide(old, 99);
  require(history.plan().target->id == "remember" && history.plan().last_seen_tick == 0 &&
              history.plan().weighted_cost->total == 5,
          "old resource was penalized or forgotten");
  auto duplicate = old;
  duplicate.visible_entities.clear();
  const auto cached = history.decide(duplicate, 99);
  require(cached.type == "move", "duplicate frame lost cached action");
  require(history.decide(observe(2, {1, 1}, {}, {9, 9}), 98).type == "move" &&
              !history.plan().target && history.plan().state == rmtc::ResourceState::Exploring,
          "confirmed missing resource remained weighted target");
  auto dead = observe(3, {1, 1}, {resource("r", 2, 1)}, {9, 9});
  dead.self.alive = false;
  require(
      history.decide(dead, 97).type == "wait" && history.plan().state == rmtc::ResourceState::Dead,
      "dead robot attempted weighted movement");
}

/** @brief 检查系数、坐标、价值与剩余Tick的非法输入及浮点溢出会被明确拒绝。 */
void check_bad_inputs() {
  for (const auto& weights :
       std::vector<ResourceWeights>{{-1, 0},
                                    {0, -1},
                                    {std::numeric_limits<double>::quiet_NaN(), 0},
                                    {0, std::numeric_limits<double>::infinity()}})
    rejects([&] { ResourceStrategy bad(ResourceOrdering::Weighted, weights); });
  rejects([] { (void)rmtc::planning::score_resource_candidate(1, {1, 1}, 0, 2, 10, {}); });
  rejects([] { (void)rmtc::planning::score_resource_candidate(1, {-1, 0}, 2, 2, 10, {}); });
  rejects([] { (void)rmtc::planning::score_resource_candidate(-1, {0, 0}, 2, 2, 10, {}); });
  rejects([] { (void)rmtc::planning::score_resource_candidate(1, {0, 0}, 2, 2, -10, {}); });
  rejects([] {
    (void)rmtc::planning::score_resource_candidate(1, {5, 5}, 18, 18, 10,
                                                   {std::numeric_limits<double>::max(), 0});
  });
  rejects([] {
    ResourceStrategy strategy(ResourceOrdering::Weighted);
    strategy.decide(observe(0, {0, 0}, {}), -1);
  });
  for (int field = 0; field < 3; ++field) {
    ResourceWeights invalid;
    if (field == 0)
      invalid.peek_reward = -1;
    if (field == 1)
      invalid.peek_detour_limit = -1;
    if (field == 2)
      invalid.peek_patch_limit = -1;
    rejects([&] { ResourceStrategy strategy(ResourceOrdering::Weighted, invalid); });
  }
}

/**
 * @brief 回归检查：小正数不能靠“探索点近”抢走资源，收益超过成本后才允许主动探索。
 * 两帧视野覆盖走廊x=0..12，第二帧在x=8，已知资源在x=4。
 * 探索向右走n步恰好揭开n格，代价为n×(1-0.2K)；K=5恰好收支相抵。
 * 固定全部系数，不依赖用户正在调整的默认值；这里只验证同一局面的决策阈值。
 */
void check_exploration_profit_gate() {
  const auto scene = std::vector<rmtc::VisibleEntity>{resource("known", 4, 0)};
  for (const double unknown_value : {0.0, 0.00001, 0.001, 0.1, 1.0, 5.0, 5.01, 10.0}) {
    ResourceStrategy strategy(ResourceOrdering::Weighted, {0.5, 2, 1, unknown_value, 20, 0, 4});
    strategy.decide(observe(0, {4, 0}, scene, {20, 1}), 100);
    const auto action = strategy.decide(observe(1, {8, 0}, scene, {20, 1}), 99);
    const bool should_explore = unknown_value > 5;
    require((strategy.plan().state == rmtc::ResourceState::Exploring) == should_explore,
            "exploration profit gate incorrect for unknown_value=" + std::to_string(unknown_value));
    require(action.direction == (should_explore ? "right" : "left"),
            "coefficient sweep did not change actual movement at the profit threshold");
    if (!should_explore)
      require(strategy.plan().target && strategy.plan().target->id == "known",
              "unprofitable exploration replaced the known resource");
    std::cout << "profit gate unknown_value=" << unknown_value << ": "
              << (should_explore ? "explore" : "collect") << '\n';

    // K=0和很小的正数只抑制主动竞争，不能使没有已知资源的机器人停在原地。
    ResourceStrategy fallback(ResourceOrdering::Weighted, {0.5, 2, 1, unknown_value, 20, 0, 4});
    require(fallback.decide(observe(0, {0, 0}, {}, {20, 1}), 100).direction == "right" &&
                fallback.plan().state == rmtc::ResourceState::Exploring,
            "profit gate disabled no-resource exploration fallback");
  }

  // 探索收益已经为正仍不够：若采资源的代价更低，必须继续采资源。
  ResourceStrategy valuable(ResourceOrdering::Weighted, {0.5, 2, 1, 6, 20, 0, 4});
  valuable.decide(observe(0, {8, 0}, {resource("forward", 10, 0)}, {20, 1}), 100);
  require(valuable.plan().target && valuable.plan().target->id == "forward",
          "profitable exploration overrode a better actual resource");
}

/** @brief 核对真实资源和探索使用同一尺度，增大未知价值可主动探索，末段保留采集时间。 */
void check_joint_exploration() {
  using namespace rmtc::planning;
  const ResourceWeights joint_weights{0.5, 2, 1, 1, 20, 0, 4, 0, 2};  // 等值未知格基线，补看另测。
  const auto resource_score = score_resource_candidate(4, {2, 2}, 20, 20, 10, joint_weights, 0);
  const auto explore_score = score_resource_candidate(1, {2, 2}, 20, 20, 0, joint_weights, 9);
  require(resource_score.total == 3 && std::abs(explore_score.total - 0.2) < 1e-12,
          "one unknown cell did not represent one unit of virtual resource value");
  ResourceWeights distance_weights;
  distance_weights.distance = 3;
  require(score_resource_candidate(2, {0, 0}, 20, 20, 10, distance_weights).distance_cost == 6,
          "configurable A* distance coefficient ignored");
  require(exploration_reward_scale(2, 13, 20) == 0.5 && exploration_reward_scale(2, 3, 20) == 0 &&
              exploration_reward_scale(2, std::nullopt, 20) == 1 &&
              exploration_reward_scale(2, 13, 0) == 1,
          "remaining-window exploration discount incorrect");
  const auto discounted = score_resource_candidate(4, {2, 2}, 20, 20, 10, joint_weights, 9, 0.5);
  require(discounted.value_reward == 2 && std::abs(discounted.exploration_reward - 0.9) < 1e-12,
          "late-game discount incorrectly reduced actual resource value");

  // 两帧公开视野覆盖x=0..12,y=0..8；旧资源位于已观察区域深处，新边界仍有信息。
  const auto scene = std::vector<rmtc::VisibleEntity>{resource("known", 4, 3)};
  ResourceWeights no_exploration;
  no_exploration.unknown_cell_value = 0;
  ResourceStrategy old_policy(ResourceOrdering::Weighted, no_exploration);
  ResourceStrategy joint(ResourceOrdering::Weighted, joint_weights);
  for (auto* strategy : {&old_policy, &joint})
    strategy->decide(observe(0, {4, 4}, scene, {20, 20}), 100);
  old_policy.decide(observe(1, {8, 4}, scene, {20, 20}), 99);
  joint.decide(observe(1, {8, 4}, scene, {20, 20}), 99);
  require(old_policy.plan().target && old_policy.plan().target->id == "known",
          "zero unknown value did not restore collection priority");
  require(joint.plan().state == rmtc::ResourceState::Exploring && !joint.plan().target &&
              joint.plan().candidates == 1 && joint.plan().best_resource_cost &&
              joint.plan().best_exploration_cost &&
              *joint.plan().best_exploration_cost < *joint.plan().best_resource_cost,
          "known resource still forced collection when exploration was cheaper");
  require(joint.plan().exploration->target->reward > 0,
          "chosen exploration had no new information reward");
  const auto first_reward = joint.plan().exploration->target->reward;
  auto stronger = joint_weights;
  stronger.unknown_cell_value = 2;
  ResourceStrategy eager(ResourceOrdering::Weighted, stronger);
  eager.decide(observe(0, {4, 4}, scene, {20, 20}), 100);
  eager.decide(observe(1, {8, 4}, scene, {20, 20}), 99);
  require(eager.plan().exploration && eager.plan().exploration->target->reward >= first_reward,
          "increasing unknown-cell value did not strengthen exploration reward");

  ResourceStrategy late(ResourceOrdering::Weighted, joint_weights);
  late.decide(observe(0, {4, 4}, scene, {20, 20}), 100);
  late.decide(observe(1, {8, 4}, scene, {20, 20}), 6);
  require(late.plan().target && late.plan().target->id == "known" && late.plan().route.cost == 5,
          "late exploration consumed the time needed for the known resource");
  ResourceStrategy foot(ResourceOrdering::Weighted, stronger);
  require(foot.decide(observe(0, {4, 4}, {resource("foot", 4, 4)}, {20, 20}), 100).type == "wait" &&
              foot.plan().state == rmtc::ResourceState::AtTarget,
          "exploration abandoned resource under robot");

  ResourceStrategy on_the_way(ResourceOrdering::Weighted, joint_weights);
  on_the_way.decide(observe(0, {4, 4}, {resource("forward", 6, 4)}, {20, 20}), 100);
  require(on_the_way.plan().target && on_the_way.plan().weighted_cost->new_cells == 18 &&
              std::abs(on_the_way.plan().weighted_cost->exploration_reward - 3.6) < 1e-12,
          "actual resource position did not receive the same exploration reward");
  auto frame = observe(0, {4, 4}, {}, {20, 20});
  rmtc::MapMemory memory;
  memory.update(frame);
  GridMap grid{20, 20, memory.cells(), RowOrder::BottomToTop};
  require(count_new_visible_cells(grid, {6, 4}, 4) == 18,
          "incorrect square visibility gain or row order");
  frame.tick = 1;
  frame.self.position = {6, 4};
  memory.update(frame);
  grid.cells = memory.cells();
  require(count_new_visible_cells(grid, {6, 4}, 4) == 0,
          "already observed cells earned repeated exploration reward");
  for (const auto& weights : {ResourceWeights{0.5, 2, 0}, ResourceWeights{0.5, 2, 1, -1},
                              ResourceWeights{0.5, 2, 1, 1, -1}})
    rejects([&] { ResourceStrategy invalid(ResourceOrdering::Weighted, weights); });
  std::cout << "joint exploration: profitable information gain, late collection and no repeated "
               "gain passed\n";
}

/** @brief 验证边界加成同时作用于资源路线和纯探索，且不绕过小系数/收支门槛。 */
void check_boundary_joint_reward() {
  const auto scene = std::vector<rmtc::VisibleEntity>{resource("known", 4, 0)};
  for (double k : {0.0, 0.00001, 0.1, 2.5, 2.51}) {
    ResourceStrategy strategy(ResourceOrdering::Weighted, {0.5, 2, 1, k, 20, 1, 4});
    strategy.decide(observe(0, {4, 0}, scene, {20, 1}), 100);
    const auto action = strategy.decide(observe(1, {8, 0}, scene, {20, 1}), 99);
    // 单行地图所有未知格都贴边，奖励翻倍，原先K>5的阈值变为K>2.5。
    const bool explore = k > 2.5;
    require((strategy.plan().state == rmtc::ResourceState::Exploring) == explore &&
                action.direction == (explore ? "right" : "left"),
            "boundary bonus bypassed profit gate or did not affect threshold");
    std::cout << "boundary profit gate K=" << k << ": " << (explore ? "explore" : "collect")
              << '\n';
  }
  ResourceStrategy resource_bonus(ResourceOrdering::Weighted, {0.5, 2, 1, 1, 20, 1, 4});
  resource_bonus.decide(observe(0, {4, 4}, {resource("forward-high", 6, 4, 20)}, {20, 20}), 100);
  const auto& plan = resource_bonus.plan();
  require(plan.target && plan.target->id == "forward-high" && plan.weighted_cost &&
              plan.weighted_cost->new_cells == 18 && plan.weighted_cost->weighted_new_cells == 23 &&
              std::abs(plan.weighted_cost->exploration_reward - 4.6) < 1e-12 &&
              plan.weighted_cost->value_reward == 4,
          "resource route did not use the same weighted unknown-cell reward as exploration");
  for (const auto& weights :
       {ResourceWeights{0.5, 2, 1, 1, 20, -1, 4}, ResourceWeights{0.5, 2, 1, 1, 20, 1, -1}})
    rejects([&] { ResourceStrategy invalid(ResourceOrdering::Weighted, weights); });
}

/** @brief 检查顺路补看只支付额外距离：等长路线、两步绕路、预算、失败动作和已见格。 */
void check_resource_peek() {
  using namespace rmtc::planning;
  GridMap map{12, 12, std::vector<std::int8_t>(144, 0), RowOrder::TopToBottom};
  map.cells[7 * 12] = -1;
  AStarPlanner planner;
  const auto direct = planner.searchAStar(map, {2, 6}, {4, 6});
  ResourceWeights weights{0, 0, 1, 0, 0, 0, 4, 3, 2};
  const auto peek = plan_resource_peek(map, direct, 1, {}, weights, 2, 10);
  require(peek && peek->route.path[1] == GridIndex{1, 6} && peek->extra_steps == 2 &&
              peek->route.cost == direct.cost + 2 && peek->extra_cells == 1,
          "small side peek did not preserve resource goal or account for extra distance");
  require(!plan_resource_peek(map, direct, 1, {}, weights, 0, 10) &&
              !plan_resource_peek(map, direct, 1, {{1, 6}}, weights, 2, 10) &&
              !plan_resource_peek(map, direct, 1, {}, weights, 2, 4),
          "peek ignored spent detour budget, enemy blocker or collection deadline");
  map.cells[8 * 12] = -1;
  weights.peek_patch_limit = 1;
  require(!plan_resource_peek(map, direct, 1, {}, weights, 2, 10),
          "paid peek expanded a patch larger than its configured limit");
  weights.peek_patch_limit = 0;
  require(plan_resource_peek(map, direct, 1, {}, weights, 2, 10).has_value(),
          "zero patch limit did not restore unrestricted paid peek");
  map.cells[8 * 12] = 0;
  weights.peek_reward = 1;
  require(!plan_resource_peek(map, direct, 1, {}, weights, 2, 10),
          "unprofitable peek was accepted");
  map.cells[7 * 12] = 0;
  weights.peek_reward = 100;
  require(!plan_resource_peek(map, direct, 1, {}, weights, 2, 10),
          "observed cell kept triggering side peeks");

  GridMap equal{9, 9, std::vector<std::int8_t>(81, 0), RowOrder::TopToBottom};
  equal.cells[5 * 9 + 2] = -1;
  auto shortest = planner.searchAStar(equal, {3, 3}, {5, 5});
  shortest.path = {{3, 3}, {4, 3}, {5, 3}, {5, 4}, {5, 5}};  // 指定一条合法等长基准。
  weights.peek_reward = 0.01;
  const auto free = plan_resource_peek(equal, shortest, 1, {}, weights, 0, 10);
  require(free && free->extra_steps == 0 && free->route.path[1] == GridIndex{3, 4},
          "equal-length route did not combine collection and observation for free");

  ResourceWeights limited{0, 0, 1, 0, 0, 0, 4, 0.3, 2, 0};
  ResourceStrategy strategy(ResourceOrdering::Weighted, limited);
  const auto scene = std::vector<rmtc::VisibleEntity>{resource("known", 5, 4)};
  const auto first = strategy.decide(observe(0, {4, 4}, scene, {20, 20}), 100);
  require(
      first.direction == "down" && strategy.plan().peek && strategy.plan().peek->extra_steps == 2,
      "strategy did not execute bounded incremental peek");
  strategy.decide(observe(1, {4, 5}, scene, {20, 20}), 99);
  require(strategy.plan().peek_detour_used == 2 &&
              (!strategy.plan().peek || strategy.plan().peek->extra_steps == 0),
          "executed peek did not consume cumulative detour budget");
  strategy.reset();
  strategy.decide(observe(0, {4, 4}, scene, {20, 20}), 100);
  strategy.decide(observe(1, {4, 4}, scene, {20, 20}), 99);
  require(strategy.plan().peek_detour_used == 0, "failed peek move consumed detour budget");
  auto collected = observe(2, {4, 5}, scene, {20, 20});
  collected.score.resource_value = 10;
  strategy.decide(collected, 98);
  require(strategy.plan().peek_detour_used == 0, "actual collection did not replenish peek budget");
  std::cout
      << "resource peek: equal paths, marginal gain, bounded detours and failed movement passed\n";
}

/// 测试执行器的全场真值，绝不直接作为策略输入。
struct Scenario {
  std::string name;
  rmtc::MapSize map;
  rmtc::Position start;
  int ticks{100};
  std::vector<rmtc::VisibleEntity> scene;
};

/// 相同场景下的资源、动作数量和收齐时刻；未收齐时finish_tick=-1。
struct Outcome {
  double value{0};
  int collected{0};
  int moves{0};
  int finish_tick{-1};
};

/** @brief 在严格视野裁剪下逐Tick执行单步动作，检查边界/障碍并模拟移动后的自动采集。 */
Outcome simulate(const Scenario& scenario, ResourceOrdering ordering, ResourceWeights weights) {
  ResourceStrategy strategy(ordering, weights);
  auto scene = scenario.scene;
  auto position = scenario.start;
  const auto total = std::count_if(scene.begin(), scene.end(), [](const auto& entity) {
    return entity.type == "resource_normal" || entity.type == "resource_high";
  });
  Outcome result;
  for (int tick = 0; tick < scenario.ticks; ++tick) {
    auto frame = observe(tick, position, scene, scenario.map);
    frame.score.resource_value = result.value;
    const auto action = strategy.decide(frame, scenario.ticks - tick);
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
        throw std::runtime_error("invalid simulated direction");
      ++result.moves;
    }
    require(position.x >= 0 && position.y >= 0 && position.x < scenario.map.width &&
                position.y < scenario.map.height,
            "comparison route left map");
    for (const auto& entity : scene)
      require(entity.type != "obstacle" || entity.position.x != position.x ||
                  entity.position.y != position.y,
              "comparison route crossed obstacle");
    scene.erase(std::remove_if(scene.begin(), scene.end(),
                               [&](const auto& entity) {
                                 if (entity.type == "obstacle" || entity.position.x != position.x ||
                                     entity.position.y != position.y)
                                   return false;
                                 ++result.collected;
                                 result.value += entity.value.value_or(0);
                                 return true;
                               }),
                scene.end());
    if (result.collected == total) {
      result.finish_tick = tick + 1;
      break;
    }
  }
  return result;
}

/** @brief 构造公开练习布局、不同出生点/时限及全可见走廊、中央绕墙、四角清理场景。 */
std::vector<Scenario> scenarios() {
  Scenario practice{"practice", {18, 18}, {1, 1}, 100, {}};
  int id = 0;
  for (const auto p : std::vector<rmtc::Position>{{2, 1},
                                                  {15, 1},
                                                  {1, 15},
                                                  {16, 15},
                                                  {4, 2},
                                                  {13, 2},
                                                  {2, 13},
                                                  {15, 13},
                                                  {4, 8},
                                                  {13, 9},
                                                  {8, 4},
                                                  {9, 13},
                                                  {5, 5},
                                                  {12, 5},
                                                  {5, 12},
                                                  {12, 12}}) {
    const auto value = id < 12 ? 10 : 20;
    practice.scene.push_back(resource("practice_" + std::to_string(id++), p.x, p.y, value));
  }
  for (const auto p :
       std::vector<rmtc::Position>{{6, 3}, {6, 4}, {11, 13}, {11, 14}, {3, 11}, {14, 6}})
    practice.scene.push_back(blocker(p.x, p.y));
  auto northeast = practice;
  northeast.name = "practice-NE";
  northeast.start = {16, 1};
  auto short_match = practice;
  short_match.name = "practice-50tick";
  short_match.ticks = 50;
  Scenario corridor{"corridor",
                    {9, 1},
                    {4, 0},
                    25,
                    {resource("left", 2, 0), resource("near", 5, 0), resource("right", 8, 0)}};
  Scenario pocket{"center-wall",
                  {9, 9},
                  {4, 4},
                  60,
                  {resource("near", 5, 4), resource("left", 1, 4, 20), resource("NE", 7, 1),
                   resource("NW", 1, 1), resource("SE", 7, 7, 20), blocker(3, 2), blocker(3, 3),
                   blocker(3, 4), blocker(3, 5)}};
  Scenario corners{
      "four-corners",
      {9, 9},
      {4, 4},
      60,
      {resource("NW", 0, 0), resource("NE", 8, 0), resource("SW", 0, 8), resource("SE", 8, 8),
       resource("center-high", 4, 3, 20), resource("center-normal", 5, 4)}};
  return {practice, northeast, short_match, corridor, pocket, corners};
}

/** @brief 比较三模式和20组权重；结果仅适用于这些固定场景，不作为正式地图最优参数证明。 */
void compare_strategies(std::ostream* csv) {
  const ResourceWeights trial_weights{0.5, 2, 1, 1, 20, 0, 4, 0, 2};
  if (csv)
    *csv << "scenario,horizon,mode,k_edge,k_value,resource_value,collected,moves,finish_tick,k_"
            "distance,unknown_cell_value,fade_ticks\n";
  for (const auto& scenario : scenarios()) {
    for (const auto ordering :
         {ResourceOrdering::Tour, ResourceOrdering::Nearest, ResourceOrdering::Weighted}) {
      const auto result = simulate(scenario, ordering, trial_weights);
      std::cout << scenario.name << ' ' << rmtc::resource_order_name(ordering)
                << ": value=" << result.value << " moves=" << result.moves
                << " finish=" << result.finish_tick << '\n';
      if (scenario.name == "practice" && ordering != ResourceOrdering::Weighted) {
        require(result.value == 200 && result.collected == 16,
                "practice baseline failed to collect 200 resources");
        if (ordering == ResourceOrdering::Tour)
          require(result.moves == 78, "tour baseline changed");
        if (ordering == ResourceOrdering::Nearest)
          require(result.moves == 83, "nearest baseline changed");
      }
      if (csv)
        *csv << scenario.name << ',' << scenario.ticks << ',' << rmtc::resource_order_name(ordering)
             << ',' << trial_weights.edge << ',' << trial_weights.value << ',' << result.value
             << ',' << result.collected << ',' << result.moves << ',' << result.finish_tick << ','
             << trial_weights.distance << ',' << trial_weights.unknown_cell_value << ','
             << trial_weights.exploration_fade_ticks << '\n';
    }
    auto resource_first = trial_weights;
    resource_first.unknown_cell_value = 0;
    const auto baseline = simulate(scenario, ResourceOrdering::Weighted, resource_first);
    std::cout << scenario.name << " weighted-resource-first: value=" << baseline.value
              << " moves=" << baseline.moves << " finish=" << baseline.finish_tick << '\n';
    if (scenario.name == "practice")
      require(baseline.value == 200 && baseline.moves == 76,
              "legacy weighted collection regression");
    if (csv)
      *csv << scenario.name << ',' << scenario.ticks << ",weighted-resource-first,"
           << resource_first.edge << ',' << resource_first.value << ',' << baseline.value << ','
           << baseline.collected << ',' << baseline.moves << ',' << baseline.finish_tick << ','
           << resource_first.distance << ",0," << resource_first.exploration_fade_ticks << '\n';
    for (double edge : {0.0, 0.25, 0.5, 1.0, 2.0}) {
      for (double value : {0.0, 1.0, 2.0, 4.0}) {
        const auto result =
            simulate(scenario, ResourceOrdering::Weighted, {edge, value, 1, 1, 20, 0, 4, 0, 2});
        if (csv)
          *csv << scenario.name << ',' << scenario.ticks << ",weighted-grid," << edge << ','
               << value << ',' << result.value << ',' << result.collected << ',' << result.moves
               << ',' << result.finish_tick << ',' << trial_weights.distance << ','
               << trial_weights.unknown_cell_value << ',' << trial_weights.exploration_fade_ticks
               << '\n';
        if (scenario.name == "practice" && edge == 0 && value == 0)
          require(result.moves == 83 && result.value == 200,
                  "zero weights failed to reproduce nearest baseline");
      }
    }
    if (scenario.name == "practice") {
      for (const int fade : {0, 20, 40, 60, 100}) {
        auto experiment = trial_weights;
        experiment.exploration_fade_ticks = fade;
        const auto outcome = simulate(scenario, ResourceOrdering::Weighted, experiment);
        std::cout << "exploration fade=" << fade << " unknown_value=1: value=" << outcome.value
                  << " finish=" << outcome.finish_tick << '\n';
        if (csv)
          *csv << scenario.name << ',' << scenario.ticks << ",fade-grid," << experiment.edge << ','
               << experiment.value << ',' << outcome.value << ',' << outcome.collected << ','
               << outcome.moves << ',' << outcome.finish_tick << ',' << experiment.distance << ",1,"
               << fade << '\n';
      }
    }
    if (scenario.name == "practice" || scenario.name == "practice-50tick") {
      for (const double unknown_value : {0.0, 0.00001, 0.001, 0.1, 0.25, 0.5, 1.0, 2.0}) {
        auto experiment = trial_weights;
        experiment.unknown_cell_value = unknown_value;
        const auto outcome = simulate(scenario, ResourceOrdering::Weighted, experiment);
        if (unknown_value <= 0.001)
          require(std::tie(outcome.value, outcome.collected, outcome.moves, outcome.finish_tick) ==
                      std::tie(baseline.value, baseline.collected, baseline.moves,
                               baseline.finish_tick),
                  "tiny exploration value changed resource-first benchmark behavior");
        std::cout << scenario.name << " unknown_value=" << unknown_value
                  << ": value=" << outcome.value << " finish=" << outcome.finish_tick << '\n';
        if (csv)
          *csv << scenario.name << ',' << scenario.ticks << ",unknown-value-grid,"
               << experiment.edge << ',' << experiment.value << ',' << outcome.value << ','
               << outcome.collected << ',' << outcome.moves << ',' << outcome.finish_tick << ','
               << experiment.distance << ',' << unknown_value << ','
               << experiment.exploration_fade_ticks << '\n';
      }
    }
  }
}

/** @brief 执行所有离线验证；可选--csv导出对比，失败打印原因并返回1。 */
int main(int argc, char** argv) {
  try {
    std::ofstream csv;
    if (argc == 3 && std::string(argv[1]) == "--csv") {
      csv.open(argv[2]);
      require(csv.good(), "cannot open comparison CSV");
    } else
      require(argc == 1, "usage: weighted_resource_test [--csv output.csv]");
    check_exploration_profit_gate();
    check_weight_effects();
    check_safety_and_memory();
    check_bad_inputs();
    check_joint_exploration();
    check_boundary_joint_reward();
    check_resource_peek();
    compare_strategies(csv.is_open() ? &csv : nullptr);
    require(!csv.is_open() || csv.good(), "cannot write comparison CSV");
    std::cout << "weighted resource selection, Tick budget and comparison checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
