/**
 * @file frontier_exploration_test.cpp
 * @brief 验证探索优先级、安全通行、资源抢占及多种随机地图中的持续探索。
 *
 * 场景执行器持有地图真值，仅向策略发送当前方形视野，禁止策略获取隐藏资源。
 * 不启动ROS节点、不干扰实际比赛；旧策略基线在没有资源目标时模拟WAIT。
 */
#include "rmtc_connection/frontier_exploration.hpp"
#include "rmtc_connection/map_generator.hpp"
#include "rmtc_connection/resource_strategy.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

using namespace rmtc;
using namespace rmtc::planning;

/** @brief Release构建也保留性质检查，失败时输出明确场景说明。 */
void require(bool value, const std::string& message) {
  if (!value)
    throw std::runtime_error(message);
}

/** @brief 构造局部视野数据；场景真值只在测试执行器内裁剪，不传入策略。 */
PublicPerception observe(int tick, Position position, const std::vector<VisibleEntity>& scene,
                         MapSize size, int radius = 4) {
  PublicPerception frame;
  frame.session_id = "exploration-test";
  frame.tick = tick;
  frame.map = size;
  frame.vision_radius = radius;
  frame.self.team = "A";
  frame.self.robot_id = "A";
  frame.self.position = position;
  frame.self.alive = true;
  frame.self.hp = 100;
  for (const auto& entity : scene)
    if (std::abs(entity.position.x - position.x) <= radius &&
        std::abs(entity.position.y - position.y) <= radius)
      frame.visible_entities.push_back(entity);
  return frame;
}

/** @brief 生成测试实体；资源有稳定身份，障碍/敌人用于检查通行限制。 */
VisibleEntity entity(std::string type, int x, int y, std::string id = "test") {
  VisibleEntity result;
  result.type = std::move(type);
  result.position = {x, y};
  result.id = std::move(id);
  if (result.type == "resource_normal" || result.type == "resource_high")
    result.value = result.type == "resource_high" ? 20 : 10;
  return result;
}

/** @brief 执行单步动作并独立检查边界、已知地形及障碍，防止把未知格当作可走。 */
void move(const Action& action, Position& position, const ResourceStrategy& strategy) {
  if (action.type == "wait")
    return;
  require(action.type == "move", "unexpected exploration action");
  if (action.direction == "right")
    ++position.x;
  else if (action.direction == "left")
    --position.x;
  else if (action.direction == "down")
    ++position.y;
  else if (action.direction == "up")
    --position.y;
  else
    throw std::runtime_error("invalid exploration movement direction");
  const auto size = strategy.memory().current()->map;
  require(position.x >= 0 && position.y >= 0 && position.x < size.width && position.y < size.height,
          "exploration moved outside map");
  const auto offset =
      static_cast<std::size_t>(size.height - 1 - position.y) * size.width + position.x;
  require(strategy.memory().cells().at(offset) == 0,
          "exploration moved into unknown terrain or wall");
}

/** @brief 分别增大两项权重，核对就近与贴边偏好，并检查行顺序、预算和非法参数。 */
void check_priority() {
  GridMap map{9, 9, std::vector<std::int8_t>(81, 0), RowOrder::TopToBottom};
  map.cells[4] = -1;  // 顶边唯一未知格；近观察点[4,1]，贴边观察点[3,0]/[5,0]。
  const auto near = plan_frontier_exploration(map, {4, 4}, 1, {}, {1, 0});
  const auto edge = plan_frontier_exploration(map, {4, 4}, 1, {}, {1, 10});
  require(near.target && near.target->position == GridIndex{4, 1} && near.route.cost == 3,
          "shorter actual route did not receive priority");
  require(edge.target && edge.target->edge_distance == 0 && edge.route.cost == 5,
          "larger edge weight did not prefer map boundary");
  require(plan_frontier_exploration(map, {4, 4}, 1, {}, {10, 1}).target->position ==
              near.target->position,
          "larger distance weight did not restore nearby priority");
  const auto stable = plan_frontier_exploration(map, {4, 4}, 1, {}, {1, 10}, GridIndex{5, 0});
  require(stable.target->position == GridIndex{5, 0},
          "equal-score previous exploration goal oscillated");
  require(!plan_frontier_exploration(map, {4, 4}, 1, {}, {}, {}, 2).target,
          "exploration ignored shortest-route step budget");
  const auto blocked = plan_frontier_exploration(map, {4, 4}, 1, {{4, 3}}, {1, 0});
  require(blocked.target && blocked.route.cost == 4 && blocked.route.path[1] != GridIndex{4, 3},
          "temporary blocked cell ignored or geometric distance used");
  GridMap reversed = map;
  reversed.row_order = RowOrder::BottomToTop;
  for (int y = 0; y < 9; ++y)
    std::copy_n(map.cells.begin() + y * 9, 9, reversed.cells.begin() + (8 - y) * 9);
  require(plan_frontier_exploration(reversed, {4, 4}, 1, {}, {1, 0}).target->position ==
              near.target->position,
          "RViz row order reversed exploration direction");
  require(!plan_frontier_exploration(map, {4, 4}, 0).target,
          "zero-radius vision attempted unsafe exploration");
  map.cells[4] = 0;
  require(!plan_frontier_exploration(map, {4, 4}, 4).target, "fully observed map kept patrolling");
  for (const auto weights : {ExplorationWeights{0, 1}, ExplorationWeights{1, -1},
                             ExplorationWeights{1, std::numeric_limits<double>::quiet_NaN()}}) {
    bool rejected = false;
    try {
      (void)plan_frontier_exploration(map, {4, 4}, 1, {}, weights);
    } catch (const std::invalid_argument&) {
      rejected = true;
    }
    require(rejected, "invalid exploration weights accepted");
  }
}

/** @brief 同分优先新增视野更多的位置；未知隔在不可穿越墙后时应安全等待。 */
void check_gain_and_disconnection() {
  GridMap map{9, 9, std::vector<std::int8_t>(81, 0), RowOrder::TopToBottom};
  map.cells[4 * 9 + 2] = -1;
  map.cells[4 * 9 + 6] = -1;
  map.cells[3 * 9 + 6] = -1;
  const auto plan = plan_frontier_exploration(map, {4, 4}, 1, {}, {1, 0});
  require(plan.target && plan.target->position == GridIndex{5, 4} && plan.target->new_cells == 2,
          "equal-score frontier did not prefer more unseen cells");
  GridMap sealed{10, 1, {0, 0, 100, 0, 0, -1, -1, -1, -1, -1}, RowOrder::TopToBottom};
  require(!plan_frontier_exploration(sealed, {0, 0}, 1).target,
          "exploration crossed sealed wall or unknown cells");
}

/**
 * @brief 验证未知格越靠外边越值钱，四角不重复叠加，已观察后立即取消奖励。
 * 等距离/等格数候选原本保留旧内侧目标；只启用B后必须真实改选边界方向。
 */
void check_boundary_reward() {
  require(unknown_cell_edge_factor({0, 5}, 12, 12, 1, 4) == 2 &&
              unknown_cell_edge_factor({2, 5}, 12, 12, 1, 4) == 1.5 &&
              unknown_cell_edge_factor({4, 5}, 12, 12, 1, 4) == 1 &&
              unknown_cell_edge_factor({0, 0}, 12, 12, 1, 4) == 2,
          "boundary band factors or corner cap incorrect");
  GridMap sample{12, 12, std::vector<std::int8_t>(144, 0), RowOrder::TopToBottom};
  for (int x : {0, 2, 4})
    sample.cells[5 * 12 + x] = -1;
  const auto gain = new_visibility_gain(sample, {2, 5}, 4, 1, 4);
  require(gain.new_cells == 3 && gain.weighted_cells == 4.5 &&
              new_visibility_gain(sample, {2, 5}, 4, 0, 4).weighted_cells == 3 &&
              new_visibility_gain(sample, {2, 5}, 4, 1, 0).weighted_cells == 3,
          "raw/weighted visibility gain or disabled edge bonus incorrect");
  GridMap reversed = sample;
  reversed.row_order = RowOrder::BottomToTop;
  for (int y = 0; y < 12; ++y)
    std::copy_n(sample.cells.begin() + y * 12, 12, reversed.cells.begin() + (11 - y) * 12);
  require(new_visibility_gain(reversed, {2, 5}, 4, 1, 4).weighted_cells == 4.5,
          "boundary reward depended on memory row order");

  GridMap map{11, 11, std::vector<std::int8_t>(121, 0), RowOrder::TopToBottom};
  map.cells[5 * 11] = -1;
  map.cells[5 * 11 + 6] = -1;
  const auto plain =
      plan_frontier_exploration(map, {3, 5}, 1, {}, {1, 0, 1, 0, 0, 4}, GridIndex{5, 5});
  const auto border =
      plan_frontier_exploration(map, {3, 5}, 1, {}, {1, 0, 1, 0, 1, 4}, GridIndex{5, 5});
  require(plain.target && plain.target->position == GridIndex{5, 5} && border.target &&
              border.target->position == GridIndex{1, 5} && border.target->new_cells == 1 &&
              border.target->weighted_new_cells == 2 && border.target->reward == 2,
          "edge bonus did not change actual frontier choice with distance and raw gain equal");
  const auto direct = new_visibility_gain(map, border.target->position, 1, 1, 4);
  require(direct.weighted_cells == border.target->weighted_new_cells,
          "frontier prefix sum disagreed with direct weighted visibility count");
  map.cells[5 * 11] = 0;
  const auto seen = plan_frontier_exploration(map, {3, 5}, 1, {}, {1, 0, 1, 0, 1, 4});
  require(seen.target && seen.target->position == GridIndex{5, 5} &&
              new_visibility_gain(map, {1, 5}, 1, 1, 4).weighted_cells == 0,
          "observed border cell kept rewarding exploration");
  for (const auto weights :
       {ExplorationWeights{1, 0, 1, 0, -1, 4}, ExplorationWeights{1, 0, 1, 0, 1, -1},
        ExplorationWeights{1, 0, 1, 0, std::numeric_limits<double>::infinity(), 4}}) {
    bool rejected = false;
    try {
      (void)plan_frontier_exploration(map, {3, 5}, 1, {}, weights);
    } catch (const std::invalid_argument&) {
      rejected = true;
    }
    require(rejected, "invalid boundary exploration weights accepted");
  }
  std::cout << "boundary bonus: weighted gain, target change and no repeated reward passed\n";
}

/** @brief 所有资源模式都能探索到远处隐藏资源；发现后立即抢占，敌车消失后重新通行。 */
void check_hidden_resource() {
  for (const auto ordering :
       {ResourceOrdering::Tour, ResourceOrdering::Nearest, ResourceOrdering::Weighted}) {
    ResourceStrategy strategy(ordering);
    Position position{0, 0};
    std::vector<VisibleEntity> scene{entity("resource_high", 20, 0)};
    bool explored = false, pursued = false;
    for (int tick = 0; tick < 40 && position.x < 20; ++tick) {
      const auto action = strategy.decide(observe(tick, position, scene, {25, 1}), 40 - tick);
      require(action.type == "move", "no-resource corridor got stuck waiting");
      if (position.x < 16) {
        require(strategy.plan().state == ResourceState::Exploring && !strategy.plan().target,
                "strategy received or invented a hidden resource");
        explored = true;
      } else {
        require(strategy.plan().target && !strategy.plan().exploration,
                "visible resource did not preempt exploration");
        pursued = true;
      }
      move(action, position, strategy);
    }
    require(explored && pursued && position.x == 20, "did not reach resource through exploration");
    ResourceStrategy budget(ordering);
    require(budget.decide(observe(0, {0, 0}, {}, {25, 1}), 1).type == "wait",
            "last window wasted on exploration with no time to collect");
    require(budget.decide(observe(1, {0, 0}, {}, {25, 1}), 2).type == "move",
            "two available windows did not allow one-step exploration");
  }
  ResourceStrategy strategy;
  require(strategy.decide(observe(0, {0, 0}, {entity("robot", 1, 0)}, {25, 1})).type == "wait",
          "exploration walked through enemy in narrow corridor");
  const auto action = strategy.decide(observe(1, {0, 0}, {}, {25, 1}));
  require(action.direction == "right", "historical enemy permanently blocked exploration");
  require(strategy.decide(observe(1, {0, 0}, {}, {25, 1})).direction == action.direction,
          "duplicate frame changed cached exploration action");
  require(strategy.decide(observe(2, {0, 0}, {}, {25, 1})).direction == action.direction &&
              strategy.plan().route.path.front() == GridIndex{0, 0},
          "failed move advanced exploration position");
}

/// 独立执行器的统计，不将真值或资源总数交给策略。
struct RunResult {
  int value{0};
  int moves{0};
  int explores{0};
  int finish_tick{0};
};

/** @brief 在生成地图中逐Tick模拟公开感知、合法移动与自动采集，可选择旧WAIT基线。 */
RunResult run_seed(int seed, ResourceOrdering ordering, int windows, bool old_wait = false) {
  const nlohmann::json base{{"events", nlohmann::json::object()}};
  const auto config = rmtc::practice::generate_practice_map(base, 1, seed).config;
  const auto& map = config.at("map");
  const MapSize size{map.at("width").get<int>(), map.at("height").get<int>()};
  Position position{config["robots"][0]["spawn"][0].get<int>(),
                    config["robots"][0]["spawn"][1].get<int>()};
  std::vector<VisibleEntity> scene;
  for (const auto& obstacle : map.at("obstacles"))
    scene.push_back(entity("obstacle", obstacle[0].get<int>(), obstacle[1].get<int>()));
  for (const auto& resource : map.at("resources"))
    scene.push_back(entity(resource.at("value") == 20 ? "resource_high" : "resource_normal",
                           resource["position"][0].get<int>(), resource["position"][1].get<int>(),
                           resource["id"].get<std::string>()));
  ResourceStrategy strategy(ordering);
  RunResult result;
  for (int tick = 0; tick < windows; ++tick) {
    auto frame = observe(tick, position, scene, size);
    frame.score.resource_value = result.value;
    auto action = strategy.decide(frame, windows - tick);
    if (strategy.plan().state == ResourceState::Exploring) {
      if (old_wait)
        action = Action::wait();
      else
        ++result.explores;
    }
    if (action.type == "move")
      ++result.moves;
    move(action, position, strategy);
    for (const auto& e : scene)
      require(e.type != "obstacle" || e.position.x != position.x || e.position.y != position.y,
              "generated map wall collision");
    scene.erase(std::remove_if(scene.begin(), scene.end(),
                               [&](const auto& e) {
                                 if (e.type == "obstacle" || e.position.x != position.x ||
                                     e.position.y != position.y)
                                   return false;
                                 result.value += static_cast<int>(e.value.value_or(0));
                                 if (result.value == 200)
                                   result.finish_tick = tick + 1;
                                 return true;
                               }),
                scene.end());
    if (result.value == 200 &&
        strategy.memory().observed_count() == static_cast<std::size_t>(size.width * size.height))
      break;
  }
  if (windows == 1000) {
    require(result.value == 200,
            "seed " + std::to_string(seed) + " did not finish resource collection");
    require(
        strategy.memory().observed_count() == static_cast<std::size_t>(size.width * size.height),
        "seed " + std::to_string(seed) + " did not finish exploring connected map");
  }
  return result;
}

/** @brief 非零种子覆盖持续探索，与无资源就WAIT的旧行为对比，检查至少一张地图改善。 */
void check_seeded_maps() {
  int improved = 0;
  for (const auto ordering :
       {ResourceOrdering::Tour, ResourceOrdering::Nearest, ResourceOrdering::Weighted}) {
    for (int seed = 1; seed <= 16; ++seed) {
      (void)run_seed(seed, ordering, 1000);
      const auto before = run_seed(seed, ordering, 100, true);
      const auto after = run_seed(seed, ordering, 100);
      if (after.value > before.value)
        ++improved;
    }
  }
  require(improved > 0, "exploration did not improve any stuck baseline");
  std::cout << "48 seeded comparisons: " << improved << " improved resource collection\n";
}

/** @brief 运行优先级与集成回归场景；异常时返回非零，便于ctest定位。 */
int main() {
  try {
    check_priority();
    check_gain_and_disconnection();
    check_boundary_reward();
    check_hidden_resource();
    check_seeded_maps();
    std::cout << "exploration priority, safety, preemption and 48 seeded runs passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
