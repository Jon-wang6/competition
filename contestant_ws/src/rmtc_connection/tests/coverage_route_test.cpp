/**
 * @file coverage_route_test.cpp
 * @brief 检验联合规划的关键约束：未知估计不允许盲走、障碍变化重算、无资源继续开图。
 *
 * @details 模块协作与维护约定
 * 验证对象：coverage_route.hpp、resource_strategy.hpp。
 * 运行约定：由独立测试可执行文件/CTest执行，断言失败用于定位回归，不是正式选手的逐帧逻辑。
 * 结果边界：通过表示这些构造场景满足约束，不证明所有随机地图均最优或稳定达到某个总分。
 */
#include "rmtc_connection/coverage_route.hpp"
#include "rmtc_connection/resource_strategy.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
using namespace rmtc::planning;
/// 不使用assert，确保Release构建仍实际执行检查。
void require(bool condition, const char* message) {
  if (!condition)
    throw std::runtime_error(message);
}
/// 用公开方形视野更新一片空地，留出的其他格子仍然为未知。
void reveal(GridMap& grid, GridIndex position, int radius) {
  for (int y = std::max(0, position.y - radius);
       y <= std::min(grid.height - 1, position.y + radius); ++y)
    for (int x = std::max(0, position.x - radius);
         x <= std::min(grid.width - 1, position.x + radius); ++x)
      grid.cells[y * grid.width + x] = 0;
}
/// 检验实际执行的一步在视野内已知空闲，并且四方向相邻。
void safe_step(const GridMap& grid, const CoverageTour& tour, GridIndex start) {
  require(tour.first_leg.success(), "no coverage route");
  require(tour.first_leg.path.size() > 1, "coverage did not move");
  const auto next = tour.first_leg.path[1];
  require(std::abs(start.x - next.x) + std::abs(start.y - next.y) == 1, "non-adjacent move");
  require(grid.cells[next.y * grid.width + next.x] == 0, "blind or blocked move");
}
/// 小型全图中验证准确资源顺序及临时障碍禁行，不受探索权重影响。
void known_tour() {
  GridMap grid{8, 8, std::vector<std::int8_t>(64, 0)};
  CoverageRoutePlanner planner;
  std::vector<ResourceGoal> goals{
      {"near", {2, 1}, 10, 0}, {"far", {6, 1}, 10, 0}, {"last", {6, 6}, 20, 0}};
  auto tour = planner.plan(grid, {1, 1}, goals, 2, 16, {});
  require(tour.cost == 10 && tour.keys.size() == 3 && tour.resource_key == "near",
          "known route not optimal");
  auto blocked = planner.plan(grid, {1, 1}, goals, 2, 16, {{2, 1}});
  safe_step(grid, blocked, {1, 1});
  require(blocked.first_leg.path[1] != GridIndex{2, 1}, "temporary blocked cell entered");
}
/// 仅提供已观察格子；无资源及收齐资源后都应继续探索，不把远期估计当真实地形。
void safe_exploration() {
  GridMap grid{20, 20, std::vector<std::int8_t>(400, -1)};
  GridIndex start{1, 1};
  reveal(grid, start, 4);
  CoverageRoutePlanner planner;
  auto tour = planner.plan(grid, start, {}, 4, 0, {});
  safe_step(grid, tour, start);
  const auto next = tour.first_leg.path[1];
  grid.cells[next.y * 20 + next.x] = 100;
  auto changed = planner.plan(grid, start, {}, 4, 0, {});
  safe_step(grid, changed, start);
  require(changed.first_leg.path[1] != next, "new obstacle ignored by cached route");
  planner.reset();
  safe_step(grid, planner.plan(grid, start, {}, 4, 16, {}), start);
}
/// 输入非法时抛异常，不因缓存或越界索引造成崩溃或任意移动。
void rejects_invalid() {
  CoverageRoutePlanner planner;
  GridMap grid{3, 3, std::vector<std::int8_t>(9, 0)};
  bool threw = false;
  try {
    planner.plan(grid, {3, 0}, {}, 1, 0, {});
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  require(threw, "out-of-bounds start accepted");
  CoverageWeights weights;
  weights.gain_exponent = -1;
  threw = false;
  try {
    planner.plan(grid, {0, 0}, {}, 1, 0, {}, weights);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  require(threw, "negative information exponent accepted");
  // 新增估算比例和换线门槛必须拒绝非法输入，不能将NaN带入路线排序。
  for (int invalid = 0; invalid < 3; ++invalid) {
    weights = CoverageWeights{};
    if (invalid == 0)
      weights.causal_future_fraction = 1.01;
    else if (invalid == 1)
      weights.refresh_margin = -2;
    else
      weights.causal_future_fraction = std::numeric_limits<double>::quiet_NaN();
    threw = false;
    try {
      planner.plan(grid, {0, 0}, {}, 1, 0, {}, weights);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    require(threw, "invalid future/refresh weight accepted");
  }
}
/// ROS地图存储自下向上，比赛坐标自上向下；翻转存储不应改变首步和路线成本。
void row_order_equivalence() {
  GridMap top{20, 20, std::vector<std::int8_t>(400, -1)};
  reveal(top, {1, 1}, 4);
  top.cells[2 * 20 + 3] = 100;
  auto bottom = top;
  bottom.row_order = RowOrder::BottomToTop;
  for (int y = 0; y < 20; ++y)
    for (int x = 0; x < 20; ++x)
      bottom.cells[(19 - y) * 20 + x] = top.cells[y * 20 + x];
  CoverageRoutePlanner a, b;
  const auto first = a.plan(top, {1, 1}, {}, 4, 0, {}),
             second = b.plan(bottom, {1, 1}, {}, 4, 0, {});
  require(first.cost == second.cost && first.first_leg.path == second.first_leg.path,
          "row storage changes decisions");
}
/// 脚下待结算资源要等待；窗口耗尽时不能继续向远处探索或发移动意图。
void strategy_boundary_cases() {
  rmtc::PublicPerception frame;
  frame.session_id = "joint-boundary";
  frame.map = {20, 20};
  frame.vision_radius = 4;
  frame.self.alive = true;
  frame.self.position = {1, 1};
  frame.self.team = "A";
  rmtc::VisibleEntity resource;
  resource.type = "resource_normal";
  resource.id = "underfoot";
  resource.position = {1, 1};
  resource.value = 10;
  frame.visible_entities.push_back(resource);
  rmtc::ResourceStrategy strategy(rmtc::ResourceOrdering::Joint);
  require(strategy.decide(frame, 100).type == "wait" &&
              strategy.plan().state == rmtc::ResourceState::AtTarget,
          "left foot resource");
  frame.tick = 1;
  frame.visible_entities.clear();
  frame.score.resource_value = 10;
  require(strategy.decide(frame, 0).type == "wait" &&
              strategy.plan().state == rmtc::ResourceState::NoResourceWithinBudget,
          "moved with no remaining action window");
}
}  // namespace
/// 执行固定小场景，失败时输出具体约束，不启动ROS或读取裁判隐藏地图。
int main() {
  try {
    known_tour();
    safe_exploration();
    rejects_invalid();
    row_order_equivalence();
    strategy_boundary_cases();
    std::cout << "coverage route tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
