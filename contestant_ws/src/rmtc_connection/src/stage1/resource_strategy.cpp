/**
 * @file resource_strategy.cpp
 * @brief 资源记忆、多资源顺序规划及单步动作生成；与ROS收发/裁判内部状态无关。
 *
 * 只从当前公开观察累计地形。确认旧资源位置为空后停止把它当作采集目标，
 * 视野外尚未确认消失的资源仍可追踪。Weighted把资源与探索放在同一评分中；
 * Tour与Nearest保留原有采集优先规则，便于对照。Joint将资源与必要观察点合排，
 * 远期未知地形按空地估算，实际只执行已知安全首步；其参数与Weighted独立。
 * Weighted先排除来不及的资源，脚下资源先等待自动采集，其余候选按分数比较。
 * 资源位置同样获得新增视野奖励；只奖励仍未知的格子，临近结束降低探索收益。
 * 已有资源时，纯探索必须收益大于自身代价且优于采集；无资源时仍探索兜底。
 * 每次只提交路径上的一格，下一Tick重新使用裁判给出的实际位置规划。
 *
 * @details 模块协作与维护约定
 * 每个Tick刷新自身地图记忆、过滤确认消失的资源，再交给所选规划器生成下一步。
 * 计划路线是建议，不是一次发送整条路线；下一帧会按新观察重算或复用有效部分。
 * 切换算法应同时核对ResourceOrdering、启动覆盖和安装目录，避免编辑一版运行另一版。
 */
#include "rmtc_connection/resource_strategy.hpp"

#include <algorithm>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace rmtc {
namespace {

/** @brief 将规划路线的首个相邻格转换为比赛动作；同格目标等待自动采集。 */
Action first_step(const planning::SearchResult& route) {
  if (route.path.size() <= 1)
    return Action::wait();
  const auto from = route.path[0];
  const auto to = route.path[1];
  const auto dx = to.x - from.x;
  const auto dy = to.y - from.y;
  if (dx == 1 && dy == 0)
    return Action::move("right");
  if (dx == -1 && dy == 0)
    return Action::move("left");
  if (dx == 0 && dy == 1)
    return Action::move("down");
  if (dx == 0 && dy == -1)
    return Action::move("up");
  throw std::runtime_error("A* planned path contains a non-adjacent first step");
}

}  // namespace

/** @brief 保存策略与集中配置的权重并检查合法性，不读取裁判真值或创建ROS节点。 */
ResourceStrategy::ResourceStrategy(ResourceOrdering ordering, planning::ResourceWeights weights,
                                   planning::ExplorationWeights exploration_weights)
    : ordering_(ordering), weights_(weights), exploration_weights_(exploration_weights) {
  planning::validate_resource_weights(weights_);
  planning::validate_exploration_weights(exploration_weights_);
}

/** @brief 清空本局记忆和路线缓存，保留启动时选择的策略及权重。 */
void ResourceStrategy::reset() {
  memory_ = MapMemory{};
  plan_ = ResourcePlan{};
  cached_action_ = Action::wait();
  route_planner_.reset();
  coverage_planner_.reset();
  peek_detour_used_ = 0;
  pending_peek_position_.reset();
  pending_peek_extra_ = 0;
}

/**
 * @brief Joint专用决策：把仍存在的已知资源和必要观察点放入同一条开放路线。
 * frame/memory_已由decide更新；grid与blocked_cells只含本队实际观察到的信息。
 * 累计资源数包含已采集记录，用来判断是否仍需寻找新资源；待采集目标排除已消失记录。
 * 本函数更新plan_和cached_action_；只返回安全首步，远期路线随下一Tick重新估计。
 */
Action ResourceStrategy::decide_joint(const PublicPerception& frame, const planning::GridMap& grid,
                                      planning::GridIndex start,
                                      const std::vector<planning::GridIndex>& blocked_cells,
                                      std::optional<std::int64_t> remaining_ticks) {
  if (remaining_ticks && *remaining_ticks == 0) {
    plan_.state = ResourceState::NoResourceWithinBudget;
    return cached_action_;
  }
  std::vector<planning::ResourceGoal> goals;
  int known_resources = 0;
  for (const auto& [key, record] : memory_.entities()) {
    const auto& e = record.entity;
    if (e.type != "resource_normal" && e.type != "resource_high")
      continue;
    ++known_resources;
    if (!record.first_missing_tick)
      goals.push_back(
          {key, {e.position.x, e.position.y}, e.value.value_or(0), record.last_seen_tick});
  }
  // 裁判若在脚下保留待结算资源，先给它一个WAIT窗口，不为未来路线把它落下。
  for (const auto& goal : goals)
    if (goal.position == start) {
      const auto& record = memory_.entities().at(goal.key);
      plan_.resource_key = goal.key;
      plan_.target = record.entity;
      plan_.last_seen_tick = record.last_seen_tick;
      plan_.state = ResourceState::AtTarget;
      plan_.order_method = "joint_foot_resource";
      plan_.route.status = planning::SearchStatus::Found;
      plan_.route.path = {start};
      return cached_action_;
    }
  const auto joint = coverage_planner_.plan(grid, start, goals, frame.vision_radius,
                                            known_resources, blocked_cells, coverage_weights_);
  plan_.route = joint.first_leg;
  plan_.total_cost = joint.cost;
  plan_.visit_order = joint.keys;
  plan_.resource_key = joint.resource_key;
  plan_.candidates = goals.size();
  plan_.order_method = "joint_coverage_estimate";
  plan_.order_replanned = true;
  if (!joint.resource_key.empty()) {
    const auto& record = memory_.entities().at(joint.resource_key);
    plan_.target = record.entity;
    plan_.last_seen_tick = record.last_seen_tick;
  }
  plan_.state = joint.first_leg.success()
                    ? (plan_.target ? (joint.first_leg.path.size() > 1 ? ResourceState::Moving
                                                                       : ResourceState::AtTarget)
                                    : ResourceState::Exploring)
                    : ResourceState::NoReachableResource;
  if (joint.first_leg.success())
    cached_action_ = first_step(joint.first_leg);
  return cached_action_;
}

/** @brief 用新观察选择目标，每Tick只返回一步意图，重复Tick保持原动作。 */
Action ResourceStrategy::decide(const PublicPerception& frame,
                                std::optional<std::int64_t> remaining_ticks) {
  if (remaining_ticks && *remaining_ticks < 0)
    throw std::invalid_argument("remaining ticks must be nonnegative");
  const auto& previous = memory_.current();
  if (previous && previous->session_id == frame.session_id && previous->tick == frame.tick)
    return cached_action_;
  const auto previous_key =
      previous && previous->session_id == frame.session_id ? plan_.resource_key : std::string{};
  const bool new_session = !previous || previous->session_id != frame.session_id;
  const bool collected_more =
      !new_session && frame.score.resource_value > previous->score.resource_value;
  const auto previous_exploration =
      !new_session && plan_.exploration && plan_.exploration->target
          ? std::optional<planning::GridIndex>{plan_.exploration->target->position}
          : std::nullopt;
  if (!memory_.update(frame))
    return Action::wait();
  if (new_session) {
    route_planner_.reset();
    coverage_planner_.reset();
  }
  if (new_session || collected_more || !frame.self.alive)
    peek_detour_used_ = 0;
  else if (pending_peek_position_ && frame.self.position.x == pending_peek_position_->x &&
           frame.self.position.y == pending_peek_position_->y)
    peek_detour_used_ += pending_peek_extra_;
  pending_peek_position_.reset();
  pending_peek_extra_ = 0;
  plan_ = ResourcePlan{};
  plan_.peek_detour_used = peek_detour_used_;
  plan_.remaining_ticks = remaining_ticks;
  cached_action_ = Action::wait();
  if (!frame.self.alive) {
    plan_.state = ResourceState::Dead;
    return cached_action_;
  }

  planning::GridMap grid{frame.map.width, frame.map.height, memory_.cells(),
                         planning::RowOrder::BottomToTop};
  planning::SearchOptions options;  // 默认未知格不可通行，已知自由格代价均为1。
  for (const auto& entity : frame.visible_entities) {
    if (entity.type == "robot" && (entity.position.x != frame.self.position.x ||
                                   entity.position.y != frame.self.position.y)) {
      options.blocked_cells.push_back({entity.position.x, entity.position.y});
    }
  }
  const planning::GridIndex start{frame.self.position.x, frame.self.position.y};
  if (ordering_ == ResourceOrdering::Joint)
    return decide_joint(frame, grid, start, options.blocked_cells, remaining_ticks);
  if (ordering_ == ResourceOrdering::Tour) {
    std::vector<planning::ResourceGoal> goals;
    for (const auto& [key, record] : memory_.entities()) {
      const auto& entity = record.entity;
      if ((entity.type != "resource_normal" && entity.type != "resource_high") ||
          record.first_missing_tick)
        continue;
      goals.push_back({key,
                       {entity.position.x, entity.position.y},
                       entity.value.value_or(0),
                       record.last_seen_tick});
    }
    plan_.candidates = goals.size();
    // 多资源规划独立封装；只取其首目标的一步动作，其余顺序留给后续Tick。
    const auto& tour = route_planner_.plan(grid, start, goals, options.blocked_cells, previous_key);
    plan_.visit_order = tour.keys;
    plan_.total_cost = tour.cost;
    plan_.order_method =
        tour.keys.empty() ? "none" : (tour.exact ? "optimal_tour" : "nearest_fallback");
    plan_.order_replanned = tour.replanned;
    if (!tour.keys.empty()) {
      const auto& record = memory_.entities().at(tour.keys.front());
      plan_.resource_key = tour.keys.front();
      plan_.target = record.entity;
      plan_.last_seen_tick = record.last_seen_tick;
      plan_.route = tour.first_leg;
    }
  } else {
    for (const auto& [key, record] : memory_.entities()) {
      const auto& entity = record.entity;
      if ((entity.type != "resource_normal" && entity.type != "resource_high") ||
          record.first_missing_tick)
        continue;
      ++plan_.candidates;
      auto route =
          planner_.searchAStar(grid, start, {entity.position.x, entity.position.y}, options);
      if (!route.success()) {
        if (route.status == planning::SearchStatus::InvalidMap ||
            route.status == planning::SearchStatus::InvalidOptions ||
            route.status == planning::SearchStatus::StartOutOfBounds ||
            route.status == planning::SearchStatus::GoalOutOfBounds) {
          throw std::runtime_error("invalid map/options in resource A* search");
        }
        continue;
      }
      bool better = !plan_.target;
      std::optional<planning::WeightedResourceCost> weighted_cost;
      if (ordering_ == ResourceOrdering::Weighted) {
        // 脚下资源也需要一个WAIT窗口；不能用直线距离判定赶不赶得及。
        if (remaining_ticks && std::max<std::int64_t>(1, route.cost) > *remaining_ticks) {
          ++plan_.budget_filtered;
          continue;
        }
        const auto visibility = planning::new_visibility_gain(
            grid, {entity.position.x, entity.position.y}, frame.vision_radius,
            weights_.unknown_edge_bonus, weights_.unknown_edge_band);
        weighted_cost = planning::score_resource_candidate(
            route.cost, {entity.position.x, entity.position.y}, frame.map.width, frame.map.height,
            entity.value.value_or(0), weights_, visibility.new_cells,
            planning::exploration_reward_scale(route.cost, remaining_ticks,
                                               weights_.exploration_fade_ticks),
            visibility.weighted_cells);
        if (plan_.target) {
          // 不为了高权重离开脚下资源；其余先比分数，同分优先保留旧目标。
          const auto priority =
              std::make_tuple(route.cost != 0, weighted_cost->total, key != previous_key,
                              route.cost, -record.last_seen_tick, -entity.value.value_or(0), key);
          const auto best = std::make_tuple(plan_.route.cost != 0, plan_.weighted_cost->total,
                                            plan_.resource_key != previous_key, plan_.route.cost,
                                            -plan_.last_seen_tick, -plan_.target->value.value_or(0),
                                            plan_.resource_key);
          better = priority < best;
        }
      } else if (plan_.target) {
        // 旧版模式：距离始终是第一关键字，保留旧目标和价值只影响同距离选择。
        const auto priority =
            std::make_tuple(route.cost, key != previous_key, -record.last_seen_tick,
                            -entity.value.value_or(0), key);
        const auto best = std::make_tuple(plan_.route.cost, plan_.resource_key != previous_key,
                                          -plan_.last_seen_tick, -plan_.target->value.value_or(0),
                                          plan_.resource_key);
        better = priority < best;
      }
      if (!better)
        continue;
      plan_.resource_key = key;
      plan_.target = entity;
      plan_.last_seen_tick = record.last_seen_tick;
      plan_.route = std::move(route);
      plan_.weighted_cost = weighted_cost;
    }
    if (plan_.target) {
      plan_.visit_order = {plan_.resource_key};
      plan_.total_cost = plan_.route.cost;
      plan_.order_method = ordering_ == ResourceOrdering::Weighted ? "weighted" : "nearest";
      plan_.order_replanned = true;
    }
  }

  if (plan_.weighted_cost)
    plan_.best_resource_cost = plan_.weighted_cost->total;
  const bool compete =
      ordering_ == ResourceOrdering::Weighted && (!plan_.target || plan_.route.cost > 0);
  // 探索代价=路程+边缘-信息奖励。只有负值才表示信息收益覆盖了自身成本；
  // 再与真实资源比较，避免仅因“探索走1步、采集走几步”就在奖励近0时放弃采集。
  // 不以K>0作为开关：K=0和小正数统一经过此判断，自然回到有资源先采集。
  if (compete && plan_.weighted_cost)
    plan_.exploration_cost_limit = std::min(0.0, plan_.weighted_cost->total);
  if (!plan_.target || compete) {
    // Weighted资源与探索必须使用相同路程/边缘/价值尺度；其余策略仍只在无资源时探索。
    auto explore_weights = exploration_weights_;
    if (ordering_ == ResourceOrdering::Weighted) {
      explore_weights = {weights_.distance,
                         weights_.edge,
                         weights_.value * weights_.unknown_cell_value / 10.0,
                         weights_.exploration_fade_ticks,
                         weights_.unknown_edge_bonus,
                         weights_.unknown_edge_band};
    }
    // 已有资源可选时，只让有后续利用时间的探索目标参加竞争；没有资源时保留一步探索兜底。
    const auto reserve = plan_.target ? 2 : 1;
    auto max_steps =
        remaining_ticks
            ? std::optional<std::int64_t>{std::max<std::int64_t>(0, *remaining_ticks - reserve)}
            : std::nullopt;
    if (compete && plan_.target && remaining_ticks) {
      // 静态已知地图中，从探索点返回资源最多多花同等探索步数。
      // 预留2×探索路程+当前资源路程，避免探索完却连已知资源也赶不及收。
      const auto spare = std::max<std::int64_t>(0, *remaining_ticks - plan_.route.cost);
      max_steps = std::min(*max_steps, spare / 2);
    }
    auto exploration = planning::plan_frontier_exploration(
        grid, start, frame.vision_radius, options.blocked_cells, explore_weights,
        previous_exploration, max_steps, remaining_ticks);
    if (exploration.target)
      plan_.best_exploration_cost = exploration.target->total;
    // 有资源时须同时低于0和资源代价，收支相抵或同分都采资源；没有资源则继续探索。
    // 脚下资源在compete中已保护，不会为了探索离开。
    if (exploration.target &&
        (!plan_.target || (plan_.exploration_cost_limit &&
                           exploration.target->total < *plan_.exploration_cost_limit))) {
      plan_.resource_key.clear();
      plan_.target.reset();
      plan_.weighted_cost.reset();
      plan_.last_seen_tick = 0;
      plan_.visit_order.clear();
      plan_.exploration = std::move(exploration);
      plan_.state = ResourceState::Exploring;
      plan_.order_method =
          ordering_ == ResourceOrdering::Weighted ? "weighted_explore" : "frontier";
      plan_.route = plan_.exploration->route;
      plan_.total_cost = plan_.route.cost;
      cached_action_ = first_step(plan_.route);
      return cached_action_;
    }
    if (!plan_.target)
      plan_.exploration = std::move(exploration);
  }
  if (!plan_.target) {
    if (plan_.budget_filtered > 0)
      plan_.state = ResourceState::NoResourceWithinBudget;
    return cached_action_;
  }
  if (ordering_ == ResourceOrdering::Weighted && plan_.route.cost > 0) {
    // 只为原采集路线看不到的格子付额外绕路费；同长路径也可顺便多看。
    plan_.peek = planning::plan_resource_peek(
        grid, plan_.route, frame.vision_radius, options.blocked_cells, weights_,
        std::max<std::int64_t>(0, weights_.peek_detour_limit - peek_detour_used_), remaining_ticks);
    if (plan_.peek) {
      plan_.route = plan_.peek->route;
      plan_.total_cost = plan_.route.cost;
      pending_peek_position_ = plan_.route.path[1];
      pending_peek_extra_ = plan_.peek->extra_steps;
    }
  }
  plan_.state = plan_.route.path.size() <= 1 ? ResourceState::AtTarget : ResourceState::Moving;
  cached_action_ = first_step(plan_.route);
  return cached_action_;
}

/** @brief 返回采集、探索或等待的稳定英文状态名，供结构化日志与调试使用。 */
const char* resource_state_name(ResourceState state) noexcept {
  switch (state) {
    case ResourceState::Moving:
      return "moving";
    case ResourceState::AtTarget:
      return "at_target";
    case ResourceState::Exploring:
      return "exploring";
    case ResourceState::NoReachableResource:
      return "no_reachable_resource";
    case ResourceState::NoResourceWithinBudget:
      return "no_resource_within_budget";
    case ResourceState::Dead:
      return "dead";
  }
  return "unknown";
}

}  // namespace rmtc
