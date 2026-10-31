/**
 * @file stage2_resource_race.cpp
 * @brief 已知资源的期限子集动态规划；等长首段偏好沿途开图，不增加实际步数。
 */
#include "rmtc_connection/stage2_resource_race.hpp"
#include "rmtc_connection/stage2_tactical_navigation.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>

namespace rmtc {
ResourceRacePlan plan_resource_race(
    MapSize size, Position start, std::vector<RaceResource> resources,
    int remaining, const ResourceRaceWeights& w,
    const std::function<bool(Position)>& passable,
    const std::function<double(Position)>& information, double mission_budget) {
  ResourceRacePlan result;
  if (!w.enabled || remaining <= 0 || resources.empty()) return result;
  const auto began = std::chrono::steady_clock::now();
  const auto ms = [&] { return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - began).count(); };
  const auto index = [&](Position p) { return p.y * size.width + p.x; };
  const auto direct = goal_distance_field(size, start, passable);
  const int horizon = std::min(remaining, w.horizon);
  resources.erase(std::remove_if(resources.begin(), resources.end(), [&](const auto& r) {
    return r.position.x < 0 || r.position.y < 0 || r.position.x >= size.width ||
           r.position.y >= size.height || r.value <= 0 || r.confidence <= 0 ||
           direct[index(r.position)] <= 0 || direct[index(r.position)] > horizon;
  }), resources.end());
  // 早到期限优先只影响预算中断时的探索顺序；完整DP不依赖输入顺序。
  std::stable_sort(resources.begin(), resources.end(), [&](const auto& a, const auto& b) {
    const int ad = a.enemy_steps < 0 ? horizon + 1 : a.enemy_steps;
    const int bd = b.enemy_steps < 0 ? horizon + 1 : b.enemy_steps;
    if (ad != bd) return ad < bd;
    return a.value * a.confidence / direct[index(a.position)] >
           b.value * b.confidence / direct[index(b.position)];
  });
  if (resources.size() > static_cast<std::size_t>(w.target_limit)) resources.resize(w.target_limit);
  const int n = static_cast<int>(resources.size());
  result.targets = n;
  if (n == 0) { result.elapsed_ms = ms(); return result; }
  std::vector<std::vector<int>> fields;
  for (const auto& r : resources) fields.push_back(goal_distance_field(size, r.position, passable));
  const unsigned masks = 1U << n;
  constexpr std::int16_t unreachable = std::numeric_limits<std::int16_t>::max();
  std::vector<std::int16_t> arrival(static_cast<std::size_t>(masks) * n, unreachable);
  std::vector<std::int8_t> parent(arrival.size(), -1);
  std::vector<double> value(masks, 0);
  for (unsigned mask = 1; mask < masks; ++mask) {
    const unsigned bit = mask & (~mask + 1);
    const int i = __builtin_ctz(bit);
    value[mask] = value[mask ^ bit] + resources[i].value * resources[i].confidence;
  }
  if (w.mission_weight > 0 && mission_budget > 0) {
    std::map<std::string,std::vector<double>> task_gains;
    for (int i=0;i<n;++i)
      for (const auto& [id,gain]:resources[i].mission_values) {
        auto& gains=task_gains[id];
        if (gains.empty()) gains.resize(n,0);
        gains[i]=gain;
      }
    for (const auto& [id,gains]:task_gains) {
      (void)id;
      std::vector<double> sums(masks,0);
      for (unsigned mask=1;mask<masks;++mask) {
        const unsigned bit=mask&(~mask+1);
        sums[mask]=sums[mask^bit]+gains[__builtin_ctz(bit)];
        value[mask]+=w.mission_weight*std::min(mission_budget,sums[mask]);
      }
    }
  }
  std::vector<double> terminal_information;
  for (const auto& r:resources) terminal_information.push_back(information(r.position));
  const auto deadline = [&](int i) {
    return resources[i].enemy_steps < 0 ? horizon :
           std::min(horizon, resources[i].enemy_steps + w.deadline_slack);
  };
  unsigned best_mask = 0;
  int best_last = -1, best_time = horizon + 1;
  double best_objective = -std::numeric_limits<double>::infinity();
  const auto choose = [&](unsigned mask, int last, int when) {
    const double objective = value[mask] - w.distance_cost * when +
                             w.terminal_information * terminal_information[last];
    if (objective > best_objective + 1e-9 ||
        (std::abs(objective - best_objective) <= 1e-9 && when < best_time)) {
      best_objective = objective; best_mask = mask; best_last = last; best_time = when;
    }
  };
  for (int i = 0; i < n; ++i) {
    const int when = direct[index(resources[i].position)];
    if (when <= deadline(i)) {
      arrival[(1U << i) * n + i] = static_cast<std::int16_t>(when);
      choose(1U << i, i, when);
    }
  }
  for (unsigned mask = 1; mask < masks; ++mask) {
    if ((mask & 63U) == 0 && ms() >= w.budget_ms) { result.complete = false; break; }
    for (int last = 0; last < n; ++last) {
      const int when = arrival[mask * n + last];
      if (when == unreachable) continue;
      ++result.expanded;
      choose(mask, last, when);
      for (int next = 0; next < n; ++next) {
        if (mask & (1U << next)) continue;
        const int leg = fields[last][index(resources[next].position)];
        const int next_time = when + leg;
        if (leg < 0 || next_time > deadline(next)) continue;
        const unsigned next_mask = mask | (1U << next);
        const auto at = next_mask * n + next;
        if (next_time < arrival[at]) {
          arrival[at] = static_cast<std::int16_t>(next_time);
          parent[at] = static_cast<std::int8_t>(last);
          choose(next_mask, next, next_time);
        }
      }
    }
  }
  if (best_last < 0) {
    // 对手可能转向、停留或交战：最早到达估计不是确定消失时刻。
    // 没有“能抢先”的路线时保留一条直接争取路线，下一帧看消失证据再取消。
    for (int i = 0; i < n; ++i) {
      const int when = direct[index(resources[i].position)];
      const double utility = resources[i].value * resources[i].confidence / (when + 3.0);
      if (best_last < 0 || utility > best_objective) {
        best_objective = utility; best_last = i; best_time = when; best_mask = 1U << i;
      }
    }
    result.contested_fallback = true;
  }
  for (int i=0;i<n;++i)
    if (best_mask & (1U<<i)) result.forecast_value += resources[i].value * resources[i].confidence;
  result.objective = best_objective;
  result.route_steps = best_time;
  unsigned mask = best_mask;
  int last = best_last;
  while (last >= 0) {
    result.order.push_back(resources[last].key);
    const auto at = mask * n + last;
    result.arrivals.push_back(result.contested_fallback ? best_time : arrival[at]);
    const int previous = parent[at]; mask ^= 1U << last; last = previous;
  }
  std::reverse(result.order.begin(), result.order.end());
  std::reverse(result.arrivals.begin(), result.arrivals.end());
  const auto first = std::find_if(resources.begin(), resources.end(), [&](const auto& r) {
    return r.key == result.order.front();
  });
  const auto to_goal = fields[static_cast<std::size_t>(first - resources.begin())];
  // 只比较等长下一步，优先能看更多新区域的落点；资源路线绝不因此额外绕行。
  constexpr Position steps[]{{0,-1},{1,0},{0,1},{-1,0}};
  Position at = start;
  result.route.path.push_back({at.x, at.y});
  while (to_goal[index(at)] > 0) {
    Position next = at;
    double potential = -std::numeric_limits<double>::infinity();
    for (const auto delta : steps) {
      const Position p{at.x + delta.x, at.y + delta.y};
      if (p.x < 0 || p.y < 0 || p.x >= size.width || p.y >= size.height ||
          !passable(p) || to_goal[index(p)] != to_goal[index(at)] - 1) continue;
      const double gain = information(p);
      if (gain > potential) { potential = gain; next = p; }
    }
    if (next.x == at.x && next.y == at.y) { result.route = {}; break; }
    at = next; result.route.path.push_back({at.x, at.y});
  }
  if (result.route.path.size() > 1) {
    result.route.status = planning::SearchStatus::Found;
    result.route.cost = static_cast<int>(result.route.path.size()) - 1;
  }
  result.elapsed_ms = ms();
  return result;
}
}  // namespace rmtc
