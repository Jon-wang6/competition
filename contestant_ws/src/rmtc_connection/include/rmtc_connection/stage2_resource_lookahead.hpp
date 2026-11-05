/**
 * @file stage2_resource_lookahead.hpp
 * @brief 第二阶段短程采集前瞻：比较先吃一块后，已知区域内还能顺路吃哪些资源。
 *
 * 每个候选资源建立一次真实距离场，最多枚举三个不同资源；只给首目标增加后续
 * 收益率，执行仍每Tick重新规划。未知地形、已消失资源、超时路线不参与计算。
 * 资源分和每项任务的预测收益分别封顶，避免同一奖励在路线中被重复许诺。
 *
 * @details 模块协作与维护约定
 * 为近期采集比较连续访问机会，按公开可达路径估计路程并避免重复采集/观察计分。
 * 只执行本帧合法首步，后续路径是下一帧可被新信息替换的预测。
 */
#pragma once

#include "rmtc_connection/stage2_joint_planner.hpp"
#include "rmtc_connection/stage2_tactical_navigation.hpp"

#include <algorithm>
#include <functional>
#include <map>
#include <vector>

namespace rmtc {

/**
 * @brief 返回每项首资源的后续采集收益率；不含首资源自身，不替代实际裁判计分。
 * @param opportunities 已按记忆和可见对手争抢折扣的可达资源机会。
 * @param start_distances 本车BFS步数，世界坐标y*width+x索引。
 * @param resource_budget 尚可得到的真实资源分；mission_budget为每项未完成任务的潜在分。
 * @param resource_weight 当前阶段采集偏好；mission_weight为采集任务偏好。
 * @param passable 当前本队已知空地，已排除敌车和危险格，不读取对手私有信息。
 * 复杂度由limit<=10、depth<=3限制；距离场O(limit*格数)，路线枚举O(limit^depth)。
 */
template <typename Passable>
std::map<std::string, double> resource_lookahead(MapSize size,
                                                 const std::vector<JointOpportunity>& offered,
                                                 const std::vector<int>& start_distances,
                                                 double resource_budget, double mission_budget,
                                                 double resource_weight, double mission_weight,
                                                 std::int64_t remaining_ticks,
                                                 const Stage2Weights& weights, Passable passable) {
  std::map<std::string, double> result;
  if (weights.resource_lookahead_weight <= 0 || offered.size() < 2)
    return result;
  auto opportunities = offered;
  std::stable_sort(opportunities.begin(), opportunities.end(),
                   [](const auto& a, const auto& b) { return a.priority > b.priority; });
  if (opportunities.size() > static_cast<std::size_t>(weights.resource_lookahead_limit))
    opportunities.resize(weights.resource_lookahead_limit);
  std::vector<std::vector<int>> fields;
  for (const auto& opportunity : opportunities)
    fields.push_back(goal_distance_field(size, opportunity.position, passable));
  const int horizon =
      static_cast<int>(std::min<std::int64_t>(remaining_ticks, weights.resource_lookahead_steps));
  const auto index = [&](Position p) { return p.y * size.width + p.x; };
  // 每个递归分支独立扣减分数预算；访问位图保证同一资源只算一次。
  const auto collect = [&](const JointOpportunity& item, double& budget,
                           std::map<std::string, double>& missions) {
    const double resource = std::min(budget, item.resource_points);
    budget -= resource;
    double mission = 0;
    for (const auto& [id, points] : item.mission_points) {
      auto [it, inserted] = missions.emplace(id, mission_budget);
      (void)inserted;
      const double gain = std::min(it->second, points);
      it->second -= gain;
      mission += gain;
    }
    return resource * resource_weight + mission * mission_weight;
  };
  std::function<double(int, int, unsigned, int, double, std::map<std::string, double>)> extend;
  extend = [&](int at, int elapsed, unsigned visited, int depth, double budget,
               std::map<std::string, double> missions) {
    if (depth >= weights.resource_lookahead_depth)
      return 0.0;
    double best = 0;
    for (std::size_t next = 0; next < opportunities.size(); ++next) {
      if (visited & (1U << next))
        continue;
      const int leg = fields[at][index(opportunities[next].position)];
      const int arrival = elapsed + leg;
      if (leg <= 0 || arrival > horizon)
        continue;
      double next_budget = budget;
      auto next_missions = missions;
      const double gain = collect(opportunities[next], next_budget, next_missions);
      // 后续越远越不确定；0.7为每多一站的折扣，不声称路线一定能全部吃到。
      const double utility = gain / (arrival + weights.travel_offset) +
                             0.7 * extend(static_cast<int>(next), arrival, visited | (1U << next),
                                          depth + 1, next_budget, std::move(next_missions));
      best = std::max(best, utility);
    }
    return best;
  };
  for (std::size_t first = 0; first < opportunities.size(); ++first) {
    const int elapsed = start_distances[index(opportunities[first].position)];
    if (elapsed <= 0 || elapsed >= horizon)
      continue;
    double budget = resource_budget;
    std::map<std::string, double> missions;
    collect(opportunities[first], budget, missions);
    result[opportunities[first].key] =
        weights.resource_lookahead_weight *
        extend(static_cast<int>(first), elapsed, 1U << first, 1, budget, std::move(missions));
  }
  return result;
}

}  // namespace rmtc
