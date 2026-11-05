/**
 * @file stage2_joint_planner.cpp
 * @brief 将第一阶段联合覆盖路线适配为第二阶段有限前视的采集建议。
 *
 * @details 模块协作与维护约定
 * 第一阶段路线负责提出访问顺序，第二阶段再按剩余时间、风险和收益解释该提案。
 * 复用规划器不等于把第一阶段目标直接照搬；敌人抢夺与任务等仍由第二阶段评价。
 */
#include "rmtc_connection/stage2_joint_planner.hpp"
#include <algorithm>
#include <cmath>

namespace rmtc {
Stage2JointPlan Stage2JointPlanner::propose(const PublicPerception& frame, const MapMemory& memory,
                                            std::vector<JointOpportunity> opportunities,
                                            const std::vector<planning::GridIndex>& blocked,
                                            std::int64_t remaining_ticks,
                                            const Stage2Weights& weights) {
  Stage2JointPlan result;
  std::stable_sort(opportunities.begin(), opportunities.end(),
                   [](const auto& a, const auto& b) { return a.priority > b.priority; });
  if (opportunities.size() > static_cast<std::size_t>(weights.joint_resource_limit))
    opportunities.resize(weights.joint_resource_limit);
  std::vector<planning::ResourceGoal> goals;
  for (const auto& o : opportunities)
    goals.push_back({o.key, {o.position.x, o.position.y}, o.value, frame.tick});
  int known = 0;
  for (const auto& item : memory.entities())
    if (item.second.entity.type == "resource_normal" || item.second.entity.type == "resource_high")
      ++known;
  planning::CoverageWeights config;
  config.expected_resources = 16;
  config.insertion_offset = weights.joint_observation_offset;
  config.future_resource_weight = weights.joint_unknown_weight;
  config.refresh_margin = 0;
  config.relocate = false;
  const planning::GridMap grid{frame.map.width, frame.map.height, memory.cells(),
                               planning::RowOrder::BottomToTop};
  const auto tour = planner_.plan(grid, {frame.self.position.x, frame.self.position.y}, goals,
                                  frame.vision_radius, known, blocked, config);
  if (!tour.first_leg.success() || tour.first_leg.path.size() < 2)
    return result;
  result.route.status = planning::SearchStatus::Found;
  for (const auto p : tour.first_leg.path) {
    if (memory.cells()[MapMemory::index({p.x, p.y}, frame.map)] != 0 ||
        result.route.path.size() > static_cast<std::size_t>(std::min<std::int64_t>(
                                       remaining_ticks, weights.joint_horizon)))
      break;
    result.route.path.push_back(p);
  }
  if (result.route.path.size() < 2) {
    result.route = {};
    return result;
  }
  result.route.cost = static_cast<int>(result.route.path.size()) - 1;
  result.visit_order = tour.keys;
  result.estimated_cost = tour.cost;
  auto at = result.route.path.back();
  double available_resource = weights.resource_points / weights.resource_cap *
                              std::max(0.0, weights.resource_cap - frame.score.resource_value);
  std::map<std::string, double> mission_budget;
  for (const auto& m : frame.missions)
    if ((m.type == "normal_resources" || m.type == "high_resources" ||
         m.type == "resource_value") &&
        std::find(frame.score.mission_completed_ids.begin(),
                  frame.score.mission_completed_ids.end(),
                  m.id) == frame.score.mission_completed_ids.end())
      mission_budget[m.id] = weights.mission_points * weights.mission_potential;
  // 第一段若真能到达一个资源，先扣除其收益，后续前视不重复计入。
  for (const auto& o : opportunities) {
    if (o.position.x != at.x || o.position.y != at.y)
      continue;
    result.resource_key = o.key;
    result.resource_points = std::min(available_resource, o.resource_points);
    available_resource -= result.resource_points;
    for (const auto& [id, gain] : o.mission_points) {
      const double earned = std::min(mission_budget[id], gain);
      result.mission_points += earned;
      mission_budget[id] -= earned;
    }
    break;
  }
  planning::SearchOptions search;
  search.blocked_cells = blocked;
  int elapsed = result.route.cost, credited = 0;
  for (const auto& key : tour.keys) {
    if (key == result.resource_key)
      continue;
    const auto found = std::find_if(opportunities.begin(), opportunities.end(),
                                    [&](const auto& o) { return o.key == key; });
    if (found == opportunities.end())
      continue;
    const auto leg = planning::AStarPlanner{}.searchAStar(
        grid, at, {found->position.x, found->position.y}, search);
    if (!leg.success() ||
        elapsed + leg.cost > std::min<std::int64_t>(remaining_ticks, weights.joint_horizon))
      break;
    elapsed += leg.cost;
    at = {found->position.x, found->position.y};
    const double resource = std::min(available_resource, found->resource_points);
    available_resource -= resource;
    double mission = 0;
    for (const auto& [id, gain] : found->mission_points) {
      const double earned = std::min(mission_budget[id], gain);
      mission += earned;
      mission_budget[id] -= earned;
    }
    result.future_utility += weights.joint_future_weight * std::pow(0.7, credited) *
                             (resource + mission) / (elapsed + weights.travel_offset);
    if (++credited >= 3)
      break;
  }
  return result;
}
}  // namespace rmtc
