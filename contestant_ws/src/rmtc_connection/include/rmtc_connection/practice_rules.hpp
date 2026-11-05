/**
 * @file practice_rules.hpp
 * @brief 本地裁判v1.2计分与排名；仅用于裁判，不允许选手策略读取全图或AI审计日志。
 *
 * @details 模块协作与维护约定
 * 本地裁判扩展计算分项、资格与排名；它与选手内部用于动作排序的权重是两套职责。
 * 读取结算或审计信息的权限仅属于练习裁判，不能把本文件接入选手来获取隐藏状态。
 */
#pragma once
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <set>
#include <string>
#include <tuple>

namespace rmtc::practice {
using RulesJson = nlohmann::json;

/** @brief 根据本场、本席位的真实成功记录判定练习计分资格；模拟响应及其他场次无效。 */
inline std::set<std::string> qualified_teams(const RulesJson& records, const std::string& session) {
  std::set<std::string> teams;
  for (const auto& row : records)
    if (row.value("session_id", "") == session && row.value("mode", "") == "real" &&
        row.value("success", false) && row.value("status", 0) >= 200 && row.value("status", 0) < 300)
      teams.insert(row.value("seat", ""));
  return teams;
}

/** @brief 九位小数比较键；界面显示的小数位数不能改变同分判定。 */
inline long long comparison_units(double value) {
  return std::llround(value * 1e9);
}

/** @brief 以得分、获资格任务数、资源、未截断伤害、无效动作依次生成排名键。 */
inline auto ranking_key(const RulesJson& row) {
  return std::make_tuple(comparison_units(row.at("score").get<double>()),
      row.at("mission_completed_eligible").get<int>(),
      comparison_units(row.at("resource_value").get<double>()),
      comparison_units(row.at("effective_damage").get<double>()),
      -row.at("invalid_actions").get<int>());
}

/** @brief 从原始结算状态重算资格、分项与排名；保留原始任务数，不把完全同分强判胜负。 */
inline RulesJson rules_scoreboard(const RulesJson& batch, const std::set<std::string>& qualified) {
  const auto& state = batch.at("state");
  const int stage = state.at("stage");
  RulesJson board{{"tick", state.at("tick")}, {"action_tick", batch.at("tick")},
      {"stage", stage}, {"session_id", batch.at("session_id")},
      {"rules_version", "1.2-local"}, {"teams", RulesJson::array()}, {"actions", RulesJson::array()}};
  for (const auto& robot : state.at("robots")) {
    const std::string team = robot.at("team");
    const int raw = static_cast<int>(robot.at("mission_completed").size());
    const bool eligible = stage == 2 && qualified.count(team);
    const int missions = eligible ? raw : 0;
    const double resource = robot.at("resource_value"), damage = robot.at("effective_damage");
    const double resource_score = stage == 1 ? 40.0 * resource / 200.0 : 30.0 * std::min(resource / 120.0, 1.0);
    const double damage_score = stage == 1 ? 0.0 : 15.0 * std::min(damage / 500.0, 1.0);
    const double mission_score = stage == 1 ? 0.0 : 5.0 * missions;
    const auto names = batch.value("team_names", RulesJson::object());
    board["teams"].push_back({{"team", team}, {"team_name", names.value(team, team)},
        {"score", resource_score + damage_score + mission_score},
        {"resource_value", resource}, {"effective_damage", damage},
        {"mission_completed", raw}, {"mission_completed_raw", raw},
        {"mission_completed_eligible", missions}, {"ai_eligible", eligible},
        {"mission_completed_ids", robot.at("mission_completed")},
        {"invalid_actions", robot.at("invalid_actions")}, {"hp", robot.at("hp")}, {"alive", robot.at("alive")},
        {"score_breakdown", {{"resource", resource_score}, {"combat", damage_score}, {"missions", mission_score}}}});
    auto action = robot.at("last_action");
    if (!action.is_object()) action = RulesJson{{"action", "wait"}};
    action["team"] = team;
    action["source"] = batch.value("action_inputs", RulesJson::object()).contains(team) ? "submitted" : "default";
    const std::string error = action.value("error", "");
    action["status"] = error.empty() || error == "E_OOB" || error == "E_BLOCKED" ? "executed" : "rejected";
    board["actions"].push_back(action);
  }
  auto& teams = board["teams"];
  std::stable_sort(teams.begin(), teams.end(), [](const auto& a, const auto& b) {return ranking_key(a) > ranking_key(b);});
  for (std::size_t i = 0; i < teams.size(); ++i) {
    const bool previous = i && ranking_key(teams[i]) == ranking_key(teams[i-1]);
    const bool next = i+1 < teams.size() && ranking_key(teams[i]) == ranking_key(teams[i+1]);
    teams[i]["rank"] = previous ? teams[i-1]["rank"] : RulesJson(i+1);
    teams[i]["tied"] = previous || next;
  }
  board["map_view"] = {{"map", state.at("map")}, {"obstacles", state.at("obstacles")},
      {"resources", state.at("resources")}, {"robots", state.at("robots")}};
  return board;
}
}  // namespace rmtc::practice
