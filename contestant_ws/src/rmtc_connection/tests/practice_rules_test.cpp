/** @file practice_rules_test.cpp
 * @brief 检查裁判v1.2的资格门槛、分项封顶、跨场隔离及同分比较。
 *
 * @details 模块协作与维护约定
 * 验证对象：practice_rules.hpp。
 * 运行约定：由独立测试可执行文件/CTest执行，断言失败用于定位回归，不是正式选手的逐帧逻辑。
 * 结果边界：通过表示这些构造场景满足约束，不证明所有随机地图均最优或稳定达到某个总分。
 */
#include "rmtc_connection/practice_rules.hpp"
#include <iostream>
#include <stdexcept>
using Json = nlohmann::json;
/** @brief 对规则结果作独立断言，发布构建下也保留检查。 */
void require(bool value, const char* message) {if (!value) throw std::runtime_error(message);}
/** @brief 构造具有三个原始已完成任务的结算记录，用来验证“完成”与“计分资格”不同。 */
Json robot(const std::string& team, double resource, double damage) {
  return {{"team",team}, {"resource_value",resource}, {"effective_damage",damage},
      {"mission_completed",{"normal","high","reach"}}, {"invalid_actions",0},
      {"hp",100}, {"alive",true}, {"last_action",{{"action","wait"}}}};
}
/** @brief 逐条核对封顶、任务资格、舍入排序与完全同分，避免沿用旧练习包的裸任务分。 */
int main() {
  try {
    Json batch{{"tick",499}, {"session_id","current"}, {"state",{{"tick",500},{"stage",2},
      {"map",{{"width",24},{"height",24}}},{"obstacles",Json::array()}, {"resources",Json::array()},
      {"robots",Json::array({robot("A",120,500),robot("B",120,600)})}}}};
    auto score = rmtc::practice::rules_scoreboard(batch, {});
    require(score["teams"][0]["score"] == 45, "AI资格缺失时任务必须不计分");
    require(score["teams"][0]["team"] == "B", "超过500的伤害必须用于同分比较");
    require(score["teams"][0]["mission_completed_raw"] == 3 && score["teams"][0]["mission_completed_eligible"] == 0, "保留原始任务并清零计分任务");
    const Json records = Json::array({
      {{"session_id","current"},{"seat","A"},{"mode","mock"},{"success",true},{"status",200}},
      {{"session_id","old"},{"seat","A"},{"mode","real"},{"success",true},{"status",200}},
      {{"session_id","current"},{"seat","B"},{"mode","real"},{"success",true},{"status",201}}});
    auto qualified = rmtc::practice::qualified_teams(records,"current");
    require(qualified.size() == 1 && qualified.count("B"), "仅当前席位真实成功调用授予资格，2xx均有效");
    score = rmtc::practice::rules_scoreboard(batch, qualified);
    require(score["teams"][0]["score"] == 60, "符合资格的满分必须为60");
    batch["state"]["robots"][0] = robot("A",120,600);
    score = rmtc::practice::rules_scoreboard(batch, {"A","B"});
    require(score["teams"][0]["tied"] == true && score["teams"][1]["rank"] == 1, "完全同分不得按席位字母判胜负");
    batch["state"]["robots"][0]["effective_damage"] = 600.0000000001;
    score = rmtc::practice::rules_scoreboard(batch, {"A","B"});
    require(score["teams"][0]["tied"] == true, "九位以后差值不改变排名");
    batch["state"]["stage"] = 1; batch["state"]["robots"] = Json::array({robot("A",200,999)});
    score = rmtc::practice::rules_scoreboard(batch, {});
    require(score["teams"][0]["score"] == 40, "第一阶段不需要AI资格、伤害或任务");
    std::cout << "practice rules: all checks passed\n";
  } catch (const std::exception& error) {std::cerr << error.what() << '\n';return 1;}
}
