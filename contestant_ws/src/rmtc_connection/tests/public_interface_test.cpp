/**
 * @file public_interface_test.cpp
 * @brief 离线验证完整公开数据接口；不启动ROS、裁判或新订阅。
 *
 * 测试覆盖全部公开感知字段、四类实体坐标、可选字段、原文保留和坏数据，
 * 并确认v1.1伤害统计下降、超过500及死亡后任务完成记录仍可正确接收。
 * 最后确认接收器仅构造时保持停用，符合“先做好接口，暂不调用”的要求。
 */
#include "rmtc_connection/public_receiver.hpp"

#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

using rmtc::Json;

/** @brief 测试条件失败时抛出说明，Release构建下也执行检查。 */
void require(bool condition, const std::string& message) {
  if (!condition)
    throw std::runtime_error(message);
}

/** @brief 验证非法输入被标准异常拒绝，不启动任何ROS通信。 */
void rejects(const std::function<void()>& operation, const std::string& message) {
  bool failed = false;
  try {
    operation();
  } catch (const std::exception&) {
    failed = true;
  }
  require(failed, message);
}

/** @brief 为[-4,4]范围内的测试位移生成89字符位置图；同格只保留中心1。 */
std::string entity_map(int dx, int dy) {
  std::string result;
  for (int row = 0; row < 9; ++row) {
    if (row)
      result += '\n';
    for (int col = 0; col < 9; ++col) {
      result += row == 4 && col == 4 ? '1' : (row == dy + 4 && col == dx + 4 ? '2' : '0');
    }
  }
  return result;
}

/** @brief 构造带全部公开字段的测试帧，坐标和公告仅用于离线解析测试。 */
Json sample() {
  return {
      {"tick", 148},
      {"session_id", "test-public"},
      {"vision_radius", 4},
      {"self",
       {{"team", "A"},
        {"robot_id", "A"},
        {"position", {8, 8}},
        {"hp", 70.5},
        {"alive", true},
        {"defending", false},
        {"respawn_in", 0},
        {"protected", true},
        {"last_action",
         {{"tick", 147}, {"action", "move"}, {"moved", true}, {"effective_direction", "right"}}}}},
      {"visible_entities", Json::array({{{"type", "robot"},
                                         {"id", "robot_B"},
                                         {"team", "B"},
                                         {"robot_id", "B"},
                                         {"position", entity_map(1, -1)},
                                         {"hp", 60.25},
                                         {"defending", true},
                                         {"protected", false}},
                                        {{"type", "resource_normal"},
                                         {"id", "n01"},
                                         {"position", entity_map(1, 0)},
                                         {"value", 10}},
                                        {{"type", "resource_high"},
                                         {"id", "h01"},
                                         {"position", entity_map(0, 0)},
                                         {"value", 20}},
                                        {{"type", "obstacle"}, {"position", entity_map(-1, 2)}}})},
      {"score",
       {{"resource_value", 30},
        {"effective_damage", 12.5},
        {"mission_completed", 1},
        {"mission_completed_ids", {"collect_high_value"}},
        {"invalid_actions", 0}}},
      {"mission_text", "收集两个普通资源、一个高价值资源，并到达[9,9]。"},
      {"missions",
       Json::array({{{"id", "collect_two_normal"}, {"type", "normal_resources"}, {"target", 2}},
                    {{"id", "collect_high_value"}, {"type", "high_resources"}, {"target", 1}},
                    {{"id", "reach_target"}, {"type", "reach"}, {"position", {9, 9}}}})},
      {"map", {{"width", 24}, {"height", 24}}},
      {"event_text", "第一条公开公告。\n第二条公开公告，保留原文。"}};
}

/**
 * @brief 验证v1.1结算后的伤害快照与任务记录完整保留。
 * 连续构造D超过500、死亡扣减、扣至0和复活的公开帧，确保接口不截断、
 * 不强制单调递增，也不根据D自行撤销裁判已完成的任务或重算资源统计。
 * 本测试仅检查接收端语义，不替代官方裁判的伤害、奖励和死亡结算。
 */
void check_v11_score_snapshots() {
  auto raw = sample();
  raw["score"]["mission_completed"] = 2;
  raw["score"]["mission_completed_ids"] = {"collect_high_value", "damage_goal"};
  const std::vector<double> snapshots{560.5, 500.5, 440.5, 0.0, 12.5};
  for (std::size_t index = 0; index < snapshots.size(); ++index) {
    raw["tick"] = 148 + index;
    raw["self"]["last_action"]["tick"] = 147 + index;
    raw["score"]["effective_damage"] = snapshots[index];
    raw["self"]["alive"] = index == 0 || index == snapshots.size() - 1;
    raw["self"]["hp"] = raw["self"]["alive"] == true ? 100 : 0;
    raw["self"]["respawn_in"] = raw["self"]["alive"] == true ? 0 : 2;
    const auto frame = rmtc::parse_public_perception(raw.dump(), "A");
    require(frame.score.effective_damage == snapshots[index],
            "v1.1 damage snapshot was capped, accumulated or refused after decrease");
    require(frame.score.resource_value == 30 && frame.score.mission_completed == 2 &&
                frame.score.mission_completed_ids ==
                    std::vector<std::string>{"collect_high_value", "damage_goal"},
            "v1.1 death/decreased damage changed retained resources or completed missions");
    require(frame.raw == raw, "v1.1 score snapshot changed raw perception");
  }
}

/** @brief 执行离线检查；成功返回0，任一检查失败则打印原因并返回1。 */
int main() {
  try {
    check_v11_score_snapshots();
    const auto raw = sample();
    const auto frame = rmtc::parse_public_perception(raw.dump(), "A");
    require(frame.tick == 148 && frame.session_id == "test-public" && frame.vision_radius == 4,
            "frame identity not preserved");
    require(frame.self.team == "A" && frame.self.robot_id == "A" && frame.self.hp == 70.5 &&
                frame.self.alive && !frame.self.defending && frame.self.spawn_protected &&
                frame.self.respawn_in == 0 && !frame.self.position_reason,
            "self fields not preserved");
    require(frame.self.last_action && frame.self.last_action->tick == 147 &&
                frame.self.last_action->action == "move" && frame.self.last_action->moved &&
                frame.self.last_action->effective_direction == "right" &&
                !frame.self.last_action->error,
            "receipt fields not preserved");
    require(frame.visible_entities.size() == 4, "entity list incomplete");
    const auto& robot = frame.visible_entities[0];
    require(robot.type == "robot" && robot.id == "robot_B" && robot.team == "B" &&
                robot.robot_id == "B" && robot.hp == 60.25 && robot.defending == true &&
                robot.spawn_protected == false && robot.relative_position.x == 1 &&
                robot.relative_position.y == -1 && robot.position.x == 9 && robot.position.y == 7 &&
                robot.position_text == entity_map(1, -1),
            "robot fields or coordinates incorrect");
    require(frame.visible_entities[1].value == 10 && frame.visible_entities[1].position.x == 9 &&
                frame.visible_entities[2].value == 20 &&
                frame.visible_entities[2].position.x == 8 &&
                frame.visible_entities[2].position.y == 8 &&
                frame.visible_entities[3].position.x == 7 &&
                frame.visible_entities[3].position.y == 10 && !frame.visible_entities[3].id &&
                !frame.visible_entities[3].hp,
            "resource/obstacle fields or overlap coordinates incorrect");
    require(frame.score.resource_value == 30 && frame.score.effective_damage == 12.5 &&
                frame.score.mission_completed == 1 && frame.score.invalid_actions == 0 &&
                frame.score.mission_completed_ids == std::vector<std::string>{"collect_high_value"},
            "score fields not preserved");
    require(frame.missions.size() == 3 && frame.missions[0].target == 2 &&
                !frame.missions[0].position && !frame.missions[2].target &&
                frame.missions[2].type == "reach" && frame.missions[2].position->x == 9 &&
                frame.missions[2].position->y == 9 && frame.map.width == 24 &&
                frame.map.height == 24,
            "mission or map data incorrect");
    require(frame.mission_text == raw.at("mission_text") &&
                frame.event_text == raw.at("event_text") && frame.raw == raw &&
                robot.raw == raw["visible_entities"][0],
            "raw public input was changed");

    // 开局、死亡与重生：可选信息缺省不应被当成一个伪造的回执或定位原因。
    auto value = sample();
    value["self"]["last_action"] = nullptr;
    value["self"]["position_reason"] = "initial";
    require(!rmtc::parse_public_perception(value.dump(), "A").self.last_action,
            "null receipt rejected");
    value["self"]["position_reason"] = "respawn";
    value["self"]["position"] = {4, 3};
    value["visible_entities"] = Json::array();
    require(rmtc::parse_public_perception(value.dump(), "A").self.position_reason == "respawn",
            "respawn position_reason missing");
    value["self"]["alive"] = false;
    value["self"]["hp"] = 0;
    value["self"]["respawn_in"] = 2;
    require(rmtc::parse_public_perception(value.dump(), "A").self.respawn_in == 2,
            "dead frame rejected");
    value = sample();
    value["self"]["last_action"] = {
        {"tick", 147}, {"action", "move"}, {"moved", false}, {"error", "E_BLOCKED"}};
    require(rmtc::parse_public_perception(value.dump(), "A").self.last_action->error == "E_BLOCKED",
            "receipt error missing");

    // 坏数据必须拒绝；保持接口不依赖任何结构化事件或全图真值。
    rejects([] { rmtc::parse_public_perception("{", "A"); }, "bad JSON accepted");
    rejects([&] { rmtc::parse_public_perception(raw.dump(), "B"); }, "wrong team accepted");
    value = sample();
    value["vision_radius"] = 5;
    rejects([&] { rmtc::parse_public_perception(value.dump(), "A"); }, "bad radius accepted");
    value = sample();
    value["self"].erase("protected");
    rejects([&] { rmtc::parse_public_perception(value.dump(), "A"); }, "missing status accepted");
    value = sample();
    value["self"]["hp"] = "70";
    rejects([&] { rmtc::parse_public_perception(value.dump(), "A"); }, "string hp accepted");
    value = sample();
    value["map"]["width"] = 24.5;
    rejects([&] { rmtc::parse_public_perception(value.dump(), "A"); },
            "floating map size accepted");
    value = sample();
    value["score"]["effective_damage"] = true;
    rejects([&] { rmtc::parse_public_perception(value.dump(), "A"); }, "boolean damage accepted");
    value = sample();
    value["tick"] = std::numeric_limits<std::uint64_t>::max();
    rejects([&] { rmtc::parse_public_perception(value.dump(), "A"); }, "tick overflow accepted");
    value = sample();
    value["visible_entities"][0]["position"] = "bad";
    rejects([&] { rmtc::parse_public_perception(value.dump(), "A"); }, "bad entity map accepted");
    value = sample();
    value["event_text"] = Json::object();
    rejects([&] { rmtc::parse_public_perception(value.dump(), "A"); },
            "structured event input accepted");

    const auto teams = rmtc::parse_team_registry(
        R"({"teams":[{"team":"A","name":"本队"},{"team":"B","name":"另一队"}]})");
    require(teams.size() == 2 && teams[1].team == "B" && teams[1].name == "另一队",
            "registry incorrect");
    rejects(
        [] {
          rmtc::parse_team_registry(
              R"({"teams":[{"team":"A","name":"一"},{"team":"A","name":"二"}]})");
        },
        "duplicate seat accepted");
    rejects([] { rmtc::parse_team_registry(R"({"teams":{}})"); }, "registry object accepted");

    // 仅构造接口：没有rclcpp::init或start，不会改变运行中的ROS节点。
    require(!rclcpp::ok(), "test unexpectedly initialized ROS");
    int callbacks = 0;
    rmtc::PublicCallbacks handlers;
    handlers.on_perception = [&](const rmtc::PublicPerception&) { ++callbacks; };
    rmtc::PublicReceiver receiver("A", handlers);
    require(!receiver.active() && receiver.topics().perception == "/team_A/perception" &&
                receiver.topics().mission == "/referee/mission" &&
                receiver.topics().event_text == "/referee/event_text" &&
                receiver.topics().team_registry == "/referee/team_registry",
            "receiver topics or initial state incorrect");
    receiver.stop();
    require(!receiver.active() && callbacks == 0 && !rclcpp::ok(),
            "dormant interface activated ROS");
    rejects([] { rmtc::PublicReceiver invalid("Z"); }, "invalid receiver seat accepted");
    std::cout << "full public data parsing and dormant receiver checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
