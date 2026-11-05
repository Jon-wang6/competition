/**
 * @file mapping_events_test.cpp
 * @brief 隔离ROS域验证八类事件显示、全长分裂墙、到期还原及AI来源校验。
 * 不调用模型、不连接裁判；构造的AI结构通过与真实回复相同的接收与校验路径。
 *
 * @details 模块协作与维护约定
 * 验证对象：mapping_node.hpp。
 * 运行约定：由独立测试可执行文件/CTest执行，断言失败用于定位回归，不是正式选手的逐帧逻辑。
 * 结果边界：通过表示这些构造场景满足约束，不证明所有随机地图均最优或稳定达到某个总分。
 */
#include "rmtc_connection/mapping_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace {
using rmtc::Json;
using Marker = visualization_msgs::msg::Marker;

/** @brief Release构建仍执行断言，失败时打印可定位的场景说明。 */
void require(bool condition, const char* reason) {
  if (!condition)
    throw std::runtime_error(reason);
}

/** @brief 最多等待5秒驱动隔离域回调，不依赖固定的DDS发现耗时。 */
void until(rclcpp::executors::SingleThreadedExecutor& executor,
           const std::function<bool()>& condition) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < end) {
    executor.spin_some();
    if (condition()) return;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  throw std::runtime_error("event visualization exchange timed out");
}

/** @brief 仅构造本队公开字段，地图大于视野以验证区域外的事件墙也能显示。 */
Json frame(int tick, const std::string& session = "mapping-events") {
  std::string relative;
  for (int y = 0; y < 9; ++y) {
    if (y) relative += '\n';
    relative += y == 4 ? "000010000" : "000000000";
  }
  return {{"session_id", session}, {"tick", tick}, {"vision_radius", 1},
          {"map", {{"width", 24}, {"height", 24}}},
          {"self", {{"team", "C"}, {"robot_id", "C"}, {"position", {1, 1}},
                    {"hp", 100}, {"alive", true}, {"defending", false},
                    {"respawn_in", 0}, {"protected", false}, {"last_action", nullptr}}},
          {"visible_entities", Json::array()}, {"entity_map", relative},
          {"score", {{"resource_value", 0}, {"effective_damage", 0},
                     {"mission_completed", 0}, {"mission_completed_ids", Json::array()},
                     {"invalid_actions", 0}}},
          {"missions", Json::array()}, {"mission_text", ""},
          {"event_text", "opaque public announcement"}};
}

/** @brief 八类有效时间线含完整十字坐标依据；模型解析内容不依赖公告模板匹配。 */
Json reply(bool mock = false) {
  Json events = Json::array({
      {{"kind", "explosion"}, {"begin_tick", 10}, {"end_tick", 11},
       {"rectangles", {{3, 3, 5, 5}}}, {"damage", 80}},
      {{"kind", "illusion"}, {"begin_tick", 10}, {"end_tick", 11},
       {"rectangles", {{6, 3, 7, 5}}}, {"effect_ticks", 5}},
      {{"kind", "angel"}, {"begin_tick", 10}, {"end_tick", 20},
       {"points", {{3, 9}, {14, 9}}}, {"effect_ticks", 10}, {"heal_per_tick", 10}},
      {{"kind", "unrivaled"}, {"begin_tick", 10}, {"end_tick", 20},
       {"rectangles", {{11, 11, 13, 13}}}, {"stay_ticks", 10}, {"attack_radius", 2},
       {"attack_damage", 50}, {"defended_damage", 10}},
      {{"kind", "world_split"}, {"begin_tick", 10}, {"end_tick", 20}, {"cross_point", {8, 8}}},
      {{"kind", "pepper"}, {"begin_tick", 10}, {"end_tick", 20},
       {"points", {{5, 6}, {12, 11}}}, {"damage", 50}},
      {{"kind", "purity"}, {"begin_tick", 10}, {"end_tick", 20}, {"reflection_damage", 20}},
      {{"kind", "pan"}, {"begin_tick", 10}, {"end_tick", 30},
       {"movement_window_ticks", 10}, {"damage_multiplier", 2}}});
  return {{"session_id", "mapping-events"}, {"team", "C"}, {"source_tick", 1},
          {"source_event_text", "opaque public announcement"}, {"completion_received", true},
          {"mock", mock}, {"interpretation_valid", true}, {"event_understood", true},
          {"event_kind", "multiple"}, {"event_plan", {{"events", events}}}};
}
}  // namespace

/** @brief 验证实际订阅、发布、消息乱序和换局清理；所有状态通过ROS消息交换检查。 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto mapper = std::make_shared<rmtc::MappingNode>(
        rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("team", "C")}));
    auto fixture = std::make_shared<rclcpp::Node>("mapping_event_fixture");
    const auto latched = rclcpp::QoS(4).reliable().transient_local();
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;
    visualization_msgs::msg::MarkerArray::ConstSharedPtr markers;
    int maps = 0;
    int scenes = 0;
    auto map_sub = fixture->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/rmtc/team_C/map", latched, [&](nav_msgs::msg::OccupancyGrid::ConstSharedPtr value) {
          map = std::move(value); ++maps;
        });
    auto marker_sub = fixture->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/rmtc/team_C/markers", latched,
        [&](visualization_msgs::msg::MarkerArray::ConstSharedPtr value) {
          markers = std::move(value); ++scenes;
        });
    auto perception = fixture->create_publisher<std_msgs::msg::String>("/team_C/perception", rclcpp::QoS(1));
    auto ai = fixture->create_publisher<std_msgs::msg::String>("/rmtc/team_C/ai_result", latched);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(mapper); executor.add_node(fixture);
    until(executor, [&] { return perception->get_subscription_count() == 1 && ai->get_subscription_count() == 1; });
    const auto send = [&](const auto& publisher, const Json& data) {
      std_msgs::msg::String message; message.data = data.dump();
      const int old = maps;
      const int old_scene = scenes;
      publisher->publish(message);
      until(executor, [&] { return maps > old && scenes > old_scene && markers != nullptr; });
    };
    const auto cell = [&](int x, int y) { return map->data[rmtc::MapMemory::index({x, y}, {24, 24})]; };
    const auto has_text = [&](const std::string& value) {
      return std::any_of(markers->markers.begin(), markers->markers.end(), [&](const auto& marker) {
        return marker.text.find(value) != std::string::npos;
      });
    };
    auto f = frame(0);
    f["self"]["position"] = {1, 7};
    auto relative_wall = f["entity_map"].get<std::string>();
    relative_wall[5 * 10 + 5] = '2';
    f["visible_entities"] = Json::array({{{"type", "obstacle"}, {"position", relative_wall}}});
    send(perception, f);
    send(perception, frame(1));
    send(ai, reply(true));
    require(!has_text("WORLD SPLIT"), "mock created event markers");
    send(ai, reply());
    require(cell(8, 23) == -1, "preview blocked unseen terrain early");
    for (const auto* kind : {"EXPLOSION", "ILLUSION", "ANGEL", "UNRIVALED", "WORLD SPLIT", "PEPPER", "PURITY", "VULNERABILITY"})
      require(has_text(kind), "one of eight event previews was missing");
    const auto heading = [&] {
      for (const auto& m : markers->markers) if (m.ns == "status" && m.id == 0) return m.text.substr(m.text.find("Explored"));
      return std::string{};
    };
    const auto explored = heading();
    send(perception, frame(10));
    for (int i = 0; i < 24; ++i)
      require(cell(8, i) == 100 && cell(i, 8) == 100, "world split did not cover the whole row and column");
    require(heading() == explored, "inferred event walls inflated actual exploration");
    require(has_text("WORLD SPLIT ACTIVE 10t"), "active countdown missing");
    require(std::any_of(markers->markers.begin(), markers->markers.end(), [](const auto& marker) {
      return marker.ns == "event_areas" && marker.points.size() == 47;
    }), "complete split wall marker missing");

    // 一项公开授予的幻术Buff，即使施放区域已经消失也要继续标注。
    auto grant = reply(); grant["source_tick"] = 11;
    grant["event_plan"]["events"] = Json::array({{{"kind", "illusion"}, {"actor", "C"},
        {"begin_tick", 10}, {"end_tick", 25}}});
    send(ai, grant);  // 先于对应感知到达，必须等到Tick11才能合并。
    require(!has_text("C: ILLUSION BUFF"), "future AI result applied before its source tick");
    send(perception, frame(11));
    require(has_text("C: ILLUSION BUFF") && has_text("14t left"), "individual buff countdown missing");
    send(perception, frame(20));
    require(cell(1, 8) == 0 && cell(2, 8) == 100 && cell(8, 23) == -1,
            "split expiry did not restore free/permanent/unknown terrain correctly");
    require(!has_text("WORLD SPLIT") && has_text("C: ILLUSION BUFF"), "expired region or lasting buff was drawn incorrectly");
    send(perception, frame(40));
    require(std::none_of(markers->markers.begin(), markers->markers.end(), [](const auto& marker) {
      return marker.ns.rfind("event_", 0) == 0;
    }), "expired event markers survived");
    send(perception, frame(1, "next-session"));
    std_msgs::msg::String old; old.data = reply().dump(); ai->publish(old);
    send(perception, frame(2, "next-session"));
    require(!has_text("WORLD SPLIT") && cell(8, 23) == -1, "old session AI contaminated new map");
    auto changed=frame(3,"next-session");
    auto added=changed["entity_map"].get<std::string>();added[4*10+5]='2';
    changed["visible_entities"]=Json::array({{{"type","obstacle"},{"position",added}}});
    send(perception,changed);
    require(cell(2,1)==100&&has_text("SUSPECTED WORLD SPLIT"),
            "observed new wall without AI lacked temporary-layer marker");
    auto away=frame(4,"next-session");away["self"]["position"]={18,18};
    away["visible_entities"]=changed["visible_entities"];
    send(perception,away);
    require(cell(2,1)==100&&std::any_of(markers->markers.begin(),markers->markers.end(),[](const auto& m){
      return m.ns=="event_inferred_split"&&m.points.size()==1;}),
      "unseen static wall was guessed temporary or offscreen evidence disappeared");
    send(perception,frame(5,"next-session"));
    require(cell(2,1)==0&&cell(19,18)==100&&!has_text("SUSPECTED WORLD SPLIT"),
            "observed split recovery changed first-seen static wall or left stale marker");
    executor.remove_node(fixture); executor.remove_node(mapper);
    rclcpp::shutdown();
    std::cout << "Eight event overlays, full split wall, expiry, buff and session binding passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
