/**
 * @file mapping_node_test.cpp
 * @brief 在隔离ROS域内验证真实订阅、地图发布和文字快照的清理约定。
 *
 * 检查场景刷新、文字清理、历史实体淡色/确认空位标记及Tick年龄数字更新。
 * 只向本测试域内的C队感知话题发布构造帧，不连接或控制实际裁判。
 */
#include "rmtc_connection/mapping_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <set>
#include <stdexcept>
#include <thread>
#include <utility>

/** @brief 检查必须成立的条件，Release构建也不会省略。 */
void require(bool condition, const std::string& reason) {
  if (!condition)
    throw std::runtime_error(reason);
}

/** @brief 驱动真实ROS回调直到条件成立，5秒超时则报告通信未完成。 */
void until(rclcpp::executors::SingleThreadedExecutor& executor,
           const std::function<bool()>& condition) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (condition())
      return;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  throw std::runtime_error("isolated ROS message exchange timed out");
}

/** @brief 为隔离测试生成某实体相对本车的位置图，不使用裁判内部数据。 */
std::string entity_map(int dx, int dy) {
  std::string relative;
  for (int y = 0; y < 9; ++y) {
    if (y)
      relative += '\n';
    for (int x = 0; x < 9; ++x)
      relative += (y == 4 && x == 4) ? '1' : ((y == 4 + dy && x == 4 + dx) ? '2' : '0');
  }
  return relative;
}

/** @brief 构造资源、敌车和合法完整公开字段；仅供独立ROS域的接收/显示测试。 */
rmtc::Json sample() {
  return {{"session_id", "isolated-mapping-test"},
          {"tick", 7},
          {"vision_radius", 1},
          {"self",
           {{"team", "C"},
            {"robot_id", "C"},
            {"position", {1, 1}},
            {"hp", 100},
            {"alive", true},
            {"defending", false},
            {"respawn_in", 0},
            {"protected", false},
            {"last_action", nullptr}}},
          {"map", {{"width", 5}, {"height", 4}}},
          {"visible_entities", rmtc::Json::array({{{"type", "resource_normal"},
                                                   {"id", "test-r"},
                                                   {"position", entity_map(1, 0)},
                                                   {"value", 10}},
                                                  {{"type", "robot"},
                                                   {"id", "test-b"},
                                                   {"team", "B"},
                                                   {"robot_id", "B"},
                                                   {"position", entity_map(-1, 0)},
                                                   {"hp", 80}}})},
          {"score",
           {{"resource_value", 0},
            {"effective_damage", 0},
            {"mission_completed", 0},
            {"mission_completed_ids", rmtc::Json::array()},
            {"invalid_actions", 0}}},
          {"mission_text", ""},
          {"missions", rmtc::Json::array()},
          {"event_text", ""}};
}

/** @brief 运行独立节点通信测试；正常退出0，失败打印原因并清理ROS上下文。 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto mapper = std::make_shared<rmtc::MappingNode>(
        rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("team", "C")}));
    auto fixture = std::make_shared<rclcpp::Node>("mapping_fixture");
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;
    visualization_msgs::msg::MarkerArray::ConstSharedPtr markers;
    int map_count = 0;
    auto map_sub = fixture->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/rmtc/team_C/map", latched, [&](nav_msgs::msg::OccupancyGrid::ConstSharedPtr value) {
          map = std::move(value);
          ++map_count;
        });
    auto marker_sub = fixture->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/rmtc/team_C/markers", latched,
        [&](visualization_msgs::msg::MarkerArray::ConstSharedPtr value) {
          markers = std::move(value);
        });
    auto input = fixture->create_publisher<std_msgs::msg::String>("/team_C/perception",
                                                                  rclcpp::QoS(1).reliable());
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(mapper);
    executor.add_node(fixture);
    until(executor, [&]() { return input->get_subscription_count() == 1 && markers != nullptr; });
    require(!map, "invented map before first perception");

    auto frame = sample();
    std_msgs::msg::String message;
    message.data = frame.dump();
    input->publish(message);
    until(executor, [&]() { return map_count == 1 && markers->markers.size() > 10; });
    require(map->header.frame_id == "rmtc_C_map" && map->info.width == 5 && map->info.height == 4 &&
                map->data[rmtc::MapMemory::index({4, 3}, {5, 4})] == -1,
            "grid frame/dimensions/unknown incorrect");
    require(markers->markers.front().action == visualization_msgs::msg::Marker::DELETEALL &&
                markers->markers.front().ns.empty(),
            "DELETEALL must clear all namespaces");
    std::set<std::pair<std::string, int>> ids;
    bool resource = false;
    float visible_alpha = 0;
    for (const auto& marker : markers->markers) {
      require(ids.insert({marker.ns, marker.id}).second, "duplicate marker identity in snapshot");
      require(marker.text.find("Waiting") == std::string::npos,
              "waiting text survived valid perception");
      if (marker.ns == "visible_resources") {
        resource = true;
        visible_alpha = marker.color.a;
      }
      if (marker.ns == "status" && marker.id == 0)
        require(marker.text.find("Tick 7") != std::string::npos, "status ID is not stable");
    }
    require(resource, "current visible resource not rendered");

    frame["tick"] = 8;
    frame["visible_entities"] = rmtc::Json::array();
    message.data = frame.dump();
    input->publish(message);
    until(executor, [&]() {
      return map_count == 2 &&
             std::any_of(markers->markers.begin(), markers->markers.end(), [](const auto& marker) {
               return marker.ns == "status" && marker.id == 0 &&
                      marker.text.find("Tick 8") != std::string::npos;
             });
    });
    // 消失后不能继续显示实时实体；记录保留为X，并标明最后看到是1Tick前。
    bool absent_resource = false;
    bool absent_robot = false;
    bool memory_age = false;
    for (const auto& marker : markers->markers) {
      require(marker.ns != "visible_resources" && marker.ns != "visible_robots" &&
                  marker.ns != "entity_labels",
              "missing entity was still drawn as currently visible");
      if (marker.ns == "absent_resources")
        absent_resource = marker.type == visualization_msgs::msg::Marker::LINE_LIST;
      if (marker.ns == "absent_robots")
        absent_robot = marker.type == visualization_msgs::msg::Marker::LINE_LIST;
      if (marker.ns == "memory_labels" && marker.text.find("\n1t") != std::string::npos)
        memory_age = true;
      if (marker.ns == "status" && marker.id == 0)
        require(marker.text.find("Tick 8") != std::string::npos, "header retained previous Tick");
    }
    require(absent_resource && absent_robot && memory_age,
            "absent entities/age labels not rendered");

    // 新场次移出视野：实体保持原形状但变淡，不能被错误标为确认空位。
    frame = sample();
    frame["session_id"] = "out-of-view-test";
    frame["tick"] = 20;
    message.data = frame.dump();
    input->publish(message);
    until(executor, [&]() { return map_count == 3; });
    frame["tick"] = 22;
    frame["self"]["position"] = {4, 3};
    frame["visible_entities"] = rmtc::Json::array();
    message.data = frame.dump();
    input->publish(message);
    until(executor, [&]() {
      return map_count == 4 &&
             std::any_of(markers->markers.begin(), markers->markers.end(), [](const auto& marker) {
               return marker.ns == "memory_labels" && marker.text.find("\n2t") != std::string::npos;
             });
    });
    bool remembered_robot = false;
    bool remembered_resource = false;
    for (const auto& marker : markers->markers) {
      require(marker.ns != "absent_robots" && marker.ns != "absent_resources",
              "out-of-view became confirmed empty");
      if (marker.ns == "remembered_robots")
        remembered_robot = marker.type == visualization_msgs::msg::Marker::CYLINDER &&
                           marker.color.a > 0 && marker.color.a < visible_alpha;
      if (marker.ns == "remembered_resources")
        remembered_resource = marker.type == visualization_msgs::msg::Marker::SPHERE &&
                              marker.color.a > 0 && marker.color.a < visible_alpha;
    }
    require(remembered_robot && remembered_resource,
            "historical entities lost their shape or were not faded");
    // 两阶段尺寸下检查四角、四边与中部出生；真实经过JSON相对实体解析和ROS发布。
    // 每次切换场次应清图，网格原点按棋盘固定，车辆/资源/视野不能按出生点平移。
    for (const int size : {20, 24}) {
      for (const rmtc::Position spawn : {
               rmtc::Position{0, 0}, {size - 1, 0}, {0, size - 1}, {size - 1, size - 1},
               {size / 2, 0}, {0, size / 2}, {size - 1, size / 2}, {size / 2, size - 1},
               {size / 2, size / 2}, {size / 3, size * 2 / 3}}) {
        const int dx = spawn.x + 1 < size ? 1 : -1;
        const int expected_count = map_count + 1;
        frame = sample();
        frame["session_id"] = "random-birth-display-" + std::to_string(expected_count);
        frame["tick"] = 17;
        frame["vision_radius"] = 4;
        frame["map"] = {{"width", size}, {"height", size}};
        frame["self"]["position"] = {spawn.x, spawn.y};
        frame["visible_entities"] = rmtc::Json::array(
            {{{"type", "resource_normal"}, {"id", "birth-resource"},
              {"position", entity_map(dx, 0)}, {"value", 10}}});
        message.data = frame.dump();
        input->publish(message);
        until(executor, [&]() {
          return map_count == expected_count &&
                 std::any_of(markers->markers.begin(), markers->markers.end(),
                     [&](const auto& marker) {
                       return marker.ns == "self" &&
                              marker.pose.position.x == spawn.x + 0.5 - size / 2.0 &&
                              marker.pose.position.y == size / 2.0 - spawn.y - 0.5;
                     });
        });
        require(map->info.width == static_cast<unsigned>(size) &&
                    map->info.height == static_cast<unsigned>(size) &&
                    map->info.origin.position.x == -size / 2.0 &&
                    map->info.origin.position.y == -size / 2.0,
                "RViz grid origin moved with random spawn");
        for (int y = 0; y < size; ++y)
          for (int x = 0; x < size; ++x)
            require(map->data[rmtc::MapMemory::index({x, y}, {size, size})] ==
                        (std::abs(x - spawn.x) <= 4 && std::abs(y - spawn.y) <= 4 ? 0 : -1),
                    "random spawn exposed terrain beyond its clipped vision");
        bool self_position = false, resource_position = false, vision_position = false;
        for (const auto& marker : markers->markers) {
          const double cx = spawn.x + 0.5 - size / 2.0;
          const double cy = size / 2.0 - spawn.y - 0.5;
          if (marker.ns == "self")
            self_position = marker.pose.position.x == cx && marker.pose.position.y == cy;
          if (marker.ns == "visible_resources")
            resource_position = marker.pose.position.x == cx + dx && marker.pose.position.y == cy;
          if (marker.ns == "vision") {
            vision_position = marker.points.size() == 5 &&
                marker.points[0].x == -size / 2.0 + std::max(0, spawn.x - 4) &&
                marker.points[0].y == size / 2.0 - std::max(0, spawn.y - 4) &&
                marker.points[2].x == -size / 2.0 + std::min(size, spawn.x + 5) &&
                marker.points[2].y == size / 2.0 - std::min(size, spawn.y + 5);
          }
        }
        require(self_position && resource_position && vision_position,
                "random birth misplaced RViz robot/resource/vision markers");
      }
    }
    executor.remove_node(fixture);
    executor.remove_node(mapper);
    rclcpp::shutdown();
    std::cout << "isolated ROS mapping, scene cleanup and stable text IDs passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
