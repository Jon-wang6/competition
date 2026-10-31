/** @file public_probe.cpp
 * @brief 隔离域191的公开协议回归驱动，逐Tick按测试数据发送候选及畸形消息。
 * 仅用于裁判审计，不安装到选手程序，不连接默认比赛域27。
 */
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <thread>

/** @brief 根据构造脚本回填当前场次；同Tick重发只重发最后一次合法缓存动作。 */
int main(int argc, char** argv) {
  if (argc != 3 || !std::getenv("ROS_DOMAIN_ID") || std::string(std::getenv("ROS_DOMAIN_ID")) != "191") return 2;
  using Json = nlohmann::json;
  const auto script = Json::parse(std::ifstream(argv[1]));
  const int max_tick = script.at("max_ticks");
  std::ofstream output(argv[2]);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("public_rule_probe");
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subs;
  std::map<std::string,rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> pubs;
  std::map<std::string,rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> registrations;
  std::map<std::string,int> seen;
  int received_final = 0;
  for (const auto& seat_value : script.at("seats")) {
    const std::string seat = seat_value;
    pubs[seat] = node->create_publisher<std_msgs::msg::String>("/team_"+seat+"/action",rclcpp::QoS(1));
    registrations[seat] = node->create_publisher<std_msgs::msg::String>("/team_"+seat+"/registration",rclcpp::QoS(1).transient_local());
    seen[seat] = -1;
    subs.push_back(node->create_subscription<std_msgs::msg::String>("/team_"+seat+"/perception",rclcpp::QoS(1),
      [&,seat](std_msgs::msg::String::SharedPtr message) {
        auto frame = Json::parse(message->data);
        const int tick = frame.at("tick");
        if (seen[seat] > tick) return;
        const bool first = seen[seat] < tick;
        seen[seat] = tick; output << Json{{"seat",seat},{"frame",frame}}.dump() << std::endl;
        if (tick >= max_tick) {
          if (first && script.value("send_after_final",false)) {
            std_msgs::msg::String action;
            action.data = Json{{"tick",tick},{"session_id",frame.at("session_id")},
                {"action","move"},{"direction","right"}}.dump();
            pubs[seat]->publish(action);
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
          }
          if (first && ++received_final == static_cast<int>(script.at("seats").size())) rclcpp::shutdown();
          return;
        }
        const auto steps = script.value("steps",Json::object());
        const auto plan = steps.value(std::to_string(tick),Json::object());
        for (auto item : plan.value(seat,Json::array())) {
          std_msgs::msg::String action;
          if (item.is_object() && item.contains("_registration")) {
            action.data = item.at("_registration").dump(); registrations[seat]->publish(action);
            std::this_thread::sleep_for(std::chrono::milliseconds(15)); continue;
          }
          if (item.is_string()) action.data = item.get<std::string>();
          else {
            if (!item.contains("tick")) item["tick"] = tick;
            if (!item.contains("session_id")) item["session_id"] = frame.at("session_id");
            if (item.value("omit_session",false)) {item.erase("session_id");item.erase("omit_session");}
            action.data = item.dump();
          }
          pubs[seat]->publish(action);
          // 审计连续候选时让每条消息完成一次分发；避免DDS KEEP_LAST(1)合并测试输入。
          std::this_thread::sleep_for(std::chrono::milliseconds(15));
        }
      }));
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(25);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node); std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  const bool complete = received_final == static_cast<int>(script.at("seats").size());
  if (rclcpp::ok()) rclcpp::shutdown();
  return complete ? 0 : 3;
}
