/**
 * @file practice_scoring_main.cpp
 * @brief 本地修正版裁判计分节点：隔离域内读取结算、AI日志，按v1.2发布和持久化计分。
 * 原始模拟器负责战斗；本节点修正练习包缺失的AI资格门槛和九位小数排名。
 */
#include "rmtc_connection/practice_rules.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace {
using Json = nlohmann::json;
/** @brief 读取裁判私有审计记录；失败返回无资格，不从选手消息中采信成功声明。 */
Json read_records(const std::string& path) {
  Json result = Json::array();
  std::ifstream stream(path);
  std::string line;
  while (std::getline(stream, line)) {
    auto record = Json::parse(line, nullptr, false);
    if (record.is_object()) result.push_back(std::move(record));
  }
  return result;
}

/** @brief 裁判内部节点；终帧冻结资格，晚到的网络响应不能追补已结束场次的成绩。 */
class Scoring : public rclcpp::Node {
 public:
  /** @brief 订阅仅含实时消息的结算流，并用定时器接收异步完成的本场AI资格。 */
  Scoring() : Node("scoring_node") {
    score_file_ = declare_parameter("score_file", std::string("/tmp/rmtc_scoreboard.json"));
    ai_log_ = declare_parameter("ai_log", std::string());
    ready_file_ = declare_parameter("ready_file", std::string());
    max_ticks_ = declare_parameter("max_ticks", 500);
    declare_parameter("stage", 2);
    pub_ = create_publisher<std_msgs::msg::String>("/referee/scoreboard", rclcpp::QoS(2048).transient_local());
    sub_ = create_subscription<std_msgs::msg::String>("/simulator/events", rclcpp::QoS(2048).durability_volatile(),
        [this](std_msgs::msg::String::SharedPtr message) {
          try {
            auto batch = Json::parse(message->data);
            const std::string session = batch.at("session_id");
            if (session != session_) {session_ = session; frozen_ = false;}
            if (frozen_) return;
            batch_ = std::move(batch);
            publish_score();
            frozen_ = batch_.at("state").at("tick").get<int>() >= max_ticks_;
          } catch (const std::exception& error) {
            RCLCPP_FATAL(get_logger(), "scoring: %s", error.what()); rclcpp::shutdown();
          }
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      if (!ready_ && sub_->get_publisher_count() > 0 && pub_->get_subscription_count() > 0) {
        if (!ready_file_.empty()) {
          std::ofstream ready(ready_file_);
          if (!ready) {RCLCPP_FATAL(get_logger(), "cannot write scoring readiness");rclcpp::shutdown();return;}
          ready << "ready\n";
        }
        ready_ = true;
      }
      if (!batch_.is_null() && !frozen_) {
        try {publish_score();} catch (const std::exception& error) {
          RCLCPP_FATAL(get_logger(), "scoring: %s", error.what()); rclcpp::shutdown();
        }
      }
    });
  }
 private:
  /** @brief 重算同场资格，原子写盘后发布；磁盘错误不发布未持久化成绩。 */
  void publish_score() {
    const auto board = rmtc::practice::rules_scoreboard(batch_, rmtc::practice::qualified_teams(read_records(ai_log_), session_));
    const auto text = board.dump();
    if (text == previous_) return;
    std::ofstream stream;
    stream.exceptions(std::ios::failbit | std::ios::badbit);
    stream.open(score_file_ + ".tmp"); stream << text << '\n'; stream.close();
    std::filesystem::rename(score_file_ + ".tmp", score_file_);
    std_msgs::msg::String message; message.data = text; pub_->publish(message); previous_ = text;
  }
  std::string score_file_, ai_log_, ready_file_, session_, previous_;
  int max_ticks_{};
  bool frozen_{};
  bool ready_{};
  Json batch_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}

/** @brief 仅初始化回环内部ROS域，拒绝与选手公开域相同的配置。 */
int main(int argc, char** argv) {
  try {
    const int public_domain = std::stoi(std::getenv("ROS_DOMAIN_ID") ? std::getenv("ROS_DOMAIN_ID") : "27");
    const int internal = std::getenv("RMTC_INTERNAL_DOMAIN_ID") ? std::stoi(std::getenv("RMTC_INTERNAL_DOMAIN_ID")) : (public_domain+1)%233;
    if (internal < 0 || internal > 232 || internal == public_domain) throw std::runtime_error("invalid internal domain");
    setenv("ROS_DOMAIN_ID", std::to_string(internal).c_str(), 1); setenv("ROS_LOCALHOST_ONLY", "1", 1);
    rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<Scoring>()); rclcpp::shutdown();
  } catch (const std::exception& error) {std::cerr << error.what() << '\n'; return 1;}
}
