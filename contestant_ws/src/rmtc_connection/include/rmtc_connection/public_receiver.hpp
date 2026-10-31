/**
 * @file public_receiver.hpp
 * @brief 声明需要显式启用的公开信息接收器，目前不接入现有节点。
 *
 * 构造只保存席位与回调，不创建订阅；以后调用start(node)才接收本队
 * perception及公开mission/event_text/team_registry，共4个话题。
 * 回调只提供数据，不生成动作、不调用AI、不访问其他队专属感知。
 * 文本话题没有结构化session_id，不能据此假定来自当前感知场次。
 * 使用与当前程序相同的单线程执行方式；start/stop不得与回调并发执行。
 */
#pragma once

#include "rmtc_connection/public_data.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <functional>
#include <string>

namespace rmtc {

/// 接收回调可以逐项设置；未设置的回调不向使用方转交该类数据。
struct PublicCallbacks {
  std::function<void(const PublicPerception&)> on_perception;
  std::function<void(const std::string&)> on_mission_text;
  std::function<void(const std::string&)> on_event_text;
  std::function<void(const std::vector<TeamInfo>&)> on_team_registry;
  std::function<void(const std::string&, const std::string&)> on_error;  ///< 话题、错误说明。
};

/// 接收器会建立的全部话题名，只有感知话题随本队席位改变。
struct PublicTopics {
  std::string perception;
  std::string mission{"/referee/mission"};
  std::string event_text{"/referee/event_text"};
  std::string team_registry{"/referee/team_registry"};
};

class PublicReceiver {
 public:
  /**
   * @brief 保存席位及回调，构造后仍未启用任何ROS订阅。
   * @param team 本队席位A/B/C/D。
   * @param callbacks 对应四类数据的处理函数；参数引用仅在回调期间有效，
   *                  需要长期保存时应复制数据，不能持有临时对象的引用。
   * @throws std::runtime_error 席位非法时抛出。
   */
  explicit PublicReceiver(std::string team, PublicCallbacks callbacks = {});

  /// 释放本接收器的订阅；销毁须发生在没有并发回调执行的时候。
  ~PublicReceiver();

  // ROS回调捕获this，因此禁止复制或移动接收器使地址发生变化。
  PublicReceiver(const PublicReceiver&) = delete;
  PublicReceiver& operator=(const PublicReceiver&) = delete;
  PublicReceiver(PublicReceiver&&) = delete;
  PublicReceiver& operator=(PublicReceiver&&) = delete;

  /**
   * @brief 显式创建4个合法订阅。当前main/ConnectionNode没有调用它。
   * @param node 用于创建订阅的ROS节点，运行期间需由执行器spin。
   * 重复start不会叠加订阅；创建失败时清理本次订阅并向调用方抛异常。
   * 收到的数据逐条转交，不去重、不跨帧缓存，不推断事件含义或场次归属。
   */
  void start(rclcpp::Node& node);

  /// 停止本接收器的全部订阅，可重复调用，不停止外部节点或现有移动测试。
  void stop() noexcept;

  /// 返回是否已建立全部4个订阅；不代表已收到数据或完成双向通信。
  bool active() const noexcept;

  /// 返回预先计算的话题名；构造后、start之前也可查询。
  const PublicTopics& topics() const noexcept;

 private:
  /// 执行一次解析/用户回调，捕获标准异常，记录错误并调用可选on_error。
  void receive(const std::string& topic, const std::function<void()>& operation) const;

  std::string team_;
  PublicCallbacks callbacks_;
  PublicTopics topics_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr perception_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr registry_sub_;
};

}  // namespace rmtc
