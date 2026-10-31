/**
 * @file connection_node.hpp
 * @brief 声明负责公开ROS通信、场次管理和动作回执检查的节点。
 *
 * ConnectionNode继承rclcpp::Node，接收本队感知、发布本队动作并注册队名。
 * 协议解析交给protocol/public_data接口，决策按模式交给直行或资源跟踪策略。
 * 默认由main中的rclcpp::spin使用单线程执行器处理回调；成员状态不加互斥锁，
 * 若以后改为可并发的回调或后台线程，需要另外设计状态同步。
 */
#pragma once

#include "rmtc_connection/probe_strategy.hpp"
#include "rmtc_connection/resource_strategy.hpp"
#include "rmtc_connection/stage2_strategy.hpp"
#include "rmtc_connection/ai_client.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <cstdint>
#include <chrono>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace rmtc {

// ROS通信与回执管理；协议格式和移动策略分别封装到独立头文件。
class ConnectionNode final : public rclcpp::Node {
 public:
  /// 读取并校验启动参数，创建ROS收发对象、日志输出及保留的队名注册消息。
  ConnectionNode();
  /// 输出当前进程累计统计；正常退出spin后由main调用，不会重置统计值。
  void summary();

 private:
  /// 第二阶段两种模式共享通信/AI入口；具体规划开关和参数各自独立。
  bool is_stage2_mode() const noexcept {return mode_=="stage2"||mode_=="stage2_planning";}
  /// 感知回调主流程：解析、场次/Tick检查、回执核对、决策和动作发布。
  void on_perception(const std::string& data);
  /// 接受当前/新场次并按需清空缓存；已退出过的旧场次返回false。
  bool accept_session(const Perception& frame);
  /// 用self.last_action核对pending_中的提交记录，避免重复统计回执。
  void inspect_receipt(const Json& self, Position position);
  /// 向本队action话题发布JSON；replay仅决定日志标记，不重新运行策略。
  void publish(const Json& action, bool replay);
  /// 向可选JSONL日志追加一条带时间戳和当前场次的记录。
  void record(Json value);
  /// 缓存本队独立AI进程结果，不执行网络调用；最终校验在对应感知到达后进行。
  void on_ai_result(const std::string& data);
  /// 校验场次/席位/时间及模型结构，返回可用于当前帧的危险区域约束。
  std::optional<Stage2EventConstraints> consume_ai_result(const PublicPerception& frame);

  // 启动配置、当前场次及帧进度；-1表示尚未处理过对应Tick。
  std::string team_, mode_, session_;
  std::int64_t last_tick_{-1}, last_receipt_{-1};
  std::int64_t action_ticks_{100};
  // 以下统计跨场次累计；sent_包含重发，frames_不重复计算相同Tick。
  std::uint64_t frames_{0}, sent_{0}, receipts_{0}, moves_{0};
  std::uint64_t parse_errors_{0}, receipt_errors_{0}, sessions_{0};
  std::set<std::string> retired_sessions_;  ///< 已离开的场次，阻止迟到消息切回旧局。
  std::map<std::int64_t, Json> pending_;    ///< Tick到已提交动作的映射，等待回执核对。
  Json cached_;                             ///< 当前Tick动作；空值表示不可重发。
  std::ofstream log_;                       ///< 未配置log_file时保持关闭。
  ProbeStrategy probe_strategy_;
  ResourceStrategy resource_strategy_;
  Stage2Strategy stage2_strategy_;  ///< 第二阶段状态独立；仅第二阶段两种模式运行。
  std::chrono::steady_clock::time_point run_started_{std::chrono::steady_clock::now()};
  std::vector<std::int64_t> stage2_decision_us_;  ///< 每帧策略计算耗时，退出时汇总。
  Json pending_ai_result_;
  bool ai_response_received_{false}, ai_response_mock_{true};
  // 保存ROS对象的共享指针，确保订阅与发布者在节点存活期间持续有效。
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_pub_, registration_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr perception_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ai_result_sub_;
};

}  // namespace rmtc
