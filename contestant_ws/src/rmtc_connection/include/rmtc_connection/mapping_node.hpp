/**
 * @file mapping_node.hpp
 * @brief 声明独立的C++建图/可视化节点，与选手控制节点分开运行。
 *
 * 订阅本队公开perception和自有AI解释结果；不发送动作或注册队伍。
 * 发布累积OccupancyGrid、实体观察历史和轨迹MarkerArray，供RViz订阅。
 * 所有回调由单线程执行器串行执行；仅在本节点内保存地图记忆。
 */
#pragma once

#include "rmtc_connection/map_memory.hpp"
#include "rmtc_connection/stage2_events.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <deque>
#include <memory>
#include <set>
#include <string>

namespace rmtc {

class MappingNode : public rclcpp::Node {
 public:
  /**
   * @brief 校验席位/格子比例，建立一个感知订阅和两个持久化可视化发布器。
   * 参数team默认A，cell_size默认1.0（每格对应的显示米数）。
   * 坐标系rmtc_<team>_map以地图中心为原点，比赛坐标仍在标签中显示。
   */
  explicit MappingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  /// 解析一条本队感知；拒绝坏包、重复/过期Tick，接受后立即刷新地图和标记。
  void on_perception(const std_msgs::msg::String& message);

  /// 有界缓存本队AI回复；同场、同公告且经结构校验的真实事件才可进入显示层。
  void on_ai_result(const std_msgs::msg::String& message);

  /// 将已经匹配当前感知的AI计划合并到事件时间线；未来Tick的回复暂存等待。
  void consume_ai_results();

  /// 在场景快照中添加八类事件的区域/点位、预告、持续时间和已确认Buff。
  void append_event_markers(visualization_msgs::msg::MarkerArray& output);

  /// 发布累积地形；尺寸来自感知，未知为-1，行方向经过翻转以对应比赛画面。
  void publish_map();

  /**
   * @brief 发布底图、本车、当前实体/历史位置、视野、任务和轨迹。
   * 每包先DELETEALL再ADD，避免历史/实时状态切换后残留旧样式或上一场标记。
   * 实体标签的Nt由当前比赛Tick减last_seen_tick得到；墙钟重发不增加Tick。
   * 定时重发也更新断流提示；不把重发时间当作新的比赛Tick。
   */
  void publish_markers();

  std::string team_;
  std::string frame_id_;
  double cell_size_{1.0};
  MapMemory memory_;
  // 事件地图独立保存临时墙及原地形，不让公告推导的格子增加“亲眼探索”比例。
  MapMemory event_memory_;
  Stage2Events events_;
  std::deque<Json> pending_ai_results_;
  std::set<std::string> retired_sessions_;
  std::int64_t action_ticks_{500};
  std::chrono::steady_clock::time_point received_at_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr perception_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ai_result_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> frame_broadcaster_;
  rclcpp::TimerBase::SharedPtr refresh_timer_;
};

}  // namespace rmtc
