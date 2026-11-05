/**
 * @file connection_main.cpp
 * @brief 参赛程序入口，管理ROS初始化、节点运行和退出清理。
 *
 * 本文件只组装运行流程，不解析协议、不决定动作。
 * 创建ConnectionNode后进入spin，感知处理由节点回调驱动。
 * 可执行文件由ros2 run rmtc_connection connection_node启动。
 *
 * @details 模块协作与维护约定
 * 执行链：初始化ROS → 构造ConnectionNode → spin处理回调 → 退出时汇总并关闭ROS。
 * 本入口不计算路线；默认单线程执行器串行处理节点回调，不代表中间件没有后台线程。
 */
#include "rmtc_connection/connection_node.hpp"

#include <exception>
#include <iostream>
#include <memory>

/**
 * @brief 初始化ROS，创建节点并持续处理消息，退出时输出统计。
 * @param argc 命令行参数数量，由运行环境传入。
 * @param argv 命令行参数数组，其中--ros-args等参数交由rclcpp解析。
 * @return 正常结束返回0；节点构造或运行中有未处理的标准异常时返回1。
 *
 * spin默认使用单线程执行器，收到感知时调用节点注册的回调。
 * Ctrl+C触发ROS关闭后spin返回，再打印summary并释放ROS资源。
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    // 共享指针使节点在spin运行期间保持存活，离开作用域时自动释放。
    const auto node = std::make_shared<rmtc::ConnectionNode>();
    rclcpp::spin(node);
    node->summary();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "connection_node: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
