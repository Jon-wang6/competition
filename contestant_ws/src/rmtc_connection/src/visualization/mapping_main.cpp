/**
 * @file mapping_main.cpp
 * @brief 独立建图程序入口；只启动MappingNode，不启动控制节点或裁判。
 */
#include "rmtc_connection/mapping_node.hpp"

#include <exception>
#include <iostream>
#include <memory>

/**
 * @brief 初始化ROS并以单线程执行建图回调，Ctrl+C后释放订阅和发布器。
 * @return 正常退出为0；参数错误或启动异常打印原因并返回1。
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<rmtc::MappingNode>());
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "建图节点启动失败：" << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
