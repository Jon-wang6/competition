/**
 * @file probe_strategy.hpp
 * @brief 封装第一步接入使用的简单移动测试策略。
 *
 * 输入为当前公开感知Perception，输出为Action意图，不直接发布ROS消息。
 * 策略固定向下直行（比赛坐标y+1），便于查看RViz逐步扩展地图的效果。
 * 下方有可见障碍/车辆或已到地图下边界时等待，不自动转弯或绕路。
 * 不保存方向状态；阵亡时等待，复活后仍然尝试向下。
 * 这是验证通信和位置解析的策略，尚未实现采集规划、战斗或AI事件理解。
 */
#pragma once

#include "rmtc_connection/protocol.hpp"

namespace rmtc {

// 简单移动探测，仅使用公开感知，不依赖ROS，后续可替换为采集/战斗策略。
class ProbeStrategy {
 public:
  /**
   * @brief 保留新场次的统一重置入口；向下直行策略没有需要清理的状态。
   * 新场次确认后由ConnectionNode调用；同Tick重发时不能调用此函数。
   * 不清空地图或任务，因为本测试策略尚未保存这些跨帧信息。
   */
  void reset() noexcept {}

  /**
   * @brief 检查正下方是否可尝试进入，决定向下移动或原地等待。
   * @param frame 已完成基础字段检查的当前感知；本函数进一步检查地图与实体。
   * @return 存活且下方未越界/被占据时返回move down，否则返回wait。
   * @throws std::runtime_error 地图、坐标或实体字段不满足当前策略要求。
   * @throws nlohmann::json::exception 需要的字段缺失或JSON转换失败。
   *
   * 每次都选择同一个方向，连续调用或新场次重置均不会轮换方向。
   * 这只是根据当前感知提出动作，其他车辆争格等情况仍由裁判最终判定。
   * 死亡时立即等待，不要求解析周围实体；异常由节点捕获后回退到WAIT。
   */
  Action decide(const Perception& frame) const {
    if (!frame.alive)
      return Action::wait();
    const auto& map = frame.data.at("map");
    if (!map.at("width").is_number_integer() || !map.at("height").is_number_integer()) {
      throw std::runtime_error("map dimensions must be integers");
    }
    const auto width = map.at("width").get<int>();
    const auto height = map.at("height").get<int>();
    const auto [x, y] = frame.position;
    if (width <= 0 || height <= 0 || x < 0 || y < 0 || x >= width || y >= height) {
      throw std::runtime_error("invalid map dimensions or self position");
    }
    const auto& entities = frame.data.at("visible_entities");
    if (!entities.is_array())
      throw std::runtime_error("visible_entities must be an array");
    // 下边界直接等待；不能为了继续移动而改成左右或向上。
    if (y + 1 >= height)
      return Action::wait();
    // 只检查正下方一格。资源可以进入并采集，不作为移动障碍。
    for (const auto& entity : entities) {
      const auto type = entity.at("type").get<std::string>();
      if (type != "obstacle" && type != "robot")
        continue;
      const auto [dx, dy] = parse_relative_position(entity.at("position"));
      if (dx == 0 && dy == 1)
        return Action::wait();
    }
    return Action::move("down");
  }
};

}  // namespace rmtc
