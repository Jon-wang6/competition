/**
 * @file stage2_tactical_navigation.hpp
 * @brief 第二阶段目标距离场：一次反向BFS，比较所有合法首步的实际路程进展。
 *
 * 只使用本车已知可通行格；敌车和已校验危险格仍由调用方的passable排除。
 * 距离场仅提供候选移动收益，不改变攻击先结算规则，也不将墙当成掩体。
 *
 * @details 模块协作与维护约定
 * 目标距离场复用一次反向BFS结果比较多个下一步，避免每个邻居都重新做完整寻路。
 * 数组中的距离来自当前通行快照，地形改变后旧距离不能继续作为可达性事实。
 */
#pragma once

#include "rmtc_connection/protocol.hpp"

#include <array>
#include <queue>
#include <vector>

namespace rmtc {

/**
 * @brief 从目标反向扩展已知四邻接网格，返回每格到目标的最短路步数。
 * @param size 地图大小；数组按世界坐标y*width+x索引。
 * @param goal 目标坐标；非法或不可通行时返回全-1。
 * @param passable 对合法坐标判断是否为当前已知可通行格，不读取裁判隐藏地图。
 * @return -1表示未连通，其余为含动态占位约束的最短距离。
 * 每次时间和内存均为O(width*height)；调用方应复用结果，不枚举对手动作树。
 */
template <typename Passable>
std::vector<int> goal_distance_field(MapSize size, Position goal, Passable passable) {
  const auto inside = [&](Position p) {
    return p.x >= 0 && p.y >= 0 && p.x < size.width && p.y < size.height;
  };
  std::vector<int> distances(static_cast<std::size_t>(size.width * size.height), -1);
  if (!inside(goal) || !passable(goal))
    return distances;
  constexpr std::array<Position, 4> steps{{{0, -1}, {0, 1}, {-1, 0}, {1, 0}}};
  std::queue<Position> pending;
  distances[goal.y * size.width + goal.x] = 0;
  pending.push(goal);
  while (!pending.empty()) {
    const Position at = pending.front();
    pending.pop();
    for (const auto step : steps) {
      const Position next{at.x + step.x, at.y + step.y};
      if (!inside(next) || !passable(next) || distances[next.y * size.width + next.x] >= 0)
        continue;
      distances[next.y * size.width + next.x] = distances[at.y * size.width + at.x] + 1;
      pending.push(next);
    }
  }
  return distances;
}

}  // namespace rmtc
