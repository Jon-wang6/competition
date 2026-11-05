/**
 * @file stage2_reach_finish.hpp
 * @brief 末局到达任务的导航与交战余裕；只用本车地图记忆，不读取隐藏地图。
 *
 * 未开图时仅用“未知格可能可走”估计方向，实际途经点始终落在已知可达空地。
 * 先沿估计最短路接近并观察，再根据新地形重规划，避免曼哈顿贪心卡在凹形墙外。
 *
 * @details 模块协作与维护约定
 * 剩余Tick与预计到达步数共同决定任务余裕，途中的战斗不能侵占已不足的完成窗口。
 * 规划到达不等于任务已完成，最终完成状态仍以裁判公开回报为准。
 */
#pragma once
#include "rmtc_connection/map_memory.hpp"
#include "rmtc_connection/stage2_tactical_navigation.hpp"
#include <algorithm>
#include <optional>

namespace rmtc {
struct ReachApproach {
  Position waypoint;
  int estimated_steps{0}; ///< 已知路程或忽略未知墙的乐观估计；后者不是到达保证。
  bool exploring{false};
};

/**
 * @brief 给公开任务坐标找已知路线，未连通则找估计路线上的最后一个已知可达点。
 * known_distance按y*width+x索引；passable已排除当前占位、临时封锁等不可执行落点。
 * 估计路线允许未知格、忽略瞬时车位，仍严格避开已知墙（包括生效中的分裂墙）。
 */
template <typename Passable>
std::optional<ReachApproach> reach_finish_approach(const MapMemory& memory, Position start,
    Position target, const std::vector<int>& known_distance, Passable passable) {
  const auto size = memory.current()->map;
  const auto index = [&](Position p) { return p.y * size.width + p.x; };
  if (target.x < 0 || target.y < 0 || target.x >= size.width || target.y >= size.height)
    return std::nullopt;
  if (known_distance[index(target)] >= 0)
    return ReachApproach{target, known_distance[index(target)], false};
  const auto estimate = goal_distance_field(size, target, [&](Position p) {
    return memory.cells()[MapMemory::index(p, size)] != 100;
  });
  if (estimate[index(start)] <= 0) return std::nullopt;
  constexpr Position directions[]{{0,-1},{0,1},{-1,0},{1,0}};
  Position at = start;
  while (estimate[index(at)] > 0) {
    std::optional<Position> next;
    for (auto delta : directions) {
      const Position p{at.x + delta.x, at.y + delta.y};
      if (p.x < 0 || p.y < 0 || p.x >= size.width || p.y >= size.height ||
          estimate[index(p)] != estimate[index(at)] - 1) continue;
      // 同长路线先用可执行的已知格，减少被短暂车位或未知格截断的机会。
      if (!next || (passable(p) && !passable(*next))) next = p;
    }
    if (!next || !passable(*next)) break;
    at = *next;
  }
  if (at.x == start.x && at.y == start.y) return std::nullopt;
  return ReachApproach{at, known_distance[index(at)] + estimate[index(at)], true};
}

/** @brief 平均每步可花的交战余裕；预留安全缓冲，路程紧张时不再安排原地停留。 */
inline int reach_attack_allowance(std::int64_t remaining, int steps, int reserve, int maximum) {
  if (steps <= 0) return 0;
  const auto spare_per_step = (remaining - reserve) / steps - 1;
  return static_cast<int>(std::clamp<std::int64_t>(spare_per_step, 0, maximum));
}
}  // namespace rmtc
