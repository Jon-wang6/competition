/**
 * @file stage2_event_route.hpp
 * @brief 第二阶段事件软代价A*；不修改第一阶段的A*、代价或路线缓存。
 *
 * 静态/临时墙仍不可走；辣椒等非墙事件只增加有限的非负代价。启发式仍为
 * 曼哈顿距离，每条边至少1，保证本次给定软代价下A*正确。
 */
#pragma once
#include "rmtc_connection/astar.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>

namespace rmtc {
/// risk按世界坐标y*width+x排列；输出cost保留实际步数，避免把风险代价误报成Tick。
template <class Passable>
planning::SearchResult event_route(MapSize size, Position start, Position goal,
                                   const std::vector<double>& risk, Passable passable) {
  planning::SearchResult result;
  const int n = size.width * size.height;
  if (risk.size() != static_cast<std::size_t>(n) || !passable(start) || !passable(goal))
    return result;
  const auto index = [&](Position p) { return p.y * size.width + p.x; };
  const auto heuristic = [&](Position p) {
    return std::abs(p.x - goal.x) + std::abs(p.y - goal.y);
  };
  struct Entry {
    double f, g;
    int at, sequence;
  };
  const auto compare = [](const Entry& a, const Entry& b) {
    return a.f != b.f ? a.f > b.f : a.sequence > b.sequence;
  };
  std::priority_queue<Entry, std::vector<Entry>, decltype(compare)> queue(compare);
  std::vector<double> distances(n, std::numeric_limits<double>::infinity());
  std::vector<int> parent(n, -1);
  int sequence = 0;
  distances[index(start)] = 0;
  queue.push({double(heuristic(start)), 0, index(start), sequence++});
  constexpr std::array<Position, 4> steps{{{0, -1}, {0, 1}, {-1, 0}, {1, 0}}};
  while (!queue.empty()) {
    const auto entry = queue.top();
    queue.pop();
    if (entry.g > distances[entry.at] + 1e-9)
      continue;
    Position at{entry.at % size.width, entry.at / size.width};
    if (entry.at == index(goal)) {
      for (int p = entry.at; p != -1; p = parent[p])
        result.path.push_back({p % size.width, p / size.width});
      std::reverse(result.path.begin(), result.path.end());
      result.cost = result.path.size() - 1;
      result.status = planning::SearchStatus::Found;
      return result;
    }
    for (auto step : steps) {
      Position next{at.x + step.x, at.y + step.y};
      if (!passable(next))
        continue;
      const int i = index(next);
      const double cost = entry.g + 1 + std::max(0.0, risk[i]);
      if (cost + 1e-9 >= distances[i])
        continue;
      distances[i] = cost;
      parent[i] = entry.at;
      queue.push({cost + heuristic(next), cost, i, sequence++});
    }
  }
  return result;
}
}  // namespace rmtc
