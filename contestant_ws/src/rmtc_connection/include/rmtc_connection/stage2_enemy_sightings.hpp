/**
 * @file stage2_enemy_sightings.hpp
 * @brief 根据敌人最后被看到的位置和Tick，给可能重新发现敌人的区域建立软搜索场。
 *
 * 敌人离开旧格不等于失去全部线索：按每Tick最多移动一格扩散，但排除已知墙和
 * 当前确认为没有该敌人的视野。未知地形只参与概率假设，不变成已知空地或可走路线。
 * 质量按半衰期减小，到最大记忆Tick后归零；不用于对历史坐标直接开火。
 */
#pragma once

#include "rmtc_connection/map_memory.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <queue>
#include <vector>

namespace rmtc {

/// 最近敌人线索的二维前缀和；每项候选观察位置可O(1)估计再次发现敌人的机会。
class EnemySightingsField {
 public:
  Json details = Json::array();  ///< 日志保留线索年龄与质量，区分历史猜测和当前真实观测。
  /**
   * @brief 从当前公开记忆生成搜索质量，允许原位置已被确认空置的移动敌人继续扩散。
   * @param half_life 可信度每经过多少Tick减半，不是对手移动速度。
   * @param max_age 超过多少Tick彻底放弃该线索；当前可见敌人不重复进入历史搜索场。
   * 每名敌人的总质量不超过2^(-age/half_life)，不同对手可累加，查询时上限1。
   */
  EnemySightingsField(const PublicPerception& frame, const MapMemory& memory, double half_life,
                      int max_age)
      : size_(frame.map), prefix_((size_.width + 1) * (size_.height + 1), 0) {
    std::vector<double> mass(size_.width * size_.height, 0);
    const auto inside = [&](Position p) {
      return p.x >= 0 && p.y >= 0 && p.x < size_.width && p.y < size_.height;
    };
    const auto index = [&](Position p) { return p.y * size_.width + p.x; };
    const auto currently_seen = [&](Position p) {
      return std::abs(p.x - frame.self.position.x) <= frame.vision_radius &&
             std::abs(p.y - frame.self.position.y) <= frame.vision_radius;
    };
    constexpr std::array<Position, 4> steps{{{0, -1}, {0, 1}, {-1, 0}, {1, 0}}};
    for (const auto& [key, remembered] : memory.entities()) {
      (void)key;
      const auto& enemy = remembered.entity;
      const auto age = frame.tick - remembered.last_seen_tick;
      if (enemy.type != "robot" || enemy.team == frame.self.team || (enemy.hp && *enemy.hp <= 0) ||
          age <= 0 || age > max_age || !inside(enemy.position))
        continue;
      std::vector<int> distances(size_.width * size_.height, -1);
      std::vector<Position> possible;
      std::queue<Position> pending;
      distances[index(enemy.position)] = 0;
      pending.push(enemy.position);
      while (!pending.empty()) {
        const auto at = pending.front();
        pending.pop();
        if (!currently_seen(at))
          possible.push_back(at);
        if (distances[index(at)] >= age)
          continue;
        for (const auto step : steps) {
          const Position next{at.x + step.x, at.y + step.y};
          if (!inside(next) || distances[index(next)] >= 0 ||
              memory.cells()[MapMemory::index(next, size_)] == 100)
            continue;
          distances[index(next)] = distances[index(at)] + 1;
          pending.push(next);
        }
      }
      if (possible.empty())
        continue;
      const double probability = std::exp2(-static_cast<double>(age) / half_life) / possible.size();
      details.push_back(
          {{"id", enemy.id.value_or(enemy.team.value_or("enemy"))},
           {"last_seen_tick", remembered.last_seen_tick},
           {"age", age},
           {"possible_cells", possible.size()},
           {"confidence", std::exp2(-static_cast<double>(age) / half_life)},
           {"old_position_confirmed_empty", remembered.first_missing_tick.has_value()}});
      for (const auto at : possible)
        mass[index(at)] += probability;
    }
    const int stride = size_.width + 1;
    for (int y = 0; y < size_.height; ++y)
      for (int x = 0; x < size_.width; ++x)
        prefix_[(y + 1) * stride + x + 1] = mass[y * size_.width + x] +
                                            prefix_[y * stride + x + 1] +
                                            prefix_[(y + 1) * stride + x] - prefix_[y * stride + x];
  }

  /// 返回在此位置的方形视野可覆盖多少尚未排除的敌人质量；舍去浮点消去误差。
  double in_view(Position position, int radius) const {
    const int x0 = std::max(0, position.x - radius), y0 = std::max(0, position.y - radius);
    const int x1 = std::min(size_.width, position.x + radius + 1);
    const int y1 = std::min(size_.height, position.y + radius + 1), stride = size_.width + 1;
    const double value = prefix_[y1 * stride + x1] - prefix_[y0 * stride + x1] -
                         prefix_[y1 * stride + x0] + prefix_[y0 * stride + x0];
    return value < 1e-9 ? 0.0 : std::min(1.0, value);
  }

 private:
  MapSize size_;
  std::vector<double> prefix_;
};

}  // namespace rmtc
