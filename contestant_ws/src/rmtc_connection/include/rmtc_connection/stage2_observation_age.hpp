/**
 * @file stage2_observation_age.hpp
 * @brief 已探索区域的观测时效：用二维前缀和快速查询一个视野内的平均陈旧度。
 *
 * 只奖励已知空地；未知格继续由开图奖励处理，障碍不作为搜索敌人的可达区域。
 * 本车本Tick刚看到的格子应先更新last_observed，再建立本场。不会推断敌人实际位置。
 */
#pragma once

#include "rmtc_connection/map_memory.hpp"

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rmtc {

/// 一次O(地图格数)建表，之后每个目标视野只需O(1)查询，避免逐候选重复扫描老格子。
class ObservationAgeField {
 public:
  /**
   * @brief 为当前公开记忆建立年龄与已知空地数量的前缀和。
   * fresh_ticks以内视为新鲜，stale_ticks及以上为完全陈旧，中间线性过渡。
   * 未观察过的格子不计入分母，也不混入“已探索区域再次搜索”的奖励。
   * 调用方须提供匹配地图的两个数组，且0<=fresh_ticks<stale_ticks。
   */
  ObservationAgeField(MapSize size, const std::vector<std::int8_t>& cells,
                      const std::vector<std::int64_t>& last_observed, std::int64_t tick,
                      int fresh_ticks, int stale_ticks)
      : size_(size),
        stride_(size.width + 1),
        ages_(static_cast<std::size_t>((size.width + 1) * (size.height + 1)), 0),
        counts_(ages_.size(), 0) {
    for (int y = 0; y < size.height; ++y) {
      for (int x = 0; x < size.width; ++x) {
        const Position at{x, y};
        const auto last = last_observed[y * size.width + x];
        const bool known = cells[MapMemory::index(at, size)] == 0 && last >= 0;
        const double age = known ? std::clamp(static_cast<double>(tick - last - fresh_ticks) /
                                                  (stale_ticks - fresh_ticks),
                                              0.0, 1.0)
                                 : 0.0;
        const int i = (y + 1) * stride_ + x + 1;
        ages_[i] = age + ages_[i - 1] + ages_[i - stride_] - ages_[i - stride_ - 1];
        counts_[i] = known + counts_[i - 1] + counts_[i - stride_] - counts_[i - stride_ - 1];
      }
    }
  }

  /// 查询方形视野内已知空地的平均陈旧度[0,1]；空视野返回0，地图边缘自动裁剪。
  double mean_in_view(Position center, int radius) const {
    const int x0 = std::clamp(center.x - radius, 0, size_.width);
    const int x1 = std::clamp(center.x + radius + 1, 0, size_.width);
    const int y0 = std::clamp(center.y - radius, 0, size_.height);
    const int y1 = std::clamp(center.y + radius + 1, 0, size_.height);
    if (x0 >= x1 || y0 >= y1)
      return 0;
    const auto sum = [&](const auto& values) {
      return values[y1 * stride_ + x1] - values[y0 * stride_ + x1] - values[y1 * stride_ + x0] +
             values[y0 * stride_ + x0];
    };
    const int count = sum(counts_);
    const double total_age = sum(ages_);
    // 前缀和相减可能留下浮点舍入残差，不能让它变成虚假的微小搜索奖励。
    return count > 0 && total_age > 1e-9 ? std::clamp(total_age / count, 0.0, 1.0) : 0.0;
  }

 private:
  MapSize size_;
  int stride_;
  std::vector<double> ages_;
  std::vector<int> counts_;
};

}  // namespace rmtc
