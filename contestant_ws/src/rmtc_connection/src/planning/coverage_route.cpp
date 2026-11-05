/**
 * @file coverage_route.cpp
 * @brief 根据已观察障碍计算乐观距离，将资源和补看位置放进同一开放路线。
 * 隐藏布局从不进入本文件；方形视野由公开半径计算，执行前检查首步已知空地。
 *
 * @details 模块协作与维护约定
 * 规划把资源访问与补看位置放在同一条路线中，覆盖收益按集合去重，避免重复奖励同一未知格。
 * 远期距离可对未知地形作乐观估计，但实际下一步必须服从已观察通行性；发现新墙需更新距离缓存。
 */
#include "rmtc_connection/coverage_route.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <unordered_map>

namespace rmtc::planning {
namespace {
constexpr int kUnreachableDistance = 100000;
/// 计算固定起点的开放路线长度；最后一个目标后不需要返回。
int route_length(const std::vector<int>& order, int start,
                 const std::vector<std::vector<int>>& distance) {
  int result = 0, previous = start;
  for (int p : order) {
    result += distance[previous][p];
    previous = p;
  }
  return result;
}
/**
 * @brief 小规模路线调用精确动态规划；超过精确上限时最多进行20轮局部优化。
 * 大规模分支先反转一段访问顺序，再尝试单点移位，只有严格缩短才接受。
 * 起点固定、终点开放；遍历和同分处理保持确定性，不在同长路线之间反复切换。
 */
void improve_visit_order(std::vector<int>& route, int start,
                         const std::vector<std::vector<int>>& distance) {
  if (route.size() <= kExactResourceLimit) {
    std::vector<int> points{start};
    points.insert(points.end(), route.begin(), route.end());
    std::vector<std::vector<std::int64_t>> matrix(points.size(),
                                                  std::vector<std::int64_t>(points.size()));
    for (std::size_t i = 0; i < points.size(); ++i)
      for (std::size_t j = 0; j < points.size(); ++j)
        matrix[i][j] = distance[points[i]][points[j]];
    const auto optimal = plan_resource_visit_order(matrix);
    auto previous = route;
    for (std::size_t i = 0; i < route.size(); ++i)
      route[i] = previous[optimal.indices[i]];
    return;
  }
  for (int pass = 0; pass < 20; ++pass) {
    int best = 0, a = -1, b = -1;
    for (int i = 0; i < static_cast<int>(route.size()); ++i)
      for (int j = i + 1; j < static_cast<int>(route.size()); ++j) {
        int before = i ? route[i - 1] : start;
        int delta = distance[before][route[j]] - distance[before][route[i]];
        if (j + 1 < static_cast<int>(route.size()))
          delta += distance[route[i]][route[j + 1]] - distance[route[j]][route[j + 1]];
        if (delta < best) {
          best = delta;
          a = i;
          b = j;
        }
      }
    if (a >= 0) {
      std::reverse(route.begin() + a, route.begin() + b + 1);
      continue;
    }
    bool moved = false;
    const int old = route_length(route, start, distance);
    for (std::size_t i = 0; i < route.size() && !moved; ++i) {
      auto removed = route;
      int point = removed[i];
      removed.erase(removed.begin() + i);
      for (std::size_t j = 0; j <= removed.size(); ++j) {
        auto candidate = removed;
        candidate.insert(candidate.begin() + j, point);
        if (route_length(candidate, start, distance) < old) {
          route = std::move(candidate);
          moved = true;
          break;
        }
      }
    }
    if (!moved)
      break;
  }
}

}  // namespace

/// 不保留前局的障碍假设或距离表。
void CoverageRoutePlanner::reset() {
  width_ = height_ = 0;
  walls_.clear();
  distances_.clear();
  route_.clear();
  resource_points_.clear();
}

/**
 * @brief 障碍集合或尺寸变化时，重新计算各非障碍格之间的四方向单位步长距离。
 * 未知格在远期估算中按可通行处理；已知空地增加不会使此缓存失效。
 * 不可达距离保留哨兵值；邻居顺序固定，保证同输入下结果可复现。
 */
void CoverageRoutePlanner::update_distances(int w, int h, const std::vector<bool>& walls) {
  const int n = w * h;
  if (w != width_ || h != height_ || walls != walls_) {
    width_ = w;
    height_ = h;
    walls_ = walls;
    distances_.assign(n, std::vector<int>(n, kUnreachableDistance));
    std::vector<int> queue(n);
    for (int source = 0; source < n; ++source) {
      if (walls[source])
        continue;
      auto& distance = distances_[source];
      distance[source] = 0;
      int head = 0, tail = 0;
      queue[tail++] = source;
      while (head < tail) {
        const int p = queue[head++];
        for (int q : {p % w ? p - 1 : -1, p % w + 1 < w ? p + 1 : -1, p >= w ? p - w : -1,
                      p + w < n ? p + w : -1}) {
          if (q < 0 || walls[q] || distance[q] != kUnreachableDistance)
            continue;
          distance[q] = distance[p] + 1;
          queue[tail++] = q;
        }
      }
    }
  }
}

/**
 * @brief 校验公开地图，复用访问顺序，补入观察点，再返回首段路线和整条路线的估算。
 * 所有覆盖和未来资源成本都从未知格位置推算，不访问地图真值；只允许执行已知空地首步。
 * 本函数持有单帧路径/覆盖缓存；跨帧仅保留障碍距离表、路线顺序和已知资源位置。
 */
CoverageTour CoverageRoutePlanner::plan(const GridMap& grid, GridIndex start_position,
                                        const std::vector<ResourceGoal>& resources, int radius,
                                        int known_resources, const std::vector<GridIndex>& blocked,
                                        CoverageWeights weights) {
  CoverageTour result;
  const int w = grid.width, h = grid.height;
  const auto count = static_cast<std::int64_t>(w) * h;
  if (w <= 0 || h <= 0 || count > 1600 || grid.cells.size() != static_cast<std::size_t>(count) ||
      start_position.x < 0 || start_position.y < 0 || start_position.x >= w ||
      start_position.y >= h || radius < 1 || radius > 40)
    throw std::invalid_argument("invalid joint coverage grid/start/radius");
  const int n = static_cast<int>(count);
  if (std::any_of(grid.cells.begin(), grid.cells.end(),
                  [](int value) { return value < -1 || value > 100; }) ||
      (grid.row_order != RowOrder::BottomToTop && grid.row_order != RowOrder::TopToBottom))
    throw std::invalid_argument("invalid joint coverage cell storage");
  if (width_ != w || height_ != h)
    reset();
  if (!std::isfinite(weights.insertion_offset) || weights.insertion_offset < 0 ||
      weights.insertion_offset > 1000 || !std::isfinite(weights.gain_exponent) ||
      weights.gain_exponent < 0 || weights.gain_exponent > 4 ||
      !std::isfinite(weights.discovery_urgency) || weights.discovery_urgency < 0 ||
      weights.discovery_urgency > 100 || !std::isfinite(weights.path_edge_bonus) ||
      weights.path_edge_bonus < 0 || weights.path_edge_bonus > 100 ||
      !std::isfinite(weights.path_potential_exponent) || weights.path_potential_exponent < 0 ||
      weights.path_potential_exponent > 4 || !std::isfinite(weights.future_resource_weight) ||
      weights.future_resource_weight < 0 || weights.future_resource_weight > 10 ||
      !std::isfinite(weights.causal_future_fraction) || weights.causal_future_fraction < 0 ||
      weights.causal_future_fraction > 1 || !std::isfinite(weights.refresh_margin) ||
      weights.refresh_margin < -1 || weights.refresh_margin > 100 || weights.path_edge_band < 1 ||
      weights.starts < 1 || weights.starts > 32 || weights.expected_resources < 0)
    throw std::invalid_argument("invalid joint coverage weights");
  for (const auto& r : resources)
    if (r.position.x < 0 || r.position.y < 0 || r.position.x >= w || r.position.y >= h)
      throw std::invalid_argument("joint resource outside grid");
  // 规划内部统一使用左上原点的线性编号；读取时才转换ROS地图的行存储顺序。
  const auto linear_index = [w](GridIndex p) { return p.y * w + p.x; };
  const auto cell_value = [&](int p) {
    return grid
        .cells[(grid.row_order == RowOrder::BottomToTop ? h - 1 - p / w : p / w) * w + p % w];
  };
  const int start = linear_index(start_position);
  if (cell_value(start) != 0)
    throw std::invalid_argument("joint coverage start must be observed free");
  std::vector<bool> walls(n);
  for (int p = 0; p < n; ++p)
    walls[p] = cell_value(p) >= 50;
  for (auto p : blocked)
    if (p.x >= 0 && p.x < w && p.y >= 0 && p.y < h)
      walls[linear_index(p)] = true;
  walls[start] = false;
  update_distances(w, h, walls);
  const auto& distance = distances_;
  std::vector<int> resource_points;
  for (const auto& r : resources)
    if (distance[start][linear_index(r.position)] < kUnreachableDistance)
      resource_points.push_back(linear_index(r.position));
  // 保留上帧整体计划作为起点，避免局部搜索在相邻格间选出不同近似解而来回走。
  auto retained = route_;
  retained.erase(
      std::remove_if(
          retained.begin(), retained.end(),
          [&](int p) {
            if (p >= n || walls[p] || distance[start][p] >= kUnreachableDistance)
              return true;
            if (std::find(resource_points.begin(), resource_points.end(), p) !=
                resource_points.end())
              return false;
            if (std::find(resource_points_.begin(), resource_points_.end(), p) !=
                resource_points_.end())
              return true;
            if (!resources.empty() && weights.expected_resources > 0 &&
                known_resources >= weights.expected_resources)
              return true;
            for (int y = std::max(0, p / w - radius); y <= std::min(h - 1, p / w + radius); ++y)
              for (int x = std::max(0, p % w - radius); x <= std::min(w - 1, p % w + radius); ++x)
                if (cell_value(y * w + x) == -1)
                  return false;
            return true;
          }),
      retained.end());
  if (!retained.empty())
    for (int p : resource_points) {
      if (std::find(retained.begin(), retained.end(), p) != retained.end())
        continue;
      int best = kUnreachableDistance, insertion = 0;
      for (int i = 0; i <= static_cast<int>(retained.size()); ++i) {
        int before = i ? retained[i - 1] : start;
        int extra = distance[before][p];
        if (i < static_cast<int>(retained.size()))
          extra += distance[p][retained[i]] - distance[before][retained[i]];
        if (extra < best) {
          best = extra;
          insertion = i;
        }
      }
      retained.insert(retained.begin() + insertion, p);
    }
  const bool had_retained_route = !retained.empty();
  std::vector<int> route;
  int best_length = kUnreachableDistance;
  for (int trial = 0; trial < std::max(1, weights.starts); ++trial) {
    auto remaining = resource_points;
    std::vector<int> candidate;
    int current = start;
    if (!remaining.empty() && trial) {
      std::size_t choice = static_cast<std::size_t>(trial - 1) % remaining.size();
      current = remaining[choice];
      candidate.push_back(current);
      remaining.erase(remaining.begin() + choice);
    }
    while (!remaining.empty()) {
      auto p = std::min_element(remaining.begin(), remaining.end(), [&](int a, int b) {
        return distance[current][a] < distance[current][b];
      });
      current = *p;
      candidate.push_back(current);
      remaining.erase(p);
    }
    improve_visit_order(candidate, start, distance);
    int cost = route_length(candidate, start, distance);
    if (cost < best_length) {
      route = std::move(candidate);
      best_length = cost;
    }
  }
  if (!retained.empty()) {
    route = std::move(retained);
    improve_visit_order(route, start, distance);
  }
  resource_points_ = resource_points;
  const int visibility_words = (n + 63) / 64;
  std::vector<std::vector<std::uint64_t>> visibility_masks(
      n, std::vector<std::uint64_t>(visibility_words));
  std::vector<int> view_gain(n);
  for (int p = 0; p < n; ++p)
    for (int y = std::max(0, p / w - radius); y <= std::min(h - 1, p / w + radius); ++y)
      for (int x = std::max(0, p % w - radius); x <= std::min(w - 1, p % w + radius); ++x)
        if (cell_value(y * w + x) == -1) {
          int q = y * w + x;
          visibility_masks[p][q / 64] |= std::uint64_t{1} << (q % 64);
          const int edge = std::min({x, y, w - 1 - x, h - 1 - y});
          view_gain[p] +=
              100 + static_cast<int>(100 * weights.path_edge_bonus *
                                     std::max(0.0, 1.0 - static_cast<double>(edge) /
                                                             std::max(1, weights.path_edge_band)));
        }
  // 同长度路线也有不同的信息量；在距离严格递减的DAG上累计视野潜力，禁止额外绕路。
  std::vector<std::vector<int>> path_next(n);
  const auto build_shortest_path = [&](int from, int to) {
    if (path_next[to].empty()) {
      auto& next = path_next[to];
      next.assign(n, -1);
      std::vector<double> potential(n);
      std::vector<int> points(n);
      std::iota(points.begin(), points.end(), 0);
      std::stable_sort(points.begin(), points.end(),
                       [&](int a, int b) { return distance[a][to] < distance[b][to]; });
      for (int p : points) {
        if (p == to || distance[p][to] >= kUnreachableDistance)
          continue;
        double best = -1;
        for (int q : {p % w ? p - 1 : -1, p % w + 1 < w ? p + 1 : -1, p >= w ? p - w : -1,
                      p + w < n ? p + w : -1}) {
          if (q < 0 || distance[q][to] + 1 != distance[p][to])
            continue;
          double information = view_gain[q] > 0 ? 100 * std::pow(view_gain[q] / 100.0,
                                                                 weights.path_potential_exponent)
                                                : 0;
          double gain = weights.prefer_information_paths ? information + potential[q] : 0;
          if (gain > best) {
            best = gain;
            next[p] = q;
            potential[p] = gain;
          }
        }
      }
    }
    std::vector<int> steps{from};
    while (from != to && distance[from][to] < kUnreachableDistance) {
      int next = path_next[to][from];
      if (next < 0)
        break;
      from = next;
      steps.push_back(from);
    }
    return steps;
  };
  std::unordered_map<int, std::vector<std::uint64_t>> leg_masks;
  // 合并整条计划沿途的新增视野；已知格直接视为已覆盖，分段结果在本帧复用。
  const auto predict_coverage = [&](const std::vector<int>& order) {
    auto bits = visibility_masks[start];
    int from = start;
    for (int to : order) {
      auto [it, inserted] = leg_masks.try_emplace(from * n + to);
      auto& mask = it->second;
      if (inserted) {
        mask.assign(visibility_words, 0);
        for (int p : build_shortest_path(from, to))
          for (int word = 0; word < visibility_words; ++word)
            mask[word] |= visibility_masks[p][word];
      }
      for (int word = 0; word < visibility_words; ++word)
        bits[word] |= mask[word];
      from = to;
    }
    std::vector<bool> covered(n);
    for (int p = 0; p < n; ++p)
      covered[p] = cell_value(p) != -1 || (bits[p / 64] & (std::uint64_t{1} << (p % 64)));
    return covered;
  };
  double final_objective = 0;
  bool has_objective = false;
  if (resources.empty() || weights.expected_resources <= 0 ||
      known_resources < weights.expected_resources) {
    for (int addition = 0; addition < 16; ++addition) {
      const auto covered = predict_coverage(route);
      if (std::find(covered.begin(), covered.end(), false) == covered.end())
        break;
      double best = std::numeric_limits<double>::infinity();
      int chosen = -1, insert = 0;
      for (int p = 0; p < n; ++p) {
        if (walls[p] || distance[start][p] >= kUnreachableDistance)
          continue;
        int gain = 0;
        for (int y = std::max(0, p / w - radius); y <= std::min(h - 1, p / w + radius); ++y)
          for (int x = std::max(0, p % w - radius); x <= std::min(w - 1, p % w + radius); ++x)
            if (!covered[y * w + x])
              ++gain;
        if (!gain)
          continue;
        for (int i = 0; i <= static_cast<int>(route.size()); ++i) {
          int before = i ? route[i - 1] : start;
          int extra = distance[before][p];
          if (i < static_cast<int>(route.size()))
            extra += distance[p][route[i]] - distance[before][route[i]];
          double cost = (extra + weights.insertion_offset) / std::pow(gain, weights.gain_exponent);
          if (cost < best) {
            best = cost;
            chosen = p;
            insert = i;
          }
        }
      }
      if (chosen < 0)
        break;
      route.insert(route.begin() + insert, chosen);
      improve_visit_order(route, start, distance);
    }
    // 删除仅因路线重排变得多余的观察点；不能删真实资源。
    for (int i = static_cast<int>(route.size()) - 1; i >= 0; --i) {
      if (std::find(resource_points.begin(), resource_points.end(), route[i]) !=
          resource_points.end())
        continue;
      auto candidate = route;
      candidate.erase(candidate.begin() + i);
      auto c = predict_coverage(candidate);
      if (std::find(c.begin(), c.end(), false) == c.end())
        route = std::move(candidate);
    }
    if (weights.discovery_urgency > 0 || weights.future_resource_weight > 0) {
      // 只优化总路程会把探索安排到末尾。额外比较未知格第一次进入视野的时间，
      // 用公开未知格的平均等待时间鼓励早发现，并与绕路步数明确交换。
      const int words = visibility_words;
      const auto& views = visibility_masks;
      int unknown_count = 0;
      for (int p = 0; p < n; ++p)
        if (cell_value(p) == -1)
          ++unknown_count;
      std::vector<int> unknown;
      for (int p = 0; p < n; ++p)
        if (cell_value(p) == -1 && distance[start][p] < kUnreachableDistance)
          unknown.push_back(p);
      const int unknown_positions = static_cast<int>(unknown.size());
      // 总路程 + 平均发现延迟 + 未来资源的平均插入成本；逐个未知格枚举，不随机采样。
      const auto score_route = [&](const std::vector<int>& order) {
        auto seen = views[start];
        int from = start, elapsed = 0;
        double latency = 0;
        // 按公开未知格逐格假设，记录首次可见时所在位置和路线段；不读取隐藏资源。
        std::vector<int> discovery_position, discovery_leg;
        if (weights.causal_future_fraction > 0) {
          discovery_position.assign(n, -1);
          discovery_leg.assign(n, -1);
        }
        int leg = 0;
        for (int to : order) {
          auto steps = build_shortest_path(from, to);
          for (std::size_t i = 1; i < steps.size(); ++i) {
            ++elapsed;
            for (int word = 0; word < words; ++word) {
              auto extra = views[steps[i]][word] & ~seen[word];
              latency += elapsed * __builtin_popcountll(extra);
              if (weights.causal_future_fraction > 0) {
                auto newly_seen = extra;
                while (newly_seen) {
                  const int point = word * 64 + __builtin_ctzll(newly_seen);
                  discovery_position[point] = steps[i];
                  discovery_leg[point] = leg;
                  newly_seen &= newly_seen - 1;
                }
              }
              seen[word] |= extra;
            }
          }
          from = to;
          ++leg;
        }
        double future = 0;
        for (int point : unknown) {
          int insertion = kUnreachableDistance, previous = start;
          for (int next : order) {
            insertion = std::min(insertion, distance[previous][point] + distance[point][next] -
                                                distance[previous][next]);
            previous = next;
          }
          insertion = std::min(insertion, distance[previous][point]);
          if (weights.causal_future_fraction > 0 && discovery_position[point] >= 0) {
            int before = discovery_position[point];
            int causal_insertion = kUnreachableDistance;
            // 只在首次发现后的剩余路线插入，不把已经走过的路段当作免费采集机会。
            for (int i = discovery_leg[point]; i < static_cast<int>(order.size()); ++i) {
              const int next = order[i];
              causal_insertion =
                  std::min(causal_insertion, distance[before][point] + distance[point][next] -
                                                 distance[before][next]);
              before = next;
            }
            causal_insertion = std::min(causal_insertion, distance[before][point]);
            future += insertion + weights.causal_future_fraction * (causal_insertion - insertion);
          } else {
            future += insertion;
          }
        }
        future *= static_cast<double>(std::max(0, weights.expected_resources - known_resources)) /
                  std::max(1, unknown_positions);
        return route_length(order, start, distance) +
               weights.discovery_urgency * latency / std::max(1, unknown_count) +
               weights.future_resource_weight * future;
      };
      for (int pass = 0; pass < 10; ++pass) {
        double best = score_route(route);
        std::vector<int> chosen = route;
        for (int i = 0; i < static_cast<int>(route.size()); ++i)
          for (int j = i + 1; j < static_cast<int>(route.size()); ++j) {
            auto candidate = route;
            std::reverse(candidate.begin() + i, candidate.begin() + j + 1);
            // 原路线沿途承担的覆盖不能因重排而丢失，防止为了早看大块漏掉角落。
            auto c = predict_coverage(candidate);
            if (std::find(c.begin(), c.end(), false) != c.end())
              continue;
            double cost = score_route(candidate);
            if (cost < best - 1e-9) {
              best = cost;
              chosen = std::move(candidate);
            }
          }
        // 反转只能交换连续子路线；补充单点移位，允许观察点跨过一整段资源路线。
        for (std::size_t i = 0; weights.relocate && i < route.size(); ++i) {
          auto removed = route;
          const int point = removed[i];
          removed.erase(removed.begin() + i);
          for (std::size_t j = 0; j <= removed.size(); ++j) {
            if (j == i)
              continue;
            auto candidate = removed;
            candidate.insert(candidate.begin() + j, point);
            const auto covered = predict_coverage(candidate);
            if (std::find(covered.begin(), covered.end(), false) != covered.end())
              continue;
            const double cost = score_route(candidate);
            if (cost < best - 1e-9) {
              best = cost;
              chosen = std::move(candidate);
            }
          }
        }
        if (chosen == route)
          break;
        route = std::move(chosen);
      }
      final_objective = score_route(route);
      has_objective = true;
    }
  }
  route_ = route;
  if (route.empty())
    return result;
  auto steps = build_shortest_path(start, route.front());
  if (steps.size() > 1 && cell_value(steps[1]) != 0)
    return result;  // 估算未知不等于允许盲走。
  result.first_leg.status = SearchStatus::Found;
  for (int p : steps)
    result.first_leg.path.push_back({p % w, p / w});
  result.first_leg.cost = static_cast<int>(steps.size()) - 1;
  result.cost = route_length(route, start, distance);
  result.objective = has_objective ? final_objective : result.cost;
  const auto covered = predict_coverage(route);
  result.uncovered = std::count(covered.begin(), covered.end(), false);
  for (int p : route)
    for (const auto& r : resources)
      if (linear_index(r.position) == p) {
        result.keys.push_back(r.key);
        if (p == route.front())
          result.resource_key = r.key;
      }
  // 用相同公开观察重建一次探索点布局，与保留路线公平比较；只有明确改善才换。
  // 副本复用已知障碍距离表；递归关闭此功能，最多比较一个新布局，不读取真值。
  if (weights.refresh_margin >= 0 && known_resources < weights.expected_resources &&
      had_retained_route) {
    auto fresh = *this;
    fresh.route_.clear();
    const auto margin = weights.refresh_margin;
    weights.refresh_margin = -1;
    auto candidate =
        fresh.plan(grid, start_position, resources, radius, known_resources, blocked, weights);
    if (candidate.first_leg.success() && candidate.uncovered <= result.uncovered &&
        candidate.objective + margin < result.objective - 1e-9) {
      *this = std::move(fresh);
      return candidate;
    }
  }
  return result;
}
}  // namespace rmtc::planning
