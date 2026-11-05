/**
 * @file resource_peek.hpp
 * @brief 在采集路线上选择兼顾观察的下一步，只用公开记忆及已知可通行地形。
 *
 * 优先选择有额外信息的等长路线；小绕路须收益覆盖额外距离并受累计预算限制。
 * 不替换资源目标，不为原路线本来就能看到的格子重复付费，不读取隐藏资源。
 *
 * @details 模块协作与维护约定
 * 输入已有资源目标/路径，在保持采集目标的前提下评估下一步能多看多少未知区域。
 * 绕路预算与原路线可见集合共同限制探查，不将同一批新格子在每一步重复计入收益。
 */
#pragma once

#include "rmtc_connection/frontier_exploration.hpp"
#include "rmtc_connection/weighted_resource.hpp"

namespace rmtc::planning {
/// 补看后的完整采集路线及增量评分；仍保留原资源目标，实际只执行首步。
struct ResourcePeek {
  SearchResult route;
  std::int64_t extra_steps{0};
  std::size_t extra_cells{0};
  double weighted_extra_cells{0};
  double reward{0};
  double total{0};
};

/**
 * @brief 比较四个已知可走邻格，奖励原采集路线无法观察到的新增格。
 * @param direct 以当前实际位置开头、以已知资源结尾的成功A*路线；脚下资源不补看。
 * @param detour_budget 两次实际采集之间尚余的额外绕路步数，只按A*真实差值扣除。
 * @param remaining_ticks 剩余动作窗口；至少预留走完整条采集路线及一个后续窗口。
 * 代价=距离系数×额外步数-peek_reward×额外加权格数×时间折扣，严格为负才采用。
 * 额外绕路只奖励不超过peek_patch_limit格的未知连通块；0关闭此限制，等长路线不受限。
 * 同分优先较少绕路、较多新格及固定坐标顺序；邻格被阻挡/未知或路线不可达时跳过。
 */
std::optional<ResourcePeek> plan_resource_peek(
    const GridMap& map, const SearchResult& direct, int vision_radius,
    const std::vector<GridIndex>& blocked_cells, const ResourceWeights& weights,
    std::int64_t detour_budget, std::optional<std::int64_t> remaining_ticks = std::nullopt);
}  // namespace rmtc::planning
