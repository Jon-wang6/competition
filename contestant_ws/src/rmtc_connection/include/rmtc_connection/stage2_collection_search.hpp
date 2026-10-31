/**
 * @file stage2_collection_search.hpp
 * @brief 第二阶段滚动多目标路线搜索：联合采集、沿途开图、争抢到达时间和时空危险。
 *
 * 参考第一阶段资源排序与覆盖去重的思路，但不要求访问全部资源。只执行本帧已知
 * 可通行的第一段；后续收益是带时间折扣的预测，下一Tick重新观察，不读取隐藏地图。
 */
#pragma once
#include "rmtc_connection/stage2_joint_planner.hpp"
#include <functional>

namespace rmtc {
/// 第一执行段及整条预测路线的审计数据；forecast字段绝不写入真实分数。
struct CollectionSearchPlan {
  planning::SearchResult route;
  std::string first_resource;
  double utility{0}, resource_points{0}, mission_points{0}, information_points{0};
  double forecast_resource{0}, forecast_mission{0};
  int expanded{0}, completed_depth{0}, targets{0};
  bool ran{false}, budget_hit{false};
  double elapsed_ms{0};
};
/**
 * @brief 在本队公开地图上比较多段采集/观察顺序，只返回可立即执行的首段。
 * @param passable 当前已知可走格，已含可见车辆及临时墙体约束。
 * @param information 对累计唯一未知格和边界权重估值；同格不重复领取信息奖励。
 * @param risk 查询某格在第arrival步到达时的软危险；硬阻挡仍由passable决定。
 * 不同资源按累计到达时间重新计算争抢折扣；资源、每项任务均受剩余收益预算约束。
 * 时间预算覆盖准备和搜索阶段，超时保留上一完整搜索层的结果，交由原动作层验险。
 */
CollectionSearchPlan search_collection_route(
    const PublicPerception& frame, const MapMemory& memory,
    std::vector<JointOpportunity> resources, int remaining, const Stage2Weights& weights,
    double resource_weight, double mission_weight,
    const std::function<bool(Position)>& passable,
    const std::function<double(double,double)>& information,
    const std::function<double(Position,int)>& risk,
    const std::vector<double>& unknown_weights = {});
}  // namespace rmtc
