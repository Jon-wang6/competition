/**
 * @file stage2_collection_neural.hpp
 * @brief 独立采集网络的公开路线特征；与战斗网络严格分域。
 */
#pragma once
#include "rmtc_connection/stage2_planning.hpp"

namespace rmtc {
constexpr double kCollectionNeuralScale=64.0; ///< 统一训练标签和部署修正的数值尺度。
/// 记录静态采集评分尺度；部署时拒绝把不同基线训练的网络直接混用。
Json collection_baseline_signature(const Stage2Weights& weights);
/**
 * @brief 为一条根动作预测路线构建64维特征，输出全部限定在[-1,1]。
 * 仅使用公开资源记忆、双方估计距离、任务缺口和已有规划分解；不读隐藏资源。
 * root顺序与规划器一致：WAIT/UP/DOWN/LEFT/RIGHT。
 */
NeuralFeatures collection_route_features(const PublicPerception& frame,const MapMemory& memory,
    const PlanningResult& planning,std::size_t root,int normal,int high,
    const std::vector<int>& enemy_steps,const std::vector<double>& supply);
} // namespace rmtc
