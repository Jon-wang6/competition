/**
 * @file stage2_rollout.hpp
 * @brief 有预算的多情景束搜索：只推演已知地形、可见机器人及已校验事件。
 *
 * 预测用于比较局部动作，不宣称知道对手未来。每一完整深度都公平计算全部根动作；
 * 超时只采用上一完整深度，避免先枚举的方向得到额外计算优势。不开启时零推演。
 */
#pragma once
#include "rmtc_connection/stage2_config.hpp"
#include "rmtc_connection/stage2_events.hpp"
#include <optional>

namespace rmtc {
/// 由原策略给出的合法根动作及即时收益；MOVE方向已经是希望实际移动的世界方向。
struct RolloutRoot {
  Action action;
  double utility{0};
};
struct RolloutResult {
  Action action = Action::wait();
  bool ran{false}, changed{false}, budget_hit{false};
  int completed_depth{0};
  std::size_t transitions{0};
  double elapsed_ms{0}, utility{0};
  Json roots = Json::array();
};

/**
 * @brief 按攻击→同时移动→入点→驻留/回血→资源任务的顺序，比较共同短动作序列。
 * @param roots 原策略动作（可含重复项，内部保留同动作的最高即时收益）。
 * @param original 原策略最终选择；模型置信不足或预算不足时保持该动作。
 * @param goal_distances 到当前战略目标的已知地图距离，空数组表示不加目标进展奖励。
 * @param normal_count/high_count 公开连续帧确认的本车采集数量下界。
 * 本函数不连接ROS，不读取裁判配置，不推测视野外机器人；阵亡后的位置未知，局部
 * 预测不虚构复活点。已知事件允许推演到期、反伤、群攻、回血和成功移动触发。
 */
RolloutResult tactical_rollout(const PublicPerception& frame, const MapMemory& memory,
                               const Stage2Events& events,
                               const std::optional<PublicPerception>& previous,
                               const Stage2Weights& weights, const RolloutWeights& config,
                               const std::vector<RolloutRoot>& roots, Action original,
                               const std::vector<int>& goal_distances, double goal_utility,
                               int normal_count, int high_count, int remaining_ticks);
}  // namespace rmtc
