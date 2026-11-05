/**
 * @file stage2_phase_policy.hpp
 * @brief 第二阶段目标分期：先采集、再刷伤害、最后补任务；只使用本队公开状态。
 *
 * 分期确定本阶段优先比较哪些目标，原有收益仍负责同类目标排序和机会成本门槛。
 * 任务截止、事件撤离和动作层生存判断始终保留；不开启时完整沿用原目标选择。
 *
 * @details 模块协作与维护约定
 * 阶段选择用当前公开状态与阈值生成偏好；阶段名称不能替代最终动作合法性检查。
 * 开局限制攻击/防御与选择资源目标是两层条件，不能把只移动理解为必然正在抢资源。
 */
#pragma once

#include <cstdint>
#include <stdexcept>
#include <cmath>
#include <string_view>

namespace rmtc {

/// 阶段控制实验参数；下列为full_attack80候选，跨对手验证未通过前总开关保持关闭。
struct Stage2PhaseWeights {
  bool enabled{false};             ///< 总开关；新图验证通过后才设为默认。
  int resource_ticks{80};           ///< 开局采集窗口上限；资源无机会或分值满时提前结束。
  int chase_steps{8};               ///< 刷伤害阶段可优先追击/搜索的最大已知路程。
  int task_detour{0};               ///< 顺路到达任务允许增加的步数；-1关闭，0仅等长路线。
  double opportunity_floor{0.65};   ///< 阶段目标至少保留原最佳目标多少收益率；0为严格分期。
  double opening_attack_factor{1.0}; ///< 1保留原有开局攻击权重；小于1再压低主动射击，不降低防御收益。
  int visible_resource_steps{4};    ///< 开局优先确认可见的近邻资源；0关闭，仍须通过机会成本门槛。
  int resource_silence_ticks{0};     ///< 连续多少Tick无可见可达资源后提前转战；0关闭，发现资源可重返采集。
  double combat_opportunity_floor{1.0}; ///< 中期优先交战的收益保留门槛，独立于开局采集。
  double task_lookahead_weight{0.0}; ///< 交战目标之后接到达任务的收益折扣，不虚构已经完成任务。

  /// 拒绝越界/非有限参数，避免阶段预算、机会门槛或动作估值出现非法状态。
  void validate() const {
    if (resource_ticks < 0 || resource_ticks > 500 || chase_steps < 1 || chase_steps > 100 ||
        task_detour < -1 || task_detour > 8 || !std::isfinite(opportunity_floor) ||
        opportunity_floor < 0 || opportunity_floor > 1 ||
        !std::isfinite(opening_attack_factor) || opening_attack_factor < 0 ||
        opening_attack_factor > 1 || visible_resource_steps < 0 || visible_resource_steps > 24 ||
        resource_silence_ticks < 0 || resource_silence_ticks > 80 ||
        !std::isfinite(combat_opportunity_floor) || combat_opportunity_floor < 0 ||
        combat_opportunity_floor > 1 || !std::isfinite(task_lookahead_weight) ||
        task_lookahead_weight < 0 || task_lookahead_weight > 1)
      throw std::invalid_argument("invalid stage2 phase policy parameters");
  }
};

enum class Stage2Phase { Balanced, Collect, Fight, Finish };

/**
 * @brief 从公开资源机会和计分缺口决定阶段；不根据裁判隐藏资源/对手脚本切换。
 * 伤害掉分后可以回到Fight补分；资源分不会因死亡扣除，无需人为重复进入开局。
 */
inline Stage2Phase stage2_phase(const Stage2PhaseWeights& w, std::int64_t tick,
                                bool collection_needed, bool collection_possible,
                                bool damage_needed) {
  if (!w.enabled) return Stage2Phase::Balanced;
  if (tick < w.resource_ticks && collection_needed && collection_possible)
    return Stage2Phase::Collect;
  return damage_needed ? Stage2Phase::Fight : Stage2Phase::Finish;
}

/// 日志中的稳定阶段名称，直接说明当前行为目的，不代表已经取得相应积分。
inline const char* stage2_phase_name(Stage2Phase phase) {
  switch (phase) {
    case Stage2Phase::Collect: return "collect";
    case Stage2Phase::Fight: return "fight";
    case Stage2Phase::Finish: return "finish";
    default: return "balanced";
  }
}

/// 判定目标是否服务当前阶段；只有正收益、可达目标会进入上层的阶段候选池。
inline bool stage2_phase_goal(Stage2Phase phase, std::string_view kind, int steps,
                              int chase_steps) {
  switch (phase) {
    case Stage2Phase::Collect: return kind == "resource" || kind == "explore";
    case Stage2Phase::Fight:
      return steps <= chase_steps &&
             (kind == "combat" || kind == "search_enemy" || kind == "patrol");
    case Stage2Phase::Finish: return kind == "mission";
    default: return false;
  }
}

}  // namespace rmtc
