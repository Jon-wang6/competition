/**
 * @file stage2_rollout_config.hpp
 * @brief 第二阶段多情景前瞻的独立计算预算与偏好参数，不改第一阶段。
 */
#pragma once
namespace rmtc {
/// 全部权重只改变规划偏好，模拟结算中的伤害/事件参数仍遵守公开规则。
struct RolloutWeights {
  bool collection_rewards{true};   ///< 混合采集critic时关闭资源及采集任务奖励，避免重复计分。
  bool independent_opponents{false}; ///< 新模式给三名对手不同情景倾向，旧模式保持不变。
  int max_transitions{0};          ///< 0保留旧时限；正数为确定性展开预算，仍保留时间兜底。
  double weight{0.25};              ///< 未来修正强度；0完整关闭；当前默认来自本轮多图对照。
  int horizon{4};                  ///< 最多向前推演几个Tick，包含当前Tick。
  int beam_width{8};               ///< 每个根动作保留几条共同动作序列，限制指数分支。
  int scenarios{6};                ///< 不同对手倾向的共同情景数，不把预测当成确定事实。
  double budget_ms{40.0};          ///< 软时间预算；每批展开检查，不等待用满预算。
  double discount{0.85};           ///< 远期收益折扣，降低长预测误差影响。
  double worst_fraction{0.0};     ///< 最差四分之一情景的风险权重，其他权重给均值。
  double goal_progress{0.4};       ///< 第二步以后对原规划目标的进展价值。
  double override_margin{0.03};    ///< 相对原动作的最小综合收益提升，避免微小估计反复改向。
  int activation_range{4};         ///< 可见敌人足够近才启动局部对抗推演。
  int start_tick{80};              ///< 开局保护窗口，避免局部对抗覆盖一次性采集机会。
  bool preserve_collection{true};  ///< 无当前攻击危险时保持资源/到达任务的战略路线。
  double risk_scale{0.1};         ///< 远期额外掉血/死亡偏好系数；实际伤害得分扣减不打折。
  /// 启动时校验有限值和资源上限，避免误填参数把控制回调拖过动作窗口。
  void validate() const;
};

}  // namespace rmtc
