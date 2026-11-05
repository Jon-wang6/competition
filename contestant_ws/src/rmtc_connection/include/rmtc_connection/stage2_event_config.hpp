/**
 * @file stage2_event_config.hpp
 * @brief 八类事件的独立偏好系数；事件时间、区域及实际效果数值必须来自模型解释。
 *
 * 这里只调决策收益与风险，不改裁判规则。更改后重新编译并重启；也可用同名
 * event_* ROS参数做对照。死亡上界、幻术方向与临时墙生命周期不受权重关闭。
 *
 * @details 模块协作与维护约定
 * 这里的系数改变策略对危险/增益的偏好，不改变公告给出的生效Tick、伤害或区域。
 * 参数应配合受校验事件使用；没有可靠事件结构时不能用系数凭空生成规则事实。
 */
#pragma once

#include <cmath>
#include <stdexcept>

namespace rmtc {
struct Stage2EventWeights {
  double blast_risk{100.0};           ///< 爆炸/世界分裂临近生效时的空间危险代价。
  double escape_weight{0.0};          ///< 撤离距离预判权重；0关闭新增BFS，1使用完整提前风险。
  int evacuation_buffer{2};           ///< 最短撤离步数之外预留多少Tick。
  double heal_value{0.04};            ///< 每点可实际利用的回血价值，满血不凭空奖励。
  double fortress_value{4.0};         ///< 无双Buff的基础目标价值，按剩余战斗收益缩放。
  double fortress_hold{2.0};          ///< 连续驻留时保留进度的偏好，不阻止驻留期间攻击/防御。
  bool fortress_contest_enabled{true}; ///< 估计双方强化时刻；不把堡垒误认为只能一人获得的资源。
  int fortress_race_margin{1};         ///< 对手预计领先超过多少Tick且能覆盖占领位置时放弃争夺。
  int fortress_min_enemy_stay{3};      ///< 至少连续观察到对手驻守多少Tick才预测其将持续占领。
  int fortress_preempt_ticks{3};       ///< 接近对手预计强化时刻前多少Tick，提前回避其未来火力范围。
  double fortress_withdraw_priority{1.5}; ///< 争夺明显不利时的撤离偏好；攻击先结算的风险仍计算。
  double unrivaled_risk_multiplier{4.0}; ///< 已确认无双敌人的路线/后续暴露软危险倍率，不改真实伤害。
  double pepper_cost{0.05};           ///< 辣椒开关的软成本，不封路。
  double pepper_value{0.02};          ///< 触发对敌环境伤害的战术价值，绝不计作个人伤害得分。
  double purity_defend_seen{0.8};     ///< 对手上一帧防御时，本Tick继续防御的估计概率。
  double purity_defend_other{0.2};    ///< 其他对手本Tick防御的估计概率。
  double purity_defense_value{0.02};  ///< 自己防御时潜在反伤的战术偏好。
  double vulnerability_risk{0.5};     ///< 移动新触发易伤时的未来暴露附加权重。

  /// 启动时拒绝非有限值、负收益权重及非法概率；不在比赛回调中反复检查。
  void validate() const {
    for (double v :
         {blast_risk, escape_weight, heal_value, fortress_value, fortress_hold, pepper_cost, pepper_value,
          purity_defend_seen, purity_defend_other, purity_defense_value, vulnerability_risk,
          fortress_withdraw_priority, unrivaled_risk_multiplier})
      if (!std::isfinite(v) || v < 0)
        throw std::invalid_argument("invalid event weights");
    if (blast_risk < 1 || escape_weight>2 || evacuation_buffer < 0 || evacuation_buffer > 20 ||
        purity_defend_seen > 1 || purity_defend_other > 1 || fortress_race_margin < 0 ||
        fortress_race_margin > 20 || fortress_min_enemy_stay < 1 || fortress_min_enemy_stay > 50 ||
        fortress_preempt_ticks < 0 || fortress_preempt_ticks > 20 ||
        unrivaled_risk_multiplier < 1 || unrivaled_risk_multiplier > 20)
      throw std::invalid_argument("invalid event risk/probability");
  }
};
}  // namespace rmtc
