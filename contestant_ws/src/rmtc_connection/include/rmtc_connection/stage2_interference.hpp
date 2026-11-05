/**
 * @file stage2_interference.hpp
 * @brief 伤害满分后的低风险干扰：利用眼前攻击机会，限制自身可见掉血风险。
 *
 * 干扰收益属于战术估计，不写进裁判分数；不知道敌方累计得分时，不宣称对手
 * 一定扣了多少分。额外伤害只按有限的死亡扣分缓冲估值，不奖励无限刷伤害。
 *
 * @details 模块协作与维护约定
 * 满伤害后的干扰是附加竞争价值，不应冒充新增自身伤害积分。
 * 评价干扰时保留掉血风险、资源机会和路程成本，避免为了压别人分而显著损害自己。
 */
#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rmtc {
struct Stage2InterferenceWeights {
  bool modify_events{false};     ///< 默认保留原回血/Buff/辣椒规划；额外事件限制仅供独立实验。
  double weight{0.15};           ///< 轻干扰偏好；22图对照自身分数均持平，0可关闭，范围[0,1]。
  double minimum_hp{80.0};       ///< 低于本值不为干扰额外攻击；自己的采集/任务/撤离仍优先。
  double reserve_hp{60.0};       ///< 计入可见攻击/反伤上界后仍需保留的生命值。
  double maximum_incoming{20.0}; ///< 为干扰可接受的可见掉血上界，不是必然实际损失。
  double reserve_damage{60.0};   ///< 满伤害后最多估值多少额外有效伤害作为扣分缓冲。
  double reserve_weight{0.25};   ///< 缓冲价值折扣，不冒充已经取得的新分数。
  double heal_multiplier{1.25};  ///< 实验项：仅modify_events=true时放大满伤害后的可用回血收益。
  int maximum_detour{3};         ///< 实验项：仅modify_events=true时限制辣椒/夺无双点距离。

  /** @brief 启动时拒绝非法预算，禁止负风险、无限干扰权重或不可达的生命值门槛。 */
  void validate() const {
    for(double v:{weight,minimum_hp,reserve_hp,maximum_incoming,reserve_damage,reserve_weight,heal_multiplier})
      if(!std::isfinite(v)||v<0) throw std::invalid_argument("invalid postcap interference weights");
    if(weight>1||minimum_hp>100||reserve_hp>minimum_hp||maximum_incoming>100||
       reserve_damage>120||reserve_weight>1||heal_multiplier>4||maximum_detour<0||maximum_detour>12)
      throw std::invalid_argument("postcap interference budget outside bounds");
  }

  /** @brief 同时满足生命值门槛、掉血预算和剩余血量；不提高危险动作的许可上限。 */
  bool safe(double hp,double incoming) const {
    return hp>=minimum_hp&&incoming<=maximum_incoming&&hp-incoming>=reserve_hp;
  }

  /**
   * @brief 为当前已可攻击的敌人估算干扰价值；不用于生成追击目标。
   * 低血收尾更值得利用；普通消耗仅给很小的潜在击杀价值。
   * urgent表示自己的资源/任务/撤离不能耽误，此时额外干扰收益为0。
   */
  double attack_value(double own_damage,double cap,double points,double hp,double incoming,
                      double target_hp,double damage,bool urgent) const {
    if(weight==0||cap<=0||own_damage<cap||urgent||!safe(hp,incoming)||target_hp<=0||damage<=0) return 0;
    const double progress=std::min(1.0,damage/target_hp);
    const double opponent_loss_upper=points/cap*60.0;
    const double buffer_before=std::clamp(own_damage-cap,0.0,reserve_damage);
    const double buffer_after=std::clamp(own_damage+damage-cap,0.0,reserve_damage);
    const double insurance=points/cap*(buffer_after-buffer_before)*reserve_weight;
    return weight*(opponent_loss_upper*progress*progress+insurance);
  }
};
}  // namespace rmtc
