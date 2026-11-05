/**
 * @file stage2_purpose.hpp
 * @brief 可调的采集/战斗目标门槛与多步截击预测；只使用本队公开观察。
 * 没看见资源不等于全场耗尽。资源重新出现后仍可返回采集；敌人预测始终保留转向和等待。
 *
 * @details 模块协作与维护约定
 * 目标门槛帮助明确采集与战斗目的，多步截击使用的是可变的敌人运动假设。
 * 预测落点用于导航和机会评估，当前攻击对象必须满足真实可见与攻击规则。
 */
#pragma once
#include "rmtc_connection/stage2_opponent_intent.hpp"
#include <map>
#include <stdexcept>

namespace rmtc {
/// 各门槛与预测长度独立训练；默认关闭以保留已部署策略作为严格对照。
struct Stage2PurposeWeights {
  bool enabled{false};
  int start_tick{0}; ///< 可将有目的的交战/截击留到采集开局之后；0兼容原实验。
  bool protect_resources{false}; ///< 记忆中仍可能存在资源时暂停新增追击策略，优先沿原逻辑采集。
  int resource_quiet_ticks{12}; ///< 无可采目标的宽限Tick，避免开局立刻放弃开图。
  double resource_floor{0.5}; ///< 资源目标相对统一最佳收益的下限；0表示优先任何正收益资源。
  double battle_floor{0.65}; ///< 无可采资源时，战斗/机关目标至少保留多少统一最佳收益。
  double intercept_weight{0.6}; ///< 多步预测与当前目标模型的混合比例。
  int intercept_horizon{4}; ///< 只预测短期可达攻击位，防止远期不确定路线压过近期机会。
  double heading_weight{2.0}; ///< 已观察连续移动方向的惯性，随预测距离递减。
  double escape_weight{1.0}; ///< 低血敌人远离本车的软偏好；0关闭此假设。
  double patrol_visible_scale{1.0}; ///< 1兼容原候选；实验参数0禁止巡逻抢占直接交战目标。
  int chase_progress_ticks{16}; ///< 对同一敌人累计追逐移动却没有伤害的预算，之后冷却换目标。

  /// 启动时校验范围，防止批量参数搜索生成无界预测或非法机会成本。
  void validate() const {
    for(double x:{resource_floor,battle_floor,intercept_weight,heading_weight,escape_weight,patrol_visible_scale})
      if(!std::isfinite(x)||x<0) throw std::invalid_argument("invalid purpose parameter");
    if(start_tick<0||start_tick>499||resource_quiet_ticks<0||resource_quiet_ticks>100||resource_floor>1||battle_floor>1||
       intercept_weight>1||intercept_horizon<1||intercept_horizon>8||heading_weight>8||
       escape_weight>4||patrol_visible_scale>1||chase_progress_ticks<4||chase_progress_ticks>80)
      throw std::invalid_argument("purpose parameter outside bounds");
  }
};

/// 只有到达启用Tick且没有受保护的资源线索时，才允许新增追击策略改变原判断。
inline bool purpose_policy_active(const Stage2PurposeWeights& w,std::int64_t tick,bool resource) {
  return w.enabled&&tick>=w.start_tick&&!(w.protect_resources&&resource);
}

struct InterceptForecast {
  std::vector<std::vector<std::vector<IntentPosition>>> opponents; ///< [敌人][移动步数][可能位置]
  Json details=Json::array(); ///< 只输出每层概率最大的点，完整分布保留在内存。
};

/**
 * @brief 展开短期逃跑/等待/转向分布，每步重新归一化，不读取敌方指令或隐藏地形。
 * 已知墙禁止进入；未知格打折但不认作墙。事件墙通过调用方的valid回调生效。
 * 每层合并相同坐标，最多保留96个概率最高的落点，避免指数式路径增长。
 */
template<class Valid,class Known>
InterceptForecast forecast_intercepts(const PublicPerception& frame,
    const std::optional<PublicPerception>& previous,const OpponentIntentModel& intents,
    std::size_t visible_count,const Stage2PurposeWeights& w,Valid valid,Known known) {
  InterceptForecast out;
  for(std::size_t index=0;index<visible_count;++index) {
    const auto& intent=intents.opponents.at(index);
    const auto& e=intent.entity;
    Position heading{0,0};
    if(previous&&previous->session_id==frame.session_id&&previous->tick+1==frame.tick)
      for(const auto& old:previous->visible_entities)
        if(same_opponent(old,e)&&(!old.hp||*old.hp>0)&&intent_distance(old.position,e.position)==1)
          heading={e.position.x-old.position.x,e.position.y-old.position.y};
    std::vector<std::vector<IntentPosition>> layers{{{e.position,1.0}}};
    Json top=Json::array();
    for(int step=1;step<=w.intercept_horizon;++step) {
      std::map<std::pair<int,int>,double> mass;
      for(const auto& at:layers.back()) {
        std::vector<IntentPosition> choices;
        double total=0;
        for(Position d:std::vector<Position>{{0,0},{0,-1},{0,1},{-1,0},{1,0}}) {
          Position p{at.position.x+d.x,at.position.y+d.y};
          if(!valid(p))continue;
          double weight=(d.x==0&&d.y==0)?2.0:1.0;
          if(step==1) {
            for(const auto& next:intent.next)
              if(next.position.x==p.x&&next.position.y==p.y) weight=6.0*next.probability;
          } else if((heading.x||heading.y)&&d.x==heading.x&&d.y==heading.y)
            weight+=w.heading_weight/std::sqrt(static_cast<double>(step));
          const double injury=std::clamp((100.0-e.hp.value_or(100.0))/100.0,0.0,1.0);
          const int away=intent_distance(p,frame.self.position)-intent_distance(at.position,frame.self.position);
          weight*=std::exp(w.escape_weight*injury*away);
          if(!known(p))weight*=0.7;
          choices.push_back({p,weight});total+=weight;
        }
        if(total==0) { mass[{at.position.x,at.position.y}]+=at.probability;continue; }
        for(const auto& p:choices)mass[{p.position.x,p.position.y}]+=at.probability*p.probability/total;
      }
      std::vector<IntentPosition> layer;
      for(const auto& [p,prob]:mass)layer.push_back({{p.first,p.second},prob});
      std::sort(layer.begin(),layer.end(),[](const auto& a,const auto& b){return a.probability>b.probability;});
      if(layer.size()>96)layer.resize(96);
      double total=0;for(const auto& p:layer)total+=p.probability;
      for(auto& p:layer)p.probability/=total;
      if(!layer.empty())top.push_back({{"step",step},{"position",{layer[0].position.x,layer[0].position.y}},{"probability",layer[0].probability}});
      layers.push_back(std::move(layer));
    }
    out.details.push_back({{"id",e.id.value_or(e.team.value_or("enemy"))},{"likely_positions",top}});
    out.opponents.push_back(std::move(layers));
  }
  return out;
}

/// 预计走steps步后、下一Tick能合法攻击目标的概率；墙不挡子弹，机器人仍按当前身体保守挡弹。
inline double intercept_probability(Position from,std::size_t target,int steps,
    const InterceptForecast& forecast,const std::vector<VisibleEntity>& bodies,int square_radius=0) {
  if(target>=forecast.opponents.size()||steps<1||steps>=static_cast<int>(forecast.opponents[target].size()))return 0;
  double total=0;
  for(const auto& p:forecast.opponents[target][steps])
    if(square_radius>0 ? (intent_distance(from,p.position)>0&&
        std::max(std::abs(from.x-p.position.x),std::abs(from.y-p.position.y))<=square_radius)
       : intent_clear_shot(from,p.position,bodies,static_cast<int>(target)))
      total+=p.probability;
  return std::clamp(total,0.0,1.0);
}
} // namespace rmtc
