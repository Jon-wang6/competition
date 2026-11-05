/**
 * @file stage2_fortress_policy.hpp
 * @brief 堡垒争夺的公开信息估计：先比较双方强化时刻，再看对方能否覆盖占领位置。
 *
 * 堡垒允许多车分别达标。这里放弃的是火力优势不利的占领尝试，不宣称名额已被抢走。
 * 只有连续可见驻守或已确认Buff才形成威胁；从视野消失后的驻留不能凭空继续累计。
 *
 * @details 模块协作与维护约定
 * 比较预计占领/强化时刻与敌方覆盖能力，已经赶不上的争夺不应仅因Buff价值高而盲目坚持。
 * 敌方Buff与位置只有公开证据才能确认；本文件的估计不授予任何实际强化。
 */
#pragma once
#include "rmtc_connection/stage2_event_config.hpp"
#include "rmtc_connection/stage2_events.hpp"
#include <algorithm>
#include <cmath>
#include <string>

namespace rmtc {
struct FortressContest {
  bool losing{false};
  bool confirmed_buff{false};
  std::string rival;
  Position rival_position;
  std::int64_t own_ready_tick{0}, rival_ready_tick{0};
};

/**
 * @brief 估计某落点的占领时间差；对方更早强化且5×5射程覆盖该点时才建议放弃。
 * travel是本车到落点的已知步数，未开图只能传距离下界并在日志中保留估计性质。
 * 预测对手继续驻留并非事实；对手离开、死亡、Buff到期或己方已强化后即重新判断。
 */
inline FortressContest fortress_contest(const Stage2Events& events,
    const Stage2EventEffect& event, const PublicPerception& frame, Position destination,
    int travel, const Stage2EventWeights& weights) {
  FortressContest result;
  if (!weights.fortress_contest_enabled || event.kind != "unrivaled" ||
      !event.actor.empty() || event.end_tick <= frame.tick || travel < 0 ||
      events.buff(frame.self.team,"unrivaled",frame.tick)) return result;
  const int own_stay = Stage2Events::contains(event.cells,frame.self.position)
      ? events.progress(event,frame.self.team) : 0;
  result.own_ready_tick = std::max(event.begin_tick,frame.tick+travel) +
      std::max(0,event.stay_ticks-own_stay);
  for (const auto& enemy : frame.visible_entities) {
    if (enemy.type!="robot" || !enemy.team || *enemy.team==frame.self.team ||
        (enemy.hp&&*enemy.hp<=0)) continue;
    const auto buff=events.buff(*enemy.team,"unrivaled",frame.tick);
    const int progress=events.progress(event,*enemy.team);
    if (!buff && (!Stage2Events::contains(event.cells,enemy.position) ||
                  progress < weights.fortress_min_enemy_stay)) continue;
    const int radius=buff?buff->effect.attack_radius:event.attack_radius;
    if (std::max(std::abs(destination.x-enemy.position.x),
                 std::abs(destination.y-enemy.position.y))>radius) continue;
    const auto ready=buff?frame.tick:
        std::max(frame.tick,event.begin_tick)+std::max(0,event.stay_ticks-progress);
    const auto until=buff?buff->until:event.end_tick;
    // 对方增益若在本车抵达前就到期，不把过时优势算成拦路火力。
    if (ready+weights.fortress_race_margin>=result.own_ready_tick ||
        until<=std::max(event.begin_tick,frame.tick+travel) || ready>=event.end_tick) continue;
    if (!result.losing || ready<result.rival_ready_tick) {
      result.losing=true; result.confirmed_buff=buff!=nullptr;result.rival=*enemy.team;
      result.rival_position=enemy.position;result.rival_ready_tick=ready;
    }
  }
  return result;
}
}  // namespace rmtc
