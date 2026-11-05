/**
 * @file stage2_navigation_progress.hpp
 * @brief 识别没有资源、伤害、任务、新视野进展的局部往返，避免把必要折返一概禁止。
 *
 * @details 模块协作与维护约定
 * 根据位置历史和资源、伤害、任务、新视野是否增长判断无进展循环，不禁止正常折返。
 * 软惩罚有有效期，持续出口目标用于减少脱困抖动；上层仍应记录最终动作是否覆盖脱困建议。
 */
#pragma once
#include "rmtc_connection/stage2_config.hpp"
#include "rmtc_connection/public_data.hpp"
#include <cmath>
#include <deque>
#include <map>
#include <optional>
#include <set>

namespace rmtc {
class NavigationProgress {
 public:
  /** @brief 换局清除历史、软代价与触发次数，防止上一局影响新局。 */
  void reset() {history_.clear();penalties_.clear();previous_.reset();breaks_=0;exit_.reset();cycle_.clear();}
  /**
   * @brief 更新连续公开帧；有任何可确认收益、死亡、复活或漏帧时重新计时。
   * 只有窗口内至少一半帧实际移动、所经不同格数至多窗口一半时才触发。
   * 因此静止防御、直线长途移动、正在采集或持续开图均不判为局部循环。
   */
  void observe(const PublicPerception& frame,int observed,const Stage2Weights& w) {
    for(auto it=penalties_.begin();it!=penalties_.end();)
      if(it->second<=frame.tick) it=penalties_.erase(it);else ++it;
    if(w.navigation_cycle_window==0||frame.tick<w.navigation_cycle_start_tick) {history_.clear();penalties_.clear();previous_.reset();exit_.reset();cycle_.clear();return;}
    Sample current{frame.tick,frame.self.position,frame.score.resource_value,frame.score.effective_damage,
        static_cast<int>(frame.score.mission_completed_ids.size()),observed,false};
    const bool continuous=previous_&&current.tick==previous_->tick+1&&frame.self.alive&&frame.self.position_reason!="respawn";
    const bool progress=previous_&&(current.resource>previous_->resource||current.damage>previous_->damage||
        current.missions>previous_->missions||current.observed>previous_->observed);
    if(!continuous||progress) {history_.clear();penalties_.clear();exit_.reset();cycle_.clear();}
    if(exit_&&(frame.tick>=exit_until_||(current.at.x==exit_->x&&current.at.y==exit_->y))) {
      exit_.reset();cycle_.clear();history_.clear();penalties_.clear();
    }
    if(continuous) current.moved=std::abs(current.at.x-previous_->at.x)+std::abs(current.at.y-previous_->at.y)==1;
    previous_=current;
    if(!frame.self.alive) return;
    history_.push_back(current);
    while(history_.size()>static_cast<std::size_t>(w.navigation_cycle_window)) history_.pop_front();
    if(history_.size()<static_cast<std::size_t>(w.navigation_cycle_window)||!penalties_.empty()||exit_) return;
    std::set<std::pair<int,int>> positions;int moves=0;
    for(const auto& s:history_) {positions.emplace(s.at.x,s.at.y);moves+=s.moved;}
    if(moves<w.navigation_cycle_window/2||positions.size()>static_cast<std::size_t>(w.navigation_cycle_window/2)) return;
    cycle_=positions;
    for(auto p:positions) penalties_[p]=frame.tick+w.navigation_cycle_cooldown;
    history_.clear();++breaks_;
  }
  /** @brief 软代价到期即为0；绝不改变地图通行性或阻断唯一返程。 */
  double penalty(Position p,std::int64_t tick,const Stage2Weights& w) const {
    const auto it=penalties_.find({p.x,p.y});return it!=penalties_.end()&&it->second>tick?w.navigation_cycle_cost:0;
  }
  /** @brief 本帧是否仍处于脱困窗口，仅用于决策和诊断。 */
  bool active() const {return !penalties_.empty()||exit_.has_value();}
  /// 返回已承诺的退出目标；调用者仍须每Tick检查通行性和紧急危险。
  std::optional<Position> exit() const {return exit_;}
  /// 冻结目标并保留足够行程时间，不能每步把刚离开的格子重选为新目标。
  void commit_exit(Position p,std::int64_t tick,int distance,int minimum_ticks) {
    exit_=p;exit_until_=tick+std::min(100,std::max(minimum_ticks,2*distance+2));
  }
  /// 路线失效时允许重新规划；该格短期仍受原循环代价约束。
  void cancel_exit() {exit_.reset();}
  /// 新退出目标与原循环区域至少留两格距离，避免在循环边缘挪动一步就宣布成功。
  int cycle_distance(Position p) const {
    int d=100000;
    for(auto q:cycle_)d=std::min(d,std::abs(p.x-q.first)+std::abs(p.y-q.second));
    return d;
  }
  /** @brief 返回本局累计识别次数，不把“识别”当作一定成功脱困。 */
  int breaks() const {return breaks_;}
 private:
  struct Sample {std::int64_t tick;Position at;double resource,damage;int missions,observed;bool moved;};
  std::deque<Sample> history_;
  std::optional<Sample> previous_;
  std::optional<Position> exit_;
  std::int64_t exit_until_{0};
  std::set<std::pair<int,int>> cycle_;
  std::map<std::pair<int,int>,std::int64_t> penalties_;
  int breaks_{0};
};
} // namespace rmtc
