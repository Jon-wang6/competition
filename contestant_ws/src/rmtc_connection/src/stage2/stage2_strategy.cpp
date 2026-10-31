/**
 * @file stage2_strategy.cpp
 * @brief 第二阶段基础决策：收益/路程选目标，局部攻防选动作，逐帧重规划。
 *
 * BFS一次计算已知可通行格的真实步数，A*生成最终路线。资源、任务、探索、
 * 有利交战共用一个收益尺度；射程内攻击无视障碍，但移动仍遵守地形与占位。
 * 所有对手下一步行为都是估计：日志分别给出预计收益和可能受到的伤害上界。
 */
#include "rmtc_connection/stage2_strategy.hpp"
#include "rmtc_connection/stage2_event_escape.hpp"
#include "rmtc_connection/stage2_rollout.hpp"
#include "rmtc_connection/stage2_frontier_value.hpp"
#include "rmtc_connection/stage2_collection_neural.hpp"
#include "rmtc_connection/stage2_event_route.hpp"
#include "rmtc_connection/stage2_resource_lookahead.hpp"
#include "rmtc_connection/stage2_enemy_sightings.hpp"
#include "rmtc_connection/stage2_tactical_navigation.hpp"
#include "rmtc_connection/stage2_observation_age.hpp"
#include "rmtc_connection/stage2_collection.hpp"
#include "rmtc_connection/stage2_reach_finish.hpp"
#include "rmtc_connection/stage2_fortress_policy.hpp"
#include "rmtc_connection/stage2_collection_search.hpp"
#include "rmtc_connection/stage2_exploration_value.hpp"
#include "rmtc_connection/stage2_resource_sweep.hpp"
#include "rmtc_connection/stage2_opponent_motion.hpp"
#include "rmtc_connection/stage2_opponent_intent.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <queue>
#include <set>
#include <stdexcept>
#include <utility>

namespace rmtc {
namespace {

constexpr std::array<Position, 4> kSteps{{{0, -1}, {0, 1}, {-1, 0}, {1, 0}}};
constexpr std::array<const char*, 4> kDirections{{"up", "down", "left", "right"}};

/// 比较绝对坐标；不将坐标近似或历史敌人位置当成当前相同格。
bool same(Position a, Position b) {
  return a.x == b.x && a.y == b.y;
}

/// 四方向网格的曼哈顿距离，仅用于射程和风险估计；寻路步数另由BFS/A*给出。
int distance(Position a, Position b) {
  return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

/// 判断标准攻击的直线两格射程，故意不检查障碍遮挡。
bool in_range(Position from, Position to) {
  return (from.x == to.x || from.y == to.y) && distance(from, to) > 0 && distance(from, to) <= 2;
}

/// 将世界坐标映射到便于BFS使用的上到下数组；与MapMemory的底到顶数组区分。
int cell(Position p, MapSize map) {
  return p.y * map.width + p.x;
}

/// 检查地图坐标合法性，防止事件解释或任务坐标导致越界。
bool inside(Position p, MapSize map) {
  return p.x >= 0 && p.y >= 0 && p.x < map.width && p.y < map.height;
}

/**
 * @brief 收到真实移动冲突后按席位错峰重试，打破两车同等时长等待后再次相撞的同步。
 * base为配置的基础避让Tick，0严格关闭；A/B/C/D分别附加0/1/2/3 Tick。
 * 未知席位只使用base。这里只计算临时避让期限，不把空格改成永久障碍。
 */
int collision_retry_delay(const std::string& team, int base) {
  if (base <= 0)
    return 0;
  const int offset = team.size() == 1 && team[0] >= 'A' && team[0] <= 'D'
                         ? team[0] - 'A'
                         : 0;
  return base + offset;
}

/// 将资源/伤害增量换算为尚未饱和的真实比赛分数，不累计超过上限的虚假收益。
double marginal(double current, double increment, double cap, double points) {
  return points / cap * std::max(0.0, std::min(increment, cap - current));
}

/// 当前可见活敌车；历史记忆不会进入本列表，也不会造成永久占位。
std::vector<const VisibleEntity*> enemies(const PublicPerception& frame) {
  std::vector<const VisibleEntity*> result;
  for (const auto& entity : frame.visible_entities) {
    if (entity.type == "robot" && entity.team != frame.self.team && (!entity.hp || *entity.hp > 0))
      result.push_back(&entity);
  }
  return result;
}

/// 每个方向只命中最近机器人；最近目标受保护时，不能越过它攻击第二辆车。
const VisibleEntity* attack_target(Position from, Position step,
                                   const std::vector<const VisibleEntity*>& visible) {
  const VisibleEntity* nearest = nullptr;
  int best = 3;
  for (const auto* enemy : visible) {
    const auto p = enemy->position;
    const int d = distance(from, p);
    if (d >= 1 && d < best && p.x == from.x + step.x * d && p.y == from.y + step.y * d) {
      nearest = enemy;
      best = d;
    }
  }
  return nearest;
}

/// 将一次总有效伤害估计换算成边际分数；范围攻击合计后封顶，避免多目标重复满分。
double damage_gain(const PublicPerception& frame, const Stage2Weights& weights, double damage) {
  double points =
      marginal(frame.score.effective_damage, damage, weights.damage_cap, weights.damage_points);
  for (const auto& mission : frame.missions) {
    if (mission.type != "effective_damage" || !mission.target || *mission.target <= 0 ||
        std::find(frame.score.mission_completed_ids.begin(),
                  frame.score.mission_completed_ids.end(),
                  mission.id) != frame.score.mission_completed_ids.end())
      continue;
    const double left = *mission.target - frame.score.effective_damage;
    if (left > 0)
      points += weights.mission_points * weights.mission_potential * std::min(1.0, damage / left);
  }
  return points;
}

/// 事件缺省时保留原普通攻击估计；不把上一帧defending当成本Tick确定动作。
double attack_gain(const VisibleEntity& enemy, const PublicPerception& frame,
                   const Stage2Weights& weights) {
  if (enemy.spawn_protected.value_or(false))
    return 0;
  double damage = weights.attack_damage_estimate;
  if (enemy.hp)
    damage = std::min(damage, *enemy.hp);
  if (enemy.hp && *enemy.hp <= 10)
    damage += weights.kill_reward_estimate;
  return damage_gain(frame, weights, damage);
}

/// 一项候选目标；utility越大越优，字段保留收益分解以便调参定位原因。
struct Candidate {
  std::string kind, id;
  Position position;
  double resource{0}, mission{0}, combat{0}, information{0}, utility{0}, patrol{0}, safety{0};
  double event_reward{0};
  double age_information{0};
  bool joint{false};
  bool visible_resource{false};
  double reach_priority{-1};
  double future_utility{0};
  bool sweep{false};
  bool collection_search{false};
  bool resource_race{false};
  double planned_utility{0};
  double switch_bonus{0}; ///< 保持原目标的选择奖励，不是移动一步实际获得的分数。
};

}  // namespace

Stage2Strategy::Stage2Strategy(Stage2Weights weights) : weights_(weights) {
  weights_.planning.validate();
  const auto& collector=weights_.planning.collection_neural;
  collector.validate(32);
  if((collector.enabled||collector.collect)&&!weights_.planning.enabled)
    throw std::invalid_argument("collection network requires planning mode");
  if(collector.collect&&weights_.neural.collect)
    throw std::invalid_argument("collect resource and combat datasets in separate runs");
  if(collector.collect&&collector.teacher_horizon<=weights_.planning.horizon)
    throw std::invalid_argument("collection teacher must look farther than baseline");
  if(collector.enabled) {
    collection_neural_model_.load(collector.model_file,"collection");
    std::ifstream stream(collector.model_file);Json saved;stream>>saved;
    if(saved.value("training",Json::object()).value("schema","")!="planning-v3-collection")
      throw std::invalid_argument("incompatible collection model baseline schema");
    if(saved.at("training").value("collection_baseline",Json::object())!=collection_baseline_signature(weights_))
      throw std::invalid_argument("collection model was trained on a different planning configuration");
  }
  if(collector.collect) {
    const std::filesystem::path path(collector.sample_file);
    if(!path.parent_path().empty())std::filesystem::create_directories(path.parent_path());
    collection_neural_samples_.open(path,std::ios::app);
    if(!collection_neural_samples_)throw std::runtime_error("cannot open collection samples");
  }
  weights_.neural.validate();
  if(weights_.neural.enabled) {
    neural_model_.load(weights_.neural.model_file);
    if(weights_.planning.enabled) {
      std::ifstream stream(weights_.neural.model_file);Json saved;stream>>saved;
      if(saved.value("training",Json::object()).value("schema","")!="planning-v2-combat")
        throw std::invalid_argument("planning mode requires a planning-v2-combat trained model");
    }
  }
  if(weights_.neural.collect) {
    const std::filesystem::path file(weights_.neural.sample_file);
    if(!file.parent_path().empty())std::filesystem::create_directories(file.parent_path());
    neural_samples_.open(file,std::ios::app);
    if(!neural_samples_)throw std::runtime_error("cannot open neural samples file");
  }
  weights_.duel_stalemate.validate();
  weights_.purpose.validate();
  weights_.acquisition.validate();
  weights_.temporal.validate();
  const auto& race = weights.resource_race;
  if (race.target_limit < 1 || race.target_limit > 16 || race.horizon < 1 || race.horizon > 500 ||
      race.deadline_slack < 0 || race.deadline_slack > 100 || race.explore_ticks < 0 ||
      race.explore_ticks > 500 || !std::isfinite(race.distance_cost) || race.distance_cost < 0 ||
      !std::isfinite(race.move_priority) || race.move_priority < 0 || race.move_priority > 10 ||
      !std::isfinite(race.budget_ms) || race.budget_ms < 1 || race.budget_ms > 150 ||
      !std::isfinite(race.coverage_offset) || race.coverage_offset < 0)
    throw std::invalid_argument("resource race limits/weights outside bounds");
  for (double v : {race.explore_offset, race.explore_distance_power, race.explore_edge_bonus,
      race.explore_competition, race.explore_urgency, race.explore_race_scale,
      race.explore_capture_floor, race.explore_move_priority, race.terminal_information,
      race.mission_weight})
    if (!std::isfinite(v) || v < 0) throw std::invalid_argument("invalid resource race feature weights");
  if (race.explore_method < 0 || race.explore_method > 1 || race.explore_distance_power <= 0 ||
      race.explore_race_scale <= 0 || race.explore_competition > 1 ||
      race.explore_capture_floor > 1 || race.task_deadline_buffer < 0 || race.task_deadline_buffer > 100)
    throw std::invalid_argument("invalid resource race feature bounds");
  weights_.phase.validate();
  weights_.battle_curve.validate();
  weights_.collection_transfer.validate();
  if(weights_.collection_transfer.enabled&&!weights_.acquisition.enabled)
    throw std::invalid_argument("collection transfer requires acquisition curve");
  if(weights_.opening_collection_ticks<0||weights_.opening_collection_ticks>200)
    throw std::invalid_argument("opening collection ticks outside bounds");
  weights_.events.validate();
  weights_.rollout.validate();
  weights_.interference.validate();
  for(double value:{weights.combat_clue_weight,weights.combat_clue_half_life,
      weights.opening_depletion_weight,weights.opening_depletion_floor,weights.opening_depletion_half_life})
    if(!std::isfinite(value)||value<0) throw std::invalid_argument("invalid battle evidence weights");
  if(weights.combat_clue_weight>2||weights.combat_clue_half_life<=0||weights.combat_clue_max_age<1||
     weights.combat_clue_max_age>60||weights.third_party_memory_ticks<0||weights.third_party_memory_ticks>8||
     weights.opening_depletion_weight>2||weights.opening_depletion_floor<.2||weights.opening_depletion_floor>1||
     weights.opening_depletion_half_life<=0||weights.opening_depletion_ticks<0||weights.opening_depletion_ticks>500)
    throw std::invalid_argument("battle evidence window outside bounds");
  for(double value:{weights.collection_search_weight,weights.collection_search_future,
      weights.collection_search_discount,weights.collection_search_risk,weights.collection_search_budget_ms})
    if(!std::isfinite(value)||value<0) throw std::invalid_argument("invalid collection search weights");
  if(weights.collection_search_weight>3||weights.collection_search_future>2||
     weights.collection_search_discount>1||weights.collection_search_budget_ms<1||
     weights.collection_search_budget_ms>150||weights.collection_search_ticks<0||
     weights.collection_search_ticks>500||weights.collection_search_horizon<1||
     weights.collection_search_horizon>100||weights.collection_search_depth<1||
     weights.collection_search_depth>8||weights.collection_search_beam<4||
     weights.collection_search_beam>256||weights.collection_search_targets<4||
     weights.collection_search_targets>32||weights.collection_search_resources<1||
     weights.collection_search_resources>16)
    throw std::invalid_argument("collection search budget outside bounds");
  if (!std::isfinite(weights.frontier_resource_weight) || weights.frontier_resource_weight < 0 ||
      !std::isfinite(weights.frontier_mission_weight) || weights.frontier_mission_weight < 0 ||
      !std::isfinite(weights.frontier_resource_half_life) ||
      weights.frontier_resource_half_life <= 0)
    throw std::invalid_argument("invalid frontier opportunity weights");
  if (weights.resource_mission_ramp_ticks < 0 || weights.resource_mission_ramp_ticks > 500)
    throw std::invalid_argument("resource mission ramp outside bounds");
  if (weights.reach_finish_ticks < 0 || weights.reach_finish_ticks > 500 ||
      weights.reach_attack_pause_ticks < 0 || weights.reach_attack_pause_ticks > 20 ||
      weights.reach_stalemate_ticks < 0 || weights.reach_stalemate_ticks > 100 ||
      weights.reach_collision_retry_ticks < 0 || weights.reach_collision_retry_ticks > 8 ||
      !std::isfinite(weights.reach_move_priority) || weights.reach_move_priority < 0)
    throw std::invalid_argument("invalid reach finish policy");
  const std::array<double, 99> numbers{{static_cast<double>(weights.navigation_cycle_start_tick),
                                        weights.navigation_cycle_cost,
                                        static_cast<double>(weights.navigation_cycle_window),
                                        static_cast<double>(weights.navigation_cycle_cooldown),
                                        weights.sweep_weight,weights.sweep_discount,
                                        static_cast<double>(weights.sweep_ticks),
                                        static_cast<double>(weights.sweep_horizon),
                                        static_cast<double>(weights.sweep_beam),
                                        weights.information_softness,
                                        weights.recent_enemy_search_weight,
                                        weights.recent_enemy_half_life,
                                        static_cast<double>(weights.recent_enemy_max_age),
                                        static_cast<double>(weights.blocked_retry_ticks),
                                        weights.resource_lookahead_weight,
                                        static_cast<double>(weights.resource_lookahead_steps),
                                        static_cast<double>(weights.resource_lookahead_limit),
                                        static_cast<double>(weights.resource_lookahead_depth),
                                        weights.route_information_weight,
                                        weights.reach_weight_end,
                                        static_cast<double>(weights.reach_ramp_ticks),
                                        static_cast<double>(weights.reach_deadline_buffer),
                                        weights.reach_deadline_weight,
                                        weights.visible_resource_weight,
                                        weights.resource_mission_weight,
                                        weights.resource_mission_weight_end,
                                        static_cast<double>(weights.resource_mission_ramp_ticks),
                                        static_cast<double>(weights.observation_start_tick),
                                        static_cast<double>(weights.observation_ramp_ticks),
                                        weights.observation_age_weight,
                                        static_cast<double>(weights.observation_fresh_ticks),
                                        static_cast<double>(weights.observation_stale_ticks),
                                        weights.route_alternatives_weight,
                                        weights.resource_cap,
                                        weights.resource_points,
                                        weights.damage_cap,
                                        weights.damage_points,
                                        weights.mission_points,
                                        weights.mission_potential,
                                        weights.travel_offset,
                                        weights.information_per_cell,
                                        weights.information_cap,
                                        weights.edge_information_bonus,
                                        weights.resource_age_ticks,
                                        weights.contested_resource_factor,
                                        weights.remembered_risk,
                                        weights.route_risk,
                                        weights.switch_margin,
                                        weights.attack_damage_estimate,
                                        weights.attack_probability,
                                        weights.hp_loss_cost,
                                        weights.death_cost,
                                        weights.future_exposure_cost,
                                        weights.kill_reward_estimate,
                                        static_cast<double>(weights.enemy_memory_ticks),
                                        weights.patrol_value,
                                        weights.patrol_stale_ticks,
                                        static_cast<double>(weights.patrol_min_age),
                                        weights.patrol_enemy_hint,
                                        static_cast<double>(weights.opening_ticks),
                                        weights.opening_resource_weight,
                                        weights.opening_combat_weight,
                                        weights.combat_weight_end,
                                        static_cast<double>(weights.combat_ramp_ticks),
                                        weights.opening_reach_weight,
                                        weights.initial_resource_value,
                                        weights.resource_need_power,
                                        weights.resource_coverage_power,
                                        weights.resource_depletion_power,
                                        weights.exploration_floor,
                                        weights.collection_mission_weight,
                                        static_cast<double>(weights.chase_stall_ticks),
                                        static_cast<double>(weights.chase_cooldown_ticks),
                                        static_cast<double>(weights.collection_mode),
                                        static_cast<double>(weights.joint_ticks),
                                        static_cast<double>(weights.joint_resource_limit),
                                        static_cast<double>(weights.joint_horizon),
                                        weights.joint_future_weight,
                                        weights.joint_observation_offset,
                                        weights.joint_unknown_weight,
                                        static_cast<double>(weights.joint_min_resources),
                                        weights.joint_future_weight_end,
                                        weights.joint_observation_offset_end,
                                        weights.competition_softness,
                                        weights.competition_weight,
                                        weights.competition_weight_end,
                                        weights.resource_weight_end,
                                        weights.opponent_motion_weight,
                                        weights.endgame_preserve_weight,
                                        weights.low_hp_risk_weight,
                                        weights.retreat_damage_fraction,
                                        static_cast<double>(weights.retreat_buffer_ticks),
                                        static_cast<double>(weights.retreat_window_ticks),
                                        weights.intent_chase_weight,
                                        weights.intent_risk_weight,
                                        weights.third_party_risk_discount,
                                        weights.third_party_attack_bonus,
                                        weights.intent_heading_weight,
                                        static_cast<double>(weights.intent_memory_ticks)}};
  for (double value : numbers) {
    if (!std::isfinite(value) || value < 0)
      throw std::invalid_argument("stage2 weights must be finite and nonnegative");
  }
  if (weights.blocked_retry_min_failures < 1 || weights.blocked_retry_min_failures > 32 ||
      weights.recent_enemy_half_life <= 0 || weights.recent_enemy_max_age < 1 ||
      weights.recent_enemy_max_age > 60 || weights.blocked_retry_ticks > 8 ||
      weights.resource_lookahead_steps < 1 || weights.resource_lookahead_limit < 2 ||
      weights.resource_lookahead_limit > 10 || weights.resource_lookahead_depth < 2 ||
      weights.resource_lookahead_depth > 3 || weights.route_information_weight > 1 ||
      weights.resource_cap <= 0 || weights.damage_cap <= 0 || weights.travel_offset <= 0 ||
      weights.resource_age_ticks <= 0 || weights.mission_potential > 1 ||
      weights.information_softness > 1 ||
      weights.sweep_discount > 1 || weights.sweep_ticks > 500 ||
      weights.sweep_horizon < 1 || weights.sweep_horizon > 24 ||
      weights.sweep_beam < 4 || weights.sweep_beam > 64 ||
      (weights.navigation_cycle_window>0&&weights.navigation_cycle_window<8)||
      weights.navigation_cycle_window>40||weights.navigation_cycle_cooldown<1||
      weights.navigation_cycle_start_tick>500||
      weights.navigation_cycle_cooldown>100||
      weights.contested_resource_factor > 1 || weights.attack_probability > 1 ||
      weights.attack_damage_estimate < 10 || weights.attack_damage_estimate > 20 ||
      weights.patrol_stale_ticks <= 0 || weights.initial_resource_value <= 0 ||
      weights.observation_stale_ticks <= weights.observation_fresh_ticks ||
      weights.route_alternatives_weight > 1 || weights.intent_chase_weight > 1 ||
      weights.intent_risk_weight > 1 || weights.third_party_risk_discount > 1 ||
      weights.third_party_attack_bonus > 1 || weights.intent_memory_ticks > 3 ||
      weights.opponent_motion_weight > 1 || weights.retreat_damage_fraction > 1 ||
      weights.retreat_window_ticks < 1 || weights.exploration_floor > 1 ||
      weights.collection_mode > 2 || weights.joint_min_resources > weights.joint_resource_limit ||
      weights.joint_resource_limit < 1 || weights.joint_resource_limit > 10 ||
      weights.competition_softness <= 0 || weights.competition_weight > 1 ||
      weights.competition_weight_end > 1 || weights.joint_horizon < 1 ||
      weights.chase_cooldown_ticks <= 0 ||
      (weights.chase_stall_ticks > 0 && weights.chase_stall_ticks < 8))
    throw std::invalid_argument("stage2 caps/offsets/probabilities/damage outside valid range");
}

void Stage2Strategy::reset() {
  battle_evidence_.reset();
  enemy_beliefs_.reset();
  navigation_progress_.reset();
  events_.reset();
  physical_cached_ = Action::wait();
  blocked_retry_until_.clear();
  blocked_retry_target_.reset();
  blocked_retry_streak_ = 0;
  phase_last_resource_tick_ = 0;
  acquisition_last_activity_tick_ = 0;
  reach_finish_id_.clear();
  reach_finish_position_.reset();
  reach_previous_distance_ = -1;
  reach_last_progress_tick_ = -1;
  reach_defensive_stalemate_ = 0;
  stationary_defense_ticks_=0;
  duel_stalemate_.reset();
  memory_ = MapMemory{};
  joint_planner_.reset();
  race_coverage_planner_.reset();
  plan_ = Stage2Plan{};
  previous_.reset();
  intent_previous_.reset();
  event_.reset();
  normal_collected_ = high_collected_ = 0;
  previous_goal_.clear();
  pursuit_target_.clear();
  pursuit_positions_.clear();
  pursuit_targets_.clear();
  pursuit_cooldown_.clear();
  unproductive_pursuit_.clear();
  last_observed_tick_.clear();
  cached_ = Action::wait();
}

const Stage2Plan& Stage2Strategy::plan() const noexcept {
  return plan_;
}

bool Stage2Strategy::set_event_constraints(const Stage2EventConstraints& constraints) {
  if (!memory_.current())
    return false;
  const auto& frame = *memory_.current();
  if (constraints.session_id != frame.session_id || constraints.source_text.empty() ||
      (constraints.source_text != frame.event_text &&
       frame.event_text.compare(0, constraints.source_text.size(), constraints.source_text) != 0) ||
      constraints.source_tick < 0 || constraints.source_tick > frame.tick ||
      constraints.begin_tick < 0 || constraints.end_tick <= constraints.begin_tick ||
      constraints.end_tick <= frame.tick ||
      (event_ && constraints.source_tick < event_->source_tick))
    return false;
  for (auto p : constraints.dangerous_cells) {
    if (!inside(p, frame.map))
      return false;
  }
  for (const auto& effect : constraints.events) {
    if (effect.begin_tick < 0 || effect.end_tick <= effect.begin_tick)
      return false;
    for (auto p : effect.cells)
      if (!inside(p, frame.map))
        return false;
    for (auto p : effect.points)
      if (!inside(p, frame.map))
        return false;
  }
  if (constraints.timeline)
    events_.merge(constraints, frame, memory_);
  event_ = constraints;
  return true;
}

Action Stage2Strategy::decide(const PublicPerception& frame, std::int64_t remaining_ticks,
                              const Stage2EventConstraints* incoming_event) {
  auto active_weights=temporal_weights_at(weights_,frame.tick);
  // 保存参数组可能将普通碰撞避让设为0；补任务期间仍对真实阻挡回执做短暂改道。
  if(active_weights.reach_finish_ticks>0 && remaining_ticks<=active_weights.reach_finish_ticks &&
      std::any_of(frame.missions.begin(),frame.missions.end(),[&](const auto& mission) {
        return mission.type=="reach" && std::find(frame.score.mission_completed_ids.begin(),
            frame.score.mission_completed_ids.end(),mission.id)==frame.score.mission_completed_ids.end();
      }))
    active_weights.blocked_retry_ticks=std::max(active_weights.blocked_retry_ticks,
        active_weights.reach_collision_retry_ticks);
  if (memory_.current() && frame.session_id == memory_.current()->session_id &&
      frame.tick == memory_.current()->tick)
    return remaining_ticks > 0 ? cached_ : Action::wait();
  const auto old = memory_.current();
  if (!memory_.update(frame))
    return Action::wait();
  if(old&&old->session_id==frame.session_id&&old->tick+1==frame.tick&&
      old->self.alive&&frame.self.alive&&same(old->self.position,frame.self.position)&&
      frame.self.hp>=old->self.hp&&physical_cached_.type=="defend")++stationary_defense_ticks_;
  else stationary_defense_ticks_=0;
  // 新追击可以晚于采集开局才启用。尚可能存在的资源即使暂未连通，也保守保护。
  const bool purpose_resource_hint=std::any_of(memory_.entities().begin(),memory_.entities().end(),
      [](const auto& item){const auto& m=item.second;const auto& e=m.entity;
        return !m.first_missing_tick&&(e.type=="resource_normal"||e.type=="resource_high")&&
            e.value&&*e.value>0;});
  active_weights.purpose.enabled=purpose_policy_active(weights_.purpose,frame.tick,purpose_resource_hint);
  if(weights_.purpose.enabled&&!active_weights.purpose.enabled)unproductive_pursuit_.clear();
  if (!old || old->session_id != frame.session_id) {
    battle_evidence_.reset();
    enemy_beliefs_.reset();
    navigation_progress_.reset();
    events_.reset();
    physical_cached_ = Action::wait();
    joint_planner_.reset();
    race_coverage_planner_.reset();
    previous_.reset();
    intent_previous_.reset();
    event_.reset();
    normal_collected_ = high_collected_ = 0;
    phase_last_resource_tick_ = 0;
    acquisition_last_activity_tick_ = 0;
    reach_finish_id_.clear();
    reach_finish_position_.reset();
    reach_previous_distance_ = -1;
    reach_last_progress_tick_ = -1;
    reach_defensive_stalemate_ = 0;
    previous_goal_.clear();
    pursuit_target_.clear();
    pursuit_positions_.clear();
    pursuit_targets_.clear();
    pursuit_cooldown_.clear();
    unproductive_pursuit_.clear();
    last_observed_tick_.assign(memory_.cells().size(), -1);
    blocked_retry_until_.clear();
  }
  // 仅接受连续帧中、与本程序上一条MOVE对应的真实E_BLOCKED回执；复活/漏帧不猜冲突。
  for (auto it = blocked_retry_until_.begin(); it != blocked_retry_until_.end();) {
    if (it->second <= frame.tick)
      it = blocked_retry_until_.erase(it);
    else
      ++it;
  }
  bool received_collision = false;
  if (active_weights.blocked_retry_ticks > 0 && old && old->session_id == frame.session_id &&
      frame.tick == old->tick + 1 && old->self.alive && frame.self.alive &&
      frame.self.position_reason != "respawn" && same(old->self.position, frame.self.position) &&
      cached_.type == "move" && frame.self.last_action &&
      frame.self.last_action->tick == old->tick && frame.self.last_action->action == "move" &&
      !frame.self.last_action->moved && frame.self.last_action->error == "E_BLOCKED") {
    for (std::size_t i = 0; i < kDirections.size(); ++i) {
      if (physical_cached_.direction == kDirections[i]) {
        const Position failed{old->self.position.x + kSteps[i].x,
                              old->self.position.y + kSteps[i].y};
        if (inside(failed, frame.map)) {
          received_collision = true;
          blocked_retry_streak_ = blocked_retry_target_ && same(*blocked_retry_target_, failed)
                                      ? std::min(blocked_retry_streak_ + 1,
                                                 active_weights.blocked_retry_min_failures)
                                      : 1;
          blocked_retry_target_ = failed;
          // 短暂的同时抢格通常下一两Tick就能通过；只对持续相同的真实失败暂时绕行。
          if (blocked_retry_streak_ >= active_weights.blocked_retry_min_failures)
            blocked_retry_until_[{failed.x, failed.y}] =
                frame.tick + collision_retry_delay(frame.self.team, active_weights.blocked_retry_ticks);
        }
        break;
      }
    }
  }
  if (!received_collision) {
    blocked_retry_target_.reset();
    blocked_retry_streak_ = 0;
  }
  if (old && old->event_text != frame.event_text && event_ && !event_->timeline &&
      frame.event_text.compare(0, old->event_text.size(), old->event_text) != 0)
    event_.reset();  // 公告改变后不让旧结构解释因相同文本再次出现而复活。
  // 只在连续两帧、上一帧资源仍可见、本人实际到达且资源分数相应增长时计数。
  // 这是一份保守下界：漏帧或中途接入时不凭总分拆猜普通/高价值数量。
  if (previous_ && frame.tick == previous_->tick + 1 && previous_->self.alive && frame.self.alive) {
    const double delta = frame.score.resource_value - previous_->score.resource_value;
    for (const auto& entity : previous_->visible_entities) {
      if ((entity.type == "resource_normal" || entity.type == "resource_high") &&
          same(entity.position, frame.self.position) && entity.value &&
          std::abs(delta - *entity.value) < 1e-6) {
        if (entity.type == "resource_high")
          ++high_collected_;
        else
          ++normal_collected_;
        break;
      }
    }
  }
  // 追击必须兑现伤害；即使一直跑到新格子，也不能无限跟在同一敌人身后。
  if(active_weights.purpose.enabled&&old&&old->session_id==frame.session_id&&
      frame.tick==old->tick+1&&old->self.alive&&frame.self.alive) {
    if(frame.score.effective_damage>old->score.effective_damage)unproductive_pursuit_.clear();
    else if(plan_.goal=="combat"&&cached_.type=="move") {
      auto& spent=unproductive_pursuit_[plan_.target_id];
      if(++spent>=active_weights.purpose.chase_progress_ticks) {
        pursuit_cooldown_[plan_.target_id]=frame.tick+active_weights.chase_cooldown_ticks;
        spent=0;
      }
    }
  } else if(active_weights.purpose.enabled)unproductive_pursuit_.clear();
  // 只识别真实连续帧中的局部循环；长途追击经过新格子、已有伤害进展或中途漏帧均不触发。
  // 暂停的是追击目标收益，当前射程内的合法攻击与防御候选仍照常参与比较。
  for (auto it = pursuit_cooldown_.begin(); it != pursuit_cooldown_.end();) {
    if (it->second <= frame.tick)
      it = pursuit_cooldown_.erase(it);
    else
      ++it;
  }
  if (active_weights.chase_stall_ticks > 0 && old && old->session_id == frame.session_id &&
      frame.tick == old->tick + 1 && old->self.alive && frame.self.alive &&
      plan_.goal == "combat" && cached_.type == "move" &&
      frame.score.effective_damage <= old->score.effective_damage) {
    if (!active_weights.chase_shared_progress && pursuit_target_ != plan_.target_id) {
      pursuit_target_ = plan_.target_id;
      pursuit_positions_.clear();
      pursuit_targets_.clear();
    }
    pursuit_positions_.push_back(frame.self.position);
    pursuit_targets_.push_back(plan_.target_id);
    while (pursuit_positions_.size() > static_cast<std::size_t>(active_weights.chase_stall_ticks)) {
      pursuit_positions_.pop_front();
      pursuit_targets_.pop_front();
    }
    if (pursuit_positions_.size() == static_cast<std::size_t>(active_weights.chase_stall_ticks)) {
      std::set<std::pair<int, int>> visited;
      for (const auto p : pursuit_positions_)
        visited.emplace(p.x, p.y);
      if (visited.size() <= 4) {
        for (const auto& target : pursuit_targets_)
          pursuit_cooldown_[target] = frame.tick + active_weights.chase_cooldown_ticks;
        pursuit_positions_.clear();
        pursuit_targets_.clear();
      }
    }
  } else {
    pursuit_positions_.clear();
    pursuit_targets_.clear();
    pursuit_target_.clear();
  }
  previous_ = frame;
  plan_ = Stage2Plan{};
  if(weights_.purpose.enabled)
    plan_.purpose["gate"]={{"active",active_weights.purpose.enabled},{"start_tick",weights_.purpose.start_tick},
        {"resource_guard",weights_.purpose.protect_resources&&purpose_resource_hint}};
  plan_.blocked_retry_streak = blocked_retry_streak_;
  for (const auto& [position, until] : blocked_retry_until_)
    plan_.blocked_retries.push_back(
        {{"position", {position.first, position.second}}, {"until_tick", until}});
  plan_.pursuit_cooldowns = pursuit_cooldown_.size();
  cached_ = Action::wait();
  events_.observe(frame, memory_);
  if (incoming_event)
    set_event_constraints(*incoming_event);  // 先接受当前感知，再校验异步结果，避免额外晚一Tick。
  const bool matching_event = event_ && event_->source_text == frame.event_text;
  plan_.event_state = events_.describe(frame);
  plan_.event_pending = !frame.event_text.empty() && !matching_event;
  if (remaining_ticks <= 0 || !frame.self.alive) {
    plan_.goal = frame.self.alive ? "finished" : "respawn_wait";
    return cached_;
  }

  const auto visible = enemies(frame);
  if(active_weights.combat_clue_weight>0||active_weights.opening_depletion_weight>0)
    battle_evidence_.update(frame,old,events_,physical_cached_,memory_,active_weights);
  plan_.battle_evidence=battle_evidence_.details;
  plan_.resource_origins=battle_evidence_.origins;
  const auto own_unrivaled = events_.buff(frame.self.team, "unrivaled", frame.tick);
  const auto purity = events_.active("purity", frame.tick);
  const auto multiplier = [&](const std::string& team) {
    const auto state = events_.buff(team, "pan", frame.tick);
    return state ? state->effect.damage_multiplier : 1.0;
  };
  const auto square_hit = [](Position a, Position b, int radius) {
    return std::max(std::abs(a.x - b.x), std::abs(a.y - b.y)) <= radius && !same(a, b);
  };
  const auto enemy_range = [&](Position from, Position to, const VisibleEntity& enemy) {
    const auto state = events_.buff(enemy.team.value_or(""), "unrivaled", frame.tick);
    return state ? square_hit(from, to, state->effect.attack_radius) : in_range(from, to);
  };
  const auto defend_probability = [&](const VisibleEntity& enemy) {
    return enemy.defending.value_or(false) ? active_weights.events.purity_defend_seen
                                           : active_weights.events.purity_defend_other;
  };
  // 只有可确认的Buff改变攻击模型；敌方未被观察的行为保持不确定，不凭历史位置射击。
  const auto estimated_damage = [&](const VisibleEntity& enemy) {
    if (enemy.spawn_protected.value_or(false))
      return 0.0;
    const double normal = own_unrivaled ? own_unrivaled->effect.attack_damage : 20.0;
    const double defended = own_unrivaled ? own_unrivaled->effect.defended_damage : 10.0;
    double damage = purity          ? normal * (1.0 - defend_probability(enemy))
                    : own_unrivaled ? (normal + defended) * 0.5
                                    : active_weights.attack_damage_estimate;
    damage *= multiplier(enemy.team.value_or(""));
    if (enemy.hp)
      damage = std::min(damage, *enemy.hp);
    const double min_hit = defended * multiplier(enemy.team.value_or(""));
    if (enemy.hp && *enemy.hp <= min_hit)
      damage += active_weights.kill_reward_estimate * (purity ? 1.0 - defend_probability(enemy) : 1.0);
    return damage;
  };
  const auto event_attack_gain = [&](const VisibleEntity& enemy) {
    if (!own_unrivaled && !purity && multiplier(enemy.team.value_or("")) == 1.0)
      return attack_gain(enemy, frame, active_weights);
    return damage_gain(frame, active_weights, estimated_damage(enemy));
  };
  const auto clear_enemy_hit = [&](Position at, const VisibleEntity* enemy,
                                   const std::vector<const VisibleEntity*>& robots) {
    if (enemy->spawn_protected.value_or(false) || !enemy_range(enemy->position, at, *enemy))
      return false;
    if (events_.buff(enemy->team.value_or(""), "unrivaled", frame.tick))
      return true;
    for (const auto* other : robots)
      if (other != enemy && !same(other->position, at) &&
          distance(enemy->position, other->position) < distance(enemy->position, at) &&
          distance(enemy->position, other->position) + distance(other->position, at) ==
              distance(enemy->position, at))
        return false;
    return true;
  };
  const auto event_threats = [&](Position at, const std::vector<const VisibleEntity*>& robots) {
    int count = 0;
    for (const auto* enemy : robots)
      if (clear_enemy_hit(at, enemy, robots))
        ++count;
    return count;
  };
  // 软危险系数只影响路线与后续暴露；本Tick致死上界仍用真实50/10等公告伤害。
  const auto threat_risk = [&](Position at, const std::vector<const VisibleEntity*>& robots) {
    double risk=0;
    for (const auto* enemy:robots)
      if (clear_enemy_hit(at,enemy,robots))
        risk += events_.buff(enemy->team.value_or(""),"unrivaled",frame.tick)
            ? active_weights.events.unrivaled_risk_multiplier : 1.0;
    return risk;
  };
  plan_.fortress={{"contests",Json::array()},{"buff_threats",Json::array()},
                   {"pending_threats",Json::array()}};
  for (const auto* enemy:visible)
    if (const auto buff=events_.buff(enemy->team.value_or(""),"unrivaled",frame.tick))
      plan_.fortress["buff_threats"].push_back({{"team",enemy->team.value_or("")},
          {"until_tick",buff->until},{"attack_radius",buff->effect.attack_radius},
          {"attack_damage",buff->effect.attack_damage},
          {"risk_multiplier",active_weights.events.unrivaled_risk_multiplier}});
  struct PendingFortressThreat {
    const Stage2EventEffect* event;
    Position position;
    int radius;
    std::int64_t ready_tick,until;
  };
  std::vector<PendingFortressThreat> pending_fortress;
  if(active_weights.events.fortress_contest_enabled && !own_unrivaled)
    for(const auto& [id,e]:events_.effects()) {
      (void)id;
      if(e.kind!="unrivaled"||!e.actor.empty()||e.end_tick<=frame.tick) continue;
      for(const auto* enemy:visible) {
        const auto team=enemy->team.value_or("");
        if(events_.buff(team,"unrivaled",frame.tick)||
            !Stage2Events::contains(e.cells,enemy->position)||
            events_.progress(e,team)<active_weights.events.fortress_min_enemy_stay) continue;
        const auto ready=std::max(frame.tick,e.begin_tick)+
            std::max(0,e.stay_ticks-events_.progress(e,team));
        if(ready>=e.end_tick) continue;
        pending_fortress.push_back({&e,enemy->position,e.attack_radius,ready,e.end_tick});
        plan_.fortress["pending_threats"].push_back({{"team",team},{"ready_tick",ready},
            {"until_tick",e.end_tick},{"observed_stay_ticks",events_.progress(e,team)}});
      }
    }
  // 预测火力只惩罚未来位置，不改已授予Buff或本Tick真实伤害；离开后下一帧即重算。
  const auto pending_fortress_risk=[&](Position at,std::int64_t arrival) {
    double risk=0;
    for(const auto& threat:pending_fortress) {
      const bool lost_entry=Stage2Events::contains(threat.event->cells,at) &&
          fortress_contest(events_,*threat.event,frame,at,
              static_cast<int>(std::max<std::int64_t>(0,arrival-frame.tick)),active_weights.events).losing;
      if((lost_entry || arrival+active_weights.events.fortress_preempt_ticks>=threat.ready_tick) &&
          arrival<threat.until && square_hit(at,threat.position,threat.radius))
        risk+=active_weights.events.unrivaled_risk_multiplier;
    }
    return risk;
  };
  const auto current_damage_bound = [&](bool defend) {
    if (frame.self.spawn_protected)
      return 0.0;
    double total = 0;
    for (const auto* enemy : visible)
      if (clear_enemy_hit(frame.self.position, enemy, visible)) {
        const auto buff = events_.buff(enemy->team.value_or(""), "unrivaled", frame.tick);
        total += buff ? (defend ? buff->effect.defended_damage : buff->effect.attack_damage)
                      : (defend ? 10.0 : 20.0);
      }
    return total * multiplier(frame.self.team);
  };
  const auto motion = estimate_opponent_motion(frame, old, visible, memory_);
  plan_.opponent_motion = motion.details;
  std::vector<const VisibleEntity*> projected;
  for (const auto& enemy : motion.projected)
    projected.push_back(&enemy);
  const bool use_intents = active_weights.intent_chase_weight > 0 || active_weights.intent_risk_weight > 0 ||
                           active_weights.third_party_risk_discount > 0 ||
                           active_weights.third_party_attack_bonus > 0 || active_weights.purpose.enabled;
  OpponentIntentModel intents;
  std::vector<VisibleEntity> intent_bodies;
  if (use_intents) {
    intents = assess_opponent_intents(frame, old, intent_previous_, memory_, visible,
                                      active_weights.intent_memory_ticks, active_weights.intent_heading_weight);
    if(active_weights.combat_clue_weight>0) for(std::size_t i=0;i<intents.opponents.size();++i) {
      auto& intent=intents.opponents[i];
      intent.third_party_confidence=std::max(intent.third_party_confidence,
          std::min(.6,active_weights.combat_clue_weight*battle_evidence_.engagement(
              BattleEvidence::identity(intent.entity),frame.tick,active_weights.third_party_memory_ticks)));
      if(i<intents.details.size()) intents.details[i]["third_party_confidence"]=intent.third_party_confidence;
    }
    plan_.opponent_intents = intents.details;
    for (const auto* enemy : visible)
      intent_bodies.push_back(*enemy);
  }
  intent_previous_ = old;
  const auto size = frame.map;
  const Position start = frame.self.position;
  const int count = size.width * size.height;
  InterceptForecast intercepts;
  if(active_weights.purpose.enabled&&active_weights.purpose.intercept_weight>0) {
    intercepts=forecast_intercepts(frame,old,intents,visible.size(),active_weights.purpose,
      [&](Position p){return inside(p,size)&&memory_.cells()[MapMemory::index(p,size)]!=100&&
          events_.terrain_at(p,frame.tick,memory_)!=100;},
      [&](Position p){return memory_.cells()[MapMemory::index(p,size)]==0;});
    plan_.purpose["intercepts"]=intercepts.details;
  }
  // 只记录本车真正看到过的区域；全图已开不意味着一直知道敌车当前在哪里。
  for (int y = std::max(0, start.y - frame.vision_radius);
       y <= std::min(size.height - 1, start.y + frame.vision_radius); ++y) {
    for (int x = std::max(0, start.x - frame.vision_radius);
         x <= std::min(size.width - 1, start.x + frame.vision_radius); ++x)
      last_observed_tick_[cell({x, y}, size)] = frame.tick;
  }
  std::vector<bool> blocked(static_cast<std::size_t>(count), false);
  planning::SearchOptions options;
  for (const auto& [position, until] : blocked_retry_until_) {
    (void)until;
    const Position p{position.first, position.second};
    if (!same(p, start)) {
      blocked[cell(p, size)] = true;
      options.blocked_cells.push_back({p.x, p.y});
    }
  }
  for (const auto* enemy : visible) {
    blocked[cell(enemy->position, size)] = true;
    options.blocked_cells.push_back({enemy->position.x, enemy->position.y});
  }
  // 已解释的危险格在规划窗口内保守避开；不把未知自然语言自动当作已理解。
  const bool avoid_event = event_ && !event_->timeline && event_->end_tick > frame.tick &&
                           event_->begin_tick - frame.tick < remaining_ticks;
  const bool inside_danger =
      avoid_event && std::any_of(event_->dangerous_cells.begin(), event_->dangerous_cells.end(),
                                 [&](Position p) { return same(p, start); });
  // 已经身在危险区时必须允许沿区内格子走向出口，不能把自己的撤离路线全部封死。
  if (avoid_event && !inside_danger) {
    for (auto p : event_->dangerous_cells) {
      if (!same(p, start)) {
        blocked[cell(p, size)] = true;
        options.blocked_cells.push_back({p.x, p.y});
      }
    }
  }
  /// 当前地图下的实际可通行条件；未知格先观察，不直接向未知障碍迈步。
  const auto passable = [&](Position p) {
    return inside(p, size) && !blocked[cell(p, size)] &&
           memory_.cells()[MapMemory::index(p, size)] == 0;
  };
  std::vector<int> dist(static_cast<std::size_t>(count), -1);
  std::vector<Position> parent(static_cast<std::size_t>(count), start);
  std::queue<Position> pending;
  dist[cell(start, size)] = 0;
  pending.push(start);
  while (!pending.empty()) {
    const auto p = pending.front();
    pending.pop();
    for (auto step : kSteps) {
      Position next{p.x + step.x, p.y + step.y};
      if (!passable(next) || dist[cell(next, size)] >= 0)
        continue;
      dist[cell(next, size)] = dist[cell(p, size)] + 1;
      parent[cell(next, size)] = p;
      pending.push(next);
    }
  }

  // 等步长四邻接网格上，BFS距离与A*最短路长度相同。每名可见敌车只算一次，
  // 复用于全部资源；仅用已知空地/障碍，未知区未连通不推断无路，不读取敌方私有视野。
  // 其他车辆的占位和事件绕行不是永久地形，估计敌方路程时不据此封路。
  std::vector<int> nearest_enemy(static_cast<std::size_t>(count), -1);
  if (active_weights.astar_resource_competition || active_weights.resource_race.enabled) {
    for (const auto* enemy : visible) {
      std::vector<int> enemy_dist(static_cast<std::size_t>(count), -1);
      std::queue<Position> queue;
      enemy_dist[cell(enemy->position, size)] = 0;
      queue.push(enemy->position);
      while (!queue.empty()) {
        const auto at = queue.front();
        queue.pop();
        for (const auto step : kSteps) {
          const Position next{at.x + step.x, at.y + step.y};
          if (!inside(next, size) || enemy_dist[cell(next, size)] >= 0 ||
              memory_.cells()[MapMemory::index(next, size)] != 0)
            continue;
          enemy_dist[cell(next, size)] = enemy_dist[cell(at, size)] + 1;
          queue.push(next);
        }
      }
      for (int i = 0; i < count; ++i)
        if (enemy_dist[i] >= 0 && (nearest_enemy[i] < 0 || enemy_dist[i] < nearest_enemy[i]))
          nearest_enemy[i] = enemy_dist[i];
    }
  }

  // 时间只影响决策偏好，不改变裁判实际计分。越早越重视一次性资源，随后平滑恢复。
  // 仅依据公开记忆判断资源是否可能仍存在；尚有未知区域时不能宣称资源已经耗尽。
  const bool unknown_map =
      std::find(memory_.cells().begin(), memory_.cells().end(), -1) != memory_.cells().end();
  const bool known_resource =
      std::any_of(memory_.entities().begin(), memory_.entities().end(), [&](const auto& item) {
        const auto& remembered = item.second;
        const auto& e = remembered.entity;
        return (e.type == "resource_normal" || e.type == "resource_high") &&
               !remembered.first_missing_tick && e.value && *e.value > 0 &&
               inside(e.position, size) && dist[cell(e.position, size)] > 0;
      });
  // 用户选择开局不花步数攻击或防御：有资源/未知区时只移动，资源已耗尽则可提前转战。
  const bool opening_window=frame.tick<active_weights.opening_collection_ticks&&
      frame.score.resource_value<active_weights.resource_cap&&(unknown_map||known_resource);
  const bool opening_has_move=std::any_of(kSteps.begin(),kSteps.end(),[&](Position step){
    return passable({start.x+step.x,start.y+step.y});});
  const bool opening_collect_only=opening_window&&opening_has_move;
  plan_.opening_collection=opening_collect_only;
  plan_.opening_blocked_battle=opening_window&&!opening_has_move;
  // 直接交战且无已知资源、或已进入明确追击时，当前战斗模型不受采集移植影响。
  const bool transfer_eligible=(unknown_map||known_resource)&&(visible.empty()||known_resource)&&
      (!active_weights.purpose.enabled||known_resource)&&
      (!active_weights.battle_curve.enabled||frame.tick<=active_weights.battle_curve.start_tick||known_resource);
  const double transfer_alpha=collection_transfer_alpha(active_weights.collection_transfer,
      frame.tick,transfer_eligible);
  const double transfer_offset=active_weights.travel_offset+
      (active_weights.collection_transfer.travel_offset-active_weights.travel_offset)*transfer_alpha;
  const double transfer_resource_floor=1+(active_weights.collection_transfer.resource_floor-1)*transfer_alpha;
  active_weights=collection_transfer_weights_at(active_weights,transfer_alpha);
  if(active_weights.collection_transfer.enabled)
    plan_.collection_transfer={{"alpha",transfer_alpha},{"eligible",transfer_eligible},
        {"resource_weight",active_weights.acquisition.resource_weight},
        {"explore_peak",active_weights.acquisition.explore_peak},
        {"travel_offset",transfer_offset},{"resource_floor",transfer_resource_floor},
        {"selected",false}};
  const double battle_alpha=battle_curve_progress(active_weights.battle_curve,frame.tick,
      frame.score.effective_damage,active_weights.damage_cap,known_resource);
  active_weights=battle_curve_weights_at(active_weights,frame.tick,frame.score.effective_damage,known_resource);
  if(active_weights.battle_curve.enabled)
    plan_.battle_curve={{"alpha",battle_alpha},{"resource_guard",known_resource&&active_weights.battle_curve.protect_resources},
        {"hp_loss_cost",active_weights.hp_loss_cost},{"future_exposure_cost",active_weights.future_exposure_cost},
        {"route_risk",active_weights.route_risk},{"patrol_value",active_weights.patrol_value},
        {"recent_enemy_search_weight",active_weights.recent_enemy_search_weight}};
  bool visible_resource = false;
  if (active_weights.acquisition.enabled) {
    for (const auto& [id,item] : memory_.entities()) {
      (void)id;
      const auto& e=item.entity;
      if ((e.type=="resource_normal" || e.type=="resource_high") && !item.first_missing_tick &&
          item.last_seen_tick==frame.tick && e.value && *e.value>0 && inside(e.position,size) &&
          dist[cell(e.position,size)]>0) { visible_resource=true; break; }
    }
    if (visible_resource || (old && frame.score.resource_value>old->score.resource_value))
      acquisition_last_activity_tick_=frame.tick;
  }
  const auto acquisition=acquisition_preference(active_weights.acquisition,frame.tick,
      acquisition_last_activity_tick_,visible_resource);
  plan_.acquisition_exploration_weight=acquisition.exploration;
  plan_.acquisition_silence_ticks=acquisition.silence_ticks;
  if (active_weights.temporal.enabled)
    plan_.temporal_weights={{"resource_age_ticks",active_weights.resource_age_ticks},
        {"competition_weight",active_weights.competition_weight},
        {"resource_lookahead_weight",active_weights.resource_lookahead_weight},
        {"route_information_weight",active_weights.route_information_weight},
        {"third_party_attack_bonus",active_weights.third_party_attack_bonus},
        {"third_party_risk_discount",active_weights.third_party_risk_discount}};
  const double opening =
      active_weights.opening_ticks > 0 && (unknown_map || known_resource)
          ? std::clamp(1.0 - static_cast<double>(frame.tick) / active_weights.opening_ticks, 0.0, 1.0)
          : 0.0;
  // 当前攻击先于移动结算：可能被当场击毁时，撤回额外的抢资源加成，保留原有防御判断。
  const bool lethal_exposure =
      !frame.self.spawn_protected && frame.self.hp <= current_damage_bound(false);
  plan_.economy = assess_resource_economy(memory_, frame, active_weights);
  if (active_weights.enemy_belief_tracking)
    enemy_beliefs_.update(frame,memory_,active_weights.recent_enemy_half_life,active_weights.recent_enemy_max_age);
  navigation_progress_.observe(frame,static_cast<int>(std::count_if(last_observed_tick_.begin(),
      last_observed_tick_.end(),[](auto tick){return tick>=0;})),active_weights);
  plan_.navigation_cycle_active=navigation_progress_.active();
  plan_.navigation_cycle_breaks=navigation_progress_.breaks();
  // 采集欲望可独立降到小于1的终值；只调决策偏好，真实资源/任务分数不变。
  // 低血致命暴露时撤销阶段偏好，仍交给统一生存代价比较。
  const double desired_resource_weight = active_weights.acquisition.enabled ? acquisition.resource :
      active_weights.resource_weight_end +
      (active_weights.opening_resource_weight - active_weights.resource_weight_end) * opening;
  const double resource_weight = active_weights.acquisition.enabled ? desired_resource_weight :
      1.0 + (desired_resource_weight - 1.0) *
                                           (lethal_exposure ? 0.0 : 1.0) *
                                           plan_.economy.collection_factor;
  double combat_weight = collection_linear(active_weights.opening_combat_weight,
      active_weights.combat_weight_end, frame.tick, active_weights.combat_ramp_ticks);
  const double resource_mission_weight = resource_mission_weight_at(active_weights, frame.tick);
  const double reach_weight = reach_priority_at(active_weights, frame.tick, remaining_ticks, -1,
                                               unknown_map || known_resource);
  // 保分的提前量由已知安全区的最短路、伤害缺口、血量和缓冲共同决定。
  // 安全只是相对当前可见敌人的一步活动范围，不宣称视野外没有敌人。
  std::optional<Position> retreat_target;
  int escape_steps = std::numeric_limits<int>::max();
  if (active_weights.endgame_preserve_weight > 0 && !visible.empty() &&
      frame.score.effective_damage >= active_weights.damage_cap * active_weights.retreat_damage_fraction) {
    const auto safe = [&](Position p) {
      for (std::size_t i = 0; i < visible.size(); ++i)
        if (distance(p, visible[i]->position) < 4 || distance(p, projected[i]->position) < 4)
          return false;
      return true;
    };
    if (safe(start))
      escape_steps = 0;
    for (int y = 0; y < size.height; ++y) {
      for (int x = 0; x < size.width; ++x) {
        const Position at{x, y};
        const int steps = dist[cell(at, size)];
        if (steps > 0 && steps < escape_steps && safe(at)) {
          escape_steps = steps;
          retreat_target = at;
        }
      }
    }
    if (escape_steps != std::numeric_limits<int>::max()) {
      plan_.escape_steps = escape_steps;
      const int gap_ticks = static_cast<int>(
          std::ceil(std::max(0.0, active_weights.damage_cap - frame.score.effective_damage) /
                    active_weights.attack_damage_estimate));
      const int hp_buffer =
          static_cast<int>(std::ceil(std::max(0.0, 100.0 - frame.self.hp) / 25.0));
      plan_.retreat_lead_ticks =
          std::min(active_weights.retreat_window_ticks,
                   escape_steps + active_weights.retreat_buffer_ticks + gap_ticks + hp_buffer);
      plan_.preserving_score = remaining_ticks <= plan_.retreat_lead_ticks;
      if (plan_.preserving_score)
        combat_weight /= 1.0 + active_weights.endgame_preserve_weight;
    }
  }
  plan_.opening_fraction = opening;
  plan_.resource_weight = resource_weight;
  plan_.combat_weight = combat_weight;
  plan_.reach_weight = reach_weight;

  // 对预告区域计算撤离距离，首步进入后仍须来得及在结算开始前离开。
  std::map<std::string,EventEscapeField> escape_fields;
  if(active_weights.events.escape_weight>0)
    for(const auto& [id,e]:events_.effects())
      if(e.actor.empty()&&(e.kind=="explosion"||e.kind=="world_split")&&e.begin_tick>frame.tick)
        escape_fields.emplace(id,EventEscapeField(size,e.cells,passable));
  // 事件软代价按预计到达Tick查询；辣椒为有限代价，只有临时墙进入硬阻挡层。
  const auto event_penalty = [&](Position at, std::int64_t arrival, bool entering = true) {
    double penalty = 0;
    for (const auto& [id, e] : events_.effects()) {
      (void)id;
      if (!e.actor.empty())
        continue;
      if ((e.kind == "explosion" || e.kind == "world_split") &&
          Stage2Events::contains(e.cells, at) && arrival < e.end_tick &&
          arrival + active_weights.events.evacuation_buffer >= e.begin_tick) {
        const double scale = e.kind == "world_split" ? 1.0 : std::max(0.1, e.damage / 100.0);
        penalty += active_weights.events.blast_risk * scale /
                   (1 + std::max<std::int64_t>(0, e.begin_tick - arrival));
      }
      const auto escape=escape_fields.find(id);
      if(escape!=escape_fields.end()&&Stage2Events::contains(e.cells,at)&&arrival<e.begin_tick){
        const int steps=escape->second.distance(at);
        const auto available=e.begin_tick-arrival;
        if(steps<0||steps+active_weights.events.evacuation_buffer>available)
          penalty+=active_weights.events.escape_weight*active_weights.events.blast_risk;
      }
      if (e.kind == "pepper" && e.begin_tick <= arrival && arrival < e.end_tick) {
        if (entering && Stage2Events::contains(e.points, at))
          penalty += active_weights.events.pepper_cost;
        if (!frame.self.spawn_protected)
          for (const auto point : e.points) {
            if (same(point, at) || (point.x != at.x && point.y != at.y))
              continue;
            for (const auto* enemy : visible)
              if (distance(enemy->position, point) == 1)
                penalty += e.damage * active_weights.hp_loss_cost * 0.25;
          }
      }
    }
    return penalty;
  };
  std::vector<double> event_route_cost(static_cast<std::size_t>(count), 0.0);
  bool weighted_event_route = false;
  for (int y = 0; y < size.height; ++y)
    for (int x = 0; x < size.width; ++x) {
      const Position p{x, y};
      const int steps = dist[cell(p, size)];
      if (steps >= 0) {
        event_route_cost[cell(p, size)] = event_penalty(p, frame.tick + steps);
        weighted_event_route = weighted_event_route || event_route_cost[cell(p, size)] > 0;
      }
    }
  const double combat_need =
      std::clamp(1.0 - frame.score.effective_damage / active_weights.damage_cap, 0.0, 1.0);
  const auto fortress_reward = [&](const Stage2EventEffect& e, Position at) {
    if (e.kind != "unrivaled" || !e.actor.empty() || e.end_tick <= frame.tick || own_unrivaled)
      return 0.0;
    const int travel = inside(at, size) ? dist[cell(at, size)] : -1;
    const int needed = std::max(1, e.stay_ticks - events_.progress(e, frame.self.team));
    if (travel < 0 ||
        std::max<std::int64_t>(e.begin_tick, frame.tick + travel) + needed >= e.end_tick)
      return 0.0;
    if (fortress_contest(events_,e,frame,at,travel,active_weights.events).losing) return 0.0;
    return active_weights.events.fortress_value * (0.15 + 0.85 * combat_need);
  };
  const auto hold_reward = [&](Position destination) {
    double gain = 0;
    for (const auto& [id, e] : events_.effects()) {
      (void)id;
      if (e.kind == "unrivaled" && frame.tick < e.end_tick &&
          Stage2Events::contains(e.cells, start) && Stage2Events::contains(e.cells, destination)) {
        const int needed = std::max(1, e.stay_ticks - events_.progress(e, frame.self.team));
        gain = std::max(gain, active_weights.events.fortress_hold * fortress_reward(e, start) /
                                  (needed + active_weights.travel_offset));
      }
    }
    return gain;
  };
  std::vector<const MissionInfo*> unfinished;
  for (const auto& mission : frame.missions) {
    if (std::find(frame.score.mission_completed_ids.begin(),
                  frame.score.mission_completed_ids.end(),
                  mission.id) != frame.score.mission_completed_ids.end())
      continue;
    if (mission.type != "reach" && mission.type != "normal_resources" &&
        mission.type != "high_resources" && mission.type != "resource_value" &&
        mission.type != "effective_damage") {
      ++plan_.unsupported_missions;
      continue;
    }
    unfinished.push_back(&mission);
  }
  // 已开图仍会失去对敌人的实时了解。旧区域只在伤害/任务仍有收益时奖励重新观察，
  // 达到计分上限后不为了陈旧度无休止巡游；不把老资源位置当成会自动刷新的资源。
  double search_gain =
      marginal(frame.score.effective_damage, 20.0, active_weights.damage_cap, active_weights.damage_points);
  for (const auto* mission : unfinished) {
    if (mission->type == "effective_damage" && mission->target &&
        *mission->target > frame.score.effective_damage)
      search_gain += active_weights.mission_points * active_weights.mission_potential *
                     std::min(1.0, 20.0 / (*mission->target - frame.score.effective_damage));
  }
  // 观测时效可延后至采集窗口之后，再平滑增大，避免刚开局就返回旧区域找人。
  const double observation_phase =
      frame.tick < active_weights.observation_start_tick ? 0.0
      : active_weights.observation_ramp_ticks > 0
          ? std::clamp(static_cast<double>(frame.tick - active_weights.observation_start_tick) /
                           active_weights.observation_ramp_ticks,
                       0.0, 1.0)
          : 1.0;
  std::optional<ObservationAgeField> age_field;
  if (active_weights.observation_age_weight > 0 && observation_phase > 0 && search_gain > 0)
    age_field.emplace(size, memory_.cells(), last_observed_tick_, frame.tick,
                      active_weights.observation_fresh_ticks, active_weights.observation_stale_ticks);
  const FrontierOpportunityValue frontier_value(frame, memory_, active_weights, normal_collected_,
                                                high_collected_);
  std::vector<double> unknown_supply(static_cast<std::size_t>(count),1.);
  if(active_weights.opening_depletion_weight>0) for(int y=0;y<size.height;++y) for(int x=0;x<size.width;++x)
    unknown_supply[cell({x,y},size)]=battle_evidence_.supply_factor({x,y},frame.tick,active_weights);
  plan_.frontier_value = frontier_value.describe();
  if(active_weights.planning.enabled&&active_weights.planning.regional_exploration) {
    const auto regional=regional_collection_supply(frame,memory_,active_weights.planning.regional_strength,active_weights.initial_resource_value);
    for(int i=0;i<count;++i)unknown_supply[i]*=regional.factors[i];
    plan_.frontier_value["regional_supply"]=regional.regions;
  }
  /// 计算到目标后能新观察的未知格奖励；越靠近地图边界权重略高，奖励有上限。
  const auto information = [&](Position p) {
    double cells = 0;
    double expected_unknown_cells = 0;
    for (int y = std::max(0, p.y - frame.vision_radius);
         y <= std::min(size.height - 1, p.y + frame.vision_radius); ++y) {
      for (int x = std::max(0, p.x - frame.vision_radius);
           x <= std::min(size.width - 1, p.x + frame.vision_radius); ++x) {
        if (memory_.cells()[MapMemory::index({x, y}, size)] != -1)
          continue;
        expected_unknown_cells+=unknown_supply[cell({x,y},size)];
        const int edge = std::min({x, y, size.width - 1 - x, size.height - 1 - y});
        cells += 1.0 + active_weights.edge_information_bonus / (1.0 + edge);
      }
    }
    return exploration_value(
               cells * active_weights.information_per_cell + frontier_value.extra(expected_unknown_cells),
               active_weights.information_cap, active_weights.information_softness) *
           plan_.economy.exploration_factor * acquisition.exploration;
  };
  std::optional<Candidate> best;
  // 没有可见可达资源持续一段时间后，允许提前转战；发现新资源时仍能重新评估采集。
  // 沉默只是本队的机会信号，不宣称未知地图上已经没有资源，原探索候选继续保留。
  if (active_weights.phase.enabled||active_weights.purpose.enabled)
    for (const auto& entity : frame.visible_entities)
      if ((entity.type == "resource_normal" || entity.type == "resource_high") &&
          entity.value && *entity.value > 0 && inside(entity.position, size) &&
          dist[cell(entity.position, size)] > 0)
        phase_last_resource_tick_ = frame.tick;
  plan_.phase_resource_age = std::max<std::int64_t>(0, frame.tick - phase_last_resource_tick_);
  const bool collection_opportunity = (unknown_map || known_resource) &&
      (active_weights.phase.resource_silence_ticks == 0 ||
       plan_.phase_resource_age <= active_weights.phase.resource_silence_ticks);
  const auto phase = stage2_phase(active_weights.phase, frame.tick,
      frame.score.resource_value < active_weights.resource_cap ||
        std::any_of(unfinished.begin(), unfinished.end(), [](const auto* m) {
          return m->type == "normal_resources" || m->type == "high_resources" ||
                 m->type == "resource_value";
        }), collection_opportunity, search_gain > 0);
  plan_.phase = stage2_phase_name(phase);
  std::optional<Candidate> phase_best;
  std::optional<Candidate> phase_resource;
  std::optional<Candidate> deadline_mission;
  std::optional<Candidate> reach_finish;
  std::optional<Position> reach_final_target;
  int reach_finish_steps = -1;
  const bool reach_finish_window = active_weights.reach_finish_ticks > 0 &&
      remaining_ticks <= active_weights.reach_finish_ticks;
  std::optional<Candidate> purpose_resource;
  std::optional<Candidate> purpose_battle;
  // 单个任务距离场复用于全部交战候选：估计“先交战、再顺路到任务”的后续路程。
  // 只在刷伤害阶段启用，不把这笔私有任务预期叠加到开局资源欲望上。
  std::vector<std::vector<int>> task_distance_fields;
  if (phase == Stage2Phase::Fight && active_weights.phase.task_lookahead_weight > 0)
    for (const auto* m : unfinished)
      if (m->type == "reach" && m->position && inside(*m->position, size) &&
          dist[cell(*m->position, size)] >= 0)
        task_distance_fields.push_back(goal_distance_field(size, *m->position, passable));
  Stage2JointPlan joint_plan;
  ResourceSweepPlan sweep_plan;
  CollectionSearchPlan collection_search_plan;
  ResourceRacePlan resource_race_plan;
  Stage2JointPlan race_coverage_plan;
  std::optional<Candidate> resource_priority;
  std::optional<Candidate> transfer_resource;
  bool prefer_joint = false;
  /// 所有类型目标统一比较“可得分+信息价值 / 路程”，减去路径风险并保留轻微惯性。
  const auto consider = [&](Candidate candidate) {
    if (prefer_joint && !candidate.joint &&
        (candidate.kind == "resource" || candidate.kind == "explore"))
      return;
    if (!inside(candidate.position, size))
      return;
    const int steps = dist[cell(candidate.position, size)];
    if (steps <= 0 || steps > remaining_ticks)
      return;
    ++plan_.candidates;
    if (!candidate.sweep && !candidate.collection_search)
      candidate.information = information(candidate.position);
    if (age_field) {
      candidate.age_information =
          std::min(std::max(0.0, active_weights.information_cap - candidate.information),
                   active_weights.observation_age_weight * observation_phase * search_gain *
                       age_field->mean_in_view(candidate.position, frame.vision_radius) *
                       plan_.economy.exploration_factor);
      candidate.information += candidate.age_information;
    }
    if(active_weights.combat_clue_weight>0&&search_gain>0)
      candidate.patrol+=active_weights.combat_clue_weight*search_gain*
          battle_evidence_.in_view(candidate.position,frame.vision_radius);
    if (candidate.resource + candidate.mission + candidate.combat + candidate.information +
            candidate.patrol + candidate.event_reward <=
        0)
      return;  // 目标惯性不能凭空让已无收益的旧目标继续存在。
    double risk = event_penalty(candidate.position, frame.tick + steps);
    Position p = candidate.position;
    while (dist[cell(p, size)] > 2)
      p = parent[cell(p, size)];
    while (!same(p, start)) {
      risk += threat_risk(p, visible) * active_weights.route_risk;
      risk += pending_fortress_risk(p,frame.tick+dist[cell(p,size)]) * active_weights.route_risk;
      risk += active_weights.combat_clue_weight*battle_evidence_.exposure(p)*active_weights.route_risk;
      risk += event_penalty(p, frame.tick + dist[cell(p, size)]);
      p = parent[cell(p, size)];
    }
    for (const auto& [key, memory] : memory_.entities()) {
      (void)key;
      const auto age = frame.tick - memory.last_seen_tick;
      if (memory.entity.type == "robot" && (!memory.entity.hp||*memory.entity.hp>0) &&
          !memory.first_missing_tick && age > 0 &&
          age <= active_weights.enemy_memory_ticks &&
          distance(memory.entity.position, candidate.position) <= 2)
        risk += active_weights.remembered_risk / (1.0 + age) *
            (events_.buff(memory.entity.team.value_or(""),"unrivaled",frame.tick)
             ? active_weights.events.unrivaled_risk_multiplier : 1.0);
    }
    const double visible_weight =
        candidate.visible_resource ? active_weights.visible_resource_weight : 1.0;
    const double collection_weight =
        candidate.kind == "resource" ? resource_weight * visible_weight : 1.0;
    const double mission_weight =
        candidate.kind == "resource"
            ? resource_mission_weight * (active_weights.adaptive_resources
                                                      ? active_weights.collection_mission_weight
                                                      : resource_weight) * visible_weight
            : (candidate.reach_priority >= 0 ? candidate.reach_priority : reach_weight);
    // 新搜索已按整段到达时间扣除风险；这里不能重复扣一次目标/首两步风险。
    if(candidate.collection_search) risk=0;
    risk += (pending_fortress_risk(candidate.position,frame.tick+steps) +
        threat_risk(candidate.position,visible)-event_threats(candidate.position,visible)) *
        active_weights.future_exposure_cost;
    candidate.utility = (candidate.sweep||candidate.collection_search) ? candidate.planned_utility - risk :
                        (candidate.resource * collection_weight +
                         candidate.mission * mission_weight + candidate.combat * combat_weight +
                         candidate.information + candidate.patrol + candidate.event_reward) /
                            (steps + ((candidate.kind=="resource"||candidate.kind=="explore")
                                ? transfer_offset : active_weights.travel_offset)) -
                        risk + candidate.future_utility;
    if (candidate.kind + ":" + candidate.id == previous_goal_) {
      candidate.switch_bonus = active_weights.switch_margin;
      candidate.utility += candidate.switch_bonus;
    }
    if(transfer_alpha>0&&candidate.kind=="resource"&&candidate.resource+candidate.mission>0&&
        candidate.utility>0&&(!transfer_resource||candidate.utility>transfer_resource->utility))
      transfer_resource=candidate;
    if (candidate.kind == "combat" || candidate.kind == "search_enemy" || candidate.kind == "patrol")
      for (const auto& field : task_distance_fields) {
        const int after = field[cell(candidate.position, size)];
        if (after >= 0 && steps + after <= remaining_ticks)
          candidate.utility += active_weights.phase.task_lookahead_weight * active_weights.mission_points *
              active_weights.mission_potential / (steps + after + active_weights.travel_offset);
      }
    if (candidate.utility > 0 && stage2_phase_goal(phase, candidate.kind, steps,
                                                 active_weights.phase.chase_steps) &&
        (!phase_best || candidate.utility > phase_best->utility + 1e-9))
      phase_best = candidate;
    if (phase == Stage2Phase::Collect && candidate.kind == "resource" &&
        candidate.visible_resource && steps <= active_weights.phase.visible_resource_steps &&
        candidate.resource + candidate.mission > 0 && candidate.utility > 0 &&
        (!phase_resource || candidate.utility > phase_resource->utility + 1e-9))
      phase_resource = candidate;
    if (candidate.utility > 0 && candidate.kind == "mission" &&
        remaining_ticks <= steps + active_weights.reach_deadline_buffer &&
        (!deadline_mission || candidate.utility > deadline_mission->utility + 1e-9))
      deadline_mission = candidate;
    if(active_weights.purpose.enabled&&candidate.utility>0) {
      if(candidate.kind=="resource"&&candidate.resource+candidate.mission>0&&
          (!purpose_resource||candidate.utility>purpose_resource->utility+1e-9))
        purpose_resource=candidate;
      if((candidate.kind=="combat"||candidate.kind=="search_enemy"||candidate.kind=="patrol"||
          candidate.kind.rfind("event_",0)==0)&&
          (!purpose_battle||candidate.utility>purpose_battle->utility+1e-9))
        purpose_battle=candidate;
    }
    if (candidate.utility > 0 && (!best || candidate.utility > best->utility + 1e-9))
      best = candidate;
  };

  std::vector<std::pair<std::string, Candidate>> resource_candidates;
  std::vector<JointOpportunity> opportunities;
  std::vector<RaceResource> race_resources;
  for (const auto& [key, memory] : memory_.entities()) {
    const auto& entity = memory.entity;
    if ((entity.type != "resource_normal" && entity.type != "resource_high") ||
        memory.first_missing_tick || !entity.value)
      continue;
    Candidate c{"resource", entity.id.value_or(key), entity.position};
    c.visible_resource = memory.last_seen_tick == frame.tick;
    JointOpportunity opportunity;
    opportunity.key = key;
    opportunity.position = entity.position;
    opportunity.value = *entity.value;
    const double age = static_cast<double>(frame.tick - memory.last_seen_tick);
    double confidence = active_weights.planning.enabled
        && active_weights.planning.memory_model==1
        ? planning_resource_presence(static_cast<int>(age),nearest_enemy[cell(entity.position,size)],active_weights.planning)
        : 1.0 / (1.0 + age / active_weights.resource_age_ticks);
    if (active_weights.resource_race.enabled && inside(entity.position, size) &&
        dist[cell(entity.position, size)] > 0)
      race_resources.push_back({key, entity.position, *entity.value, confidence,
                                nearest_enemy[cell(entity.position, size)], {}});
    double race_factor = 1.0;
    if (inside(entity.position, size) && dist[cell(entity.position, size)] >= 0) {
      if (active_weights.astar_resource_competition) {
        race_factor = collection_race_factor(dist[cell(entity.position, size)],
                                             nearest_enemy[cell(entity.position, size)], frame.tick,
                                             active_weights);
        confidence *= race_factor;
      } else {
        for (const auto* enemy : visible) {
          if (distance(enemy->position, entity.position) <= dist[cell(entity.position, size)]) {
            confidence *= active_weights.contested_resource_factor;
            break;
          }
        }
      }
    }
    plan_.resource_races.push_back(
        {{"id", c.id},
         {"position", {entity.position.x, entity.position.y}},
         {"visible", c.visible_resource},
         {"age_ticks", frame.tick - memory.last_seen_tick},
         {"own_steps", dist[cell(entity.position, size)]},
         {"nearest_visible_enemy_steps", nearest_enemy[cell(entity.position, size)]},
         {"race_factor", race_factor},
         {"confidence", confidence}});
    c.resource = marginal(frame.score.resource_value, *entity.value, active_weights.resource_cap,
                          active_weights.resource_points) *
                 confidence;
    for (const auto* mission : unfinished) {
      if (!mission->target || *mission->target <= 0)
        continue;
      double left = 0, progress = 0;
      if (mission->type == "normal_resources" && entity.type == "resource_normal") {
        left = *mission->target - normal_collected_;
        progress = 1;
      } else if (mission->type == "high_resources" && entity.type == "resource_high") {
        left = *mission->target - high_collected_;
        progress = 1;
      } else if (mission->type == "resource_value") {
        left = *mission->target - frame.score.resource_value;
        progress = *entity.value;
      }
      if (left > 0) {
        const double gain = active_weights.mission_points * active_weights.mission_potential *
                            std::min(1.0, progress / left) * confidence;
        c.mission += gain;
        opportunity.mission_points[mission->id] = gain;
      }
    }
    if (active_weights.astar_resource_competition)
      opportunity.value *= confidence;  // 路线排序也承认争抢和记忆折扣，不能只折扣最终分数。
    const double visible_weight = c.visible_resource ? active_weights.visible_resource_weight : 1.0;
    opportunity.value *= visible_weight;
    resource_candidates.emplace_back(key, c);
    if (dist[cell(entity.position, size)] > 0 && c.resource + c.mission > 0) {
      opportunity.resource_points = c.resource;
      opportunity.direct_steps=dist[cell(entity.position,size)];
      opportunity.nearest_enemy_steps=nearest_enemy[cell(entity.position,size)];
      opportunity.preference=visible_weight;
      opportunity.priority =
          (c.resource + c.mission) * visible_weight /
          (dist[cell(entity.position, size)] + active_weights.travel_offset);
      opportunities.push_back(opportunity);
    }
  }
  const bool collection_task = std::any_of(unfinished.begin(), unfinished.end(), [](const auto* m) {
    return m->type == "normal_resources" || m->type == "high_resources" ||
           m->type == "resource_value";
  });
  if(active_weights.collection_search_weight>0&&frame.tick<active_weights.collection_search_ticks&&
     (unknown_map||frame.score.resource_value<active_weights.resource_cap||collection_task)) {
    const double mission_weight=resource_mission_weight*
        (active_weights.adaptive_resources?active_weights.collection_mission_weight:resource_weight);
    collection_search_plan=search_collection_route(frame,memory_,opportunities,
        static_cast<int>(remaining_ticks),active_weights,resource_weight,mission_weight,passable,
        [&](double cells,double edge_cells) {
          return exploration_value(edge_cells*active_weights.information_per_cell+frontier_value.extra(cells),
              active_weights.information_cap,active_weights.information_softness)*plan_.economy.exploration_factor*acquisition.exploration;
        },[&](Position at,int arrival) {
          const double future=std::exp2(-static_cast<double>(arrival-1)/6.0);
          return event_penalty(at,frame.tick+arrival)+
              future*(threat_risk(at,visible)+active_weights.combat_clue_weight*battle_evidence_.exposure(at))*active_weights.route_risk;
        },unknown_supply);
    const auto& search=collection_search_plan;
    plan_.collection_search={{"ran",search.ran},{"expanded",search.expanded},
        {"completed_depth",search.completed_depth},{"targets",search.targets},
        {"elapsed_ms",search.elapsed_ms},{"budget_hit",search.budget_hit},
        {"forecast_resource",search.forecast_resource},{"forecast_mission",search.forecast_mission}};
    if(search.route.success()&&search.route.path.size()>1) {
      const auto end=search.route.path.back();
      Candidate c{search.first_resource.empty()?"explore":"resource",
          "search:"+search.first_resource,{end.x,end.y}};
      c.collection_search=true;c.planned_utility=search.utility;
      c.resource=search.resource_points;c.mission=search.mission_points;
      c.information=search.information_points;consider(c);
    }
  }
  if (active_weights.sweep_weight>0 && frame.tick<active_weights.sweep_ticks &&
      (unknown_map || frame.score.resource_value<active_weights.resource_cap || collection_task)) {
    const double mission_weight=resource_mission_weight*
        (active_weights.adaptive_resources?active_weights.collection_mission_weight:resource_weight);
    sweep_plan=resource_sweep(frame,memory_,opportunities,static_cast<int>(remaining_ticks),active_weights,
        resource_weight,mission_weight,passable,[&](int cells,double edge_cells) {
          return exploration_value(edge_cells*active_weights.information_per_cell+frontier_value.extra(cells),
              active_weights.information_cap,active_weights.information_softness)*plan_.economy.exploration_factor*acquisition.exploration;
        });
    if(sweep_plan.path.size()>1) {
      const auto end=sweep_plan.path.back();
      Candidate c{sweep_plan.first_resource.empty()?"explore":"resource",
          "sweep:"+sweep_plan.first_resource,{end.x,end.y}};
      c.sweep=true;c.planned_utility=sweep_plan.utility;
      c.resource=sweep_plan.resource_points;c.mission=sweep_plan.mission_points;
      c.information=sweep_plan.information_points;
      consider(c);
    }
  }
  if (active_weights.collection_mode > 0 && frame.tick < active_weights.joint_ticks &&
      opportunities.size() >= static_cast<std::size_t>(active_weights.joint_min_resources) &&
      (frame.score.resource_value < active_weights.resource_cap || collection_task)) {
    plan_.joint_planned = true;
    joint_plan =
        joint_planner_.propose(frame, memory_, opportunities, options.blocked_cells,
                               remaining_ticks, collection_weights_at(active_weights, frame.tick));
    if (joint_plan.route.success() && joint_plan.route.path.size() > 1) {
      const auto end = joint_plan.route.path.back();
      Candidate c{joint_plan.resource_key.empty() ? "explore" : "resource",
                  "joint:" + joint_plan.resource_key,
                  {end.x, end.y}};
      c.resource = joint_plan.resource_points;
      c.mission = joint_plan.mission_points;
      c.joint = true;
      if (const auto found = memory_.entities().find(joint_plan.resource_key);
          found != memory_.entities().end())
        c.visible_resource = found->second.last_seen_tick == frame.tick;
      c.future_utility = joint_plan.future_utility;
      consider(c);
      prefer_joint = active_weights.collection_mode == 2 && best && best->joint;
    }
  }
  const auto future_resources = resource_lookahead(
      size, opportunities, dist,
      marginal(frame.score.resource_value, active_weights.resource_cap, active_weights.resource_cap,
               active_weights.resource_points),
      active_weights.mission_points * active_weights.mission_potential, resource_weight,
      resource_mission_weight *
          (active_weights.adaptive_resources ? active_weights.collection_mission_weight : resource_weight),
      remaining_ticks, active_weights, passable);
  for (auto& item : resource_candidates) {
    if (const auto future = future_resources.find(item.first); future != future_resources.end())
      item.second.future_utility = future->second;
    consider(item.second);
  }
  // 增益/开关是可达目标收益，不写进静态障碍；数值全部来自已校验模型事件。
  for (const auto& [id, e] : events_.effects()) {
    if (!e.actor.empty() || e.end_tick <= frame.tick)
      continue;
    std::vector<Position> goals;
    double reward = 0;
    if (e.kind == "angel") {
      const double deficit = std::max(0.0, e.hp_cap - frame.self.hp);
      const auto existing = events_.buff(frame.self.team, "angel", frame.tick);
      const double old_heal =
          existing ? std::min(deficit, existing->effect.heal_per_tick *
                                           static_cast<double>(existing->until - frame.tick))
                   : 0.0;
      reward = active_weights.events.heal_value *
               std::max(0.0, std::min(deficit, e.heal_per_tick * e.effect_ticks) - old_heal);
      if(active_weights.interference.modify_events&&active_weights.interference.weight>0&&frame.score.effective_damage>=active_weights.damage_cap)
        reward*=active_weights.interference.heal_multiplier;
      goals = e.points;
    } else if (e.kind == "unrivaled" && !own_unrivaled) {
      // 已在堡垒内时靠驻留收益维持捕获，不再为同一块区域虚构移动收益。
      if (Stage2Events::contains(e.cells, start))
        continue;
      goals = e.cells;
      reward = active_weights.events.fortress_value * (0.15 + 0.85 * combat_need);
    } else if (e.kind == "pepper")
      goals = e.points;
    for (const auto goal : goals) {
      double value = reward;
      if (e.kind == "pepper") {
        if (frame.tick < e.begin_tick)
          continue;
        for (const auto* enemy : visible)
          if (!enemy->spawn_protected.value_or(false) && !same(goal, enemy->position) &&
              (goal.x == enemy->position.x || goal.y == enemy->position.y))
            value += active_weights.events.pepper_value * std::min(e.damage, enemy->hp.value_or(100.0));
      }
      if (value <= 0)
        continue;
      const int travel = inside(goal, size) ? dist[cell(goal, size)] : -1;
      if (e.kind=="unrivaled") {
        const auto contest=fortress_contest(events_,e,frame,goal,
            travel>=0?travel:distance(start,goal),active_weights.events);
        if(contest.losing) {
          plan_.fortress["contests"].push_back({{"event_id",id},{"goal",{goal.x,goal.y}},
              {"reason","rival_power_advantage"},{"rival",contest.rival},
              {"own_ready_tick",contest.own_ready_tick},{"rival_ready_tick",contest.rival_ready_tick},
              {"confirmed_buff",contest.confirmed_buff}});
          continue;
        }
      }
      if(active_weights.interference.modify_events&&active_weights.interference.weight>0&&frame.score.effective_damage>=active_weights.damage_cap&&
          (e.kind=="pepper"||e.kind=="unrivaled")) {
        if(travel<0||travel>active_weights.interference.maximum_detour) continue;
        double incoming=current_damage_bound(false);
        // 用当前可见敌人的完整伤害估计短途机关路线，不为骚扰走进明显火力区。
        for(Position at=goal;!same(at,start)&&dist[cell(at,size)]>0;at=parent[cell(at,size)])
          for(const auto* enemy:visible) if(clear_enemy_hit(at,enemy,visible)) {
            const auto buff=events_.buff(enemy->team.value_or(""),"unrivaled",frame.tick);
            incoming+=(buff?buff->effect.attack_damage:20.0)*multiplier(frame.self.team);
          }
        if(!active_weights.interference.safe(frame.self.hp,incoming)) continue;
      }
      const int stay = e.kind == "unrivaled"
                           ? std::max(1, e.stay_ticks - events_.progress(e, frame.self.team))
                           : 0;
      if (travel >= 0 &&
          std::max<std::int64_t>(e.begin_tick, frame.tick + travel) + stay >= e.end_tick)
        continue;
      if (travel > 0) {
        Candidate c{"event_" + e.kind, id, goal};
        c.event_reward = value;
        if (stay > 0)
          c.event_reward *= static_cast<double>(travel + active_weights.travel_offset) /
                            (travel + stay + active_weights.travel_offset);
        consider(c);
      } else if (travel < 0 && e.kind != "pepper") {
        // 已公告的远方位置可以引导开图，但仍只能向本车已知空地移动。
        for (int y = 0; y < size.height; ++y)
          for (int x = 0; x < size.width; ++x) {
            Position at{x, y};
            const int d = dist[cell(at, size)];
            if (d <= 0 || distance(at, goal) >= distance(start, goal) ||
                frame.tick + d + distance(at, goal) + stay >= e.end_tick)
              continue;
            Candidate c{"event_explore", id, at};
            c.event_reward = 0.5 * value / (1 + distance(at, goal));
            consider(c);
          }
      } else if (travel == 0 && e.kind == "angel" && frame.tick >= e.begin_tick &&
                 !events_.buff(frame.self.team, "angel", frame.tick)) {
        for (const auto step : kSteps) {
          const Position at{start.x + step.x, start.y + step.y};
          if (!passable(at))
            continue;
          Candidate c{"event_angel_refresh", id, at};
          c.event_reward = value * 0.5;
          consider(c);
        }
      }
    }
  }
  for (const auto* mission : unfinished) {
    if (mission->type == "reach" && mission->position && inside(*mission->position, size)) {
      Candidate c{"mission", mission->id, *mission->position};
      c.mission = active_weights.mission_points * active_weights.mission_potential;
      const int steps = dist[cell(*mission->position, size)];
      c.reach_priority = reach_priority_at(active_weights, frame.tick, remaining_ticks,
          steps >= 0 ? steps : distance(start, *mission->position), unknown_map || known_resource);
      consider(c);
      // 末局补任务独立于资源/分期/追击开关；不靠把真实5分虚增成几十分快速抢权。
      if (reach_finish_window) {
        const auto approach = reach_finish_approach(memory_, start, *mission->position, dist, passable);
        if (approach && approach->estimated_steps > 0 &&
            (!reach_finish || approach->estimated_steps < reach_finish_steps ||
             (approach->estimated_steps == reach_finish_steps && mission->id == reach_finish_id_))) {
          Candidate finish{approach->exploring ? "mission_explore" : "mission",
                           mission->id, approach->waypoint};
          finish.mission = active_weights.mission_points * active_weights.mission_potential;
          finish.utility = std::max(active_weights.reach_move_priority,
              finish.mission * std::max(c.reach_priority, active_weights.reach_deadline_weight) /
                  (approach->estimated_steps + active_weights.travel_offset));
          reach_finish = finish;
          reach_finish_steps = approach->estimated_steps;
          reach_final_target = *mission->position;
        }
      }
    }
  }
  for (int y = 0; y < size.height; ++y) {
    for (int x = 0; x < size.width; ++x) {
      Position p{x, y};
      if (dist[cell(p, size)] <= 0)
        continue;
      Candidate explore{"explore", std::to_string(x) + "," + std::to_string(y), p};
      // 远方任务尚未连通时，以探索为主，顺带偏向能接近该公开任务坐标的位置。
      for (const auto* mission : unfinished) {
        if (mission->type == "reach" && mission->position && inside(*mission->position, size) &&
            dist[cell(*mission->position, size)] < 0 &&
            distance(p, *mission->position) < distance(start, *mission->position)) {
          explore.mission += active_weights.mission_points * active_weights.mission_potential /
                             (1.0 + distance(p, *mission->position));
          explore.reach_priority = std::max(explore.reach_priority,
              reach_priority_at(active_weights, frame.tick, remaining_ticks,
                  distance(start, *mission->position), unknown_map || known_resource));
        }
      }
      consider(explore);
      if (frame.self.spawn_protected || remaining_ticks <= dist[cell(p, size)])
        continue;
      for (const auto* enemy : visible) {
        const auto index = static_cast<std::size_t>(
            std::find(visible.begin(), visible.end(), enemy) - visible.begin());
        const bool current_range =
            own_unrivaled ? square_hit(p, enemy->position, own_unrivaled->effect.attack_radius)
                          : in_range(p, enemy->position);
        const auto predicted_position = projected[index]->position;
        const Position predicted_delta{(predicted_position.x > p.x) - (predicted_position.x < p.x),
                                       (predicted_position.y > p.y) - (predicted_position.y < p.y)};
        // 外推场景也遵守最近机器人挡弹，不能假设挡在中间的车凭空消失。
        const bool predicted_range =
            in_range(p, predicted_position) &&
            attack_target(p, predicted_delta, projected) == projected[index];
        const auto delta = Position{(enemy->position.x > p.x) - (enemy->position.x < p.x),
                                    (enemy->position.y > p.y) - (enemy->position.y < p.y)};
        const bool current_hit =
            current_range && (own_unrivaled || attack_target(p, delta, visible) == enemy);
        double opportunity = (1.0 - active_weights.opponent_motion_weight) * current_hit +
                             active_weights.opponent_motion_weight * predicted_range;
        if (active_weights.intent_chase_weight > 0 && !own_unrivaled) {
          const int arrival = dist[cell(p, size)];
          const double delay = std::max(0, arrival - 1);
          const double blend = active_weights.intent_chase_weight / (1.0 + delay * delay);
          opportunity = (1.0 - blend) * current_hit +
                        blend * intent_hit_probability(p, index, intents, intent_bodies);
        }
        const int arrival=dist[cell(p,size)];
        if(active_weights.purpose.enabled&&active_weights.purpose.intercept_weight>0&&
            arrival<=active_weights.purpose.intercept_horizon) {
          const double predicted=intercept_probability(p,index,arrival,intercepts,intent_bodies,
              own_unrivaled?own_unrivaled->effect.attack_radius:0);
          opportunity=(1.0-active_weights.purpose.intercept_weight)*opportunity+
              active_weights.purpose.intercept_weight*predicted;
        }
        if (opportunity <= 0)
          continue;
        const auto target_id = enemy->id.value_or(enemy->team.value_or("enemy"));
        if (pursuit_cooldown_.count(target_id))
          continue;
        Candidate combat{"combat", target_id, p};
        combat.combat = event_attack_gain(*enemy) * opportunity;
        consider(combat);
      }
    }
  }
  // 眼前无人而伤害仍有收益时，用最后看见Tick推算可能位置；旧格为空只排除旧格，
  // 不能把已经移动的敌人整条线索丢弃。未知格仅为软概率，目标和执行路线仍须已知可走。
  if (active_weights.recent_enemy_search_weight > 0 && visible.empty() && search_gain > 0) {
    std::optional<EnemySightingsField> sightings;
    if (!active_weights.enemy_belief_tracking)
      sightings.emplace(frame,memory_,active_weights.recent_enemy_half_life,active_weights.recent_enemy_max_age);
    plan_.recent_enemy_sightings = active_weights.enemy_belief_tracking ? enemy_beliefs_.details : sightings->details;
    for (int y = 0; y < size.height; ++y) {
      for (int x = 0; x < size.width; ++x) {
        const Position p{x, y};
        if (dist[cell(p, size)] <= 0 || dist[cell(p, size)] >= remaining_ticks)
          continue;
        const double probability = active_weights.enemy_belief_tracking
            ? enemy_beliefs_.in_view(p,frame.vision_radius) : sightings->in_view(p,frame.vision_radius);
        if (probability <= 0)
          continue;
        Candidate search{"search_enemy", std::to_string(x) + "," + std::to_string(y), p};
        search.combat =
            search_gain * active_weights.recent_enemy_search_weight * probability *
            std::exp2(-static_cast<double>(dist[cell(p, size)]) / active_weights.recent_enemy_half_life);
        consider(search);
      }
    }
  }
  // 只有常规目标全部失效、战斗仍有收益时才巡查；D已满分时保留安全等待。
  // 价值来自旧区域的信息时效，不把敌人的历史坐标当作当前真实位置。
  const bool visible_pursuit=active_weights.purpose.enabled&&!frame.self.spawn_protected&&
      std::any_of(visible.begin(),visible.end(),[&](const auto* enemy){
        const auto id=enemy->id.value_or(enemy->team.value_or("enemy"));
        return !pursuit_cooldown_.count(id)&&event_attack_gain(*enemy)>0;
      });
  const double patrol_scale=visible_pursuit?active_weights.purpose.patrol_visible_scale:1.0;
  if(active_weights.purpose.enabled)plan_.purpose["visible_pursuit"]=visible_pursuit;
  if ((!best||active_weights.purpose.enabled) && search_gain > 0 && active_weights.patrol_value > 0 && patrol_scale>0) {
    for (int y = 0; y < size.height; ++y) {
      for (int x = 0; x < size.width; ++x) {
        const Position p{x, y};
        const int steps = dist[cell(p, size)];
        if (steps <= 0 || steps >= remaining_ticks)
          continue;  // 至少留一个Tick利用巡查发现的战斗机会。
        double stale = 0;
        int free_cells = 0;
        for (int vy = std::max(0, y - frame.vision_radius);
             vy <= std::min(size.height - 1, y + frame.vision_radius); ++vy) {
          for (int vx = std::max(0, x - frame.vision_radius);
               vx <= std::min(size.width - 1, x + frame.vision_radius); ++vx) {
            if (memory_.cells()[MapMemory::index({vx, vy}, size)] != 0)
              continue;
            ++free_cells;
            const auto last = last_observed_tick_[cell({vx, vy}, size)];
            const auto age = frame.tick - last;
            if (last >= 0 && age >= active_weights.patrol_min_age)
              stale += std::min(1.0, age / active_weights.patrol_stale_ticks);
          }
        }
        if (free_cells == 0 || stale <= 0)
          continue;
        double hint = 0;
        for (const auto& [key, memory] : memory_.entities()) {
          (void)key;
          if (memory.entity.type != "robot" || (memory.entity.hp&&*memory.entity.hp<=0) || memory.first_missing_tick)
            continue;
          const auto age = frame.tick - memory.last_seen_tick;
          hint = std::max(hint, active_weights.patrol_enemy_hint /
                                    (1.0 + age / active_weights.patrol_stale_ticks) /
                                    (1.0 + distance(p, memory.entity.position)));
        }
        Candidate patrol{"patrol", std::to_string(x) + "," + std::to_string(y), p};
        patrol.patrol =
            patrol_scale * search_gain * active_weights.patrol_value * std::min(1.0, stale / free_cells + hint);
        consider(patrol);
      }
    }
  }
  // 明确阶段目标，仍保留机会成本下限；不因分期去追一个几乎不可能收益的远方目标。
  // 临近到达任务最晚出发时间时先补任务；后续紧急事件撤离仍可覆盖这里的选择。
  if (active_weights.phase.enabled) {
    const double floor = phase == Stage2Phase::Fight
        ? active_weights.phase.combat_opportunity_floor : active_weights.phase.opportunity_floor;
    if (deadline_mission) {
      best = deadline_mission;
      plan_.phase_goal_selected = true;
    } else if (phase_resource && (!best || phase_resource->utility >= floor * best->utility)) {
      best = phase_resource;
      plan_.phase_goal_selected = true;
    } else if (phase_best && (!best || phase_best->utility >= floor * best->utility)) {
      best = phase_best;
      plan_.phase_goal_selected = true;
    }
    // 对所选主目标反向BFS一次，检验任务是否真的顺路；不使用曼哈顿距离猜绕路。
    // 只插入位于主目标之前的任务，下一Tick仍重新检查，主目标收益不能重复计分。
    if (best && best->kind != "mission" && !best->joint && !best->sweep &&
        !best->collection_search && active_weights.phase.task_detour >= 0) {
      const auto to_goal = goal_distance_field(size, best->position, passable);
      const int direct = dist[cell(best->position, size)];
      std::optional<Candidate> waypoint;
      int waypoint_steps = std::numeric_limits<int>::max();
      for (const auto* m : unfinished) {
        if (m->type != "reach" || !m->position || !inside(*m->position, size)) continue;
        const int travel = dist[cell(*m->position, size)];
        const int onward = to_goal[cell(*m->position, size)];
        if (travel <= 0 || travel >= direct || onward < 0 ||
            travel + onward > direct + active_weights.phase.task_detour ||
            travel + onward > remaining_ticks || travel >= waypoint_steps) continue;
        Candidate c{"mission", m->id, *m->position};
        c.mission = active_weights.mission_points * active_weights.mission_potential;
        c.utility = best->utility; // 继承主路线价值，只改变本段途经点，不凭空添加收益。
        c.switch_bonus = best->switch_bonus;
        waypoint = c;
        waypoint_steps = travel;
      }
      if (waypoint) { best = waypoint; plan_.task_waypoint = true; }
    }
  }
  // 一次性资源优先贯穿整场：没有开局窗口，晚期新发现同样重新做完整子集规划。
  // 预计对手期限不是消失事实；无确定抢先路线时仍可直接争取，实见消失才删除。
  if (active_weights.resource_race.enabled) {
    // 路线同长时直接比较新未知格数，保留大视野间的差别，不沿用旧评分的饱和上限。
    const auto raw_information = [&](Position p) {
      double cells = 0;
      for (int y=std::max(0,p.y-frame.vision_radius);y<=std::min(size.height-1,p.y+frame.vision_radius);++y)
        for (int x=std::max(0,p.x-frame.vision_radius);x<=std::min(size.width-1,p.x+frame.vision_radius);++x)
          if (memory_.cells()[MapMemory::index({x,y},size)] == -1) cells += unknown_supply[cell({x,y},size)];
      return cells;
    };
    const double point_to_resource = active_weights.resource_cap / active_weights.resource_points;
    for (auto& r : race_resources)
      if (const auto found = std::find_if(opportunities.begin(),opportunities.end(),
          [&](const auto& o){return o.key==r.key;}); found != opportunities.end())
        for (const auto& [id,gain]:found->mission_points)
          r.mission_values[id] = gain * point_to_resource;
    resource_race_plan = plan_resource_race(size, start, race_resources,
        static_cast<int>(remaining_ticks), active_weights.resource_race, passable, raw_information,
        active_weights.mission_points * active_weights.mission_potential * point_to_resource);
    plan_.resource_race = {{"enabled", true}, {"targets", resource_race_plan.targets},
        {"order", resource_race_plan.order}, {"arrivals", resource_race_plan.arrivals},
        {"forecast_value", resource_race_plan.forecast_value},
        {"route_steps", resource_race_plan.route_steps}, {"expanded", resource_race_plan.expanded},
        {"complete", resource_race_plan.complete}, {"elapsed_ms", resource_race_plan.elapsed_ms},
        {"contested_fallback", resource_race_plan.contested_fallback}, {"coverage", false}};
    if (resource_race_plan.route.success() && resource_race_plan.route.path.size() > 1) {
      const auto& key = resource_race_plan.order.front();
      const auto at = resource_race_plan.route.path.back();
      Candidate candidate{"resource", "race:" + key, {at.x, at.y}};
      if (const auto found = std::find_if(resource_candidates.begin(), resource_candidates.end(),
          [&](const auto& item) { return item.first == key; }); found != resource_candidates.end()) {
        candidate.resource = found->second.resource;
        candidate.mission = found->second.mission;
        candidate.visible_resource = found->second.visible_resource;
      }
      candidate.resource_race = true;
      candidate.utility = std::max(active_weights.resource_race.move_priority,
          (candidate.resource * resource_weight + candidate.mission * resource_mission_weight) /
          (resource_race_plan.route.cost + active_weights.travel_offset));
      resource_priority = candidate;
    } else if (active_weights.resource_race.coverage_fallback && unknown_map &&
               frame.tick < active_weights.resource_race.explore_ticks &&
               (frame.score.resource_value < active_weights.resource_cap || collection_task)) {
      auto cover_weights = collection_weights_at(active_weights, frame.tick);
      cover_weights.joint_observation_offset = active_weights.resource_race.coverage_offset;
      if (active_weights.resource_race.explore_method == 0) {
        race_coverage_plan = race_coverage_planner_.propose(frame, memory_, {}, options.blocked_cells,
                                                         remaining_ticks, cover_weights);
      } else {
        // 未知资源探索：比较新增视野/实际路程，并用可见敌人的到达下界修正争抢价值。
        // 不硬编码向中央或边界走；边缘、争抢、紧迫性与距离指数均独立训练。
        double best_density = 0;
        std::optional<Position> observation;
        const auto& rw = active_weights.resource_race;
        for (int y=0;y<size.height;++y) for (int x=0;x<size.width;++x) {
          const Position p{x,y}; const int steps=dist[cell(p,size)];
          if (steps<=0 || steps>remaining_ticks) continue;
          double gain=0;
          for (int vy=std::max(0,y-frame.vision_radius);vy<=std::min(size.height-1,y+frame.vision_radius);++vy)
            for (int vx=std::max(0,x-frame.vision_radius);vx<=std::min(size.width-1,x+frame.vision_radius);++vx) {
              const Position unseen{vx,vy};
              if (memory_.cells()[MapMemory::index(unseen,size)]!=-1) continue;
              double competition=1;
              if (!visible.empty()) {
                int enemy_steps=size.width+size.height;
                for (const auto* e:visible) enemy_steps=std::min(enemy_steps,distance(e->position,unseen));
                const double margin=enemy_steps-steps-distance(p,unseen);
                const double probability=1/(1+std::exp(std::clamp(-margin/rw.explore_race_scale,-60.,60.)));
                const double available=rw.explore_capture_floor+(1-rw.explore_capture_floor)*probability;
                competition=(1-rw.explore_competition)+rw.explore_competition*available;
                competition*=1+rw.explore_urgency*std::exp(-std::abs(margin)/rw.explore_race_scale);
              }
              const int edge=std::min({vx,vy,size.width-1-vx,size.height-1-vy});
              gain+=unknown_supply[cell(unseen,size)]*competition*(1+rw.explore_edge_bonus/(1+edge));
            }
          const double density=gain/std::pow(steps+rw.explore_offset,rw.explore_distance_power)-
                               event_penalty(p,frame.tick+steps);
          if (density>best_density+1e-9) {best_density=density;observation=p;}
        }
        if (observation) {
          const planning::GridMap map{size.width,size.height,memory_.cells(),planning::RowOrder::BottomToTop};
          race_coverage_plan.route=planning::AStarPlanner{}.searchAStar(map,{start.x,start.y},
              {observation->x,observation->y},options);
        }
      }
      if (race_coverage_plan.route.success() && race_coverage_plan.route.path.size() > 1) {
        const auto at = race_coverage_plan.route.path.back();
        Candidate candidate{"explore", "race-cover", {at.x, at.y}};
        candidate.resource_race = true;
        candidate.information = information(candidate.position);
        candidate.utility = std::max(active_weights.resource_race.explore_move_priority * acquisition.exploration, candidate.information /
            (race_coverage_plan.route.cost + active_weights.travel_offset));
        resource_priority = candidate;
        plan_.resource_race["coverage"] = true;
      }
    }
    // 最晚必须启程的任务优先级仍兜底；不为慢慢走过去放弃即将超时的5分。
    if (resource_priority && (!active_weights.acquisition.enabled || resource_priority->kind=="resource" ||
        !best || resource_priority->utility>=best->utility) && !(deadline_mission &&
        remaining_ticks <= dist[cell(deadline_mission->position, size)] + active_weights.resource_race.task_deadline_buffer))
      best = resource_priority;
  }
  if(transfer_alpha>0&&transfer_resource&&!deadline_mission&&
      (!best||transfer_resource->utility>=transfer_resource_floor*best->utility)) {
    best=transfer_resource;plan_.collection_transfer["selected"]=true;
  }
  if(active_weights.purpose.enabled) {
    const auto& purpose=active_weights.purpose;
    std::string mode=search_gain>0?"explore":"finish";
    std::string reason="no_viable_battle_target";
    if(deadline_mission) {best=deadline_mission;mode="finish";reason="task_departure_deadline";}
    else if(purpose_resource&&(!best||purpose_resource->utility>=purpose.resource_floor*best->utility)) {
      best=purpose_resource;mode="collect";reason="positive_reachable_resource";
    } else if(!purpose_resource&&plan_.phase_resource_age>=purpose.resource_quiet_ticks&&
        purpose_battle&&(!best||purpose_battle->utility>=purpose.battle_floor*best->utility)) {
      best=purpose_battle;mode="battle";reason="no_current_resource_opportunity";
    } else if(purpose_resource)reason="resource_opportunity_cost_too_high";
    else if(plan_.phase_resource_age<purpose.resource_quiet_ticks)reason="resource_search_grace";
    plan_.purpose["mode"]=mode;
    plan_.purpose["reason"]=reason;
    plan_.purpose["resource_candidate"]=purpose_resource.has_value();
    plan_.purpose["resource_quiet_ticks"]=plan_.phase_resource_age;
    plan_.purpose["chase_spent"]=unproductive_pursuit_;
  }
  // 到达任务成为本段行程目的，沿途攻击仍在动作层比较；紧急撤离可覆盖此目的。
  if (reach_finish) {
    best = reach_finish;
    if (reach_finish_id_ != best->id || !reach_finish_position_ ||
        !same(*reach_finish_position_, *reach_final_target) || reach_last_progress_tick_ < 0) {
      reach_last_progress_tick_ = frame.tick;
      reach_defensive_stalemate_ = 0;
    } else if (reach_finish_steps < reach_previous_distance_) {
      reach_last_progress_tick_ = frame.tick;
    }
    reach_finish_id_ = best->id;
    reach_finish_position_ = reach_final_target;
    reach_previous_distance_ = reach_finish_steps;
    const int pause = reach_attack_allowance(remaining_ticks, reach_finish_steps,
        active_weights.reach_deadline_buffer, active_weights.reach_attack_pause_ticks);
    // “可能被打死”的上界不能把双方防御永远锁在原地；用连续公开姿态识别这种僵持。
    bool defenders = false;
    bool all_defending = true;
    for (const auto* enemy : visible) {
      if (enemy->spawn_protected.value_or(false) || !clear_enemy_hit(start,enemy,visible)) continue;
      defenders = true;
      all_defending = all_defending && enemy->defending.value_or(false);
    }
    if (old && old->session_id == frame.session_id && old->tick + 1 == frame.tick &&
        old->self.alive && frame.self.alive && same(old->self.position,start) &&
        frame.self.hp >= old->self.hp && physical_cached_.type == "defend" &&
        defenders && all_defending)
      ++reach_defensive_stalemate_;
    else reach_defensive_stalemate_ = 0;
    plan_.reach_finish = {{"active", true}, {"mission_id", best->id},
        {"target", {reach_final_target->x, reach_final_target->y}},
        {"remaining_ticks", remaining_ticks}, {"estimated_steps", reach_finish_steps},
        {"exploring", best->kind == "mission_explore"}, {"attack_pause_ticks", pause},
        {"no_progress_ticks", frame.tick - reach_last_progress_tick_},
        {"move_due", pause == 0 || frame.tick - reach_last_progress_tick_ >= pause},
        {"defensive_stalemate_ticks", reach_defensive_stalemate_},
        {"progress_selected", false}};
  } else {
    reach_finish_id_.clear();
    reach_finish_position_.reset();
    reach_previous_distance_ = -1;
    reach_last_progress_tick_ = -1;
    reach_defensive_stalemate_ = 0;
  }
  // 正在堡垒内却明显落后时，撤向对手强化射程外；对手离开/到期会重新开放争夺。
  // 这是战术撤离，不把堡垒涂成墙，也不宣称它被某队独占。
  bool fortress_withdraw=false;
  for (const auto& [id,e]:events_.effects()) {
    if(e.kind!="unrivaled"||!e.actor.empty()) continue;
    const auto contest=fortress_contest(events_,e,frame,start,0,active_weights.events);
    if(!contest.losing) continue;
    if(!Stage2Events::contains(e.cells,start) && !contest.confirmed_buff &&
        contest.rival_ready_tick-frame.tick>active_weights.events.evacuation_buffer+1) continue;
    std::optional<Position> exit;double cost=std::numeric_limits<double>::infinity();
    const auto rival_buff=events_.buff(contest.rival,"unrivaled",frame.tick);
    const int radius=rival_buff?rival_buff->effect.attack_radius:e.attack_radius;
    for(int y=0;y<size.height;++y) for(int x=0;x<size.width;++x) {
      const Position p{x,y};const int steps=dist[cell(p,size)];
      if(steps<=0||steps>remaining_ticks||Stage2Events::contains(e.cells,p)||
          std::max(std::abs(p.x-contest.rival_position.x),
                   std::abs(p.y-contest.rival_position.y))<=radius) continue;
      const double value=steps+10*threat_risk(p,visible)+event_penalty(p,frame.tick+steps);
      if(value<cost) {cost=value;exit=p;}
    }
    plan_.fortress["withdraw"]={{"rival",contest.rival},{"own_ready_tick",contest.own_ready_tick},
        {"rival_ready_tick",contest.rival_ready_tick},{"confirmed_buff",contest.confirmed_buff},
        {"exit_found",exit.has_value()}};
    if(exit) {
      Candidate retreat{"evade","fortress-withdraw:"+id,*exit};
      retreat.safety=active_weights.events.fortress_withdraw_priority;
      retreat.utility=active_weights.events.fortress_withdraw_priority;
      if(!best||retreat.utility>best->utility) {best=retreat;fortress_withdraw=true;}
    }
  }
  if (plan_.preserving_score && retreat_target && escape_steps <= remaining_ticks) {
    Candidate retreat{"evade", "endgame-preserve", *retreat_target};
    const double damage_loss = active_weights.planning.enabled&&active_weights.planning.exact_death_loss
        ? planning_damage_loss(frame.score.effective_damage,active_weights.damage_cap,active_weights.damage_points)
        : active_weights.damage_points / active_weights.damage_cap * std::min(60.0, frame.score.effective_damage);
    retreat.safety = active_weights.endgame_preserve_weight * (active_weights.death_cost + damage_loss);
    retreat.utility = retreat.safety / (escape_steps + active_weights.travel_offset);
    if (!best || retreat.utility > best->utility)
      best = retreat;
    ++plan_.candidates;
  }
  if (inside_danger) {
    std::optional<Position> exit;
    int exit_steps = std::numeric_limits<int>::max();
    for (int y = 0; y < size.height; ++y) {
      for (int x = 0; x < size.width; ++x) {
        const Position p{x, y};
        const int steps = dist[cell(p, size)];
        if (steps <= 0 || steps >= exit_steps || steps > remaining_ticks)
          continue;
        if (std::none_of(event_->dangerous_cells.begin(), event_->dangerous_cells.end(),
                         [&](Position danger) { return same(p, danger); })) {
          exit = p;
          exit_steps = steps;
        }
      }
    }
    // 预告期尚宽裕时保留正常策略；接近最后撤离时间后提高撤离优先级。
    if (exit && event_->begin_tick - frame.tick <= exit_steps + 2) {
      Candidate evacuation{"evade", "ai-danger-region", *exit};
      evacuation.safety = active_weights.death_cost * 2;
      evacuation.utility = evacuation.safety / (exit_steps + active_weights.travel_offset) + 0.01;
      best = evacuation;
      ++plan_.candidates;
    }
  }
  // 爆炸和世界分裂在移动前结算：必须在激活前按实际最短撤离步数预留时间。
  for (const auto& [id, e] : events_.effects()) {
    if (!e.actor.empty() || (e.kind != "explosion" && e.kind != "world_split") ||
        e.end_tick <= frame.tick || !Stage2Events::contains(e.cells, start))
      continue;
    if (e.kind == "explosion" && frame.self.spawn_protected && frame.tick >= e.begin_tick)
      continue;
    std::optional<Position> exit;
    int steps = std::numeric_limits<int>::max();
    for (int y = 0; y < size.height; ++y)
      for (int x = 0; x < size.width; ++x) {
        const Position p{x, y};
        const int d = dist[cell(p, size)];
        if (d > 0 && d < steps && !Stage2Events::contains(e.cells, p) && d <= remaining_ticks) {
          exit = p;
          steps = d;
        }
      }
    if (exit && e.begin_tick - frame.tick <= steps + active_weights.events.evacuation_buffer) {
      Candidate evacuate{"evade", id, *exit};
      evacuate.safety = active_weights.events.blast_risk;
      evacuate.utility = evacuate.safety / (steps + active_weights.travel_offset) + 0.01;
      if (!best || evacuate.utility > best->utility)
        best = evacuate;
    }
  }
  // 仅在确认无进展往返后选一个循环区域外的可达落点，所有攻防/事件风险仍由动作层比较。
  if(navigation_progress_.active()&&!reach_finish&&(!best||best->kind!="evade")&&
      (best||search_gain>0||current_damage_bound(false)>0)) {
    std::optional<Position> exit;double priority=std::numeric_limits<double>::infinity();
    const bool persistent=active_weights.planning.enabled&&active_weights.planning.persistent_escape;
    if(persistent) {
      exit=navigation_progress_.exit();
      if(exit&&(!inside(*exit,size)||dist[cell(*exit,size)]<=0||
          event_penalty(*exit,frame.tick+dist[cell(*exit,size)])>=active_weights.events.blast_risk*.1)) {
        navigation_progress_.cancel_exit();exit.reset();
      }
    }
    if(!exit) for(int y=0;y<size.height;++y) for(int x=0;x<size.width;++x) {
      const Position p{x,y};const int steps=dist[cell(p,size)];
      if(steps<=0||steps>remaining_ticks||navigation_progress_.penalty(p,frame.tick,active_weights)>0) continue;
      if(persistent&&navigation_progress_.cycle_distance(p)<2)continue;
      int enemy_distance=0;
      if(!visible.empty()) {enemy_distance=size.width+size.height;for(const auto* e:visible) enemy_distance=std::min(enemy_distance,distance(p,e->position));}
      const double value=10*threat_risk(p,visible)+steps-.3*information(p)-.05*enemy_distance+
          event_penalty(p,frame.tick+steps);
      if(value<priority) {priority=value;exit=p;}
    }
    if(exit) {
      if(persistent&&!navigation_progress_.exit())
        navigation_progress_.commit_exit(*exit,frame.tick,dist[cell(*exit,size)],active_weights.planning.escape_ticks);
      Candidate escape{"unstick","navigation-progress",*exit};
      escape.safety=active_weights.navigation_cycle_cost;
      escape.utility=(best?best->utility:0)+active_weights.navigation_cycle_cost+.01;
      best=escape;
    }
  }
  if (best) {
    if(active_weights.purpose.enabled)plan_.purpose["selected_goal"]=best->kind;
    if (active_weights.resource_race.enabled) plan_.resource_race["selected"] = best->resource_race;
    plan_.goal = best->kind;
    plan_.target_id = best->id;
    plan_.target = best->position;
    plan_.event_reward = best->event_reward;
    plan_.resource_points = best->resource;
    plan_.mission_points = best->mission;
    plan_.combat_points = best->combat;
    plan_.information_points = best->information;
    plan_.observation_age_points = best->age_information;
    plan_.patrol_potential = best->patrol;
    plan_.safety_potential = best->safety;
    // 惯性用于稳定“去哪里”，不等于一步移动会获得的积分；否则高惯性会压过就地射击。
    plan_.goal_utility = best->utility -
        (active_weights.separate_goal_switch_bonus ? best->switch_bonus : 0.0);
    const planning::GridMap map{size.width, size.height, memory_.cells(),
                                planning::RowOrder::BottomToTop};
    plan_.joint_used = best->joint;
    plan_.joint_future_utility = best->future_utility;
    if (best->resource_race) {
      plan_.route = best->kind == "resource" ? resource_race_plan.route : race_coverage_plan.route;
    } else if (best->collection_search) {
      plan_.route=collection_search_plan.route;
    } else if (best->sweep) {
      plan_.route.status=planning::SearchStatus::Found;
      plan_.route.path=sweep_plan.path;
      plan_.route.cost=static_cast<int>(sweep_plan.path.size())-1;
    } else if (best->joint) {
      plan_.route = joint_plan.route;
      plan_.joint_visit_order = joint_plan.visit_order;
    } else {
      plan_.route = planning::AStarPlanner{}.searchAStar(
          map, {start.x, start.y}, {best->position.x, best->position.y}, options);
      if (weighted_event_route) {
        const auto safer = event_route(size, start, best->position, event_route_cost, passable);
        if (safer.success())
          plan_.route = safer;
      }
    }
  }

  // 仅新模式用共同窗口比较根动作；赶任务/撤离仍由原安全与截止保护负责。
  PlanningResult collection_values;
  Json collection_learning=Json::object();
  CollectionRaceForecast collection_competition;
  if(active_weights.planning.enabled&&active_weights.planning.race_scenarios)
    collection_competition=forecast_collection_race(frame,memory_,old?&*old:nullptr,active_weights.planning.race_memory_ticks);
  const bool collection_goal=best&&(best->kind=="resource"||best->kind=="explore");
  if(active_weights.planning.enabled&&!reach_finish&&!fortress_withdraw&&
      (!best||best->kind!="evade")&&(collection_goal||!opportunities.empty())) {
    std::vector<Position> original_path;
    if(collection_goal)for(const auto p:plan_.route.path)original_path.push_back({p.x,p.y});
    const auto solve_collection=[&](const Stage2Weights& params) {return plan_collection_actions(frame,memory_,params,
        static_cast<int>(remaining_ticks),normal_collected_,high_collected_,resource_weight,
        resource_mission_weight*(active_weights.adaptive_resources?active_weights.collection_mission_weight:resource_weight),
        nearest_enemy,[&](Position p,int arrival) {
          if(!inside(p,size)||events_.terrain_at(p,frame.tick+arrival-1,memory_)!=0)return false;
          if(arrival==1)return passable(p);
          const auto retry=blocked_retry_until_.find({p.x,p.y});
          if(retry!=blocked_retry_until_.end()&&retry->second>frame.tick+arrival-1)return false;
          return !(avoid_event&&!inside_danger&&frame.tick+arrival-1>=event_->begin_tick&&
              frame.tick+arrival-1<event_->end_tick&&Stage2Events::contains(event_->dangerous_cells,p));
        },[&](double cells,double edge_cells) {
          return exploration_value(edge_cells*active_weights.information_per_cell+frontier_value.extra(cells),
              active_weights.information_cap,1.0)*plan_.economy.exploration_factor*
              std::max(acquisition.exploration,active_weights.planning.exploration_floor);
        },[&](Position p,int arrival) {
          return event_penalty(p,frame.tick+arrival)+
              std::exp2(-(arrival-1)/6.0)*threat_risk(p,visible)*active_weights.route_risk;
        },unknown_supply,best&&best->kind=="explore"?std::optional<Position>(best->position):std::nullopt,original_path,
        &collection_competition);};
    collection_values=solve_collection(active_weights);
    const auto& cfg=active_weights.planning.collection_neural;
    if((cfg.enabled||cfg.collect)&&collection_goal&&collection_values.useful&&collection_values.roots[0].valid) {
      const auto began=std::chrono::steady_clock::now();
      std::array<NeuralFeatures,5> features{};std::array<bool,5> allowed{};
      int original=-1;double best_base=-std::numeric_limits<double>::infinity();
      for(std::size_t i=0;i<5;++i)if(collection_values.roots[i].valid) {
        allowed[i]=!(i==0&&opening_collect_only)&&
            (i==0||frame.self.spawn_protected||current_damage_bound(false)<frame.self.hp);
        features[i]=collection_route_features(frame,memory_,collection_values,i,normal_collected_,high_collected_,nearest_enemy,unknown_supply);
        if(allowed[i]&&collection_values.roots[i].value>best_base){best_base=collection_values.roots[i].value;original=static_cast<int>(i);}
      }
      collection_learning={{"eligible",original>=0},{"enabled",cfg.enabled},{"changed",false},{"teacher_ran",false}};
      if(original>=0&&cfg.collect&&frame.tick%cfg.sample_stride==0) {
        auto teacher_weights=active_weights;
        teacher_weights.planning.horizon=cfg.teacher_horizon;
        teacher_weights.planning.beam_width=std::max(24,active_weights.planning.beam_width);
        teacher_weights.planning.max_nodes=std::max(30000,active_weights.planning.max_nodes);
        teacher_weights.planning.budget_ms=std::min(100.0,cfg.teacher_budget_ms);
        const auto teacher=solve_collection(teacher_weights);
        collection_learning["teacher_ran"]=true;
        collection_learning["teacher_depth"]=teacher.completed_depth;
        collection_learning["teacher_budget_hit"]=teacher.budget_hit;
        if(!teacher.budget_hit&&teacher.completed_depth==std::min<int>(remaining_ticks,cfg.teacher_horizon)) {
          Json samples=Json::array();
          constexpr std::array<const char*,5> directions{{"","up","down","left","right"}};
          for(std::size_t i=0;i<5;++i)if(collection_values.roots[i].valid&&teacher.roots[i].valid) {
            const double baseline=collection_values.roots[i].value/kCollectionNeuralScale;
            samples.push_back({{"action",i==0?"wait":"move"},{"direction",directions[i]},
                {"features",features[i]},{"baseline",baseline},
                {"target",teacher.roots[i].value/kCollectionNeuralScale-baseline},
                {"selected",static_cast<int>(i)==original},{"safety_ok",allowed[i]}});
          }
          if(samples.size()>=2) {
            collection_neural_samples_<<Json{{"schema","planning-v3-collection"},{"feature_version",1},
                {"teacher_kind","public-collection-rollout"},{"policy_clip",cfg.max_correction},
                {"collection_baseline",collection_baseline_signature(active_weights)},
                {"group",cfg.sample_group},{"session_id",frame.session_id},{"tick",frame.tick},
                {"goal",best->kind},{"samples",std::move(samples)}}.dump()<<'\n';
            if(!collection_neural_samples_)throw std::runtime_error("collection sample write failed");
            collection_learning["sample_written"]=true;
          }
        }
      }
      if(original>=0&&cfg.enabled) {
        std::array<double,5> deltas{};
        for(std::size_t i=0;i<5;++i)if(collection_values.roots[i].valid)
          deltas[i]=cfg.weight*std::clamp<double>(collection_neural_model_.predict(features[i]),-cfg.max_correction,cfg.max_correction);
        int chosen=original;double best=best_base/kCollectionNeuralScale+deltas[original]+cfg.override_margin;
        for(std::size_t i=0;i<5;++i)if(allowed[i]&&deltas[i]>deltas[original]+cfg.override_margin&&
            collection_values.roots[i].value/kCollectionNeuralScale+deltas[i]>best) {
          chosen=static_cast<int>(i);best=collection_values.roots[i].value/kCollectionNeuralScale+deltas[i];
        }
        // 只有超过原路线与安全边界的明确修正才生效；后续攻防/事件保护仍可覆盖。
        if(chosen!=original)for(std::size_t i=0;i<5;++i)if(collection_values.roots[i].valid)
          collection_values.roots[i].value+=kCollectionNeuralScale*deltas[i];
        collection_learning["changed"]=chosen!=original;
        collection_learning["original_root"]=original;collection_learning["proposed_root"]=chosen;
      }
      collection_learning["elapsed_ms"]=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
    }
  }
  if(active_weights.planning.enabled) {
    plan_.planning=collection_values.describe();plan_.planning["collection_neural"]=std::move(collection_learning);
  }
  const bool use_collection_values=collection_values.useful&&collection_values.roots[0].valid;
  const int incoming = frame.self.spawn_protected ? 0 : event_threats(start, visible);
  // 分期只降低开局主动攻击欲望；被动攻击风险、死亡边界和撤离逻辑完全沿用原判断。
  const double phase_attack_factor = phase == Stage2Phase::Collect
      ? active_weights.phase.opening_attack_factor : 1.0;
  plan_.incoming_damage_bound = current_damage_bound(false);
  // 交火线索只降低期望损伤，不降低攻击先结算的最坏致死边界。
  double expected_threats = incoming;
  if (use_intents && !frame.self.spawn_protected) {
    for (std::size_t i = 0; i < visible.size(); ++i)
      if (!visible[i]->spawn_protected.value_or(false) &&
          intent_clear_shot(visible[i]->position, start, intent_bodies, static_cast<int>(i)))
        expected_threats -=
            active_weights.third_party_risk_discount * intents.opponents[i].third_party_confidence;
    expected_threats = std::max(0.0, expected_threats);
  }
  double best_action = -std::numeric_limits<double>::infinity();
  const bool urgent_own_goal=best&&(best->kind=="evade"||
      (best->resource+best->mission)>0);
  bool duel_probe=false;
  if(active_weights.duel_stalemate.enabled) {
    std::vector<const VisibleEntity*> threats;
    bool ordinary=true;
    for(const auto* enemy:visible)if(clear_enemy_hit(start,enemy,visible)) {
      threats.push_back(enemy);
      if(events_.buff(enemy->team.value_or(""),"unrivaled",frame.tick))ordinary=false;
    }
    duel_stalemate_.observe(frame,old,physical_cached_,threats);
    const bool eligible=ordinary&&!known_resource&&!opening_collect_only&&!urgent_own_goal&&
        !purity&&!own_unrivaled&&!events_.active("pan",frame.tick)&&
        frame.score.effective_damage<active_weights.damage_cap&&
        event_penalty(start,frame.tick+1)<=0;
    duel_probe=duel_stalemate_.active(active_weights.duel_stalemate,frame,eligible);
    plan_.duel_stalemate={{"ticks",duel_stalemate_.ticks()},{"active",duel_probe},
        {"death_risk_factor",duel_probe?active_weights.duel_stalemate.death_risk_factor:1.0}};
  }
  const auto interference_gain=[&](const VisibleEntity& enemy,double banked_damage=0.0) {
    if(frame.score.effective_damage<active_weights.damage_cap) return 0.0;
    double reflection=0;
    if(purity) {
      if(own_unrivaled) {
        for(const auto* target:visible)
          if(!target->spawn_protected.value_or(false)&&
             square_hit(start,target->position,own_unrivaled->effect.attack_radius))
            reflection+=purity->reflection_damage;
      } else reflection=purity->reflection_damage;
    }
    return active_weights.interference.attack_value(frame.score.effective_damage+banked_damage,active_weights.damage_cap,
        active_weights.damage_points,frame.self.hp,current_damage_bound(false)+reflection,
        enemy.hp.value_or(100.0),estimated_damage(enemy),urgent_own_goal);
  };
  /// 每项动作都承担当前位置的本Tick攻击风险；移动只能改变后续暴露，不能回避先结算的攻击。
  const auto evaluate = [&](Action action, Position destination, double gain, double interference=0.0) {
    // 在动作层过滤，确保rollout只能从移动根节点选择；没有合法移动时才保留WAIT兜底。
    if(opening_collect_only&&action.type!="move")return;
    double collection_advantage=0;
    if(use_collection_values&&action.type=="move") {
      for(std::size_t i=0;i<kSteps.size();++i)if(action.direction==kDirections[i]&&collection_values.roots[i+1].valid) {
        collection_advantage=collection_values.roots[i+1].value-collection_values.roots[0].value;
        // 对采集/开图移动替换旧目标收益率，避免完整目标收益和未来采集重复相加。
        gain=(collection_goal?0.0:gain)+collection_advantage;
        if(collection_goal&&plan_.route.path.size()>1&&
            plan_.route.path[1].x==destination.x&&plan_.route.path[1].y==destination.y)
          gain+=active_weights.planning.route_tie_margin;
        break;
      }
    }
    double bound = current_damage_bound(action.type == "defend");
    // 高采集权重不许诺攻击先结算时无法执行的移动；防御能活下来时撤回该移动收益。
    if ((active_weights.acquisition.enabled || reach_finish || fortress_withdraw) && action.type=="move" && bound>=frame.self.hp &&
        current_damage_bound(true)<frame.self.hp)
      gain=0;
    double reflected_expected = 0, reflected_bound = 0;
    if (purity && action.type == "attack") {
      for (std::size_t i = 0; i < kDirections.size(); ++i)
        if (action.direction == kDirections[i]) {
          const auto* ordinary = own_unrivaled ? nullptr : attack_target(start, kSteps[i], visible);
          for (const auto* target : visible)
            if (!target->spawn_protected.value_or(false) &&
                (own_unrivaled
                     ? square_hit(start, target->position, own_unrivaled->effect.attack_radius)
                     : target == ordinary)) {
              reflected_expected += purity->reflection_damage * defend_probability(*target);
              reflected_bound += purity->reflection_damage;
            }
        }
      bound += reflected_bound;
    }
    const double hp_risk =
        1.0 + active_weights.low_hp_risk_weight * std::clamp(1.0 - frame.self.hp / 100.0, 0.0, 1.0);
    const double expected_bound =
        incoming > 0 ? current_damage_bound(action.type == "defend") * expected_threats / incoming
                     : 0.0;
    double utility =
        gain + interference + hold_reward(destination) -
        expected_bound * active_weights.attack_probability * active_weights.hp_loss_cost * hp_risk -
        reflected_expected * active_weights.hp_loss_cost * hp_risk;
    if(action.type=="move")
      utility-=navigation_progress_.penalty(destination,frame.tick,active_weights);
    utility -= event_penalty(destination, frame.tick + 1, action.type == "move");
    if (action.type == "defend" && purity)
      utility += purity->reflection_damage * active_weights.events.purity_defense_value *
                 expected_threats * active_weights.attack_probability;
    const auto healing = events_.buff(frame.self.team, "angel", frame.tick);
    if (healing && bound < frame.self.hp)
      utility +=
          std::min(healing->effect.heal_per_tick, expected_bound * active_weights.attack_probability) *
          active_weights.hp_loss_cost;
    if (bound > 0 && frame.self.hp <= bound) {
      const double damage_loss = active_weights.planning.enabled&&active_weights.planning.exact_death_loss
          ? planning_damage_loss(frame.score.effective_damage,active_weights.damage_cap,active_weights.damage_points)
          : active_weights.damage_points / active_weights.damage_cap * std::min(60.0, frame.score.effective_damage);
      // 用持续互防证据折扣“这Tick一定死亡”的软评分，真实死亡扣分规则不变。
      // 这不是安全保证；incoming_bound、资源保护和事件危险值均完整保留。
      const double probe_factor=duel_probe&&action.type=="attack"
          ?active_weights.duel_stalemate.death_risk_factor:1.0;
      utility -= (active_weights.death_cost + damage_loss)*probe_factor;
    }
    const bool retains_protection = frame.self.spawn_protected && same(start, destination);
    if (!retains_protection) {
      double exposure =
          (1.0 - active_weights.opponent_motion_weight) * event_threats(destination, visible) +
          active_weights.opponent_motion_weight * event_threats(destination, projected);
      if (active_weights.intent_risk_weight > 0)
        exposure = (1.0 - active_weights.intent_risk_weight) * exposure +
                   active_weights.intent_risk_weight * intents.exposure[cell(destination, size)];
      // 普通动作预测不能把已确认的无双射程/危险倍率稀释回普通敌人的水平。
      exposure += (1.0-active_weights.opponent_motion_weight) *
          (threat_risk(destination,visible)-event_threats(destination,visible)) +
          active_weights.opponent_motion_weight *
          (threat_risk(destination,projected)-event_threats(destination,projected));
      exposure += pending_fortress_risk(destination,frame.tick+1);
      double future_multiplier = multiplier(frame.self.team);
      const auto pan = events_.active("pan", frame.tick);
      if (action.type == "move" && pan && frame.tick < pan->begin_tick + pan->movement_window_ticks)
        future_multiplier = std::max(future_multiplier, 1.0 + active_weights.events.vulnerability_risk *
                                                                  (pan->damage_multiplier - 1.0));
      utility -= exposure * future_multiplier * active_weights.future_exposure_cost * hp_risk *
                 (plan_.preserving_score ? 1.0 + active_weights.endgame_preserve_weight : 1.0);
      utility-=active_weights.combat_clue_weight*battle_evidence_.exposure(destination)*
          active_weights.future_exposure_cost*hp_risk;
    }
    if (avoid_event && event_->begin_tick <= frame.tick &&
        std::any_of(event_->dangerous_cells.begin(), event_->dangerous_cells.end(),
                    [&](Position p) { return same(p, destination); }))
      utility -= active_weights.death_cost * 2;
    plan_.action_scores.push_back({{"action", action.type},
                                   {"direction", action.direction},
                                   {"utility", utility},
                                   {"collection_advantage",collection_advantage},
                                   {"interference_utility", interference},
                                   {"incoming_bound", bound}});
    if (utility > best_action + 1e-9) {
      best_action = utility;
      cached_ = std::move(action);
    }
  };
  // 无目标时先WAIT；其余动作必须有正的收益或安全改善才覆盖它，避免无意义游走。
  evaluate(Action::wait(), start, 0);
  // 只在资源/任务/开图的普通最短路上比较沿途观察；不改变联合路线顺序与战斗目标。
  const bool observe_route =
      active_weights.route_information_weight > 0 && best && !best->joint &&
      (best->kind == "resource" || best->kind == "mission" || best->kind == "explore");
  const auto step_information = [&](Position p) {
    return observe_route
               ? active_weights.route_information_weight * information(p) / (1.0 + active_weights.travel_offset)
               : 0.0;
  };
  if (plan_.route.success() && plan_.route.path.size() > 1) {
    const auto next = plan_.route.path[1];
    for (std::size_t i = 0; i < kSteps.size(); ++i) {
      if (next.x == start.x + kSteps[i].x && next.y == start.y + kSteps[i].y) {
        evaluate(Action::move(kDirections[i]), {next.x, next.y},
                 plan_.goal_utility + step_information({next.x, next.y}));
        break;
      }
    }
  }
  // 反向距离场让所有同样接近目标的首步获得收益，再由当前/未来风险择优。
  // 联合采集路线保持原访问顺序，不能用捷径跳过途中计划采集的资源。
  std::vector<int> goal_distances;
  if ((active_weights.route_alternatives_weight > 0 || observe_route) && best && !best->joint &&
      plan_.route.success() && plan_.route.path.size() > 1)
    goal_distances = goal_distance_field(size, best->position, passable);
  // 即使目标路线当前危险，也保留其他合法侧移作为局部避险候选。
  for (std::size_t i = 0; i < kSteps.size(); ++i) {
    Position p{start.x + kSteps[i].x, start.y + kSteps[i].y};
    if (passable(p)) {
      double gain = -0.01;
      if (!goal_distances.empty() && goal_distances[cell(start, size)] >= 0 &&
          goal_distances[cell(p, size)] >= 0) {
        const int progress = goal_distances[cell(start, size)] - goal_distances[cell(p, size)];
        gain += active_weights.route_alternatives_weight * plan_.goal_utility * progress;
        if (observe_route && progress == 1)
          gain = std::max(gain, plan_.goal_utility + step_information(p));
      }
      evaluate(Action::move(kDirections[i]), p, gain);
    }
  }
  if (!frame.self.spawn_protected && own_unrivaled) {
    double damage = 0;
    double interference=0;
    bool target = false;
    for (const auto* enemy : visible)
      if (!enemy->spawn_protected.value_or(false) &&
          square_hit(start, enemy->position, own_unrivaled->effect.attack_radius)) {
        target = true;
        interference+=interference_gain(*enemy,damage);
        damage += estimated_damage(*enemy);
      }
    if (target)
      evaluate({"attack", "up"}, start, damage_gain(frame, active_weights, damage) * combat_weight *
               phase_attack_factor,interference);
  }
  if (!frame.self.spawn_protected && !own_unrivaled) {
    for (std::size_t i = 0; i < kSteps.size(); ++i) {
      const auto* target = attack_target(start, kSteps[i], visible);
      if (target && !target->spawn_protected.value_or(false)) {
        const auto index = static_cast<std::size_t>(
            std::find(visible.begin(), visible.end(), target) - visible.begin());
        const double opportunity = use_intents
                                       ? 1.0 + active_weights.third_party_attack_bonus *
                                                   intents.opponents[index].third_party_confidence
                                       : 1.0;
        evaluate({"attack", kDirections[i]}, start,
                 event_attack_gain(*target) * combat_weight * opportunity * phase_attack_factor,
                 interference_gain(*target));
      }
    }
  }
  evaluate({"defend", ""}, start, 0);
  const bool preserve_collection = active_weights.rollout.preserve_collection &&
                                   (plan_.goal == "resource" || plan_.goal == "mission") &&
                                   current_damage_bound(false) <= 0;
  if (active_weights.rollout.weight > 0 && !visible.empty() && !preserve_collection && !fortress_withdraw && (!use_collection_values||active_weights.planning.tactical_bridge)) {
    std::vector<RolloutRoot> roots;
    for (const auto& score : plan_.action_scores)
      roots.push_back(
          {{score.at("action").get<std::string>(), score.at("direction").get<std::string>()},
           score.at("utility").get<double>()});
    std::vector<int> rollout_distances;
    if (best)
      rollout_distances = goal_distance_field(size, best->position, [&](Position p) {
        return inside(p, size) && memory_.cells()[MapMemory::index(p, size)] == 0;
      });
    auto tactical_config=active_weights.rollout;
    if(active_weights.planning.enabled) {
      tactical_config.max_transitions=30000;
      tactical_config.independent_opponents=active_weights.planning.independent_opponents;
    }
    if(use_collection_values) {
      tactical_config.collection_rewards=false;
      tactical_config.goal_progress=0;
    }
    const auto rollout = tactical_rollout(frame, memory_, events_, old, active_weights, tactical_config,
                                          roots, cached_, rollout_distances, plan_.goal_utility,
                                          normal_collected_, high_collected_, remaining_ticks);
    plan_.rollout = {{"ran", rollout.ran},
                     {"changed", rollout.changed},
                     {"completed_depth", rollout.completed_depth},
                     {"transitions", rollout.transitions},
                     {"elapsed_ms", rollout.elapsed_ms},
                     {"budget_hit", rollout.budget_hit},
                     {"roots", rollout.roots},
                     {"collection_rewards",tactical_config.collection_rewards},
                     {"collection_bridge",use_collection_values}};
    if (rollout.ran && rollout.completed_depth >= 2) {
      cached_ = rollout.action;
      best_action = rollout.utility;
    }
  }
  // 第一层使用原算法已有的收益、风险与记忆特征；网络只修正合法根动作的评分。
  // 教师与正式推理分开：教师仅显式离线采样时运行，绝不改变本次执行动作。
  if(active_weights.neural.enabled||active_weights.neural.collect) {
    const auto began=std::chrono::steady_clock::now();
    const auto& cfg=active_weights.neural;
    const bool eligible=!use_collection_values&&!visible.empty()&&!preserve_collection&&!fortress_withdraw&&
        (!best||best->kind!="evade")&&!plan_.action_scores.empty();
    plan_.neural={{"enabled",cfg.enabled},{"eligible",eligible},{"changed",false},
        {"weight",cfg.weight},{"teacher_ran",false}};
    if(eligible) {
      struct NeuralCandidate {Action action;Position position;NeuralFeatures features;double base,bound;};
      std::map<std::string,Json> unique;
      for(const auto& score:plan_.action_scores) {
        const auto key=score.at("action").get<std::string>()+":"+score.at("direction").get<std::string>();
        if(!unique.count(key)||score.at("utility").get<double>()>unique.at(key).at("utility").get<double>())unique[key]=score;
      }
      std::map<std::string,double> baseline;
      for(const auto& [key,score]:unique)baseline[key]=score.at("utility");
      if(plan_.rollout.value("completed_depth",0)>=2)
        for(const auto& score:plan_.rollout.at("roots"))
          baseline[score.at("action").get<std::string>()+":"+score.at("direction").get<std::string>()]=score.at("combined");
      std::vector<NeuralCandidate> candidates;
      std::vector<RolloutRoot> roots;
      int nearest=8;for(const auto* enemy:visible)nearest=std::min(nearest,distance(start,enemy->position));
      const auto norm=[](double value,double scale){return static_cast<float>(std::clamp(value/scale,-1.,1.));};
      const auto* nearest_enemy=*std::min_element(visible.begin(),visible.end(),[&](const auto* a,const auto* b){
        return distance(start,a->position)<distance(start,b->position);});
      // 新特征全部来自当前公开帧和连续历史；缺失血量另带known标志，不伪造0血。
      const bool continuous=old&&old->self.alive&&frame.tick==old->tick+1&&
          frame.self.position_reason!="respawn";
      double enemy_hp_loss=0;
      if(continuous&&nearest_enemy->hp&&nearest_enemy->team)
        for(const auto& e:old->visible_entities)
          if(e.type=="robot"&&e.team==nearest_enemy->team&&e.hp)
            enemy_hp_loss=std::max(0.,*e.hp-*nearest_enemy->hp);
      const auto buff_left=[&](const std::string& kind){
        const auto* b=events_.buff(frame.self.team,kind,frame.tick);
        return b?norm(b->until-frame.tick,100):0.F;};
      const auto event_left=[&](const std::string& kind){
        const auto* e=events_.active(kind,frame.tick);
        return e?norm(e->end_tick-frame.tick,100):0.F;};
      float explosion_until=-1;
      for(const auto& [id,e]:events_.effects()) {
        (void)id;
        if(e.kind=="explosion"&&e.end_tick>frame.tick)
          explosion_until=norm(e.begin_tick-frame.tick,50);
      }
      for(const auto& [key,score]:unique) {
        Action action{score.at("action"),score.at("direction")};Position at=start;
        Position facing{0,0};
        for(std::size_t d=0;d<kDirections.size();++d)
          if(action.direction==kDirections[d])facing=kSteps[d];
        if(action.type=="move")for(std::size_t d=0;d<kDirections.size();++d)
          if(action.direction==kDirections[d])at={start.x+kSteps[d].x,start.y+kSteps[d].y};
        NeuralFeatures x{{norm(frame.tick,500),norm(frame.self.hp,100),
          norm(frame.score.effective_damage,active_weights.damage_cap),norm(frame.score.resource_value,active_weights.resource_cap),
          norm(memory_.observed_count(),memory_.cells().size()),
          float(plan_.goal=="resource"),float(plan_.goal=="explore"),float(plan_.goal=="mission"),
          float(plan_.goal=="combat"||plan_.goal=="search_enemy"||plan_.goal=="patrol"),float(plan_.goal=="evade"),
          norm(score.at("utility"),5),norm(baseline[key],5),norm(plan_.goal_utility,5),
          norm(plan_.resource_points,5),norm(plan_.mission_points,5),norm(plan_.combat_points,5),
          norm(plan_.information_points,5),norm(plan_.patrol_potential,5),norm(visible.size(),3),norm(nearest,8),
          norm(score.at("incoming_bound"),100),norm(threat_risk(at,visible),3),norm(event_penalty(at,frame.tick+1),10),
          best?norm(distance(start,best->position)-distance(at,best->position),1):0.F,
          norm(blocked_retry_streak_,8),float(action.type=="wait"),float(action.type=="move"),
          float(action.type=="attack"),float(action.type=="defend"),float(at.x-start.x),float(at.y-start.y),
          float(frame.self.spawn_protected)}};
        const VisibleEntity* target=action.type=="attack"?
            (own_unrivaled?nearest_enemy:attack_target(start,facing,visible)):nullptr;
        x[32]=nearest_enemy->hp.has_value();x[33]=norm(nearest_enemy->hp.value_or(0),100);
        x[34]=nearest_enemy->defending.value_or(false);x[35]=nearest_enemy->spawn_protected.value_or(false);
        x[36]=events_.buff(nearest_enemy->team.value_or(""),"unrivaled",frame.tick)!=nullptr;
        x[37]=target!=nullptr;x[38]=target&&target->hp.has_value();x[39]=norm(target?target->hp.value_or(0):0,100);
        x[40]=target&&target->defending.value_or(false);x[41]=target&&target->spawn_protected.value_or(false);
        x[42]=target&&events_.buff(target->team.value_or(""),"unrivaled",frame.tick);
        const int dx=nearest_enemy->position.x-start.x,dy=nearest_enemy->position.y-start.y;
        x[43]=norm(facing.x||facing.y?dx*facing.x+dy*facing.y:nearest,8);
        x[44]=norm(dx*(-facing.y)+dy*facing.x,8);
        x[45]=norm(enemy_hp_loss,100);x[46]=norm(continuous?frame.self.hp-old->self.hp:0,100);
        x[47]=frame.self.defending;x[48]=buff_left("unrivaled");x[49]=buff_left("illusion");
        x[50]=buff_left("angel");x[51]=buff_left("purity");x[52]=buff_left("pepper");
        x[53]=event_left("pan");x[54]=event_left("world_split");x[55]=explosion_until;
        x[56]=norm(frame.score.mission_completed_ids.size(),std::max<std::size_t>(1,frame.missions.size()));
        x[57]=plan_.route.success()?norm(plan_.route.path.size()-1,40):1.F;
        int free=0,unknown=0,wall=0,attackers=0;
        for(auto d:kSteps) {
          const Position p{at.x+d.x,at.y+d.y};
          if(!inside(p,size)){++wall;continue;}
          const int terrain=events_.terrain_at(p,frame.tick+1,memory_);
          if(terrain<0)++unknown;else if(terrain==0&&!blocked[cell(p,size)])++free;else ++wall;
        }
        x[58]=norm(free,4);x[59]=norm(unknown,4);x[60]=norm(wall,4);
        int unseen=0,total=0;
        for(int y=std::max(0,at.y-frame.vision_radius);y<=std::min(size.height-1,at.y+frame.vision_radius);++y)
          for(int xx=std::max(0,at.x-frame.vision_radius);xx<=std::min(size.width-1,at.x+frame.vision_radius);++xx) {
            ++total;unseen+=memory_.cells()[MapMemory::index({xx,y},size)]<0;
          }
        x[61]=norm(unseen,std::max(1,total));
        x[62]=norm(distance(start,nearest_enemy->position)-distance(at,nearest_enemy->position),1);
        for(const auto* e:visible)attackers+=in_range(e->position,at);
        x[63]=norm(attackers,3);
        candidates.push_back({action,at,x,baseline[key],score.at("incoming_bound")});
        roots.push_back({action,score.at("utility")});
      }
      if(cfg.collect&&frame.tick%cfg.sample_stride==0&&remaining_ticks>=cfg.teacher_horizon) {
        auto teacher_cfg=active_weights.rollout;
        teacher_cfg.start_tick=0;teacher_cfg.activation_range=8;teacher_cfg.horizon=cfg.teacher_horizon;
        teacher_cfg.budget_ms=cfg.teacher_budget_ms;teacher_cfg.override_margin=0;
        if(active_weights.planning.enabled){teacher_cfg.independent_opponents=true;teacher_cfg.max_transitions=30000;}
        teacher_cfg.weight=std::max(.25,teacher_cfg.weight);
        std::vector<int> field;
        if(best)field=goal_distance_field(size,best->position,[&](Position p){
          return inside(p,size)&&memory_.cells()[MapMemory::index(p,size)]==0;});
        const auto teacher=tactical_rollout(frame,memory_,events_,old,active_weights,teacher_cfg,
            roots,cached_,field,plan_.goal_utility,normal_collected_,high_collected_,remaining_ticks);
        plan_.neural["teacher_ran"]=teacher.ran;
        plan_.neural["teacher_depth"]=teacher.completed_depth;
        plan_.neural["teacher_ms"]=teacher.elapsed_ms;
        if(teacher.completed_depth==cfg.teacher_horizon) {
          Json samples=Json::array();double total=0;
          for(const auto& candidate:candidates)for(const auto& target:teacher.roots)
            if(target.at("action")==candidate.action.type&&target.at("direction")==candidate.action.direction) {
              const double residual=std::clamp(target.at("combined").get<double>()-candidate.base,-3.,3.);
              total+=residual;
              double original_bound=0;
              for(const auto& c:candidates)if(c.action.type==cached_.type&&c.action.direction==cached_.direction)
                original_bound=c.bound;
              const bool safety_ok=!(original_bound<frame.self.hp&&candidate.bound>=frame.self.hp)&&
                  event_penalty(candidate.position,frame.tick+1)<active_weights.events.blast_risk*.1;
              samples.push_back({{"features",candidate.features},{"target",residual},
                {"baseline",candidate.base},{"action",candidate.action.type},{"direction",candidate.action.direction},
                {"selected",candidate.action.type==cached_.type&&candidate.action.direction==cached_.direction},
                {"safety_ok",safety_ok}});
            }
          if(samples.size()>=2) {
            const double mean=total/samples.size();
            for(auto& sample:samples)sample["target"]=sample.at("target").get<double>()-mean;
            neural_samples_<<Json{{"schema",active_weights.planning.enabled?"planning-v2-combat":"combat-v1"},
                {"feature_version",2},{"policy_clip",cfg.max_correction},
                {"teacher_kind","tactical-public-rollout"},{"group",cfg.sample_group},{"session_id",frame.session_id},
                {"tick",frame.tick},{"depth",teacher.completed_depth},{"samples",samples}}.dump()<<'\n';
            if(!neural_samples_)throw std::runtime_error("neural sample write failed");
          }
        }
      }
      const auto infer_began=std::chrono::steady_clock::now();
      if(cfg.enabled&&cfg.weight>0) {
        double original_bound=0;
        for(const auto& c:candidates)if(c.action.type==cached_.type&&c.action.direction==cached_.direction)original_bound=c.bound;
        double selected=-std::numeric_limits<double>::infinity(),original_correction=0;
        for(const auto& c:candidates)if(c.action.type==cached_.type&&c.action.direction==cached_.direction)
          {original_correction=cfg.weight*std::clamp<double>(neural_model_.predict(c.features),-cfg.max_correction,cfg.max_correction);
           selected=c.base+original_correction+cfg.override_margin;}
        const Action original=cached_;Json scores=Json::array();
        for(const auto& c:candidates) {
          const double prediction=neural_model_.predict(c.features);
          const double correction=cfg.weight*std::clamp(prediction,-cfg.max_correction,cfg.max_correction);
          const double value=c.base+correction;
          const bool safety_ok=!(original_bound<frame.self.hp&&c.bound>=frame.self.hp)&&
              event_penalty(c.position,frame.tick+1)<active_weights.events.blast_risk*.1;
          scores.push_back({{"action",c.action.type},{"direction",c.action.direction},{"baseline",c.base},
              {"prediction",prediction},{"correction",correction},{"combined",value},{"safety_ok",safety_ok}});
          // 原前瞻有自己的换动作门槛；零残差必须完整保留原动作，不能借新门槛偷偷改策略。
          if(safety_ok&&correction>original_correction+cfg.override_margin&&value>selected+1e-9)
            {selected=value;cached_=c.action;best_action=value;}
        }
        plan_.neural["changed"]=original.type!=cached_.type||original.direction!=cached_.direction;
        plan_.neural["proposed_action"]=cached_.type;
        plan_.neural["proposed_direction"]=cached_.direction;
        plan_.neural["scores"]=std::move(scores);
      }
      plan_.neural["inference_ms"]=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-infer_began).count();
    }
    plan_.neural["total_ms"]=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
  }
  // 有余裕时攻击可胜出；停留预算耗尽后，选择接近任务的首步。
  // 默认保留致死上界；仅连续互防且无掉血的僵持允许试探脱离所有当前射线，非必然安全。
  // 不把爆炸/分裂撤离替换为赶任务，也不让幻术改变物理落点的安全检查。
  if (reach_finish && best && best->id == reach_finish->id &&
      best->kind == reach_finish->kind && plan_.route.success() && plan_.route.path.size() > 1 &&
      plan_.reach_finish.value("move_due", false)) {
    const auto field = goal_distance_field(size, best->position, passable);
    double progress_utility = -std::numeric_limits<double>::infinity();
    std::optional<Action> progress_action;
    for (const auto& score : plan_.action_scores) {
      if (score.at("action") != "move") continue;
      for (std::size_t i = 0; i < kDirections.size(); ++i) {
        if (score.at("direction") != kDirections[i]) continue;
        const Position p{start.x + kSteps[i].x, start.y + kSteps[i].y};
        if (!passable(p) || field[cell(p,size)] < 0 || field[cell(start,size)] < 0 ||
            field[cell(p,size)] >= field[cell(start,size)] ||
            event_penalty(p, frame.tick + 1, true) >= active_weights.events.blast_risk * 0.1)
          continue;
        const bool stalemate_escape = active_weights.reach_stalemate_ticks > 0 &&
            reach_defensive_stalemate_ >= active_weights.reach_stalemate_ticks &&
            event_threats(p,visible) == 0;
        if (score.at("incoming_bound").get<double>() >= frame.self.hp && !stalemate_escape)
          continue;
        const double utility = score.at("utility").get<double>();
        if (utility > progress_utility) {
          progress_utility = utility;
          progress_action = Action::move(kDirections[i]);
        }
      }
    }
    if (progress_action) {
      cached_ = *progress_action;
      best_action = progress_utility;
      plan_.reach_finish["progress_selected"] = true;
      plan_.reach_finish["stalemate_escape"] = current_damage_bound(false) >= frame.self.hp;
    } else {
      plan_.reach_finish["progress_blocked_by_safety"] = true;
      const bool recovery=active_weights.planning.enabled&&active_weights.planning.reach_recovery&&
          stationary_defense_ticks_>=active_weights.planning.recovery_ticks&&
          frame.tick-reach_last_progress_tick_>=active_weights.planning.recovery_ticks;
      if(recovery) {
        double recovery_cost=std::numeric_limits<double>::infinity();
        std::optional<Action> exit;
        const double death_loss=planning_damage_loss(frame.score.effective_damage,
            active_weights.damage_cap,active_weights.damage_points);
        for(const auto& score:plan_.action_scores)if(score.at("action")=="move")
          for(std::size_t i=0;i<kDirections.size();++i)if(score.at("direction")==kDirections[i]) {
            const Position p{start.x+kSteps[i].x,start.y+kSteps[i].y};
            if(!passable(p)||field[cell(p,size)]<0||field[cell(p,size)]+active_weights.reach_deadline_buffer>remaining_ticks||
                event_penalty(p,frame.tick+1,true)>=active_weights.events.blast_risk*.1)continue;
            // 只有目的地脱离全部当前射线才允许此软试探；这不保证当前Tick不会转火。
            if(event_threats(p,visible)>0||death_loss>=active_weights.mission_points)continue;
            const double cost=field[cell(p,size)]+pending_fortress_risk(p,frame.tick+1)*10;
            if(cost<recovery_cost){recovery_cost=cost;exit=Action::move(kDirections[i]);}
          }
        plan_.reach_finish["recovery_considered"]=true;
        plan_.reach_finish["stationary_defense_ticks"]=stationary_defense_ticks_;
        if(exit) {
          cached_=*exit;stationary_defense_ticks_=0;
          plan_.reach_finish["recovery_selected"]=true;
          plan_.reach_finish["recovery_is_soft_probe"]=true;
          for(const auto& score:plan_.action_scores)if(score.at("action")==cached_.type&&score.at("direction")==cached_.direction)
            best_action=score.at("utility");
        }
      }
    }
  }
  if(plan_.neural.value("changed",false)) {
    const bool applied=plan_.neural.at("proposed_action")==cached_.type&&plan_.neural.at("proposed_direction")==cached_.direction;
    plan_.neural["overridden_by_final_guard"]=!applied;plan_.neural["changed"]=applied;
  }
  physical_cached_ = cached_;
  if(active_weights.planning.enabled) {
    auto& learned=plan_.planning["collection_neural"];
    if(learned.value("changed",false)) {
      int actual_root=0;
      if(cached_.type=="move")for(std::size_t i=0;i<kDirections.size();++i)
        if(cached_.direction==kDirections[i])actual_root=static_cast<int>(i)+1;
      const bool applied=actual_root==learned.at("proposed_root").get<int>();
      learned["overridden_by_final_guard"]=!applied;learned["changed"]=applied;
    }
    plan_.planning["applied"]=use_collection_values;
    plan_.planning["selected_action"]=cached_.type;
    plan_.planning["selected_direction"]=cached_.direction;
    // 紫色目标与预测路线应指向实际选择的采集顺序，避免仍显示旧A*目标。
    if(use_collection_values&&collection_goal&&cached_.type=="move")
      for(std::size_t i=0;i<kDirections.size();++i)if(cached_.direction==kDirections[i]) {
        const auto& forecast=collection_values.roots[i+1];
        plan_.route.path.clear();
        for(const auto p:forecast.path)plan_.route.path.push_back({p.x,p.y});
        plan_.route.status=planning::SearchStatus::Found;
        plan_.route.cost=static_cast<int>(forecast.path.size())-1;
        bool found=false;
        for(std::size_t j=1;j<forecast.path.size()&&!found;++j)
          for(const auto& [key,m]:memory_.entities())if(!m.first_missing_tick&&m.entity.value&&
              (m.entity.type=="resource_normal"||m.entity.type=="resource_high")&&same(m.entity.position,forecast.path[j])) {
            plan_.goal="resource";plan_.target=m.entity.position;plan_.target_id=m.entity.id.value_or(key);found=true;break;
          }
      }
  }
  for(const auto& score:plan_.action_scores)
    if(score.at("action")==cached_.type&&score.at("direction")==cached_.direction)
      plan_.interference_utility=score.value("interference_utility",0.0);
  if (cached_.type == "move" && events_.buff(frame.self.team, "illusion", frame.tick)) {
    plan_.movement_inverted = true;
    if (cached_.direction == "up")
      cached_.direction = "down";
    else if (cached_.direction == "down")
      cached_.direction = "up";
    else if (cached_.direction == "left")
      cached_.direction = "right";
    else if (cached_.direction == "right")
      cached_.direction = "left";
  }
  plan_.action_utility = best_action;
  previous_goal_ = active_weights.planning.enabled?plan_.goal+":"+plan_.target_id:
      (best ? best->kind + ":" + best->id : "");
  return cached_;
}

}  // namespace rmtc
