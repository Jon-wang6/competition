/**
 * @file connection_node.cpp
 * @brief 实现ROS通信节点，将感知接收、策略调用和动作回执连接起来。
 *
 * 输入：本队/team_X/perception中的String消息，data为JSON文本。
 * 输出：本队/team_X/action的动作，以及/team_X/registration的显示队名。
 * 每个新Tick决策一次；相同Tick只重发缓存，旧Tick丢弃。
 * 新场次清空局内缓存，累计统计保留；最终状态只检查回执、不发新动作。
 * 只使用规则允许的本队公开话题，不读取网页、全图或裁判内部文件。
 */
#include "rmtc_connection/connection_node.hpp"

#include <chrono>
#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace rmtc {

/**
 * @brief 完成节点的启动配置及ROS连接准备，尚不主动生成逐Tick动作。
 *
 * 参数通过declare_parameter读取：team指定席位，team_name指定显示名称，
 * mode选择wait/probe/resource/stage2，resource_order只选择第一阶段的资源顺序算法，
 * k_edge和k_value覆盖weighted边缘代价及每10点真实/虚拟价值奖励；
 * 路程、未知格价值及折扣窗口默认来自resource_config.hpp，便于直接改C++调参。
 * action_ticks指定动作总步数，log_file指定可选日志。
 * 创建感知订阅和动作发布后，先发布一条持久保留的队名注册消息。
 * @throws std::runtime_error 席位/模式/队名/总Tick非法，或日志文件无法打开。
 * @throws std::exception ROS参数声明或通信对象创建失败时，底层标准异常向上传播。
 */
ConnectionNode::ConnectionNode() : Node("rmtc_connection") {
  team_ = declare_parameter<std::string>("team", "A");
  const auto name = declare_parameter<std::string>("team_name", "奶龙做什么都队");
  mode_ = declare_parameter<std::string>("mode", "resource");
  if (is_stage2_mode()) {
    Stage2Weights opening;
    opening.planning.enabled=declare_parameter<bool>("planning_enabled",mode_=="stage2_planning");
    for(auto item:{std::pair<const char*,bool*>{"planning_tactical_bridge",&opening.planning.tactical_bridge},
        {"planning_persistent_escape",&opening.planning.persistent_escape},
        {"planning_race_scenarios",&opening.planning.race_scenarios},
        {"planning_regional_exploration",&opening.planning.regional_exploration},
        {"planning_independent_opponents",&opening.planning.independent_opponents},
        {"planning_exact_death_loss",&opening.planning.exact_death_loss},
        {"planning_distribution_state",&opening.planning.distribution_state},
        {"planning_safe_terminal",&opening.planning.safe_terminal},
        {"planning_continue_baseline",&opening.planning.continue_baseline},
        {"planning_reach_recovery",&opening.planning.reach_recovery}})
      *item.second=declare_parameter<bool>(item.first,*item.second);
    opening.planning.recovery_ticks=declare_parameter<int>("planning_recovery_ticks",opening.planning.recovery_ticks);
    opening.planning.memory_model=declare_parameter<int>("planning_memory_model",opening.planning.memory_model);
    for(auto item:{std::pair<const char*,int*>{"planning_horizon",&opening.planning.horizon},
        {"planning_escape_ticks",&opening.planning.escape_ticks},
        {"planning_terminal_stops",&opening.planning.terminal_stops},
        {"planning_race_memory_ticks",&opening.planning.race_memory_ticks},
        {"planning_beam_width",&opening.planning.beam_width},
        {"planning_max_nodes",&opening.planning.max_nodes}})
      *item.second=declare_parameter<int>(item.first,*item.second);
    for(auto item:{std::pair<const char*,double*>{"planning_budget_ms",&opening.planning.budget_ms},
        {"planning_regional_strength",&opening.planning.regional_strength},
        {"planning_race_scenario_weight",&opening.planning.race_scenario_weight},
        {"planning_discount",&opening.planning.discount},
        {"planning_terminal_weight",&opening.planning.terminal_weight},
        {"planning_memory_half_life",&opening.planning.memory_half_life},
        {"planning_enemy_pressure",&opening.planning.enemy_pressure},
        {"planning_exploration_floor",&opening.planning.exploration_floor},
        {"planning_route_tie_margin",&opening.planning.route_tie_margin},
        {"planning_future_risk_weight",&opening.planning.future_risk_weight}})
      *item.second=declare_parameter<double>(item.first,*item.second);
    opening.neural.enabled=declare_parameter<bool>("neural_enabled",false);
    auto& collector=opening.planning.collection_neural;
    collector.enabled=declare_parameter<bool>("collection_neural_enabled",false);
    collector.collect=declare_parameter<bool>("collection_neural_collect",false);
    collector.model_file=declare_parameter<std::string>("collection_neural_model_file","");
    collector.sample_file=declare_parameter<std::string>("collection_neural_sample_file","");
    collector.sample_group=declare_parameter<std::string>("collection_neural_sample_group","");
    collector.weight=declare_parameter<double>("collection_neural_weight",.125);
    collector.max_correction=declare_parameter<double>("collection_neural_max_correction",1.0);
    collector.override_margin=declare_parameter<double>("collection_neural_override_margin",.003);
    collector.sample_stride=declare_parameter<int>("collection_neural_sample_stride",8);
    collector.teacher_horizon=declare_parameter<int>("collection_neural_teacher_horizon",20);
    collector.teacher_budget_ms=declare_parameter<double>("collection_neural_teacher_budget_ms",80.0);
    opening.neural.collect=declare_parameter<bool>("neural_collect",false);
    opening.neural.model_file=declare_parameter<std::string>("neural_model_file","");
    opening.neural.sample_file=declare_parameter<std::string>("neural_sample_file","");
    opening.neural.sample_group=declare_parameter<std::string>("neural_sample_group","");
    opening.neural.weight=declare_parameter<double>("neural_weight",opening.neural.weight);
    opening.neural.max_correction=declare_parameter<double>("neural_max_correction",opening.neural.max_correction);
    opening.neural.override_margin=declare_parameter<double>("neural_override_margin",opening.neural.override_margin);
    opening.neural.sample_stride=declare_parameter<int>("neural_sample_stride",opening.neural.sample_stride);
    opening.neural.teacher_horizon=declare_parameter<int>("neural_teacher_horizon",opening.neural.teacher_horizon);
    opening.neural.teacher_budget_ms=declare_parameter<double>("neural_teacher_budget_ms",opening.neural.teacher_budget_ms);
    opening.duel_stalemate.enabled=declare_parameter<bool>("duel_stalemate_enabled",opening.duel_stalemate.enabled);
    opening.duel_stalemate.ticks=declare_parameter<int>("duel_stalemate_ticks",opening.duel_stalemate.ticks);
    opening.duel_stalemate.start_tick=declare_parameter<int>("duel_stalemate_start_tick",opening.duel_stalemate.start_tick);
    opening.duel_stalemate.maximum_hp=declare_parameter<double>("duel_stalemate_maximum_hp",opening.duel_stalemate.maximum_hp);
    opening.duel_stalemate.death_risk_factor=declare_parameter<double>("duel_stalemate_death_risk_factor",opening.duel_stalemate.death_risk_factor);
    opening.collection_transfer.enabled=declare_parameter<bool>("collection_transfer_enabled",opening.collection_transfer.enabled);
    opening.collection_transfer.hold_ticks=declare_parameter<int>("collection_transfer_hold_ticks",opening.collection_transfer.hold_ticks);
    opening.collection_transfer.fade_ticks=declare_parameter<int>("collection_transfer_fade_ticks",opening.collection_transfer.fade_ticks);
    for(auto item:{std::pair<const char*,double*>{"collection_transfer_shape",&opening.collection_transfer.shape},
        {"collection_transfer_resource_weight",&opening.collection_transfer.resource_weight},
        {"collection_transfer_explore_peak",&opening.collection_transfer.explore_peak},
        {"collection_transfer_explore_half_ticks",&opening.collection_transfer.explore_half_ticks},
        {"collection_transfer_silence_half_ticks",&opening.collection_transfer.silence_half_ticks},
        {"collection_transfer_contested_factor",&opening.collection_transfer.contested_factor},
        {"collection_transfer_competition_softness",&opening.collection_transfer.competition_softness},
        {"collection_transfer_travel_offset",&opening.collection_transfer.travel_offset},
        {"collection_transfer_resource_floor",&opening.collection_transfer.resource_floor}})
      *item.second=declare_parameter<double>(item.first,*item.second);
    opening.opening_collection_ticks=declare_parameter<int>("opening_collection_ticks",opening.opening_collection_ticks);
    opening.battle_curve.enabled=declare_parameter<bool>("battle_curve_enabled",opening.battle_curve.enabled);
    opening.battle_curve.protect_resources=declare_parameter<bool>("battle_curve_protect_resources",opening.battle_curve.protect_resources);
    opening.battle_curve.start_tick=declare_parameter<int>("battle_curve_start_tick",opening.battle_curve.start_tick);
    opening.battle_curve.ramp_ticks=declare_parameter<int>("battle_curve_ramp_ticks",opening.battle_curve.ramp_ticks);
    for(auto item:{std::pair<const char*,double*>{"battle_curve_shape",&opening.battle_curve.shape},
        {"battle_curve_damage_need_power",&opening.battle_curve.damage_need_power},
        {"battle_curve_combat_end",&opening.battle_curve.combat_end},
        {"battle_curve_search_end",&opening.battle_curve.search_end},
        {"battle_curve_hp_risk_end",&opening.battle_curve.hp_risk_end},
        {"battle_curve_future_risk_end",&opening.battle_curve.future_risk_end},
        {"battle_curve_route_risk_end",&opening.battle_curve.route_risk_end},
        {"battle_curve_third_party_end",&opening.battle_curve.third_party_end}})
      *item.second=declare_parameter<double>(item.first,*item.second);
    opening.temporal.enabled=declare_parameter<bool>("temporal_policy_enabled",opening.temporal.enabled);
    opening.temporal.transition_ticks=declare_parameter<int>("temporal_transition_ticks",opening.temporal.transition_ticks);
    for (auto item:{std::pair<const char*,double*>{"temporal_shape",&opening.temporal.shape},
        {"temporal_resource_age_end",&opening.temporal.resource_age_end},
        {"temporal_competition_end",&opening.temporal.competition_end},
        {"temporal_lookahead_end",&opening.temporal.lookahead_end},
        {"temporal_route_information_end",&opening.temporal.route_information_end},
        {"temporal_third_party_attack_end",&opening.temporal.third_party_attack_end},
        {"temporal_third_party_risk_end",&opening.temporal.third_party_risk_end}})
      *item.second=declare_parameter<double>(item.first,*item.second);
    opening.acquisition.enabled=declare_parameter<bool>("acquisition_curve_enabled",opening.acquisition.enabled);
    opening.acquisition.silence_grace_ticks=declare_parameter<int>("acquisition_silence_grace_ticks",opening.acquisition.silence_grace_ticks);
    for (auto item:{std::pair<const char*,double*>{"acquisition_resource_weight",&opening.acquisition.resource_weight},
        {"acquisition_explore_peak",&opening.acquisition.explore_peak},
        {"acquisition_explore_floor",&opening.acquisition.explore_floor},
        {"acquisition_explore_half_ticks",&opening.acquisition.explore_half_ticks},
        {"acquisition_explore_shape",&opening.acquisition.explore_shape},
        {"acquisition_silence_half_ticks",&opening.acquisition.silence_half_ticks}})
      *item.second=declare_parameter<double>(item.first,*item.second);
    opening.resource_race.enabled = declare_parameter<bool>("resource_race_enabled", opening.resource_race.enabled);
    opening.resource_race.coverage_fallback = declare_parameter<bool>("resource_race_coverage", opening.resource_race.coverage_fallback);
    for (auto item : {std::pair<const char*, int*>{"resource_race_limit", &opening.resource_race.target_limit},
                     {"resource_race_horizon", &opening.resource_race.horizon},
                     {"resource_race_explore_method", &opening.resource_race.explore_method},
                     {"resource_race_task_buffer", &opening.resource_race.task_deadline_buffer},
                     {"resource_race_deadline_slack", &opening.resource_race.deadline_slack},
                     {"resource_race_explore_ticks", &opening.resource_race.explore_ticks}})
      *item.second = declare_parameter<int>(item.first, *item.second);
    for (auto item : {std::pair<const char*, double*>{"resource_race_distance_cost", &opening.resource_race.distance_cost},
                     {"resource_race_move_priority", &opening.resource_race.move_priority},
                     {"resource_race_budget_ms", &opening.resource_race.budget_ms},
                     {"resource_race_mission_weight", &opening.resource_race.mission_weight},
                     {"resource_race_terminal_information", &opening.resource_race.terminal_information},
                     {"resource_race_explore_offset", &opening.resource_race.explore_offset},
                     {"resource_race_explore_distance_power", &opening.resource_race.explore_distance_power},
                     {"resource_race_explore_edge_bonus", &opening.resource_race.explore_edge_bonus},
                     {"resource_race_explore_competition", &opening.resource_race.explore_competition},
                     {"resource_race_explore_urgency", &opening.resource_race.explore_urgency},
                     {"resource_race_explore_race_scale", &opening.resource_race.explore_race_scale},
                     {"resource_race_explore_capture_floor", &opening.resource_race.explore_capture_floor},
                     {"resource_race_explore_move_priority", &opening.resource_race.explore_move_priority},
                     {"resource_race_coverage_offset", &opening.resource_race.coverage_offset}})
      *item.second = declare_parameter<double>(item.first, *item.second);
    opening.separate_goal_switch_bonus = declare_parameter<bool>("separate_goal_switch_bonus", opening.separate_goal_switch_bonus);
    opening.purpose.enabled=declare_parameter<bool>("purpose_enabled",opening.purpose.enabled);
    opening.purpose.start_tick=declare_parameter<int>("purpose_start_tick",opening.purpose.start_tick);
    opening.purpose.protect_resources=declare_parameter<bool>("purpose_protect_resources",opening.purpose.protect_resources);
    for(auto item:{std::pair<const char*,int*>{"purpose_resource_quiet_ticks",&opening.purpose.resource_quiet_ticks},
        {"purpose_intercept_horizon",&opening.purpose.intercept_horizon},
        {"purpose_chase_progress_ticks",&opening.purpose.chase_progress_ticks}})
      *item.second=declare_parameter<int>(item.first,*item.second);
    for(auto item:{std::pair<const char*,double*>{"purpose_resource_floor",&opening.purpose.resource_floor},
        {"purpose_battle_floor",&opening.purpose.battle_floor},
        {"purpose_intercept_weight",&opening.purpose.intercept_weight},
        {"purpose_heading_weight",&opening.purpose.heading_weight},
        {"purpose_escape_weight",&opening.purpose.escape_weight}})
      *item.second=declare_parameter<double>(item.first,*item.second);
    opening.purpose.patrol_visible_scale=declare_parameter<double>("purpose_patrol_visible_scale",opening.purpose.patrol_visible_scale);
    opening.phase.enabled = declare_parameter<bool>("phase_enabled", opening.phase.enabled);
    opening.phase.resource_ticks = declare_parameter<int>("phase_resource_ticks", opening.phase.resource_ticks);
    opening.phase.chase_steps = declare_parameter<int>("phase_chase_steps", opening.phase.chase_steps);
    opening.phase.task_detour = declare_parameter<int>("phase_task_detour", opening.phase.task_detour);
    opening.phase.opportunity_floor = declare_parameter<double>("phase_opportunity_floor", opening.phase.opportunity_floor);
    opening.phase.opening_attack_factor = declare_parameter<double>("phase_opening_attack_factor", opening.phase.opening_attack_factor);
    opening.phase.visible_resource_steps = declare_parameter<int>("phase_visible_resource_steps", opening.phase.visible_resource_steps);
    opening.phase.resource_silence_ticks = declare_parameter<int>("phase_resource_silence_ticks", opening.phase.resource_silence_ticks);
    opening.phase.combat_opportunity_floor = declare_parameter<double>("phase_combat_opportunity_floor", opening.phase.combat_opportunity_floor);
    opening.phase.task_lookahead_weight = declare_parameter<double>("phase_task_lookahead_weight", opening.phase.task_lookahead_weight);
    for (auto item : {std::pair<const char*, double*>{"rollout_weight", &opening.rollout.weight},
                      {"rollout_budget_ms", &opening.rollout.budget_ms},
                      {"rollout_discount", &opening.rollout.discount},
                      {"rollout_worst_fraction", &opening.rollout.worst_fraction},
                      {"rollout_goal_progress", &opening.rollout.goal_progress},
                      {"rollout_override_margin", &opening.rollout.override_margin},
                      {"rollout_risk_scale", &opening.rollout.risk_scale}})
      *item.second = declare_parameter<double>(item.first, *item.second);
    for (auto item : {std::pair<const char*, int*>{"rollout_horizon", &opening.rollout.horizon},
                      {"rollout_beam_width", &opening.rollout.beam_width},
                      {"rollout_scenarios", &opening.rollout.scenarios},
                      {"rollout_activation_range", &opening.rollout.activation_range},
                      {"rollout_start_tick", &opening.rollout.start_tick}})
      *item.second = declare_parameter<int>(item.first, *item.second);

    for (auto item :
         {std::pair<const char*, double*>{"event_blast_risk", &opening.events.blast_risk},
          {"event_escape_weight", &opening.events.escape_weight},
          {"event_heal_value", &opening.events.heal_value},
          {"event_fortress_value", &opening.events.fortress_value},
          {"event_fortress_hold", &opening.events.fortress_hold},
          {"event_fortress_withdraw_priority", &opening.events.fortress_withdraw_priority},
          {"event_unrivaled_risk_multiplier", &opening.events.unrivaled_risk_multiplier},
          {"event_pepper_cost", &opening.events.pepper_cost},
          {"event_pepper_value", &opening.events.pepper_value},
          {"event_purity_defend_seen", &opening.events.purity_defend_seen},
          {"event_purity_defend_other", &opening.events.purity_defend_other},
          {"event_purity_defense_value", &opening.events.purity_defense_value},
          {"event_vulnerability_risk", &opening.events.vulnerability_risk}})
      *item.second = declare_parameter<double>(item.first, *item.second);
    opening.events.fortress_contest_enabled = declare_parameter<bool>(
        "event_fortress_contest_enabled", opening.events.fortress_contest_enabled);
    opening.events.fortress_race_margin = declare_parameter<int>(
        "event_fortress_race_margin", opening.events.fortress_race_margin);
    opening.events.fortress_min_enemy_stay = declare_parameter<int>(
        "event_fortress_min_enemy_stay", opening.events.fortress_min_enemy_stay);
    opening.events.fortress_preempt_ticks = declare_parameter<int>(
        "event_fortress_preempt_ticks", opening.events.fortress_preempt_ticks);
    opening.rollout.preserve_collection =
        declare_parameter<bool>("rollout_preserve_collection", opening.rollout.preserve_collection);
    opening.events.evacuation_buffer =
        declare_parameter<int>("event_evacuation_buffer", opening.events.evacuation_buffer);
    opening.opening_ticks = declare_parameter<int>("opening_ticks", opening.opening_ticks);
    opening.opening_resource_weight =
        declare_parameter<double>("opening_resource_weight", opening.opening_resource_weight);
    opening.resource_weight_end =
        declare_parameter<double>("resource_weight_end", opening.resource_weight_end);
    opening.opening_combat_weight =
        declare_parameter<double>("opening_combat_weight", opening.opening_combat_weight);
    opening.combat_weight_end =
        declare_parameter<double>("combat_weight_end", opening.combat_weight_end);
    opening.combat_ramp_ticks =
        declare_parameter<int>("combat_ramp_ticks", opening.combat_ramp_ticks);
    opening.opening_reach_weight =
        declare_parameter<double>("opening_reach_weight", opening.opening_reach_weight);
    opening.reach_ramp_ticks = declare_parameter<int>("reach_ramp_ticks", opening.reach_ramp_ticks);
    opening.reach_deadline_buffer =
        declare_parameter<int>("reach_deadline_buffer", opening.reach_deadline_buffer);
    opening.reach_deadline_weight =
        declare_parameter<double>("reach_deadline_weight", opening.reach_deadline_weight);
    opening.reach_finish_ticks =
        declare_parameter<int>("reach_finish_ticks", opening.reach_finish_ticks);
    opening.reach_attack_pause_ticks =
        declare_parameter<int>("reach_attack_pause_ticks", opening.reach_attack_pause_ticks);
    opening.reach_move_priority =
        declare_parameter<double>("reach_move_priority", opening.reach_move_priority);
    opening.reach_stalemate_ticks =
        declare_parameter<int>("reach_stalemate_ticks", opening.reach_stalemate_ticks);
    opening.reach_collision_retry_ticks =
        declare_parameter<int>("reach_collision_retry_ticks", opening.reach_collision_retry_ticks);
    opening.visible_resource_weight =
        declare_parameter<double>("visible_resource_weight", opening.visible_resource_weight);
    opening.adaptive_resources =
        declare_parameter<bool>("adaptive_resources", opening.adaptive_resources);
    for (auto item :
         {std::pair<const char*, double*>{"resource_need_power", &opening.resource_need_power},
          {"resource_coverage_power", &opening.resource_coverage_power},
          {"resource_depletion_power", &opening.resource_depletion_power},
          {"exploration_floor", &opening.exploration_floor},
          {"collection_mission_weight", &opening.collection_mission_weight},
          {"information_per_cell", &opening.information_per_cell},
          {"frontier_resource_weight", &opening.frontier_resource_weight},
          {"frontier_mission_weight", &opening.frontier_mission_weight},
          {"frontier_resource_half_life", &opening.frontier_resource_half_life},
          {"information_cap", &opening.information_cap},
          {"information_softness", &opening.information_softness},
          {"sweep_weight", &opening.sweep_weight},
          {"sweep_discount", &opening.sweep_discount},
          {"navigation_cycle_cost", &opening.navigation_cycle_cost},
          {"edge_information_bonus", &opening.edge_information_bonus},
          {"contested_resource_factor", &opening.contested_resource_factor},
          {"resource_age_ticks", &opening.resource_age_ticks}})
      *item.second = declare_parameter<double>(item.first, *item.second);
    opening.recent_enemy_search_weight =
        declare_parameter<double>("recent_enemy_search_weight", opening.recent_enemy_search_weight);
    opening.enemy_belief_tracking=declare_parameter<bool>("enemy_belief_tracking",opening.enemy_belief_tracking);
    opening.interference.modify_events=declare_parameter<bool>("postcap_modify_events",opening.interference.modify_events);
    for(auto item:{std::pair<const char*,double*>{"postcap_interference_weight",&opening.interference.weight},
        {"postcap_minimum_hp",&opening.interference.minimum_hp},
        {"postcap_reserve_hp",&opening.interference.reserve_hp},
        {"postcap_maximum_incoming",&opening.interference.maximum_incoming},
        {"postcap_reserve_damage",&opening.interference.reserve_damage},
        {"postcap_reserve_weight",&opening.interference.reserve_weight},
        {"postcap_heal_multiplier",&opening.interference.heal_multiplier}})
      *item.second=declare_parameter<double>(item.first,*item.second);
    opening.interference.maximum_detour=declare_parameter<int>("postcap_maximum_detour",opening.interference.maximum_detour);
    opening.sweep_ticks=declare_parameter<int>("sweep_ticks",opening.sweep_ticks);
    opening.sweep_horizon=declare_parameter<int>("sweep_horizon",opening.sweep_horizon);
    opening.sweep_beam=declare_parameter<int>("sweep_beam",opening.sweep_beam);
    opening.sweep_collect_only=declare_parameter<bool>("sweep_collect_only",opening.sweep_collect_only);
    for(auto item:{std::pair<const char*,double*>{"collection_search_weight",&opening.collection_search_weight},
        {"combat_clue_weight",&opening.combat_clue_weight},
        {"combat_clue_half_life",&opening.combat_clue_half_life},
        {"opening_depletion_weight",&opening.opening_depletion_weight},
        {"opening_depletion_floor",&opening.opening_depletion_floor},
        {"opening_depletion_half_life",&opening.opening_depletion_half_life},
        {"collection_search_future",&opening.collection_search_future},
        {"collection_search_discount",&opening.collection_search_discount},
        {"collection_search_risk",&opening.collection_search_risk},
        {"collection_search_budget_ms",&opening.collection_search_budget_ms}})
      *item.second=declare_parameter<double>(item.first,*item.second);
    for(auto item:{std::pair<const char*,int*>{"collection_search_ticks",&opening.collection_search_ticks},
        {"combat_clue_max_age",&opening.combat_clue_max_age},
        {"third_party_memory_ticks",&opening.third_party_memory_ticks},
        {"opening_depletion_ticks",&opening.opening_depletion_ticks},
        {"collection_search_horizon",&opening.collection_search_horizon},
        {"collection_search_depth",&opening.collection_search_depth},
        {"collection_search_beam",&opening.collection_search_beam},
        {"collection_search_targets",&opening.collection_search_targets},
        {"collection_search_resources",&opening.collection_search_resources}})
      *item.second=declare_parameter<int>(item.first,*item.second);
    opening.navigation_cycle_window=declare_parameter<int>("navigation_cycle_window",opening.navigation_cycle_window);
    opening.navigation_cycle_start_tick=declare_parameter<int>("navigation_cycle_start_tick",opening.navigation_cycle_start_tick);
    opening.navigation_cycle_cooldown=declare_parameter<int>("navigation_cycle_cooldown",opening.navigation_cycle_cooldown);
    opening.recent_enemy_half_life =
        declare_parameter<double>("recent_enemy_half_life", opening.recent_enemy_half_life);
    opening.recent_enemy_max_age =
        declare_parameter<int>("recent_enemy_max_age", opening.recent_enemy_max_age);
    opening.blocked_retry_ticks =
        declare_parameter<int>("blocked_retry_ticks", opening.blocked_retry_ticks);
    opening.blocked_retry_min_failures =
        declare_parameter<int>("blocked_retry_min_failures", opening.blocked_retry_min_failures);
    opening.resource_lookahead_weight =
        declare_parameter<double>("resource_lookahead_weight", opening.resource_lookahead_weight);
    opening.resource_lookahead_steps =
        declare_parameter<int>("resource_lookahead_steps", opening.resource_lookahead_steps);
    opening.resource_lookahead_limit =
        declare_parameter<int>("resource_lookahead_limit", opening.resource_lookahead_limit);
    opening.resource_lookahead_depth =
        declare_parameter<int>("resource_lookahead_depth", opening.resource_lookahead_depth);
    opening.route_information_weight =
        declare_parameter<double>("route_information_weight", opening.route_information_weight);
    opening.reach_weight_end =
        declare_parameter<double>("reach_weight_end", opening.reach_weight_end);
    opening.resource_mission_weight =
        declare_parameter<double>("resource_mission_weight", opening.resource_mission_weight);
    opening.resource_mission_weight_end = declare_parameter<double>(
        "resource_mission_weight_end", opening.resource_mission_weight_end);
    opening.resource_mission_ramp_ticks = declare_parameter<int>(
        "resource_mission_ramp_ticks", opening.resource_mission_ramp_ticks);
    opening.mission_potential =
        declare_parameter<double>("mission_potential", opening.mission_potential);
    opening.travel_offset = declare_parameter<double>("travel_offset", opening.travel_offset);
    opening.switch_margin = declare_parameter<double>("switch_margin", opening.switch_margin);
    opening.observation_start_tick =
        declare_parameter<int>("observation_start_tick", opening.observation_start_tick);
    opening.observation_ramp_ticks =
        declare_parameter<int>("observation_ramp_ticks", opening.observation_ramp_ticks);
    opening.observation_age_weight =
        declare_parameter<double>("observation_age_weight", opening.observation_age_weight);
    opening.observation_fresh_ticks =
        declare_parameter<int>("observation_fresh_ticks", opening.observation_fresh_ticks);
    opening.observation_stale_ticks =
        declare_parameter<int>("observation_stale_ticks", opening.observation_stale_ticks);
    opening.chase_shared_progress =
        declare_parameter<bool>("chase_shared_progress", opening.chase_shared_progress);
    opening.route_alternatives_weight =
        declare_parameter<double>("route_alternatives_weight", opening.route_alternatives_weight);
    opening.chase_stall_ticks =
        declare_parameter<int>("chase_stall_ticks", opening.chase_stall_ticks);
    opening.chase_cooldown_ticks =
        declare_parameter<int>("chase_cooldown_ticks", opening.chase_cooldown_ticks);
    opening.collection_mode = declare_parameter<int>("collection_mode", opening.collection_mode);
    opening.joint_min_resources =
        declare_parameter<int>("joint_min_resources", opening.joint_min_resources);
    opening.joint_ticks = declare_parameter<int>("joint_ticks", opening.joint_ticks);
    opening.joint_resource_limit =
        declare_parameter<int>("joint_resource_limit", opening.joint_resource_limit);
    opening.joint_horizon = declare_parameter<int>("joint_horizon", opening.joint_horizon);
    opening.joint_future_weight =
        declare_parameter<double>("joint_future_weight", opening.joint_future_weight);
    opening.joint_observation_offset =
        declare_parameter<double>("joint_observation_offset", opening.joint_observation_offset);
    opening.joint_unknown_weight =
        declare_parameter<double>("joint_unknown_weight", opening.joint_unknown_weight);
    opening.joint_linear_weights =
        declare_parameter<bool>("joint_linear_weights", opening.joint_linear_weights);
    opening.astar_resource_competition =
        declare_parameter<bool>("astar_resource_competition", opening.astar_resource_competition);
    for (auto item : {std::pair<const char*, double*>{"joint_future_weight_end",
                                                      &opening.joint_future_weight_end},
                      {"joint_observation_offset_end", &opening.joint_observation_offset_end},
                      {"competition_softness", &opening.competition_softness},
                      {"competition_weight", &opening.competition_weight},
                      {"competition_weight_end", &opening.competition_weight_end}})
      *item.second = declare_parameter<double>(item.first, *item.second);
    for (auto item : {std::pair<const char*, double*>{"opponent_motion_weight",
                                                      &opening.opponent_motion_weight},
                      {"endgame_preserve_weight", &opening.endgame_preserve_weight},
                      {"low_hp_risk_weight", &opening.low_hp_risk_weight},
                      {"retreat_damage_fraction", &opening.retreat_damage_fraction},
                      {"attack_probability", &opening.attack_probability},
                      {"hp_loss_cost", &opening.hp_loss_cost},
                      {"death_cost", &opening.death_cost}})
      *item.second = declare_parameter<double>(item.first, *item.second);
    opening.retreat_buffer_ticks =
        declare_parameter<int>("retreat_buffer_ticks", opening.retreat_buffer_ticks);
    opening.retreat_window_ticks =
        declare_parameter<int>("retreat_window_ticks", opening.retreat_window_ticks);
    for (auto item :
         {std::pair<const char*, double*>{"intent_chase_weight", &opening.intent_chase_weight},
          {"intent_risk_weight", &opening.intent_risk_weight},
          {"third_party_risk_discount", &opening.third_party_risk_discount},
          {"third_party_attack_bonus", &opening.third_party_attack_bonus},
          {"intent_heading_weight", &opening.intent_heading_weight},
          {"future_exposure_cost", &opening.future_exposure_cost}})
      *item.second = declare_parameter<double>(item.first, *item.second);
    opening.intent_memory_ticks =
        declare_parameter<int>("intent_memory_ticks", opening.intent_memory_ticks);
    // 大批量联合调参也需要覆盖旧风险/巡查参数；读取和日志使用同一实际值。
    for (auto item :
         {std::pair<const char*, double*>{"remembered_risk", &opening.remembered_risk},
          {"route_risk", &opening.route_risk},
          {"attack_damage_estimate", &opening.attack_damage_estimate},
          {"kill_reward_estimate", &opening.kill_reward_estimate},
          {"patrol_value", &opening.patrol_value},
          {"patrol_stale_ticks", &opening.patrol_stale_ticks},
          {"patrol_enemy_hint", &opening.patrol_enemy_hint}})
      *item.second = declare_parameter<double>(item.first, *item.second);
    opening.enemy_memory_ticks =
        declare_parameter<int>("enemy_memory_ticks", opening.enemy_memory_ticks);
    opening.patrol_min_age = declare_parameter<int>("patrol_min_age", opening.patrol_min_age);
    stage2_strategy_ = Stage2Strategy(opening);
    if(opening.neural.enabled)
      RCLCPP_INFO(get_logger(),"NEURAL enabled: model architecture and parameters verified, weight=%.4f, model=%s",
          opening.neural.weight,opening.neural.model_file.c_str());
  }
  const auto resource_order = declare_parameter<std::string>(
      "resource_order", resource_order_name(kDefaultResourceOrdering));
  const planning::ResourceWeights weights{
      declare_parameter<double>("k_edge", kDefaultEdgeWeight),
      declare_parameter<double>("k_value", kDefaultValueWeight)};
  action_ticks_ = declare_parameter<std::int64_t>("action_ticks", is_stage2_mode() ? 500 : 100);
  const auto log_path = declare_parameter<std::string>("log_file", "");
  if (team_.size() != 1 || std::string("ABCD").find(team_) == std::string::npos) {
    throw std::runtime_error("team must be A, B, C or D");
  }
  if (mode_ != "wait" && mode_ != "probe" && mode_ != "resource" && !is_stage2_mode()) {
    throw std::runtime_error("mode must be wait, probe, resource, stage2 or stage2_planning");
  }
  if (resource_order != "tour" && resource_order != "nearest" && resource_order != "weighted" &&
      resource_order != "joint")
    throw std::runtime_error("resource_order must be tour, nearest, weighted or joint");
  const auto ordering = resource_order == "tour"      ? ResourceOrdering::Tour
                        : resource_order == "nearest" ? ResourceOrdering::Nearest
                        : resource_order == "joint"   ? ResourceOrdering::Joint
                                                      : ResourceOrdering::Weighted;
  resource_strategy_ = ResourceStrategy(ordering, weights);
  if (action_ticks_ <= 0)
    throw std::runtime_error("action_ticks must be positive");
  if (name.empty() || name.size() > 96 || name.find_first_of("\r\n") != std::string::npos) {
    throw std::runtime_error("team_name must be 1-96 UTF-8 bytes, without line breaks");
  }
  if (!log_path.empty()) {
    log_.open(log_path, std::ios::app);
    if (!log_)
      throw std::runtime_error("cannot open log_file: " + log_path);
  }

  // 感知和动作使用可靠传输、非历史保留、仅保存最新一条的队列。
  // 可靠传输不保证动作一定赶上裁判截止时间，回调仍必须及时完成。
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
  const auto prefix = "/team_" + team_;
  action_pub_ = create_publisher<std_msgs::msg::String>(prefix + "/action", qos);
  // 注册采用TRANSIENT_LOCAL，让后接入的订阅者可收到保留的队名。
  registration_pub_ = create_publisher<std_msgs::msg::String>(
      prefix + "/registration", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  // Lambda捕获this，将每条ROS消息的data交给本节点的主回调。
  perception_sub_ = create_subscription<std_msgs::msg::String>(
      prefix + "/perception", qos,
      [this](const std_msgs::msg::String::SharedPtr msg) { on_perception(msg->data); });

  std_msgs::msg::String registration;
  registration.data = Json{{"team", team_}, {"name", name}}.dump();
  registration_pub_->publish(registration);
  record({{"event", "ready"},
          {"team", team_},
          {"mode", mode_},
          {"resource_order", resource_order},
          {"k_edge", weights.edge},
          {"k_value", weights.value},
          {"k_distance", weights.distance},
          {"unknown_cell_value", weights.unknown_cell_value},
          {"exploration_fade_ticks", weights.exploration_fade_ticks},
          {"unknown_edge_bonus", weights.unknown_edge_bonus},
          {"unknown_edge_band", weights.unknown_edge_band},
          {"peek_reward", weights.peek_reward},
          {"peek_detour_limit", weights.peek_detour_limit},
          {"peek_patch_limit", weights.peek_patch_limit}});
  RCLCPP_INFO(get_logger(), "READY team=%s mode=%s resource_order=%s perception=%s/perception",
              team_.c_str(), mode_.c_str(), resource_order.c_str(), prefix.c_str());
  if (is_stage2_mode()) {
    ai_result_sub_ = create_subscription<std_msgs::msg::String>(
        "/rmtc/team_" + team_ + "/ai_result",
        rclcpp::QoS(rclcpp::KeepLast(4)).reliable().transient_local(),
        [this](const std_msgs::msg::String::SharedPtr msg) { on_ai_result(msg->data); });
    record(
        {{"event", "stage2_config"},
         {"mode",mode_},{"planning_enabled",get_parameter("planning_enabled").as_bool()},
         {"planning_horizon",get_parameter("planning_horizon").as_int()},
         {"planning_beam_width",get_parameter("planning_beam_width").as_int()},
         {"planning_max_nodes",get_parameter("planning_max_nodes").as_int()},
         {"planning_budget_ms",get_parameter("planning_budget_ms").as_double()},
         {"neural_enabled",get_parameter("neural_enabled").as_bool()},
         {"neural_collect",get_parameter("neural_collect").as_bool()},
         {"neural_weight",get_parameter("neural_weight").as_double()},
         {"neural_model_file",get_parameter("neural_model_file").as_string()},
         {"neural_override_margin",get_parameter("neural_override_margin").as_double()},
         {"duel_stalemate_enabled",get_parameter("duel_stalemate_enabled").as_bool()},
         {"duel_stalemate_ticks",get_parameter("duel_stalemate_ticks").as_int()},
         {"duel_stalemate_start_tick",get_parameter("duel_stalemate_start_tick").as_int()},
         {"duel_stalemate_maximum_hp",get_parameter("duel_stalemate_maximum_hp").as_double()},
         {"duel_stalemate_death_risk_factor",get_parameter("duel_stalemate_death_risk_factor").as_double()},
         {"collection_transfer_enabled",get_parameter("collection_transfer_enabled").as_bool()},
         {"collection_transfer_hold_ticks",get_parameter("collection_transfer_hold_ticks").as_int()},
         {"collection_transfer_fade_ticks",get_parameter("collection_transfer_fade_ticks").as_int()},
         {"collection_transfer_shape",get_parameter("collection_transfer_shape").as_double()},
         {"collection_transfer_resource_weight",get_parameter("collection_transfer_resource_weight").as_double()},
         {"collection_transfer_explore_peak",get_parameter("collection_transfer_explore_peak").as_double()},
         {"collection_transfer_explore_half_ticks",get_parameter("collection_transfer_explore_half_ticks").as_double()},
         {"collection_transfer_silence_half_ticks",get_parameter("collection_transfer_silence_half_ticks").as_double()},
         {"collection_transfer_contested_factor",get_parameter("collection_transfer_contested_factor").as_double()},
         {"collection_transfer_competition_softness",get_parameter("collection_transfer_competition_softness").as_double()},
         {"collection_transfer_travel_offset",get_parameter("collection_transfer_travel_offset").as_double()},
         {"collection_transfer_resource_floor",get_parameter("collection_transfer_resource_floor").as_double()},
         {"opening_collection_ticks",get_parameter("opening_collection_ticks").as_int()},
         {"battle_curve_enabled",get_parameter("battle_curve_enabled").as_bool()},
         {"battle_curve_protect_resources",get_parameter("battle_curve_protect_resources").as_bool()},
         {"battle_curve_start_tick",get_parameter("battle_curve_start_tick").as_int()},
         {"battle_curve_ramp_ticks",get_parameter("battle_curve_ramp_ticks").as_int()},
         {"battle_curve_shape",get_parameter("battle_curve_shape").as_double()},
         {"battle_curve_damage_need_power",get_parameter("battle_curve_damage_need_power").as_double()},
         {"battle_curve_combat_end",get_parameter("battle_curve_combat_end").as_double()},
         {"battle_curve_search_end",get_parameter("battle_curve_search_end").as_double()},
         {"battle_curve_hp_risk_end",get_parameter("battle_curve_hp_risk_end").as_double()},
         {"battle_curve_future_risk_end",get_parameter("battle_curve_future_risk_end").as_double()},
         {"battle_curve_route_risk_end",get_parameter("battle_curve_route_risk_end").as_double()},
         {"battle_curve_third_party_end",get_parameter("battle_curve_third_party_end").as_double()},
         {"temporal_policy_enabled",get_parameter("temporal_policy_enabled").as_bool()},
         {"temporal_transition_ticks",get_parameter("temporal_transition_ticks").as_int()},
         {"temporal_shape",get_parameter("temporal_shape").as_double()},
         {"temporal_resource_age_end",get_parameter("temporal_resource_age_end").as_double()},
         {"temporal_competition_end",get_parameter("temporal_competition_end").as_double()},
         {"temporal_lookahead_end",get_parameter("temporal_lookahead_end").as_double()},
         {"temporal_route_information_end",get_parameter("temporal_route_information_end").as_double()},
         {"temporal_third_party_attack_end",get_parameter("temporal_third_party_attack_end").as_double()},
         {"temporal_third_party_risk_end",get_parameter("temporal_third_party_risk_end").as_double()},
         {"acquisition_curve_enabled",get_parameter("acquisition_curve_enabled").as_bool()},
         {"acquisition_resource_weight",get_parameter("acquisition_resource_weight").as_double()},
         {"acquisition_explore_peak",get_parameter("acquisition_explore_peak").as_double()},
         {"acquisition_explore_floor",get_parameter("acquisition_explore_floor").as_double()},
         {"acquisition_explore_half_ticks",get_parameter("acquisition_explore_half_ticks").as_double()},
         {"acquisition_explore_shape",get_parameter("acquisition_explore_shape").as_double()},
         {"acquisition_silence_half_ticks",get_parameter("acquisition_silence_half_ticks").as_double()},
         {"acquisition_silence_grace_ticks",get_parameter("acquisition_silence_grace_ticks").as_int()},
         {"resource_race_enabled",get_parameter("resource_race_enabled").as_bool()},
         {"resource_race_coverage",get_parameter("resource_race_coverage").as_bool()},
         {"resource_race_limit",get_parameter("resource_race_limit").as_int()},
         {"resource_race_horizon",get_parameter("resource_race_horizon").as_int()},
         {"resource_race_deadline_slack",get_parameter("resource_race_deadline_slack").as_int()},
         {"resource_race_explore_ticks",get_parameter("resource_race_explore_ticks").as_int()},
         {"resource_race_distance_cost",get_parameter("resource_race_distance_cost").as_double()},
         {"resource_race_move_priority",get_parameter("resource_race_move_priority").as_double()},
         {"resource_race_budget_ms",get_parameter("resource_race_budget_ms").as_double()},
         {"resource_race_coverage_offset",get_parameter("resource_race_coverage_offset").as_double()},
         {"combat_clue_weight",get_parameter("combat_clue_weight").as_double()},
         {"combat_clue_half_life",get_parameter("combat_clue_half_life").as_double()},
         {"combat_clue_max_age",get_parameter("combat_clue_max_age").as_int()},
         {"third_party_memory_ticks",get_parameter("third_party_memory_ticks").as_int()},
         {"opening_depletion_weight",get_parameter("opening_depletion_weight").as_double()},
         {"opening_depletion_floor",get_parameter("opening_depletion_floor").as_double()},
         {"opening_depletion_half_life",get_parameter("opening_depletion_half_life").as_double()},
         {"opening_depletion_ticks",get_parameter("opening_depletion_ticks").as_int()},
         {"collection_search_weight",get_parameter("collection_search_weight").as_double()},
         {"collection_search_future",get_parameter("collection_search_future").as_double()},
         {"collection_search_discount",get_parameter("collection_search_discount").as_double()},
         {"collection_search_risk",get_parameter("collection_search_risk").as_double()},
         {"collection_search_budget_ms",get_parameter("collection_search_budget_ms").as_double()},
         {"collection_search_ticks",get_parameter("collection_search_ticks").as_int()},
         {"collection_search_horizon",get_parameter("collection_search_horizon").as_int()},
         {"collection_search_depth",get_parameter("collection_search_depth").as_int()},
         {"collection_search_beam",get_parameter("collection_search_beam").as_int()},
         {"collection_search_targets",get_parameter("collection_search_targets").as_int()},
         {"collection_search_resources",get_parameter("collection_search_resources").as_int()},
         {"frontier_resource_weight", get_parameter("frontier_resource_weight").as_double()},
         {"frontier_mission_weight", get_parameter("frontier_mission_weight").as_double()},
         {"frontier_resource_half_life", get_parameter("frontier_resource_half_life").as_double()},
         {"rollout_weight", get_parameter("rollout_weight").as_double()},
         {"rollout_budget_ms", get_parameter("rollout_budget_ms").as_double()},
         {"rollout_discount", get_parameter("rollout_discount").as_double()},
         {"rollout_worst_fraction", get_parameter("rollout_worst_fraction").as_double()},
         {"rollout_goal_progress", get_parameter("rollout_goal_progress").as_double()},
         {"rollout_override_margin", get_parameter("rollout_override_margin").as_double()},
         {"rollout_horizon", get_parameter("rollout_horizon").as_int()},
         {"rollout_beam_width", get_parameter("rollout_beam_width").as_int()},
         {"rollout_scenarios", get_parameter("rollout_scenarios").as_int()},
         {"rollout_activation_range", get_parameter("rollout_activation_range").as_int()},
         {"rollout_start_tick", get_parameter("rollout_start_tick").as_int()},
         {"rollout_risk_scale", get_parameter("rollout_risk_scale").as_double()},
         {"rollout_preserve_collection", get_parameter("rollout_preserve_collection").as_bool()},
         {"reach_ramp_ticks", get_parameter("reach_ramp_ticks").as_int()},
         {"reach_deadline_buffer", get_parameter("reach_deadline_buffer").as_int()},
         {"reach_deadline_weight", get_parameter("reach_deadline_weight").as_double()},
         {"reach_finish_ticks", get_parameter("reach_finish_ticks").as_int()},
         {"reach_attack_pause_ticks", get_parameter("reach_attack_pause_ticks").as_int()},
         {"reach_move_priority", get_parameter("reach_move_priority").as_double()},
         {"reach_stalemate_ticks", get_parameter("reach_stalemate_ticks").as_int()},
         {"reach_collision_retry_ticks", get_parameter("reach_collision_retry_ticks").as_int()},
         {"visible_resource_weight", get_parameter("visible_resource_weight").as_double()},
         {"combat_weight_end", get_parameter("combat_weight_end").as_double()},
         {"combat_ramp_ticks", get_parameter("combat_ramp_ticks").as_int()},
         {"event_blast_risk", get_parameter("event_blast_risk").as_double()},
         {"event_escape_weight", get_parameter("event_escape_weight").as_double()},
         {"event_heal_value", get_parameter("event_heal_value").as_double()},
         {"event_fortress_value", get_parameter("event_fortress_value").as_double()},
         {"event_fortress_hold", get_parameter("event_fortress_hold").as_double()},
         {"event_fortress_contest_enabled", get_parameter("event_fortress_contest_enabled").as_bool()},
         {"event_fortress_race_margin", get_parameter("event_fortress_race_margin").as_int()},
         {"event_fortress_min_enemy_stay", get_parameter("event_fortress_min_enemy_stay").as_int()},
         {"event_fortress_preempt_ticks", get_parameter("event_fortress_preempt_ticks").as_int()},
         {"event_fortress_withdraw_priority", get_parameter("event_fortress_withdraw_priority").as_double()},
         {"event_unrivaled_risk_multiplier", get_parameter("event_unrivaled_risk_multiplier").as_double()},
         {"event_pepper_cost", get_parameter("event_pepper_cost").as_double()},
         {"event_pepper_value", get_parameter("event_pepper_value").as_double()},
         {"event_purity_defend_seen", get_parameter("event_purity_defend_seen").as_double()},
         {"event_purity_defend_other", get_parameter("event_purity_defend_other").as_double()},
         {"event_purity_defense_value", get_parameter("event_purity_defense_value").as_double()},
         {"event_vulnerability_risk", get_parameter("event_vulnerability_risk").as_double()},
         {"event_evacuation_buffer", get_parameter("event_evacuation_buffer").as_int()},
         {"recent_enemy_search_weight", get_parameter("recent_enemy_search_weight").as_double()},
         {"recent_enemy_half_life", get_parameter("recent_enemy_half_life").as_double()},
         {"recent_enemy_max_age", get_parameter("recent_enemy_max_age").as_int()},
         {"blocked_retry_ticks", get_parameter("blocked_retry_ticks").as_int()},
         {"purpose_enabled",get_parameter("purpose_enabled").as_bool()},
         {"purpose_start_tick",get_parameter("purpose_start_tick").as_int()},
         {"purpose_protect_resources",get_parameter("purpose_protect_resources").as_bool()},
         {"purpose_resource_quiet_ticks",get_parameter("purpose_resource_quiet_ticks").as_int()},
         {"purpose_intercept_horizon",get_parameter("purpose_intercept_horizon").as_int()},
         {"purpose_chase_progress_ticks",get_parameter("purpose_chase_progress_ticks").as_int()},
         {"purpose_resource_floor",get_parameter("purpose_resource_floor").as_double()},
         {"purpose_battle_floor",get_parameter("purpose_battle_floor").as_double()},
         {"purpose_intercept_weight",get_parameter("purpose_intercept_weight").as_double()},
         {"purpose_heading_weight",get_parameter("purpose_heading_weight").as_double()},
         {"purpose_escape_weight",get_parameter("purpose_escape_weight").as_double()},
         {"purpose_patrol_visible_scale",get_parameter("purpose_patrol_visible_scale").as_double()},
         {"phase_enabled", get_parameter("phase_enabled").as_bool()},
         {"phase_resource_ticks", get_parameter("phase_resource_ticks").as_int()},
         {"phase_chase_steps", get_parameter("phase_chase_steps").as_int()},
         {"phase_task_detour", get_parameter("phase_task_detour").as_int()},
         {"phase_opportunity_floor", get_parameter("phase_opportunity_floor").as_double()},
         {"phase_opening_attack_factor", get_parameter("phase_opening_attack_factor").as_double()},
         {"phase_visible_resource_steps", get_parameter("phase_visible_resource_steps").as_int()},
         {"phase_resource_silence_ticks", get_parameter("phase_resource_silence_ticks").as_int()},
         {"phase_combat_opportunity_floor", get_parameter("phase_combat_opportunity_floor").as_double()},
         {"phase_task_lookahead_weight", get_parameter("phase_task_lookahead_weight").as_double()},
         {"blocked_retry_min_failures", get_parameter("blocked_retry_min_failures").as_int()},
         {"resource_lookahead_weight", get_parameter("resource_lookahead_weight").as_double()},
         {"resource_lookahead_steps", get_parameter("resource_lookahead_steps").as_int()},
         {"resource_lookahead_limit", get_parameter("resource_lookahead_limit").as_int()},
         {"resource_lookahead_depth", get_parameter("resource_lookahead_depth").as_int()},
         {"route_information_weight", get_parameter("route_information_weight").as_double()},
         {"opening_ticks", get_parameter("opening_ticks").as_int()},
         {"opening_resource_weight", get_parameter("opening_resource_weight").as_double()},
         {"resource_weight_end", get_parameter("resource_weight_end").as_double()},
         {"opening_combat_weight", get_parameter("opening_combat_weight").as_double()},
         {"opening_reach_weight", get_parameter("opening_reach_weight").as_double()},
         {"reach_weight_end", get_parameter("reach_weight_end").as_double()},
         {"resource_mission_weight", get_parameter("resource_mission_weight").as_double()},
         {"resource_mission_weight_end", get_parameter("resource_mission_weight_end").as_double()},
         {"resource_mission_ramp_ticks", get_parameter("resource_mission_ramp_ticks").as_int()},
         {"switch_margin", get_parameter("switch_margin").as_double()},
         {"separate_goal_switch_bonus", get_parameter("separate_goal_switch_bonus").as_bool()},
         {"collection_mode", get_parameter("collection_mode").as_int()},
         {"joint_ticks", get_parameter("joint_ticks").as_int()},
         {"joint_horizon", get_parameter("joint_horizon").as_int()},
         {"joint_resource_limit", get_parameter("joint_resource_limit").as_int()},
         {"joint_unknown_weight", get_parameter("joint_unknown_weight").as_double()},
         {"joint_future_weight", get_parameter("joint_future_weight").as_double()},
         {"intent_chase_weight", get_parameter("intent_chase_weight").as_double()},
         {"intent_risk_weight", get_parameter("intent_risk_weight").as_double()},
         {"third_party_risk_discount", get_parameter("third_party_risk_discount").as_double()},
         {"third_party_attack_bonus", get_parameter("third_party_attack_bonus").as_double()},
         {"intent_heading_weight", get_parameter("intent_heading_weight").as_double()},
         {"intent_memory_ticks", get_parameter("intent_memory_ticks").as_int()},
         {"future_exposure_cost", get_parameter("future_exposure_cost").as_double()},
         {"attack_probability", get_parameter("attack_probability").as_double()},
         {"hp_loss_cost", get_parameter("hp_loss_cost").as_double()},
         {"death_cost", get_parameter("death_cost").as_double()},
         {"opponent_motion_weight", get_parameter("opponent_motion_weight").as_double()},
         {"endgame_preserve_weight", get_parameter("endgame_preserve_weight").as_double()},
         {"low_hp_risk_weight", get_parameter("low_hp_risk_weight").as_double()},
         {"retreat_damage_fraction", get_parameter("retreat_damage_fraction").as_double()},
         {"retreat_buffer_ticks", get_parameter("retreat_buffer_ticks").as_int()},
         {"retreat_window_ticks", get_parameter("retreat_window_ticks").as_int()},
         {"joint_min_resources", get_parameter("joint_min_resources").as_int()},
         {"joint_observation_offset", get_parameter("joint_observation_offset").as_double()},
         {"joint_linear_weights", get_parameter("joint_linear_weights").as_bool()},
         {"joint_future_weight_end", get_parameter("joint_future_weight_end").as_double()},
         {"joint_observation_offset_end",
          get_parameter("joint_observation_offset_end").as_double()},
         {"astar_resource_competition", get_parameter("astar_resource_competition").as_bool()},
         {"competition_softness", get_parameter("competition_softness").as_double()},
         {"competition_weight", get_parameter("competition_weight").as_double()},
         {"competition_weight_end", get_parameter("competition_weight_end").as_double()},
         {"observation_age_weight", get_parameter("observation_age_weight").as_double()},
         {"observation_start_tick", get_parameter("observation_start_tick").as_int()},
         {"observation_ramp_ticks", get_parameter("observation_ramp_ticks").as_int()},
         {"observation_fresh_ticks", get_parameter("observation_fresh_ticks").as_int()},
         {"observation_stale_ticks", get_parameter("observation_stale_ticks").as_int()},
         {"chase_shared_progress", get_parameter("chase_shared_progress").as_bool()},
         {"route_alternatives_weight", get_parameter("route_alternatives_weight").as_double()},
         {"chase_stall_ticks", get_parameter("chase_stall_ticks").as_int()},
         {"chase_cooldown_ticks", get_parameter("chase_cooldown_ticks").as_int()},
         {"adaptive_resources", get_parameter("adaptive_resources").as_bool()},
         {"resource_need_power", get_parameter("resource_need_power").as_double()},
         {"resource_coverage_power", get_parameter("resource_coverage_power").as_double()},
         {"resource_depletion_power", get_parameter("resource_depletion_power").as_double()},
         {"exploration_floor", get_parameter("exploration_floor").as_double()},
         {"collection_mission_weight", get_parameter("collection_mission_weight").as_double()},
         {"contested_resource_factor", get_parameter("contested_resource_factor").as_double()},
         {"resource_age_ticks", get_parameter("resource_age_ticks").as_double()},
         {"action_ticks", action_ticks_},
         {"information_per_cell", get_parameter("information_per_cell").as_double()},
         {"information_cap", get_parameter("information_cap").as_double()},
         {"information_softness", get_parameter("information_softness").as_double()},
         {"sweep_weight", get_parameter("sweep_weight").as_double()},
         {"sweep_ticks", get_parameter("sweep_ticks").as_int()},
         {"sweep_horizon", get_parameter("sweep_horizon").as_int()},
         {"sweep_beam", get_parameter("sweep_beam").as_int()},
         {"sweep_discount", get_parameter("sweep_discount").as_double()},
         {"sweep_collect_only", get_parameter("sweep_collect_only").as_bool()},
         {"enemy_belief_tracking", get_parameter("enemy_belief_tracking").as_bool()},
         {"postcap_interference_weight", get_parameter("postcap_interference_weight").as_double()},
         {"postcap_modify_events", get_parameter("postcap_modify_events").as_bool()},
         {"postcap_minimum_hp", get_parameter("postcap_minimum_hp").as_double()},
         {"postcap_reserve_hp", get_parameter("postcap_reserve_hp").as_double()},
         {"postcap_maximum_incoming", get_parameter("postcap_maximum_incoming").as_double()},
         {"postcap_reserve_damage", get_parameter("postcap_reserve_damage").as_double()},
         {"postcap_reserve_weight", get_parameter("postcap_reserve_weight").as_double()},
         {"postcap_heal_multiplier", get_parameter("postcap_heal_multiplier").as_double()},
         {"postcap_maximum_detour", get_parameter("postcap_maximum_detour").as_int()},
         {"navigation_cycle_window", get_parameter("navigation_cycle_window").as_int()},
         {"navigation_cycle_start_tick", get_parameter("navigation_cycle_start_tick").as_int()},
         {"navigation_cycle_cooldown", get_parameter("navigation_cycle_cooldown").as_int()},
         {"navigation_cycle_cost", get_parameter("navigation_cycle_cost").as_double()},
         {"edge_information_bonus", get_parameter("edge_information_bonus").as_double()},
         {"travel_offset", get_parameter("travel_offset").as_double()},
         {"route_risk", get_parameter("route_risk").as_double()},
         {"remembered_risk", get_parameter("remembered_risk").as_double()},
         {"enemy_memory_ticks", get_parameter("enemy_memory_ticks").as_int()},
         {"attack_damage_estimate", get_parameter("attack_damage_estimate").as_double()},
         {"kill_reward_estimate", get_parameter("kill_reward_estimate").as_double()},
         {"patrol_value", get_parameter("patrol_value").as_double()},
         {"patrol_stale_ticks", get_parameter("patrol_stale_ticks").as_double()},
         {"patrol_min_age", get_parameter("patrol_min_age").as_int()},
         {"patrol_enemy_hint", get_parameter("patrol_enemy_hint").as_double()},
         {"mission_potential", get_parameter("mission_potential").as_double()},
         {"ai_result_topic", "/rmtc/team_" + team_ + "/ai_result"}});
    RCLCPP_INFO(get_logger(),
                "STAGE2 resource/mission/combat/explore baseline; ticks=%ld; "
                "AI handled by independent ai_node process",
                static_cast<long>(action_ticks_));
    return;  // 第二阶段不打印容易混淆的第一阶段资源调参信息。
  }
  // 终端直接显示真正传给策略的参数，便于发现编译目录错误、旧进程或启动参数覆盖。
  if (ordering != ResourceOrdering::Joint) {
    RCLCPP_INFO(get_logger(),
                "CONFIG distance=%.6g edge=%.6g value=%.6g unknown_cell=%.6g fade_ticks=%ld "
                "unknown_edge_bonus=%.6g unknown_edge_band=%d",
                weights.distance, weights.edge, weights.value, weights.unknown_cell_value,
                static_cast<long>(weights.exploration_fade_ticks), weights.unknown_edge_bonus,
                weights.unknown_edge_band);
    RCLCPP_INFO(get_logger(), "CONFIG peek_reward=%.6g peek_detour_limit=%d peek_patch_limit=%d",
                weights.peek_reward, weights.peek_detour_limit, weights.peek_patch_limit);
  }
  if (ordering == ResourceOrdering::Joint) {
    const planning::CoverageWeights joint;
    record({{"event", "joint_config"},
            {"insertion_offset", joint.insertion_offset},
            {"gain_exponent", joint.gain_exponent},
            {"expected_resources", joint.expected_resources},
            {"discovery_urgency", joint.discovery_urgency},
            {"prefer_information_paths", joint.prefer_information_paths},
            {"path_potential_exponent", joint.path_potential_exponent},
            {"path_edge_bonus", joint.path_edge_bonus},
            {"path_edge_band", joint.path_edge_band},
            {"future_resource_weight", joint.future_resource_weight}});
    RCLCPP_INFO(get_logger(),
                "JOINT CONFIG insertion=%.6g gain_exponent=%.6g urgency=%.6g information_paths=%s "
                "future_resource=%.6g",
                joint.insertion_offset, joint.gain_exponent, joint.discovery_urgency,
                joint.prefer_information_paths ? "true" : "false", joint.future_resource_weight);
  }
}

/**
 * @brief 汇总当前进程生命周期内的接入统计，并输出到日志和终端。
 *
 * frames是处理的新帧数量，sent包含新动作及缓存重发；receipts只统计
 * 找到本地提交记录且动作类型匹配的回执，moves只统计moved=true。
 * 统计不会在换场时归零，因此一次进程可以包含多局比赛的数据。
 * process_runtime_ms包含等待感知的时间；第二阶段另列策略计算均值/P95/最大值，
 * 计算口径含完整感知解析与策略，不含日志输出、ROS传输和裁判等待。
 */
void ConnectionNode::summary() {
  Json result{{"event", "summary"},
              {"frames", frames_},
              {"sent", sent_},
              {"matched_receipts", receipts_},
              {"successful_moves", moves_},
              {"parse_errors", parse_errors_},
              {"receipt_errors", receipt_errors_},
              {"sessions", sessions_}};
  result["process_runtime_ms"] =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - run_started_)
          .count();
  if (!stage2_decision_us_.empty()) {
    auto sorted = stage2_decision_us_;
    std::sort(sorted.begin(), sorted.end());
    result["stage2_decision_ms"] = {
        {"count", sorted.size()},
        {"mean", std::accumulate(sorted.begin(), sorted.end(), 0.0) / sorted.size() / 1000.0},
        {"p95", sorted[static_cast<std::size_t>(0.95 * (sorted.size() - 1))] / 1000.0},
        {"max", sorted.back() / 1000.0}};
  }
  record(result);
  RCLCPP_INFO(get_logger(), "SUMMARY %s", result.dump().c_str());
}

/**
 * @brief 为一条事件记录补充时间和场次，并追加到可选JSONL文件。
 * @param value 记录对象，通常含event以及Tick、动作或错误信息；按值传入，
 *              本函数补充字段不会修改调用方持有的JSON。
 *
 * time_ns为本机ROS时钟的纳秒值，用于排查消息顺序，不代表比赛Tick。
 * 场次尚未确定时不添加session_id；未配置log_file时不写自定义文件。
 * 每条记录独占一行并立即flush，便于运行中观察；不检查写入失败状态。
 */
void ConnectionNode::record(Json value) {
  value["time_ns"] = get_clock()->now().nanoseconds();
  if (!session_.empty())
    value["session_id"] = session_;
  if (log_) {
    log_ << value.dump() << '\n';
    log_.flush();
  }
}

/**
 * @brief 把准备好的动作JSON序列化并发布到本队action话题。
 * @param action 已绑定当前tick/session_id的动作JSON，调用方负责合法性。
 * @param replay true表示重复感知时重发缓存；false表示本Tick的新提交。
 *
 * dump()生成JSON文本后放入msg.data；发布只表示交给ROS传输，
 * 不代表裁判已经执行。执行结果要由后续inspect_receipt核对。
 * 每次调用都增加sent_并记录tx/replay，不在这里推进策略状态。
 */
void ConnectionNode::publish(const Json& action, bool replay) {
  std_msgs::msg::String msg;
  msg.data = action.dump();
  action_pub_->publish(msg);
  ++sent_;
  record({{"event", replay ? "replay" : "tx"}, {"action", action}});
}

/**
 * @brief 判断感知所属场次是否可接收，并在进入新场次时重置局内状态。
 * @param frame 已经通过基础解析的感知；使用session_id及当前定位信息。
 * @return 当前或从未见过的新场次返回true，已经离开的旧场次返回false。
 *
 * 换场会清空待核对动作、缓存动作、Tick进度和策略记忆，
 * 防止上一局动作或策略残留进入新局。旧场次ID存入retired_sessions_，
 * 拒绝其迟到消息。累计统计不清空；不自行假设新局一定从Tick0接入。
 */
bool ConnectionNode::accept_session(const Perception& frame) {
  if (frame.session_id == session_)
    return true;
  if (retired_sessions_.count(frame.session_id))
    return false;
  if (!session_.empty())
    retired_sessions_.insert(session_);
  session_ = frame.session_id;
  last_tick_ = -1;
  last_receipt_ = -1;
  probe_strategy_.reset();
  resource_strategy_.reset();
  stage2_strategy_.reset();
  if (!pending_ai_result_.is_object() || pending_ai_result_.value("session_id", "") != session_)
    pending_ai_result_ = Json();
  ai_response_received_ = false;
  ai_response_mock_ = true;
  pending_.clear();
  cached_ = Json();
  ++sessions_;
  const auto [x, y] = frame.position;
  record({{"event", "new_session"}, {"tick", frame.tick}, {"position", {x, y}}});
  RCLCPP_INFO(get_logger(), "SESSION %s tick=%ld position=[%d,%d]", session_.c_str(),
              static_cast<long>(frame.tick), x, y);
  return true;
}

/**
 * @brief 核对裁判提供的上一Tick执行回执，并清理已处理的提交记录。
 * @param self 当前感知中的本车状态JSON，回执位于self.last_action。
 * @param position 当前self.position，只用于日志展示，不用于累计定位。
 * @throws std::runtime_error 回执不是对象或缺少整数Tick时抛出。
 * @throws nlohmann::json::exception 回执字段缺失或类型转换错误时抛出。
 *
 * 开局null回执、重复回执以及未由本进程提交过的Tick会直接略过。
 * matched只比较同Tick的动作类型，不验证方向或保证移动成功；
 * moved=true才计为移动成功，裁判error或动作类型不匹配计入本地错误统计。
 * 这里的receipt_errors_是调试统计，与裁判的invalid_actions不是同一指标。
 */
void ConnectionNode::inspect_receipt(const Json& self, Position position) {
  if (!self.contains("last_action") || self["last_action"].is_null())
    return;
  const auto& ack = self.at("last_action");
  if (!ack.is_object() || !ack.contains("tick") || !ack["tick"].is_number_integer()) {
    throw std::runtime_error("invalid last_action receipt");
  }
  const auto tick = ack.at("tick").get<std::int64_t>();
  if (tick <= last_receipt_)
    return;
  last_receipt_ = tick;
  const auto it = pending_.find(tick);
  if (it == pending_.end())
    return;  // 刚接入时，上一Tick可能由默认WAIT处理。
  const bool matched = ack.at("action") == it->second.at("action");
  const bool moved = ack.value("moved", false);
  const auto error = ack.value("error", std::string{});
  if (matched)
    ++receipts_;
  if (moved)
    ++moves_;
  if (!matched || !error.empty())
    ++receipt_errors_;
  const auto [x, y] = position;
  record({{"event", "ack"},
          {"tick", tick},
          {"receipt", ack},
          {"matched", matched},
          {"position", {x, y}}});
  RCLCPP_INFO(get_logger(), "ACK tick=%ld action=%s moved=%s position=[%d,%d] matched=%s error=%s",
              static_cast<long>(tick), ack.at("action").get<std::string>().c_str(),
              moved ? "true" : "false", x, y, matched ? "true" : "false", error.c_str());
  // 本Tick及更早的记录不再等待；漏帧后无法从当前回执补回所有历史结果。
  pending_.erase(pending_.begin(), pending_.upper_bound(tick));
}

/**
 * @brief 处理一条公开感知，是“收感知→决策→发动作”的完整入口。
 * @param data ROS String消息的data字段，内容为感知JSON文本。
 *
 * 顺序：解析基础字段→确认场次→丢弃旧Tick/处理重复帧→核对回执→
 * 记录当前帧→检查终态→调用策略→绑定本帧身份→缓存并发布。
 * 重复帧只重发缓存，不运行decide，避免同一Tick重复更新资源计划。
 * 策略输入异常时退回WAIT；外层捕获解析等标准异常，记录警告而不退出进程。
 * 本回调不等待LLM、不读取裁判全图，动作节奏由收到的新感知驱动。
 */
void ConnectionNode::on_perception(const std::string& data) {
  try {
    const auto frame = Perception::parse(data, team_);
    if (!accept_session(frame) || frame.tick < last_tick_)
      return;
    const auto tick = frame.tick;
    const auto [x, y] = frame.position;
    if (tick == last_tick_) {
      if (!cached_.is_null())
        publish(cached_, true);  // 重发不重复推进策略。
      return;
    }
    inspect_receipt(frame.self(), frame.position);
    ++frames_;
    record({{"event", "rx"},
            {"tick", tick},
            {"position", {x, y}},
            {"alive", frame.alive},
            {"score", frame.data.value("score", Json::object())}});
    // 协议没有必需的结束标志，按赛前配置的总Tick停止提交，仍处理终态回执。
    if (tick >= action_ticks_) {
      last_tick_ = tick;
      cached_ = Json();
      record({{"event", "finished"}, {"tick", tick}});
      RCLCPP_INFO(get_logger(), "FINAL tick=%ld; no new action", static_cast<long>(tick));
      return;
    }
    Action intent = Action::wait();
    try {
      if (mode_ == "probe")
        intent = probe_strategy_.decide(frame);
      if (is_stage2_mode()) {
        const auto started = std::chrono::steady_clock::now();
        const auto public_frame = parse_public_perception(data, team_);
        const auto event = consume_ai_result(public_frame);
        intent =
            stage2_strategy_.decide(public_frame, action_ticks_ - tick, event ? &*event : nullptr);
        const auto decision_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                     std::chrono::steady_clock::now() - started)
                                     .count();
        stage2_decision_us_.push_back(decision_us);
        const auto& plan = stage2_strategy_.plan();
        Json detail{{"event", "stage2_plan"},
                    {"neural",plan.neural},
                    {"planning",plan.planning},
                    {"purpose",plan.purpose},
                    {"resource_race", plan.resource_race},
                    {"tick", tick},
                    {"goal", plan.goal},
                    {"phase", plan.phase},
                    {"phase_goal_selected", plan.phase_goal_selected},
                    {"phase_resource_age", plan.phase_resource_age},
                    {"task_waypoint", plan.task_waypoint},
                    {"target_id", plan.target_id},
                    {"target", nullptr},
                    {"opponent_intents", plan.opponent_intents},
                    {"recent_enemy_sightings", plan.recent_enemy_sightings},
                    {"opponent_motion", plan.opponent_motion},
                    {"escape_steps", plan.escape_steps},
                    {"retreat_lead_ticks", plan.retreat_lead_ticks},
                    {"preserving_score", plan.preserving_score},
                    {"joint_planned", plan.joint_planned},
                    {"joint_used", plan.joint_used},
                    {"joint_future_utility", plan.joint_future_utility},
                    {"joint_visit_order", plan.joint_visit_order},
                    {"pursuit_cooldowns", plan.pursuit_cooldowns},
                    {"navigation_cycle_active", plan.navigation_cycle_active},
                    {"blocked_retries", plan.blocked_retries},
                    {"blocked_retry_streak", plan.blocked_retry_streak},
                    {"navigation_cycle_breaks", plan.navigation_cycle_breaks},
                    {"opening_fraction", plan.opening_fraction},
                    {"map_coverage", plan.economy.coverage},
                    {"confirmed_removed_value", plan.economy.confirmed_removed_value},
                    {"known_available_value", plan.economy.known_available_value},
                    {"collection_factor", plan.economy.collection_factor},
                    {"resource_phase_factor", plan.economy.phase_factor},
                    {"exploration_factor", plan.economy.exploration_factor},
                    {"resource_weight", plan.resource_weight},
                    {"temporal_weights",plan.temporal_weights},
                    {"collection_transfer",plan.collection_transfer},
                    {"duel_stalemate",plan.duel_stalemate},
                    {"battle_curve",plan.battle_curve},
                    {"opening_collection",plan.opening_collection},
                    {"opening_blocked_battle",plan.opening_blocked_battle},
                    {"acquisition_exploration_weight",plan.acquisition_exploration_weight},
                    {"acquisition_silence_ticks",plan.acquisition_silence_ticks},
                    {"combat_weight", plan.combat_weight},
                    {"reach_weight", plan.reach_weight},
                    {"reach_finish", plan.reach_finish},
                    {"fortress", plan.fortress},
                    {"resource_races", plan.resource_races},
                    {"resource_points", plan.resource_points},
                    {"mission_potential_points", plan.mission_points},
                    {"combat_points", plan.combat_points},
                    {"information_points", plan.information_points},
                    {"observation_age_points", plan.observation_age_points},
                    {"patrol_potential", plan.patrol_potential},
                    {"safety_potential", plan.safety_potential},
                    {"goal_utility", plan.goal_utility},
                    {"action_utility", plan.action_utility},
                    {"interference_utility", plan.interference_utility},
                    {"incoming_damage_bound", plan.incoming_damage_bound},
                    {"event_pending", plan.event_pending},
                    {"event_state", plan.event_state},
                    {"event_reward", plan.event_reward},
                    {"movement_inverted", plan.movement_inverted},
                    {"unsupported_missions", plan.unsupported_missions},
                    {"candidates", plan.candidates},
                    {"decision_us", decision_us},
                    {"ai_response_received", ai_response_received_},
                    {"ai_response_mock", ai_response_mock_},
                    {"rollout", plan.rollout},
                    {"collection_search",plan.collection_search},
                    {"battle_evidence",plan.battle_evidence},
                    {"resource_origins",plan.resource_origins},
                    {"frontier_value", plan.frontier_value},
                    {"action_scores", plan.action_scores}};
        if (plan.target)
          detail["target"] = {plan.target->x, plan.target->y};
        if (plan.route.success())
          detail["route_distance"] = plan.route.cost;
        record(std::move(detail));
        RCLCPP_INFO(get_logger(),
                    "STAGE2 tick=%ld phase=%s goal=%s target=%s decision=%ldus event_pending=%s",
                    static_cast<long>(tick), plan.phase.c_str(), plan.goal.c_str(), plan.target_id.c_str(),
                    static_cast<long>(decision_us), plan.event_pending ? "true" : "false");
      }
      if (mode_ == "resource") {
        intent =
            resource_strategy_.decide(parse_public_perception(data, team_), action_ticks_ - tick);
        const auto& plan = resource_strategy_.plan();
        Json detail{{"event", "resource_plan"},
                    {"tick", tick},
                    {"state", resource_state_name(plan.state)},
                    {"candidates", plan.candidates},
                    {"order_method", plan.order_method},
                    {"order_replanned", plan.order_replanned},
                    {"total_cost", plan.total_cost},
                    {"visit_order", plan.visit_order},
                    {"budget_filtered", plan.budget_filtered},
                    {"remaining_ticks", nullptr},
                    {"weighted_cost", nullptr},
                    {"target_id", nullptr},
                    {"target", nullptr},
                    {"exploration", nullptr}};
        if (plan.remaining_ticks)
          detail["remaining_ticks"] = *plan.remaining_ticks;
        if (plan.best_resource_cost)
          detail["best_resource_cost"] = *plan.best_resource_cost;
        if (plan.best_exploration_cost)
          detail["best_exploration_cost"] = *plan.best_exploration_cost;
        if (plan.exploration_cost_limit)
          detail["exploration_cost_limit"] = *plan.exploration_cost_limit;
        if (plan.peek) {
          detail["peek"] = {{"extra_steps", plan.peek->extra_steps},
                            {"extra_cells", plan.peek->extra_cells},
                            {"weighted_extra_cells", plan.peek->weighted_extra_cells},
                            {"reward", plan.peek->reward},
                            {"cost", plan.peek->total},
                            {"detour_used", plan.peek_detour_used}};
          RCLCPP_INFO(get_logger(),
                      "PEEK tick=%ld extra_steps=%ld extra_cells=%zu reward=%.6g detour_used=%ld",
                      static_cast<long>(tick), static_cast<long>(plan.peek->extra_steps),
                      plan.peek->extra_cells, plan.peek->reward,
                      static_cast<long>(plan.peek_detour_used));
        }
        if (plan.weighted_cost) {
          const auto& cost = *plan.weighted_cost;
          detail["weighted_cost"] = {{"astar_distance", cost.astar_distance},
                                     {"edge_distance", cost.edge_distance},
                                     {"value_units", cost.value_units},
                                     {"edge_cost", cost.edge_cost},
                                     {"value_reward", cost.value_reward},
                                     {"distance_cost", cost.distance_cost},
                                     {"new_cells", cost.new_cells},
                                     {"weighted_new_cells", cost.weighted_new_cells},
                                     {"exploration_scale", cost.exploration_scale},
                                     {"exploration_reward", cost.exploration_reward},
                                     {"total", cost.total}};
        }
        if (plan.exploration) {
          const auto& explore = *plan.exploration;
          detail["exploration"] = {{"candidates", explore.candidates},
                                   {"unknown_cells", explore.unknown_cells},
                                   {"target", nullptr}};
          if (explore.target) {
            const auto& target = *explore.target;
            detail["exploration"]["target"] = {target.position.x, target.position.y};
            detail["exploration"]["distance"] = target.distance;
            detail["exploration"]["edge_distance"] = target.edge_distance;
            detail["exploration"]["new_cells"] = target.new_cells;
            detail["exploration"]["weighted_new_cells"] = target.weighted_new_cells;
            detail["exploration"]["score"] = target.total;
            detail["exploration"]["reward_scale"] = target.reward_scale;
            detail["exploration"]["reward"] = target.reward;
            RCLCPP_INFO(get_logger(),
                        "EXPLORE tick=%ld position=[%d,%d] distance=%ld edge=%d new_cells=%zu "
                        "weighted_cells=%.6g score=%.3f",
                        static_cast<long>(tick), target.position.x, target.position.y,
                        static_cast<long>(target.distance), target.edge_distance, target.new_cells,
                        target.weighted_new_cells, target.total);
          }
        }
        if (plan.route.success()) {
          detail["cost"] = plan.route.cost;
          detail["route_estimated"] = plan.order_method == "joint_coverage_estimate";
          if (!plan.route.path.empty())
            detail["route_goal"] = {plan.route.path.back().x, plan.route.path.back().y};
          detail["path"] = Json::array();
          for (const auto point : plan.route.path)
            detail["path"].push_back({point.x, point.y});
        }
        if (plan.target) {
          detail["target_id"] = plan.target->id.value_or(plan.resource_key);
          detail["target"] = {plan.target->position.x, plan.target->position.y};
          detail["last_seen_tick"] = plan.last_seen_tick;
          RCLCPP_INFO(
              get_logger(),
              "RESOURCE tick=%ld target=%s position=[%d,%d] distance=%ld order=%s total=%ld",
              static_cast<long>(tick), plan.target->id.value_or(plan.resource_key).c_str(),
              plan.target->position.x, plan.target->position.y, static_cast<long>(plan.route.cost),
              plan.order_method.c_str(), static_cast<long>(plan.total_cost));
          if (plan.weighted_cost)
            RCLCPP_INFO(get_logger(),
                        "WEIGHTED tick=%ld astar=%ld edge=%d edge_cost=%.3f value_reward=%.3f "
                        "new_cells=%zu explore_reward=%.3f score=%.3f",
                        static_cast<long>(tick),
                        static_cast<long>(plan.weighted_cost->astar_distance),
                        plan.weighted_cost->edge_distance, plan.weighted_cost->edge_cost,
                        plan.weighted_cost->value_reward, plan.weighted_cost->new_cells,
                        plan.weighted_cost->exploration_reward, plan.weighted_cost->total);
        }
        record(std::move(detail));
      }
    } catch (const std::exception& error) {
      // 当前帧身份可靠、规划输入异常时退回WAIT。
      ++parse_errors_;
      record({{"event", "decision_fallback"}, {"tick", tick}, {"reason", error.what()}});
      RCLCPP_WARN(get_logger(), "decision fallback to WAIT: %s", error.what());
      intent = Action::wait();
    }
    // 在发送边界统一绑定身份，策略本身不生成或猜测Tick和场次ID。
    const auto action = intent.to_json(tick, frame.session_id);
    last_tick_ = tick;
    cached_ = action;
    pending_[tick] = action;
    // 限制未回执缓存大小，避免长期缺少回执时内存持续增长。
    while (pending_.size() > 256)
      pending_.erase(pending_.begin());
    publish(action, false);
    RCLCPP_INFO(get_logger(), "RX/TX tick=%ld position=[%d,%d] action=%s", static_cast<long>(tick),
                x, y, action.dump().c_str());
  } catch (const std::exception& error) {
    ++parse_errors_;
    record({{"event", "parse_error"}, {"reason", error.what()}});
    RCLCPP_WARN(get_logger(), "invalid perception ignored: %s", error.what());
  }
}

/** @brief 缓存自有AI话题中的有界JSON；格式错误不影响本队公开感知与动作流程。 */
void ConnectionNode::on_ai_result(const std::string& data) {
  try {
    if (data.size() > 256 * 1024)
      throw std::runtime_error("AI IPC message too large");
    auto value = Json::parse(data);
    if (!value.is_object() || value.at("team") != team_ || !value.at("session_id").is_string())
      throw std::runtime_error("AI IPC identity mismatch");
    if (!retired_sessions_.count(value.at("session_id").get<std::string>()))
      pending_ai_result_ = std::move(value);
  } catch (const std::exception&) {
    record({{"event", "ai_result_rejected"}, {"reason", "invalid_ipc_message"}});
  }
}

/** @brief 模拟回复只更新通信状态；真实结果还须匹配当前公告、源Tick及合法约束。 */
std::optional<Stage2EventConstraints> ConnectionNode::consume_ai_result(
    const PublicPerception& frame) {
  if (!pending_ai_result_.is_object())
    return std::nullopt;
  if (pending_ai_result_.value("session_id", "") != frame.session_id)
    return std::nullopt;
  try {
    const auto& raw_tick = pending_ai_result_.at("source_tick");
    if (!raw_tick.is_number_integer())
      throw std::runtime_error("invalid AI source tick");
    const auto source_tick = raw_tick.get<std::int64_t>();
    if (source_tick < 0)
      throw std::runtime_error("negative AI source tick");
    if (source_tick > frame.tick)
      return std::nullopt;
    const auto value = std::exchange(pending_ai_result_, Json());
    const bool received = value.at("completion_received").get<bool>();
    const bool mock = value.at("mock").get<bool>();
    if (received) {
      ai_response_received_ = true;
      ai_response_mock_ = mock;
    }
    record({{"event", "ai_result"},
            {"tick", frame.tick},
            {"source_tick", source_tick},
            {"completion_received", received},
            {"mock", mock},
            {"elapsed_ms", value.at("elapsed_ms")},
            {"http_status", value.at("http_status")}});
    if (!received || mock || !value.at("interpretation_valid").get<bool>() ||
        !value.at("event_understood").get<bool>() || !value.at("event_plan").is_object() ||
        !value.at("source_event_text").is_string())
      return std::nullopt;
    const auto source_text = value.at("source_event_text").get<std::string>();
    if (source_text != frame.event_text &&
        (source_text.empty() || frame.event_text.compare(0, source_text.size(), source_text) != 0))
      return std::nullopt;
    auto constraints =
        decode_ai_constraints(value.at("event_plan"), frame, source_tick, action_ticks_);
    constraints.source_text = source_text;
    if (constraints.end_tick <= frame.tick)
      return std::nullopt;
    record({{"event", "ai_constraints_received"},
            {"tick", frame.tick},
            {"begin_tick", constraints.begin_tick},
            {"end_tick", constraints.end_tick},
            {"danger_cells", constraints.dangerous_cells.size()},
            {"event_count", constraints.events.size()}});
    return constraints;
  } catch (const std::exception&) {
    pending_ai_result_ = Json();
    record({{"event", "ai_result_rejected"}, {"reason", "invalid_interpretation_or_binding"}});
    return std::nullopt;
  }
}

}  // namespace rmtc
