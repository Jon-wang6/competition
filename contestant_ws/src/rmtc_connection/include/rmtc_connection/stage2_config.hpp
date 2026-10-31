/**
 * @file stage2_config.hpp
 * @brief 第二阶段独立参数：将收益换算成比赛分数，再比较路程、信息和风险。
 *
 * 修改下列默认值后重新编译、重启选手即可生效；不会改变第一阶段的参数。
 * 原有参数是可解释的启发式权重；neural另接独立训练的残差网络，默认关闭。
 * 原有默认值来自多图参数试验：
 * 开局采集/到达任务可分阶段过渡，联合路线的后续收益与观察点成本也可独立调整。
 * 争抢使用可见敌车已知地形最短路；采用多落点预判、交火线索和无效追击冷却。
 * 观测时效层可用每格最后看到的Tick引导再次搜索；新鲜期/饱和期/奖励系数独立可调。
 * 距离场首步比较、跨目标循环检测分别由下方参数开关；默认值以本轮新图验证为准。
 * 旧单点外推与额外末局保分默认关闭；风险估计从不抹掉当前攻击先结算的致死上界。
 * 要复现旧策略，collection_mode改0，并将astar_resource_competition改false。
 */
#pragma once
#include "rmtc_connection/stage2_event_config.hpp"
#include "rmtc_connection/stage2_rollout_config.hpp"
#include "rmtc_connection/stage2_interference.hpp"
#include "rmtc_connection/stage2_phase_policy.hpp"
#include "rmtc_connection/stage2_resource_race.hpp"
#include "rmtc_connection/stage2_acquisition_curve.hpp"
#include "rmtc_connection/stage2_temporal_policy.hpp"
#include "rmtc_connection/stage2_purpose.hpp"
#include "rmtc_connection/stage2_battle_curve.hpp"
#include "rmtc_connection/stage2_collection_transfer.hpp"
#include "rmtc_connection/stage2_duel_stalemate.hpp"
#include "rmtc_connection/stage2_neural.hpp"
#include "rmtc_connection/stage2_planning.hpp"

namespace rmtc {

/// 仅控制旧资源经济加权：本轮仍关闭它；联合采集/争抢另由下方字段控制。
/// 修改后须重新编译、重启；仅影响第二阶段，不影响第一阶段。
inline constexpr bool kUseAdaptiveResourceEconomy = false;

struct Stage2Weights {
  PlanningWeights planning; ///< 独立预测规划模式；旧模式默认关闭。
  NeuralWeights neural; ///< 独立残差网络与离线教师，默认全部关闭以完整保留旧策略。
  DuelStalemateWeights duel_stalemate; ///< 持续互防后有限试探攻击的软代价，默认关闭。
  Stage2CollectionTransferWeights collection_transfer; ///< 历史高采集参数的前期移植与平滑退出。
  Stage2BattleCurveWeights battle_curve; ///< 资源保护下的后期战斗收益/软风险独立曲线。
  int opening_collection_ticks{0}; ///< 开局采集窗口：仅移动、不开火、不防御；0保留旧策略作为对照。
  Stage2PurposeWeights purpose; ///< 资源机会消失后的明确作战目的与短期截击模型。
  Stage2TemporalWeights temporal; ///< 按参数含义独立过渡；每Tick计算副本，不改采集固定权重。
  AcquisitionCurveWeights acquisition; ///< 整局固定采集偏好；只让纯未知资源探索随时间衰减。
  ResourceRaceWeights resource_race; ///< 整局资源优先与对手预计到达期限的子集采集规划。
  Stage2PhaseWeights phase; ///< 采集→刷伤害→补任务的显式目标分期，开关与参数集中在独立头文件。
  double collection_search_weight{0.0}; ///< 多目标路线搜索提案权重；实验验证前默认关闭。
  int collection_search_ticks{180};     ///< 开局采集/开图搜索窗口，不覆盖末局战斗策略。
  int collection_search_horizon{40};    ///< 整条预测路线最多移动Tick数。
  int collection_search_depth{4};       ///< 每条预测路线最多连接几个资源/观察目标。
  int collection_search_beam{64};       ///< 每层保留的不同路线状态数。
  int collection_search_targets{24};    ///< 资源与观察点的总候选上限，不包括当前位置。
  int collection_search_resources{16};  ///< 最多同时预测的已知资源数。
  double collection_search_future{0.5}; ///< 第一段之后的收益折扣，防止远期承诺压过近邻资源。
  double collection_search_discount{0.98}; ///< 随预计到达Tick衰减收益。
  double collection_search_risk{1.0};   ///< 整条路线的时空危险代价系数。
  double collection_search_budget_ms{40.0}; ///< 软计算上限；有结果即返回，不等待用满。
  double combat_clue_weight{0.0};       ///< 掉血/明确死亡形成的软搜索、风险与交火记忆权重。
  double combat_clue_half_life{8.0};    ///< 未归因交火证据的可信度半衰期。
  int combat_clue_max_age{24};          ///< 超过本Tick年龄就丢弃旧战斗现场。
  int third_party_memory_ticks{3};      ///< 第三方射击证据可短暂延用多久，仍保留转火风险。
  double opening_depletion_weight{0.0}; ///< 敌人来路已被搜过的先验强度；只影响未知资源预期。
  double opening_depletion_floor{0.5};  ///< 任何方向至少保留的资源搜索比例，不直接放弃整片区域。
  double opening_depletion_half_life{40.0}; ///< 来路资源先验随时间减弱，不能当作实时事实。
  int opening_depletion_ticks{100};     ///< 只在开局窗口接收敌人来路线索。
  RolloutWeights rollout;                  ///< 多情景前瞻的独立开关、误差偏好及计算预算。
  Stage2EventWeights events;               ///< 事件独立参数，不修改原资源/战斗权重。
  Stage2InterferenceWeights interference;  ///< 伤害满分后的有限干扰与自身保分预算。
  double recent_enemy_search_weight{1.0};  ///< 最近敌人线索搜索权重；0关闭，不替代当前攻防。
  double recent_enemy_half_life{10.0};     ///< 最后看见后多少Tick，历史线索质量减半。
  int recent_enemy_max_age{30};            ///< 历史搜索线索的最长有效Tick；超过即放弃。
  bool enemy_belief_tracking{false};       ///< 连续保留空视野证据，替代逐帧重建的均匀敌人搜索云。
  int blocked_retry_ticks{0};  ///< E_BLOCKED后基础避让Tick，另按席位错峰0~3Tick；0关闭，不记成墙。
  int blocked_retry_min_failures{1};  ///< 同一落点连续失败几次才避让；1复现立即避让，较大值容忍短暂争抢。
  double resource_lookahead_weight{0.0};  ///< 首资源之后已知资源的收益率权重；0关闭短程前瞻。
  int resource_lookahead_steps{24};       ///< 整段采集前瞻最多步数，且不能超过比赛剩余Tick。
  int resource_lookahead_limit{6};        ///< 按当前收益率保留的资源数上限，范围2~10。
  int resource_lookahead_depth{3};        ///< 含首资源在内最多访问几块，范围2~3。
  double route_information_weight{0.0};   ///< 等长首步新增观察奖励；0保留原路径行为。
  int collection_mode{1};       ///< 0原策略，1联合路线参与比较，2优先联合采集/探索；攻防仍独立。
  int joint_min_resources{2};   ///< 至少有几块可达有收益资源才调用联合规划；0复现整套接管实验。
  int joint_ticks{80};          ///< 联合采集启用窗口，到期自动交回原策略。
  int joint_resource_limit{6};  ///< 同时规划的高收益已知资源上限，限制多人动态场景计算量。
  int joint_horizon{24};        ///< 后续资源收益最多前视多少步，不许诺整局路线。
  double joint_future_weight{0.25};       ///< 近期后续资源收益率的折扣系数。
  double joint_observation_offset{40.0};  ///< 第一阶段插入观察点的成本，在第二阶段独立调整。
  double joint_unknown_weight{0.4};       ///< 未发现资源在联合路线中的未来插入代价权重，不是未知敌人的风险。

  bool joint_linear_weights{true};            ///< true时下列联合参数随joint_ticks线性过渡。
  double joint_future_weight_end{0.0};        ///< 窗口末尾后续收益权重；与起值相同即固定权重。
  double joint_observation_offset_end{80.0};  ///< 窗口末尾观察点成本。
  bool astar_resource_competition{true};      ///< 用可见敌车的已知地形最短路代替旧曼哈顿争抢估计。
  double competition_softness{2.0};           ///< 抢先步数差的平滑尺度；越小越接近硬阈值。
  double competition_weight{1.0};             ///< 开局争抢折扣强度，0忽略、1完全使用估计。
  double competition_weight_end{1.0};         ///< joint_ticks之后的强度，期间线性变化。

  int opening_ticks{80};  ///< 资源偏好的线性过渡时长；到达任务与攻防使用独立时间线。
  double opening_resource_weight{
      1.3};  ///< tick0资源及采集任务偏好，随后线性过渡至resource_weight_end。
  double resource_weight_end{0.95};  ///< 采集欲望终值：小于起值会逐渐减弱；默认保持已验证参数。
  double opening_combat_weight{0.6};   ///< 开局降低主动交战偏好；防御和死亡风险不打折。
  double combat_weight_end{1.15};      ///< 后期交战偏好，伤害分已满时不会凭空生成攻击收益。
  int combat_ramp_ticks{200};          ///< 攻击偏好独立线性过渡时长，不随资源是否耗尽跳变。
  double reach_weight_end{1.0};             ///< 到达任务偏好终值；早期延后、后期补齐。
  double resource_mission_weight{1.0};      ///< 采集任务额外权重，独立于资源真实计分及旧经济模型。
  double resource_mission_weight_end{1.0};  ///< 可选时间线末端的采集任务偏好；不改变任务实际分值。
  int resource_mission_ramp_ticks{0};       ///< 0保持原固定权重；正数让开局值逐渐过渡至末值。
  // 2026-10-28：32图配对试验保留的小幅调整；仍低于开局资源偏好，并非稳定50分保证。
  double opening_reach_weight{0.6};         ///< tick0到达任务偏好；资源与采集任务仍用独立权重。
  int reach_ramp_ticks{120};                ///< 逐渐恢复到达任务偏好的窗口，兼顾向内开图与沿途采集。
  int reach_deadline_buffer{24};            ///< 剩余时间接近路程时，提前多少Tick恢复任务紧迫性。
  double reach_deadline_weight{2.0};        ///< 临近最晚出发时间的到达任务权重，仍计入路径风险。
  int reach_finish_ticks{200};             ///< 剩余多少Tick开始主动补未完成到达任务；0关闭此层。
  int reach_attack_pause_ticks{3};         ///< 赶任务途中最多连续停留几Tick交战；路程紧张时自动缩短。
  double reach_move_priority{0.3};         ///< 补任务时的基础移动偏好；有余裕仍可就地攻击。
  int reach_stalemate_ticks{12};           ///< 连续互相防御且不掉血多久才尝试脱离射线；0关闭试探。
  int reach_collision_retry_ticks{2};      ///< 补任务时至少避让失败落点多久，防止旧参数关闭避让后永久互堵。
  double visible_resource_weight{1.4};      ///< 当前亲眼确认的资源偏好；不提高过时资源记忆的权重。
  bool adaptive_resources{
      kUseAdaptiveResourceEconomy};      ///< 开启覆盖/采集/消失记录联合调节；false用于旧逻辑对比。
  double initial_resource_value{200.0};  ///< 第二阶段全场初始资源价值，来自规则；不是已知点位。
  double resource_need_power{0.5};       ///< 采集越接近120，额外采集加成衰减越多；0不衰减。
  double resource_coverage_power{
      0.5};  ///< 开图进度对搜索和阶段偏好的影响；已知资源基本分不受影响。
  double resource_depletion_power{0.5};  ///< 已确认移除资源对额外采集/搜索偏好的影响。
  double exploration_floor{0.85};        ///< 剩余角落的探索偏好下限，防止少量未知区域被直接放弃。
  double collection_mission_weight{
      1.4};                              ///< 单独保留采集任务的机会价值，不随资源计分饱和一起清零。
  double resource_cap{120.0};            ///< 资源计分饱和点，属于规则常量。
  double resource_points{30.0};          ///< 资源满分，属于规则常量。
  double damage_cap{500.0};              ///< 有效伤害计分饱和点，属于规则常量。
  double damage_points{15.0};            ///< 攻击满分，属于规则常量。
  double mission_points{5.0};            ///< 每项任务分数；正式计分还需要官方AI资格。
  double mission_potential{0.6};         ///< 任务潜在收益系数[0,1]；计分资格以主办方记录为准。
  double travel_offset{3.0};             ///< 收益除以“距离+本值”，平衡远近目标。
  double frontier_resource_weight{0.5};  ///< 未知区域预期资源分的额外权重，0关闭。
  double frontier_mission_weight{1.0};   ///< 未完成采集任务赋予未知区域的额外机会价值。
  double frontier_resource_half_life{40.0};  ///< 未见资源可能已被对手吃掉的时间折扣半衰期。
  double information_per_cell{0.025};        ///< 新未知格的探索价值，与资源分数处于同一量级。
  double information_cap{1.5};               ///< 单个目标可计入的探索奖励上限，避免盲目远游。
  double information_softness{0.0};          ///< 0旧硬上限，1平滑上限；保留大片新区域的增量价值。
  double sweep_weight{0.0};  ///< 资源与沿途开图联合束搜索的收益率权重；0完整关闭。
  int sweep_ticks{100};      ///< 只在此Tick之前生成联合采集路线，不替代后期攻防。
  int sweep_horizon{12};     ///< 每次最多模拟多少个移动Tick，仍只执行首步。
  int sweep_beam{24};        ///< 同层保留路线数，按首步分组保留，限制计算量。
  double sweep_discount{0.95};  ///< 每多一步，对仍未实际获得的收益打折。
  bool sweep_collect_only{false};  ///< true仅接受实际经过已知资源的路线，纯探索沿用原策略。
  int navigation_cycle_window{0};      ///< 0关闭；8~40Tick无收益且局部往返才触发脱困。
  int navigation_cycle_start_tick{0};  ///< 可延后到开局采集之后，避免早期误伤必要资源绕行。
  int navigation_cycle_cooldown{12};   ///< 触发后暂时降低循环格子的移动偏好，不写成墙。
  double navigation_cycle_cost{0.2};   ///< 循环格软代价；真实资源/伤害/任务/新视野会清除它。
  double edge_information_bonus{0.5};        ///< 边界附近未知格额外奖励，减少遗漏。
  double resource_age_ticks{35.0};           ///< 旧资源可信度 = 1/(1+未见Tick/本值)。
  double contested_resource_factor{0.2};     ///< 对手明显更快时仍保留的收益比例；越大越愿意争抢，越小越易放弃。
  double intent_chase_weight{0.2};  ///< 多落点预测对近距离追击的混合权重；按到达步数平方衰减。
  double intent_risk_weight{0.25};  ///< 三名对手下一Tick活动/射程分布对避险的混合权重。
  double third_party_risk_discount{
      0.35};                             ///< 疑似攻击别人的敌车转火风险折扣强度；不会减小致死上界。
  double third_party_attack_bonus{0.2};  ///< 当前可合法攻击的交火对手的机会偏好；不修改实际得分。
  double intent_heading_weight{3.0};     ///< 连续同向移动的惯性质量；仍保留等待与转向可能。
  int intent_memory_ticks{2};            ///< 视野外敌人软范围最多延用的Tick，按0.5^age衰减。
  double opponent_motion_weight{0.0};    ///< 当前敌位与一步外推的混合权重；0关闭预测影响。
  double endgame_preserve_weight{0.0};   ///< 末局保分撤离收益权重；0关闭新增提前避战。
  double low_hp_risk_weight{0.0};        ///< 血量下降时额外放大风险的强度；0保留原风险模型。
  double retreat_damage_fraction{0.9};   ///< 达到伤害上限的此比例，才启用末局保分。
  int retreat_buffer_ticks{8};           ///< 逃离距离之外，预留给敌车移动/误差的Tick。
  int retreat_window_ticks{40};          ///< 最多提前多少Tick进入保分窗口。
  double remembered_risk{0.025};         ///< 最近敌人记忆只增加软风险，不封路。
  int enemy_memory_ticks{3};             ///< 超过此时间的敌人位置不参与局部风险。
  double route_risk{0.08};               ///< 路径前两步处于敌方射程的代价。
  double switch_margin{0.03};            ///< 继续原目标的小奖励，抑制近似收益反复切换。
  bool separate_goal_switch_bonus{false}; ///< 目标惯性只参与选目标，不抬高移动动作收益，防止压过合法攻击。
  double attack_damage_estimate{15.0};   ///< 对手可能防御：标准10~20伤害中取15作估计。
  double attack_probability{0.55};       ///< 敌人本Tick选择攻击的估计概率，不是已知事实。
  double hp_loss_cost{0.012};            ///< 每点预计掉血的附加代价。
  double death_cost{3.0};                ///< 可能死亡的保守惩罚，另加实际伤害积分损失。
  double future_exposure_cost{0.12};     ///< 动作后位置仍处于可见敌人射程的代价。
  double kill_reward_estimate{30.0};     ///< 击杀池60的折扣估计，不假设必独享击杀。
  double observation_age_weight{
      0.0};  ///< 已知区域的陈旧度搜索奖励，0关闭；按仍可获得的伤害/任务分缩放。
  int observation_start_tick{0};    ///< 观测时效奖励开始启用的Tick；0复现原时效实验。
  int observation_ramp_ticks{0};    ///< 启用后线性增加到完整强度的Tick数；0立即达到设定强度。
  int observation_fresh_ticks{8};   ///< 多少Tick以内仍算新鲜，不给重复巡视奖励。
  int observation_stale_ticks{60};  ///< 到此Tick达到陈旧度上限；中间线性增加，不无限累积。
  bool chase_shared_progress{
      false};  ///< 切换敌人也累计无伤害局部循环；触发后暂停窗口内全部追击对象。
  double route_alternatives_weight{
      0.0};                      ///< 0仅原A*首步获目标收益，1让所有接近目标的合法首步参与收益比较。
  int chase_stall_ticks{8};      ///< 连续追击移动无伤害且只在至多4格循环时，暂停该追击；0关闭。
  int chase_cooldown_ticks{12};  ///< 暂停追击时长；仍允许立即攻击和防御，不封禁敌人格子。
  double patrol_value{1.0};      ///< 无可获利目标时，巡查旧区域寻找战斗机会的权重。
  double patrol_stale_ticks{60.0};  ///< 区域越久未见，巡查价值越高；达到此Tick数后饱和。
  int patrol_min_age{8};            ///< 刚观察过的格子不奖励重复巡查。
  double patrol_enemy_hint{0.5};    ///< 历史敌车附近的巡查加成，随时间和距离衰减。
};

}  // namespace rmtc
