/**
 * @file stage2_strategy.hpp
 * @brief 第二阶段纯C++策略接口：公开感知→统一目标→当前Tick动作。
 *
 * 复用地形/实体记忆与A*，不读取裁判内部状态，不改变第一阶段策略。
 * 未收到有效模型解释的事件保持待解释状态；不使用关键词替代官方要求的LLM。
 * 独立AI进程可提交受校验的EventConstraints；本策略自身不发起网络调用。
 *
 * @details 模块协作与维护约定
 * 主线：更新公开记忆与事件 → 生成目标 → 计算路线/前瞻 → 比较攻防移动 → 返回当前动作。
 * 目标层选资源不代表动作层必然移动；危险、事件、脱困和攻防判断可能覆盖路线建议。
 * 所有历史状态属于当前策略实例；换场必须重置，候选分数和预测位置不是裁判事实。
 */
#pragma once

#include "rmtc_connection/astar.hpp"
#include "rmtc_connection/map_memory.hpp"
#include "rmtc_connection/stage2_config.hpp"
#include "rmtc_connection/stage2_joint_planner.hpp"
#include "rmtc_connection/stage2_resource_economy.hpp"
#include "rmtc_connection/stage2_events.hpp"
#include "rmtc_connection/stage2_navigation_progress.hpp"
#include "rmtc_connection/stage2_enemy_belief.hpp"
#include "rmtc_connection/stage2_battle_evidence.hpp"

#include <deque>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

namespace rmtc {

/// 目标与动作解释，供日志和离线测试使用；收益是预测值，实际得分以裁判为准。
struct Stage2Plan {
  Json planning=Json::object(); ///< 同窗口采集预测、独立存在性估计、节点预算与实际选择。
  Json neural=Json::object(); ///< 模型修正、耗时、教师深度和动作变化；与真实比赛分数分开。
  Json duel_stalemate=Json::object(); ///< 连续互防证据、启用门槛和死亡软代价倍率。
  Json collection_transfer=Json::object(); ///< 前期采集混合比例、门槛、实际系数与是否优先资源。
  Json battle_curve=Json::object(); ///< 曲线进度、资源保护及本帧实际攻防系数，便于确认参数生效。
  bool opening_collection{false}; ///< 本帧处于只移动的开局采集窗口，攻击/防御不进入动作候选。
  bool opening_blocked_battle{false}; ///< 开局没有合法移动时，提前恢复战斗候选，不强制等待。
  Json purpose=Json::object(); ///< 当前模式、切换依据和敌方预测，均为公开信息下的估计。
  Json reach_finish=Json::object(); ///< 末200Tick补到达任务的路程、交战余裕及实际动作选择。
  Json fortress=Json::object(); ///< 堡垒争夺时间差、放弃/撤离原因及已确认无双敌人。
  std::string phase{"balanced"}; ///< 显式分期，便于检查当前为何采集/交战/补任务。
  bool phase_goal_selected{false}; ///< 本Tick是否采用阶段候选；它也可能与统一比较的最佳目标相同。
  bool task_waypoint{false}; ///< 是否在主目标路线中插入低绕路代价的到达任务。
  std::int64_t phase_resource_age{0}; ///< 距上次看见可达资源的Tick数，用于检查提前转战。
  std::string goal{"hold"};
  std::string target_id;
  std::optional<Position> target;
  planning::SearchResult route;
  double opening_fraction{0};  ///< 当前开局加权剩余比例；资源确认耗尽时归零。
  double resource_weight{1};   ///< 当前实际采集权重；新模式整局固定，旧线性模式独立保留。
  double acquisition_exploration_weight{1}; ///< 新未知区域的时间/资源空窗衰减倍率。
  Json temporal_weights = Json::object(); ///< 各个时变参数本帧的实际值，方便查看曲线是否生效。
  std::int64_t acquisition_silence_ticks{0}; ///< 距上次可达资源活动的Tick数。
  double combat_weight{1};     ///< 当前帧实际交战权重，不作用于防御风险。
  double reach_weight{1};      ///< 本帧到达任务基础偏好；各目标还按剩余路程判断紧迫性。
  Json resource_races = Json::array();  ///< 本车/最快可见敌车路程、争抢折扣和是否当前可见。
  ResourceEconomy economy;     ///< 本帧公开信息推算的资源机会，用于调参与检查是否重复计数。
  Json opponent_intents = Json::array();
  Json recent_enemy_sightings = Json::array();  ///< 历史搜索线索的年龄/质量，不表示当前敌方坐标。
  Json opponent_motion = Json::array();         ///< 连续公开帧形成的移动评估，不是敌人实际下一步。
  int escape_steps{-1};
  int retreat_lead_ticks{0};
  bool preserving_score{false};
  bool joint_planned{false};
  bool joint_used{false};
  double joint_future_utility{0};
  std::vector<std::string> joint_visit_order;
  double resource_points{0};
  double mission_points{0};
  double combat_points{0};
  double information_points{0};
  double observation_age_points{0};  ///< information_points中来自已知区域时效的部分。
  double patrol_potential{0};        ///< 寻找交战机会的预测价值，不是已取得的战斗分。
  double safety_potential{0};        ///< AI已校验危险区域的撤离价值，不计入比赛得分。
  double goal_utility{0};
  double action_utility{0};
  double interference_utility{0};   ///< 所选动作的额外干扰估值，不计为真实资源/伤害得分。
  double incoming_damage_bound{0};   ///< 考虑已知事件效果后可见敌人攻击的上界，不代表必然发生。
  std::size_t pursuit_cooldowns{0};  ///< 因无效局部追击暂时降级的敌人数量，仍可就地攻击。
  bool navigation_cycle_active{false};  ///< 无收益局部循环已识别，正在尝试临时脱困。
  int navigation_cycle_breaks{0};        ///< 本局识别次数，需结合轨迹检查脱困效果。
  Json blocked_retries = Json::array();  ///< 移动冲突的临时避让格与到期Tick，不是已确认墙体。
  int blocked_retry_streak{0};           ///< 同一落点连续E_BLOCKED次数，到触发门槛后饱和。
  bool event_pending{false};         ///< 尚无对应有效结构解释；不能宣称已理解事件。
  std::size_t unsupported_missions{0};
  std::size_t candidates{0};
  Json action_scores = Json::array();
  Json frontier_value = Json::object();  ///< 公开记忆推算的未知资源/任务探索收益。
  Json rollout = Json::object();         ///< 多情景前瞻深度、开销与每个根动作的未来收益。
  Json collection_search = Json::object(); ///< 多段采集/开图搜索深度、预算及预测收益。
  Json resource_race = Json::object(); ///< 已知资源采集顺序、预计到达时间与搜索预算。
  Json battle_evidence = Json::array(); ///< 排除已知来源后的掉血/死亡线索，不是真实敌人列表。
  Json resource_origins = Json::array(); ///< 开局敌人来路及年龄，用于未知资源的软折扣。
  Json event_state = Json::object();     ///< 已确认的Buff/区域与剩余Tick。
  double event_reward{0};                ///< 事件战术收益，不冒充实际伤害/资源得分。
  bool movement_inverted{false};         ///< 当前实际发出的MOVE是否经过幻术反向转换。
};

class Stage2Strategy {
 public:
  /// 校验参数；这些权重与第一阶段配置相互独立，非法配置在启动时直接报错。
  explicit Stage2Strategy(Stage2Weights weights = {});
  /// 清空本局地形、目标、采集统计和事件解释，开始新局时由通信节点调用。
  void reset();
  /**
   * @brief 使用本队公开感知选择动作；remaining_ticks包含当前Tick。
   * 每帧重新校验目标与首步；死亡/终态返回WAIT。重复帧返回缓存，旧帧返回WAIT。
   * 接入中途不猜测已采集普通/高价值数量，只记录连续帧可确认的本车采集。
   */
  Action decide(const PublicPerception& frame, std::int64_t remaining_ticks = 500,
                const Stage2EventConstraints* incoming_event = nullptr);
  /// 返回最近一次新帧的目标、收益分解与动作候选；不触发再次规划。
  const Stage2Plan& plan() const noexcept;
  /**
   * @brief 校验独立AI进程提供的场次、公告、源Tick、有效期及地图边界。
   * 返回false时不修改当前约束。只接收结构化区域、时限与效果参数，不解析任何自然语言。
   * 本函数本身不确认官方任务计分资格，正式调用记录仍以主办方为准。
   */
  bool set_event_constraints(const Stage2EventConstraints& constraints);

 private:
  Stage2Weights weights_;
  TinyNeuralNetwork neural_model_;
  TinyNeuralNetwork collection_neural_model_;
  std::ofstream collection_neural_samples_; ///< 采集教师样本独立保存，不与战斗标签混合。
  std::ofstream neural_samples_; ///< 仅显式采样时打开；模型推理不写训练文件。
  Stage2JointPlanner joint_planner_;
  Stage2JointPlanner race_coverage_planner_; ///< 无已知资源时的第一阶段覆盖规划独立缓存。
  MapMemory memory_;
  Stage2Plan plan_;
  Action cached_;
  std::optional<PublicPerception> previous_;
  std::optional<PublicPerception> intent_previous_;  ///< 上上帧，只用于验证敌方连续移动方向。
  std::optional<Stage2EventConstraints> event_;
  Stage2Events events_;
  NavigationProgress navigation_progress_;
  EnemyBeliefTracker enemy_beliefs_;
  BattleEvidence battle_evidence_;
  DuelStalemateTracker duel_stalemate_; ///< 只用连续可见威胁者和实际动作识别僵持。
  Action physical_cached_;  ///< 上一帧期望的物理动作；与幻术反转后的提交指令区分。
  int normal_collected_{0};
  int high_collected_{0};
  std::string previous_goal_;
  std::string pursuit_target_;
  std::deque<Position> pursuit_positions_;
  std::deque<std::string> pursuit_targets_;  ///< 与位置窗口对齐，切换目标不能掩盖无进展循环。
  std::map<std::string, std::int64_t> pursuit_cooldown_;
  std::map<std::string,int> unproductive_pursuit_; ///< 同一敌人无伤害追逐预算，不只检查原地循环。
  std::vector<std::int64_t> last_observed_tick_;  ///< 本局各格最后被本车观察的Tick。
  std::map<std::pair<int, int>, std::int64_t> blocked_retry_until_;  ///< 临时移动冲突，不修改地形。
  std::optional<Position> blocked_retry_target_;  ///< 连续失败的实际物理落点，幻术反向后仍正确。
  int blocked_retry_streak_{0};                  ///< 成功、改向、漏帧或复活后清零。
  std::int64_t phase_last_resource_tick_{0};      ///< 本局最后一次可见可达资源机会，不跨场次沿用。
  std::int64_t acquisition_last_activity_tick_{0}; ///< 可达资源当前可见或本人采集时更新。
  std::string reach_finish_id_;          ///< 当前补做的到达任务，完成或换场后清除。
  std::optional<Position> reach_finish_position_; ///< 同ID坐标改变时也重新建立进展记录。
  int reach_previous_distance_{-1};      ///< 上帧到该任务的已知/乐观路程，未知墙会让估计增加。
  std::int64_t reach_last_progress_tick_{-1}; ///< 最后真正接近任务的帧，攻击得分不能重置赶路预算。
  int stationary_defense_ticks_{0}; ///< 跨目标累计真实无掉血防御，仅新模式有限脱困使用。
  int reach_defensive_stalemate_{0};      ///< 连续看见威胁者防御且自身防御未掉血的真实帧数。
};

}  // namespace rmtc
