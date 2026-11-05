/**
 * @file stage2_planning.hpp
 * @brief 独立预测规划模式：以相同Tick窗口比较移动与停留后的采集机会。
 *
 * 只读取本队地图记忆。资源存在性与到达竞争分别估计，均不是校准后的真实概率。
 * 束搜索逐格展开，同一资源/未知格只计一次；首步仍交给原动作层检查攻击和事件风险。
 *
 * @details 模块协作与维护约定
 * 同一帧比较WAIT/UP/DOWN/LEFT/RIGHT五个根动作，按共同完成的搜索深度提交结果。
 * 资源计分与任务阈值在各采集结果分支中计算后再求期望，避免先平均再封顶造成高估。
 * 时间预算使用毫秒，路线到达时间使用相对Tick；预算不足时返回有效的已完成结果供上层回退。
 */
#pragma once
#include "rmtc_connection/map_memory.hpp"
#include "rmtc_connection/stage2_collection_forecast.hpp"
#include "rmtc_connection/stage2_neural.hpp"
#include <array>
#include <functional>
#include <optional>

namespace rmtc {
struct Stage2Weights;

/// 新模式独立参数；默认关闭，旧stage2和第一阶段不启用本规划器。
struct PlanningWeights {
  bool enabled{false};
  bool persistent_escape{false};   ///< 脱困目标跨Tick保持，抵达/失效/取得收益后才结束。
  bool race_scenarios{false};      ///< 每种情景内让敌人走完整采集路线，避免同时争抢所有资源。
  bool regional_exploration{false}; ///< 未知资源按分区观测和敌人来路估计，保持有限探索机会。
  int escape_ticks{20};            ///< 脱困目标的最短有效窗口；仍允许事件撤离覆盖。
  int terminal_stops{1};           ///< 完成束搜索后延伸的资源个数，1严格沿用旧末端估值。
  int race_memory_ticks{20};        ///< 失踪对手参与软竞争情景的最大年龄。
  double race_scenario_weight{0.5}; ///< 敌方共同路线与原静态距离估计的混合比例。
  double regional_strength{0.5};   ///< 分区证据相对均匀先验的混合比例。
  NeuralWeights collection_neural; ///< 独立采集评分器及教师采样；不共享战斗模型或样本。
  bool tactical_bridge{true};       ///< 采集预测生效时保留不重复计资源收益的战斗前瞻。
  bool independent_opponents{true}; ///< 对手情景是否分别分配，独立开关供对照。
  bool exact_death_loss{true};      ///< 独立校准死亡扣分，便于消融而不捆绑搜索器。
  bool distribution_state{true};    ///< 去重必须检查概率分布，不能仅比较几何状态。
  bool safe_terminal{true};         ///< 窗口外机会也核验到达时通行性与路线危险。
  bool continue_baseline{true};     ///< 原路线到点后继续采集/观察，不在预测里一直等待。
  bool reach_recovery{true};        ///< 长期互防无进展时允许有限的脱离射线试探。
  int recovery_ticks{16};           ///< 真实连续防御无掉血的观察窗口，不代表敌人必不攻击。
  int memory_model{1};              ///< 0沿用旧记忆折扣；1独立存在性估计；供消融/训练。
  int horizon{12};                 ///< 所有根动作共同的前瞻Tick数，含当前动作。
  int beam_width{16};              ///< 每个根动作每层保留的路线数。
  int max_nodes{12000};            ///< 确定性展开上限；超限丢弃未完成的整层。
  double budget_ms{30.0};          ///< 墙钟仅作兜底，正常由节点数控制工作量。
  double discount{0.97};           ///< 未来机会折扣，不改变资源本身的真实积分。
  double terminal_weight{0.55};    ///< 窗口外最近后续机会的保守估值。
  double memory_half_life{80.0};   ///< 没有敌人接近证据时，旧资源存在性估计的半衰期。
  double enemy_pressure{2.0};      ///< 可见敌人靠近时，加快旧资源存在性衰减。
  double exploration_floor{0.15};  ///< 保留少量未知区域机会；仍乘原资源剩余机会估计。
  double route_tie_margin{0.005};  ///< 微小收益差时保留原A*首步，不进入后续价值估计。
  double future_risk_weight{0.15}; ///< 仅第二步以后扣路线风险，首步由原动作层负责。
  /// 校验计算规模及有限值，启动时拒绝不可能在Tick内完成的配置。
  void validate() const;
};

/// 计算正式封顶战斗分的死亡损失；不包含等待复活、位置或任务的机会损失。
double planning_damage_loss(double damage, double cap, double points, double deduction = 60.0);
/// 分离的存在性估计：亲眼看到时为1，无敌人距离证据时不伪造敌人已来过。
double planning_resource_presence(int age, int enemy_steps, const PlanningWeights& weights);

struct PlanningRoot {
  bool valid{false};
  double value{0}, resource_points{0}, mission_points{0}, information{0}, terminal{0};
  std::vector<Position> path; ///< 含当前起点；仅供解释预测，不当作已执行轨迹。
};

/// 根顺序为WAIT、UP、DOWN、LEFT、RIGHT；ATTACK/DEFEND共享WAIT的采集后续价值。
struct PlanningResult {
  bool ran{false}, budget_hit{false}, useful{false};
  int completed_depth{0}, expanded{0};
  double elapsed_ms{0};
  std::array<PlanningRoot,5> roots;
  Json resources = Json::array();
  Json competition = Json::object(); ///< 整条敌方路线共同情景，不是真实未来位置。
  bool terminal_extended{false}; ///< 所有根动作都完成相同末端扩展时才为true。
  /// 生成无密钥、无私有裁判状态的解释日志，包括完整层数与每个根动作的预测。
  Json describe() const;
};

/**
 * @brief 用相同前瞻长度计算五个根动作的后续采集价值，每Tick只执行一个动作。
 * @param frame 当前本队公开帧，坐标为比赛x右/y下。
 * @param memory 当前公开记忆；地图尺寸应与frame一致。
 * @param weights 已校验的策略参数，本函数不改变它。
 * @param remaining 本场剩余可行动Tick，非正时不运行搜索。
 * @param normal_collected 已采集普通资源数量。
 * @param high_collected 已采集高价值资源数量。
 * @param resource_weight 采集积分偏好，不修改实际资源计分。
 * @param mission_weight 任务机会偏好，不授予任务资格。
 * @param enemy_steps 本队已知地形上最快可见敌车的距离场，按裁判y向下逐行排列。
 * @param passable 查询第arrival步的通行性；未知地形必须返回false。
 * @param information 对累计唯一未知格估值，属于机会价值而非正式开图积分。
 * @param future_risk 第arrival步的软危险，首步不在此重复扣除。
 * @param exploration_target 原策略探索目标；用于窗口外估值，保留原规划提案。
 * @param unknown_supply 每格未知供给估计，可为空；非空时须与地图格数一致。
 * @param original_path 原有路线建议，供比较/保留，不代表已执行路线。
 * @param competition 可空的共同敌方采集情景，避免独立资源预测相互矛盾。
 * @return 搜索状态、共同完成深度和各根动作评分；调用方检查useful后才采用。
 * 不假设能知道未来观察；未知格只估值，不在本次预测里虚构资源或空地。
 */
PlanningResult plan_collection_actions(
    const PublicPerception& frame, const MapMemory& memory, const Stage2Weights& weights,
    int remaining, int normal_collected, int high_collected,
    double resource_weight, double mission_weight, const std::vector<int>& enemy_steps,
    const std::function<bool(Position,int)>& passable,
    const std::function<double(double,double)>& information,
    const std::function<double(Position,int)>& future_risk,
    const std::vector<double>& unknown_supply,
    std::optional<Position> exploration_target = std::nullopt,
    const std::vector<Position>& original_path = {},
    const CollectionRaceForecast* competition = nullptr);
}  // namespace rmtc
