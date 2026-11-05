/**
 * @file resource_config.hpp
 * @brief 资源策略统一配置；改动后在/home/jon/robot/competition根目录colcon build。
 *
 * 切换算法只改kDefaultResourceOrdering这一行：
 * Tour=整体顺序，Nearest=最近资源，Weighted=采集与探索统一加权；Joint=覆盖联合规划。
 * launch未显式指定策略时使用本文件；k_edge/k_value仅覆盖Weighted的对应参数。
 * Joint参数直接在下方修改，重新编译并重启选手端后生效。
 *
 * @details 模块协作与维护约定
 * 生效顺序：这里提供编译期默认值；启动参数显式覆盖时，修改默认值未必改变最终策略。
 * 权重是选手的排序偏好，不能修改裁判得分；比较实验时应记录实际模式与覆盖参数。
 */
#pragma once

namespace rmtc {

/// 保留原策略作对照；Joint仅用于第一阶段静态资源练习。
enum class ResourceOrdering { Tour, Nearest, Weighted, Joint };

// 平时只改下面这一行即可更换默认算法，然后重新编译并重启选手端。
inline constexpr ResourceOrdering kDefaultResourceOrdering = ResourceOrdering::Joint;

// ===== Joint当前默认策略：同一条路线中安排采集与观察点 =====
// 当前按用户要求回到单点移位前的Joint版本（Relocate=false），进行相同地图的速度对比。
// 上一轮4096张独立图：此版本全采集率93.38%；启用单点移位为93.46%，两版结果均已保存。
// 上一轮结果：runtime/stage1-95-20261025-191053/report.json。
// Joint只使用本队公开记忆；对未知区域的估计不是真实全图，实际每次只走已知安全一步。
// 以下是Joint参数，原有Weighted参数仅在切回ResourceOrdering::Weighted后生效。
inline constexpr double kJointInsertionOffset =
    12.0;  // 插入探索点时的固定成本，避免把零散小收益排到前面。
inline constexpr double kJointGainExponent =
    1.0;  // 新覆盖格数的指数；插入比较为(额外路程+固定成本)/格数^指数。
inline constexpr double kJointDiscoveryUrgency =
    0.0;  // 未知区域平均首次观察时间的代价，0关闭此项；未来资源项仍独立生效。
inline constexpr bool kJointPreferInformationPaths = true;  // 等长路径优先开图，不额外增加步数。
inline constexpr double kJointPathPotentialExponent = 1.0;  // 视野潜力的曲线，1为线性。
inline constexpr double kJointPathEdgeBonus = 0.0;  // 等长路径对边缘未知格的额外偏好；0关闭。
inline constexpr int kJointPathEdgeBand = 1;        // 边缘偏好的作用宽度，单位为格。
inline constexpr double kJointFutureResourceWeight =
    0.4;  // 估计尚未发现资源将来插入路线的额外距离，属于假设并非真值。
inline constexpr int kJointExpectedResources =
    16;  // 第一阶段公开规则；0表示始终以完整覆盖作为目标。
inline constexpr double kJointCausalFutureFraction =
    1.0;  // 将来采集成本按首次看见之后的剩余路线估算的比例；0保留旧估算，1全部使用新估算。
inline constexpr double kJointRefreshMargin =
    0.0;  // 新探索点布局比保留路线改善超过此值才切换；负数关闭比较，防止路线抖动。

inline constexpr bool kJointRelocate = false;  // 联合代价优化时允许单点移位，补充整段反转。

// ===== Weighted模式：资源与探索统一比较，常用系数集中在这里 =====
// 代价 = 路程系数×A*距离 + 边缘系数×边缘距离
//      - 价值系数×(资源价值 + 未知格基础价值×加权新增格数×时间折扣)/10。
// 加权新增格数=Σ[1+B×max(0,1-d/L)]，只加目标视野内仍未知的地图格。
// d是未知格到地图四条外边的最短距离，不是机器人目标位置的边缘距离。
// 数字越小越优先；纯探索点的资源价值为0，资源位置同样计算新增视野奖励。
// 有已知资源时：纯探索代价必须<0且低于资源代价，收益不够时先采资源。
// 没有可达资源时：继续探索兜底，不受上述收益门槛影响。
// 第一阶段多图调参：41张训练图、128张独立验证图；评价采集、覆盖率和路过漏区。
// 种子50回归：89Tick收齐200价值并观察400格。结果在runtime/multimap-tuning。
inline constexpr double kDefaultDistanceWeight =
    1.0;  // 保留路程基准；同时缩放距离/边缘/价值不改变排序。
inline constexpr double kDefaultEdgeWeight = 0.5;    // 采集目标的边缘偏好；不强制沿边走。
inline constexpr double kDefaultValueWeight = 0.75;  // 真实资源奖励；距离仍为主要成本。
inline constexpr double kDefaultUnknownCellValue =
    0.0;  // 关闭大范围探索竞争；仍保留补看和无资源探索。
inline constexpr int kDefaultExplorationFadeTicks =
    100;  // 信息奖励随剩余时间逐步降低，末段优先采集。
inline constexpr double kDefaultUnknownEdgeBonus = 2.0;  // B：最外圈未知格最高3倍，0不加成。
inline constexpr int kDefaultUnknownEdgeBand = 1;  // L：仅加成最外一圈，避免整片内侧都被抬高。
inline constexpr double kDefaultPeekReward =
    0.2;  // 顺路补看系数：只奖励原采集路线看不到的新格，0关闭。
inline constexpr int kDefaultPeekDetourLimit = 2;  // 两次实际采集之间，补看累计最多额外绕路几步。
inline constexpr int kDefaultPeekPatchLimit =
    80;  // 付费补看仅面向不超过此格数的未知块，避免追逐大片前沿；0不限制。
// 调参建议：UnknownCellValue接近0时优先采集，增大后才可能主动探索；不是探索概率。
// UnknownCellValue与ValueWeight共同控制整体探索；PeekReward独立控制采集途中的小补看。
// 奖励须超过探索路程/边缘成本，且探索代价低于采集代价；并非每次小幅调整都会换动作。
// 这些是实验参数，不保证某张练习图或所有隐藏地图上最优。

// 以下仅供Tour/Nearest没有资源时的旧探索兜底，Weighted统一使用上面那组系数。
// 兜底代价=路程系数×实际步数+边缘系数×距地图外边缘。
// 分数越小越优先；增加边缘系数更偏向贴边，增加路程系数更偏向就近探索。
// 只有能揭开未知格的位置才参加比较，同分再按新增视野格数排序。
inline constexpr double kExplorationDistanceWeight = 1.0;
inline constexpr double kExplorationEdgeWeight = 0.5;

/** @brief 把策略枚举转成ROS参数名；非法枚举返回invalid，由启动校验拒绝。 */
constexpr const char* resource_order_name(ResourceOrdering ordering) {
  switch (ordering) {
    case ResourceOrdering::Tour:
      return "tour";
    case ResourceOrdering::Nearest:
      return "nearest";
    case ResourceOrdering::Weighted:
      return "weighted";
    case ResourceOrdering::Joint:
      return "joint";
  }
  return "invalid";
}

}  // namespace rmtc
