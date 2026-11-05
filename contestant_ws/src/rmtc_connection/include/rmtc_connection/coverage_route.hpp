/**
 * @file coverage_route.hpp
 * @brief 采集/覆盖联合路线；仅使用公开记忆，未观察地形只用于估算远期路程。
 *
 * 已知资源先安排整体顺序，再插入必要观察点；等长道路优先多看未知区域。
 * 对尚未发现资源的插入路程逐格估计，只是均匀假设，不读取真实布局。
 * 这是滚动启发式规划，不保证未知地图全局最优或每张图都能在100Tick收齐。
 *
 * @details 模块协作与维护约定
 * 规划把资源访问与补看位置放在同一条路线中，覆盖收益按集合去重，避免重复奖励同一未知格。
 * 远期距离可对未知地形作乐观估计，但实际下一步必须服从已观察通行性；发现新墙需更新距离缓存。
 */
#pragma once
#include "rmtc_connection/resource_route.hpp"
#include "rmtc_connection/resource_config.hpp"

namespace rmtc::planning {
/// 联合路线参数；探索点按增加路程/新增覆盖比较，不改变100Tick的比赛时限。
struct CoverageWeights {
  double insertion_offset{kJointInsertionOffset};  ///< 插入观察点的固定成本，和额外路程一起计分。
  double gain_exponent{kJointGainExponent};        ///< 新增覆盖格数的指数，作用于插入代价的分母。
  int starts{1};  ///< 大规模非精确路线的确定性起点数，不是地图种子。
  int expected_resources{
      kJointExpectedResources};  ///< 第一阶段公开固定资源数；0表示不使用数量停止探索。
  double discovery_urgency{kJointDiscoveryUrgency};  ///< 每格未知区域平均首次观察时间的权重。
  bool prefer_information_paths{
      kJointPreferInformationPaths};  ///< 等长道路优先多看未知格，不是概率或连续奖励系数。
  double path_edge_bonus{kJointPathEdgeBonus};  ///< 对靠地图外边的未知格增加同路程补看偏好。
  int path_edge_band{kJointPathEdgeBand};       ///< 外边缘加成作用的格数范围，至少为1。
  double path_potential_exponent{kJointPathPotentialExponent};  ///< 等长路径视野潜力的指数。
  double future_resource_weight{
      kJointFutureResourceWeight};  ///< 逐格平均未知区域，估算未来插入采集路线的额外路程。
  double causal_future_fraction{
      kJointCausalFutureFraction};  ///< 新资源只能在首次进入视野后安排采集，按此比例混合其回头成本。
  double refresh_margin{
      kJointRefreshMargin};       ///< 允许重选探索点布局的最小收益；负数保持旧缓存行为。
  bool relocate{kJointRelocate};  ///< 允许把单个目标移到其他访问位置，保持完整预计覆盖。
};
/// 本帧执行段和整条计划；远期未知路段是假设，必须逐帧重新观察和规划。
struct CoverageTour {
  SearchResult first_leg;         ///< 到第一个计划点的估算路径；只保证可执行首步已知安全。
  std::string resource_key;       ///< 首计划点为实际资源时填键；纯观察点留空。
  std::vector<std::string> keys;  ///< 整条计划中的实际资源访问顺序，不包含纯观察点。
  int cost{0};                    ///< 整条开放路线的估算步数；未知障碍可能使真实路程更长。
  double objective{0};       ///< 同帧同权重下的路线比较分数，包含未来资源估算，不是实际比赛得分。
  std::size_t uncovered{0};  ///< 预计走完整条路线后仍未观察的格数，用于防止换路线造成遗漏。
};
class CoverageRoutePlanner {
 public:
  /// 清除距离表和保留的路线；由上层在换场或显式重置策略时调用。
  void reset();
  /**
   * @brief 先安排资源顺序，再插入观察位置；小规模精确排序，大规模局部优化。
   * 已知障碍始终禁行，未知格仅在远期路线中按空地估算；返回的第一步必须已知可走。
   * known_resources为本局累计实际看到的不同资源数，不能传入裁判隐藏资源列表。
   */
  CoverageTour plan(const GridMap& grid, GridIndex start,
                    const std::vector<ResourceGoal>& resources, int radius, int known_resources,
                    const std::vector<GridIndex>& blocked, CoverageWeights weights = {});

 private:
  /// 尺寸或障碍集合变化时重建全对BFS距离；只供plan在完成输入校验后调用。
  void update_distances(int width, int height, const std::vector<bool>& walls);

  int width_{0}, height_{0};
  std::vector<bool> walls_;                   ///< 已知障碍与当前可见机器人，不把未知格当作障碍。
  std::vector<std::vector<int>> distances_;   ///< 与walls_对应的单位步长距离表。
  std::vector<int> route_, resource_points_;  ///< 上帧计划点和资源点，用于复用及清除失效目标。
};
}  // namespace rmtc::planning
