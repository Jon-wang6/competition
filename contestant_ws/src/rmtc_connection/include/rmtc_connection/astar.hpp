/**
 * @file astar.hpp
 * @brief 独立C++17四方向A*库，只计算路径，不接收ROS消息或执行移动。
 *
 * 参考本机ws11/src/grid_path_searcher_2d中的GridIndex/SearchResult和A*结构。
 * 比赛坐标为左上角[0,0]、x向右、y向下；路径只含上下左右相邻格。
 * 地图使用-1未知、0..100占据值，支持两种行存储顺序与未知格通行选项。
 * 资源跟踪策略显式调用本库；库本身不启动节点或发布动作。
 *
 * @details 模块协作与维护约定
 * 输入为地图快照、起终点与通行选项；返回状态、路径、代价和扩展记录，调用方再转成动作。
 * 开放队列允许旧条目残留，弹出时用g_score/closed跳过；不要求就地修改堆中的优先级。
 * 无路、越界和起终点受阻是不同状态；路径最优性的前提是当前地图与所选代价模型。
 */
#pragma once

#include <cstdint>
#include <vector>

namespace rmtc::planning {

/// 比赛整数格坐标；与ws11的y向上不同，这里y向下，不是RViz的米制坐标。
struct GridIndex {
  int x{0};
  int y{0};

  /// 比较两个格子的坐标；用于起终点判断，不受地图数据存储顺序影响。
  bool operator==(const GridIndex& other) const noexcept {
    return x == other.x && y == other.y;
  }

  /// 判断是否为不同格子；不使用浮点距离或近似比较。
  bool operator!=(const GridIndex& other) const noexcept {
    return !(*this == other);
  }
};

/// 只规定cells如何逐行排列；无论选哪种，输入/输出格坐标的y始终向下。
enum class RowOrder { TopToBottom, BottomToTop };

/**
 * @brief 一次搜索使用的地图快照，不读取裁判文件、不自动接入MapMemory。
 * cells长度须等于width*height，总格数须能用int索引；格值只接受-1或0..100。
 * 手工按比赛行顺序创建时使用TopToBottom；将来从MapMemory::cells()或当前
 * 可视化OccupancyGrid复制数据时，显式使用BottomToTop以还原比赛y坐标。
 */
struct GridMap {
  int width{0};
  int height{0};
  std::vector<std::int8_t> cells;
  RowOrder row_order{RowOrder::TopToBottom};
};

/// 一次搜索的通行规则；没有静态或跨次状态，不会修改传入的地图快照。
struct SearchOptions {
  int occupied_threshold{50};            ///< 已知格值>=阈值为障碍；有效阈值1..100。
  bool allow_unknown{false};             ///< 默认不穿过未知格；探索也可只走已知空地来扩展视野。
  int unknown_cost{3};                   ///< 进入未知格的代价，须>=1；已知自由格代价为1。
  std::vector<GridIndex> blocked_cells;  ///< 可选临时禁行格，如调用方选择避开的当前敌车。
};

/// 无路可达与输入非法分别返回，不将空路径误认为“已经到达”。
enum class SearchStatus {
  Found,
  NoPath,
  InvalidMap,
  InvalidOptions,
  StartOutOfBounds,
  GoalOutOfBounds,
  StartBlocked,
  GoalBlocked
};

/// 搜索输出；失败时path为空、cost为0，须先检查status或success()。
struct SearchResult {
  SearchStatus status{SearchStatus::NoPath};
  std::vector<GridIndex> path;     ///< 包含起点和终点，按实际行走顺序排列。
  std::vector<GridIndex> visited;  ///< 按扩展顺序保存已访问格，留给以后调试/显示。
  std::int64_t cost{0};            ///< 格子通行代价之和，不是米数，也不保证等于实际Tick数。

  /// 只有找到完整路线才返回true；起终点同格且可通行时路径长度为1、代价为0。
  bool success() const noexcept {
    return status == SearchStatus::Found;
  }
};

class AStarPlanner {
 public:
  /**
   * @brief 使用标准A*搜索一条总代价最小的四方向格子路线。
   * @param map 合法地图快照；算法只读，不修改占据值或行顺序。
   * @param start 当前起点，使用比赛格坐标；未知/障碍规则同样适用于起点。
   * @param goal 目标格，不会自动吸附到附近可通行格。
   * @param options 未知格、占据阈值和临时禁行选项；每次调用独立生效。
   * @return 路径、访问格、代价及明确状态。非法输入通过状态返回；内存分配
   *         等标准库异常保留给调用方处理，不转换成“无路可达”。
   *
   * f=g+h，h为曼哈顿距离，已知格单步代价1、未知格代价>=1，因此保证最优。
   * 等代价候选按h和入队顺序稳定排序；不斜走、不平滑成跨格直线。
   * 本类无成员状态，重复搜索不会沿用上次父节点、障碍或搜索结果。
   */
  SearchResult searchAStar(const GridMap& map, GridIndex start, GridIndex goal,
                           const SearchOptions& options = {}) const;
};

}  // namespace rmtc::planning
