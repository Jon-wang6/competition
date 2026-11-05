/**
 * @file public_data.hpp
 * @brief 定义本队完整感知与公开队伍名录的数据接口。
 *
 * 仅处理通信协议列出的公开信息。实体位置保留原始9x9图，同时提供
 * 相对位移和绝对坐标；任务、事件文本保持原文，不进行自然语言判断。
 * 数据解析函数在public_interface.cpp中实现，不创建ROS订阅或发送动作。
 * raw保留原始JSON，便于检查未单独提取的字段；optional表示字段缺省，
 * 不把“没有收到”伪装成0、false或其他确定的游戏状态。
 *
 * @details 模块协作与维护约定
 * 输入边界：实体局部坐标需转换为比赛绝对格坐标，可选字段缺失不能冒充数值0。
 * 输出供控制与显示复用；保留原始字段不意味着策略可以把未经校验的内容直接执行。
 */
#pragma once

#include "rmtc_connection/protocol.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace rmtc {

/// 上一Tick动作回执；effective_direction/error在未提供时为空。
struct ActionReceipt {
  std::int64_t tick{0};
  std::string action;
  bool moved{false};
  std::optional<std::string> effective_direction;
  std::optional<std::string> error;
  Json raw;
};

/// 本车状态；position始终直接取自裁判，不累加位移。
struct SelfInfo {
  std::string team;
  std::string robot_id;
  Position position;
  double hp{0};
  bool alive{false};
  bool defending{false};
  std::int64_t respawn_in{0};
  bool spawn_protected{false};  ///< 对应JSON的protected；protected是C++关键字。
  std::optional<std::string> position_reason;
  std::optional<ActionReceipt> last_action;  ///< 开局null回执对应nullopt。
};

/// 单个可见实体；不同类型不共有的属性使用optional，raw保留原始条目。
struct VisibleEntity {
  std::string type;
  std::string position_text;
  Position relative_position;
  Position position;  ///< self.position + relative_position得到的绝对坐标。
  std::optional<std::string> id;
  std::optional<std::string> team;
  std::optional<std::string> robot_id;
  std::optional<double> hp;
  std::optional<double> value;
  std::optional<bool> defending;
  std::optional<bool> spawn_protected;
  Json raw;
};

/// 裁判给出的当前统计快照；本结构不自行推算AI资格或最终任务得分。
struct ScoreInfo {
  double resource_value{0};
  /// v1.1的有效伤害D：已含击杀奖励和死亡扣减，可有小数、下降或超过500。
  /// 每帧直接使用裁判值；500是第二阶段战斗分满分门槛，不是D的存储上限。
  double effective_damage{0};
  std::int64_t mission_completed{0};
  /// 完成记录以裁判为准；D下降不会自行撤销已经完成的任务。
  std::vector<std::string> mission_completed_ids;
  std::int64_t invalid_actions{0};
};

/// 结构化任务；reach任务可不带target，目标坐标从position读取。
struct MissionInfo {
  std::string id;
  std::string type;
  std::optional<double> target;
  std::optional<Position> position;
  Json raw;
};

/// 公开地图尺寸，不包含全图障碍、资源或其他隐藏布局。
struct MapSize {
  int width{0};
  int height{0};
};

/// 一帧完整的公开感知；实体仅表示这一帧视野内的状态。
struct PublicPerception {
  std::int64_t tick{0};
  std::string session_id;
  int vision_radius{0};
  SelfInfo self;
  std::vector<VisibleEntity> visible_entities;
  ScoreInfo score;
  std::string mission_text;
  std::vector<MissionInfo> missions;
  MapSize map;
  std::string event_text;  ///< 公告原文，不是结构化事件参数。
  Json raw;
};

/// 队伍名录只提供席位与显示名称，不代表其他队的实时车辆状态。
struct TeamInfo {
  std::string team;
  std::string name;
};

/**
 * @brief 解析本队完整公开感知，校验字段类型并转换可见实体坐标。
 * @param text perception话题的msg.data，必须为JSON对象文本。
 * @param expected_team 预期席位A/B/C/D，用于检查self.team。
 * @return 新的完整感知对象，既含便于使用的字段，也保留原始JSON。
 * @throws std::exception 字段缺失、类型/范围错误或位置图非法时抛出。
 * 不判断Tick新旧或场次切换；这些跨帧决策由将来的调用方负责。
 */
PublicPerception parse_public_perception(const std::string& text, const std::string& expected_team);

/**
 * @brief 解析/referee/team_registry中的席位与显示名称数组。
 * @param text 形如{"teams":[{"team":"A","name":"队名"}]}的JSON文本。
 * @return 保持公告顺序的队伍列表；不把A-D之外的条目视为合法席位。
 * @throws std::exception JSON格式、字段类型、名称或席位重复等检查失败时抛出。
 */
std::vector<TeamInfo> parse_team_registry(const std::string& text);

}  // namespace rmtc
