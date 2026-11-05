/**
 * @file public_interface.cpp
 * @brief 实现完整公开数据解析，以及尚未启用的ROS接收接口。
 *
 * JSON只解析公开感知和队伍名录；任务/事件公告按纯文本原样转交。
 * 此文件编译为独立库，不加入当前connection_node，避免提前启用新接口。
 * 没有全局对象或自动订阅；只有调用PublicReceiver::start才连接ROS话题。
 *
 * @details 模块协作与维护约定
 * 输入边界：实体局部坐标需转换为比赛绝对格坐标，可选字段缺失不能冒充数值0。
 * 输出供控制与显示复用；保留原始字段不意味着策略可以把未经校验的内容直接执行。
 */
#include "rmtc_connection/public_receiver.hpp"

#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace rmtc {
namespace {

/** @brief 统一验证席位字符串；错误时抛出异常，不推测其他席位权限。 */
void validate_team(const std::string& team) {
  if (team.size() != 1 || std::string("ABCD").find(team) == std::string::npos) {
    throw std::runtime_error("team must be A, B, C or D");
  }
}

/** @brief 读取有范围约束的JSON整数，拒绝布尔、浮点及无符号溢出。 */
std::int64_t integer(const Json& value, std::int64_t low, std::int64_t high) {
  if (!value.is_number_integer() ||
      (value.is_number_unsigned() &&
       value.get<std::uint64_t>() > static_cast<std::uint64_t>(high))) {
    throw std::runtime_error("expected an integer within allowed range");
  }
  const auto result = value.get<std::int64_t>();
  if (result < low || result > high)
    throw std::runtime_error("integer outside allowed range");
  return result;
}

/** @brief 读取有限非负数，支持有效伤害和HP的小数，拒绝布尔及字符串。 */
double number(const Json& value) {
  if (!value.is_number())
    throw std::runtime_error("expected a number");
  const double result = value.get<double>();
  if (!std::isfinite(result) || result < 0)
    throw std::runtime_error("invalid non-negative number");
  return result;
}

/** @brief 严格读取字符串；不解析其中的自然语言、Tick、坐标或事件语义。 */
std::string text(const Json& value) {
  if (!value.is_string())
    throw std::runtime_error("expected a string");
  return value.get<std::string>();
}

/** @brief 严格读取JSON布尔，不将0、1或字符串自动转换成状态。 */
bool boolean(const Json& value) {
  if (!value.is_boolean())
    throw std::runtime_error("expected a boolean");
  return value.get<bool>();
}

/** @brief 读取可选字段；不存在时返回nullopt，存在但类型不对时抛异常。 */
template <typename Reader>
auto optional_field(const Json& object, const char* key, Reader reader)
    -> std::optional<decltype(reader(object.at(key)))> {
  if (!object.contains(key))
    return std::nullopt;
  return reader(object.at(key));
}

/** @brief 读取绝对整数坐标数组，限制为int可表示的非负范围。 */
Position position(const Json& value) {
  if (!value.is_array() || value.size() != 2)
    throw std::runtime_error("expected [x,y]");
  const auto limit = std::numeric_limits<int>::max();
  return {static_cast<int>(integer(value[0], 0, limit)),
          static_cast<int>(integer(value[1], 0, limit))};
}

/** @brief 检查公开绝对坐标是否在本场地图内，防止错误坐标进入后续规划。 */
void check_position(Position pos, MapSize map) {
  if (pos.x < 0 || pos.y < 0 || pos.x >= map.width || pos.y >= map.height) {
    throw std::runtime_error("position outside map");
  }
}

/** @brief 解析上一Tick回执，包括可选的实际方向及拒绝原因。 */
ActionReceipt receipt(const Json& value) {
  ActionReceipt result;
  result.tick = integer(value.at("tick"), 0, std::numeric_limits<std::int64_t>::max());
  result.action = text(value.at("action"));
  if (result.action != "move" && result.action != "attack" && result.action != "defend" &&
      result.action != "wait") {
    throw std::runtime_error("invalid receipt action");
  }
  result.moved = boolean(value.at("moved"));
  result.effective_direction = optional_field(value, "effective_direction", text);
  result.error = optional_field(value, "error", text);
  result.raw = value;
  return result;
}

/** @brief 解析完整本车状态，保留缺省的position_reason和开局null回执。 */
SelfInfo self_info(const Json& value) {
  SelfInfo result;
  result.team = text(value.at("team"));
  result.robot_id = text(value.at("robot_id"));
  result.position = position(value.at("position"));
  result.hp = number(value.at("hp"));
  result.alive = boolean(value.at("alive"));
  result.defending = boolean(value.at("defending"));
  result.respawn_in = integer(value.at("respawn_in"), 0, std::numeric_limits<std::int64_t>::max());
  result.spawn_protected = boolean(value.at("protected"));
  result.position_reason = optional_field(value, "position_reason", text);
  const auto& last_action = value.at("last_action");
  if (!last_action.is_null())
    result.last_action = receipt(last_action);
  return result;
}

/**
 * @brief 直接读取裁判统计快照，保留完成任务ID列表，不自行计算最终得分。
 * v1.1的effective_damage已结算击杀奖励和死亡扣减，可下降且可超过500。
 * 此处保留浮点值，不跨帧累加、不截断，也不重复加减奖励或撤销任务。
 */
ScoreInfo score_info(const Json& value) {
  ScoreInfo result;
  result.resource_value = number(value.at("resource_value"));
  result.effective_damage = number(value.at("effective_damage"));
  result.mission_completed =
      integer(value.at("mission_completed"), 0, std::numeric_limits<std::int64_t>::max());
  result.invalid_actions =
      integer(value.at("invalid_actions"), 0, std::numeric_limits<std::int64_t>::max());
  const auto& ids = value.at("mission_completed_ids");
  if (!ids.is_array())
    throw std::runtime_error("mission_completed_ids must be an array");
  for (const auto& id : ids)
    result.mission_completed_ids.push_back(text(id));
  return result;
}

/** @brief 解析一个可见实体，并将相对位置转换为当前帧的绝对坐标。 */
VisibleEntity visible_entity(const Json& value, const PublicPerception& frame) {
  VisibleEntity result;
  result.type = text(value.at("type"));
  result.position_text = text(value.at("position"));
  result.relative_position = parse_relative_position(result.position_text);
  const auto [dx, dy] = result.relative_position;
  if (std::abs(dx) > frame.vision_radius || std::abs(dy) > frame.vision_radius) {
    throw std::runtime_error("visible entity outside announced vision radius");
  }
  const auto x = static_cast<std::int64_t>(frame.self.position.x) + dx;
  const auto y = static_cast<std::int64_t>(frame.self.position.y) + dy;
  if (x < 0 || y < 0 || x >= frame.map.width || y >= frame.map.height) {
    throw std::runtime_error("visible entity outside map");
  }
  result.position = {static_cast<int>(x), static_cast<int>(y)};
  result.id = optional_field(value, "id", text);
  result.team = optional_field(value, "team", text);
  result.robot_id = optional_field(value, "robot_id", text);
  result.hp = optional_field(value, "hp", number);
  result.value = optional_field(value, "value", number);
  result.defending = optional_field(value, "defending", boolean);
  result.spawn_protected = optional_field(value, "protected", boolean);
  result.raw = value;
  return result;
}

/** @brief 解析结构化任务；reach可无target，未知类型仍保留type和raw供以后处理。 */
MissionInfo mission_info(const Json& value) {
  MissionInfo result;
  result.id = text(value.at("id"));
  result.type = text(value.at("type"));
  result.target = optional_field(value, "target", number);
  result.position = optional_field(value, "position", position);
  result.raw = value;
  return result;
}

}  // namespace

/**
 * @brief 将本队感知JSON转换为类型化数据，同时保留原始JSON以供查询新增字段。
 * 校验席位、场次、Tick、视野、位置和实体类型；不从自然语言中推测隐藏状态。
 * 缺失必需字段或类型/范围错误时抛异常，调用方决定记录警告还是回退等待。
 */
PublicPerception parse_public_perception(const std::string& input,
                                         const std::string& expected_team) {
  validate_team(expected_team);
  auto raw = Json::parse(input);
  PublicPerception result;
  result.tick = integer(raw.at("tick"), 0, std::numeric_limits<std::int64_t>::max());
  result.session_id = text(raw.at("session_id"));
  if (result.session_id.empty())
    throw std::runtime_error("empty session_id");
  result.vision_radius = static_cast<int>(integer(raw.at("vision_radius"), 0, 4));
  result.self = self_info(raw.at("self"));
  if (result.self.team != expected_team)
    throw std::runtime_error("wrong team perception");
  const auto& map = raw.at("map");
  result.map.width = static_cast<int>(integer(map.at("width"), 1, std::numeric_limits<int>::max()));
  result.map.height =
      static_cast<int>(integer(map.at("height"), 1, std::numeric_limits<int>::max()));
  check_position(result.self.position, result.map);
  const auto& entities = raw.at("visible_entities");
  if (!entities.is_array())
    throw std::runtime_error("visible_entities must be an array");
  for (const auto& value : entities)
    result.visible_entities.push_back(visible_entity(value, result));
  result.score = score_info(raw.at("score"));
  result.mission_text = text(raw.at("mission_text"));
  const auto& missions = raw.at("missions");
  if (!missions.is_array())
    throw std::runtime_error("missions must be an array");
  for (const auto& value : missions)
    result.missions.push_back(mission_info(value));
  result.event_text = text(raw.at("event_text"));
  result.raw = std::move(raw);
  return result;
}

/**
 * @brief 解析公开队伍注册表，校验席位唯一性和显示名长度/换行限制。
 * 仅返回实际公布的队伍；无效结构或重复席位抛异常，由接收层统一报告。
 */
std::vector<TeamInfo> parse_team_registry(const std::string& input) {
  const auto raw = Json::parse(input);
  const auto& teams = raw.at("teams");
  if (!teams.is_array())
    throw std::runtime_error("teams must be an array");
  std::set<std::string> seats;
  std::vector<TeamInfo> result;
  for (const auto& value : teams) {
    TeamInfo entry{text(value.at("team")), text(value.at("name"))};
    validate_team(entry.team);
    if (!seats.insert(entry.team).second)
      throw std::runtime_error("duplicate team seat");
    if (entry.name.empty() || entry.name.size() > 96 ||
        entry.name.find_first_of("\r\n") != std::string::npos) {
      throw std::runtime_error("invalid team display name");
    }
    result.push_back(std::move(entry));
  }
  return result;
}

/// 保存回调并按合法席位生成本队感知话题；此时不订阅，start后才开始接收。
PublicReceiver::PublicReceiver(std::string team, PublicCallbacks callbacks)
    : team_(std::move(team)), callbacks_(std::move(callbacks)) {
  validate_team(team_);
  topics_.perception = "/team_" + team_ + "/perception";
}

/// 析构时释放本接收器持有的订阅，不影响传入节点的其他通信。
PublicReceiver::~PublicReceiver() {
  stop();
}

/**
 * @brief 在给定节点创建四个公开话题订阅，已启动时直接返回。
 * 感知只接收实时数据，任务/注册表保留最新值，事件保留一定历史；具体QoS见下方。
 * 创建失败时释放已创建的订阅再抛出异常，避免出现部分启动状态。
 */
void PublicReceiver::start(rclcpp::Node& node) {
  if (active())
    return;
  try {
    const auto live = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    const auto retained = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    const auto events = rclcpp::QoS(rclcpp::KeepLast(128)).reliable().transient_local();
    perception_sub_ = node.create_subscription<std_msgs::msg::String>(
        topics_.perception, live, [this](const std_msgs::msg::String::SharedPtr msg) {
          receive(topics_.perception, [this, &msg] {
            const auto value = parse_public_perception(msg->data, team_);
            if (callbacks_.on_perception)
              callbacks_.on_perception(value);
          });
        });
    mission_sub_ = node.create_subscription<std_msgs::msg::String>(
        topics_.mission, retained, [this](const std_msgs::msg::String::SharedPtr msg) {
          receive(topics_.mission, [this, &msg] {
            if (callbacks_.on_mission_text)
              callbacks_.on_mission_text(msg->data);
          });
        });
    event_sub_ = node.create_subscription<std_msgs::msg::String>(
        topics_.event_text, events, [this](const std_msgs::msg::String::SharedPtr msg) {
          receive(topics_.event_text, [this, &msg] {
            if (callbacks_.on_event_text)
              callbacks_.on_event_text(msg->data);
          });
        });
    registry_sub_ = node.create_subscription<std_msgs::msg::String>(
        topics_.team_registry, retained, [this](const std_msgs::msg::String::SharedPtr msg) {
          receive(topics_.team_registry, [this, &msg] {
            const auto value = parse_team_registry(msg->data);
            if (callbacks_.on_team_registry)
              callbacks_.on_team_registry(value);
          });
        });
  } catch (...) {
    stop();
    throw;
  }
}

/// 释放全部订阅，可重复调用；回调配置与席位保留，允许稍后重新start。
void PublicReceiver::stop() noexcept {
  perception_sub_.reset();
  mission_sub_.reset();
  event_sub_.reset();
  registry_sub_.reset();
}

/// 仅当四个订阅都已创建时返回true；不表示已经收到裁判消息。
bool PublicReceiver::active() const noexcept {
  return perception_sub_ && mission_sub_ && event_sub_ && registry_sub_;
}

/// 返回本接收器的话题配置，只读引用在接收器生命周期内有效。
const PublicTopics& PublicReceiver::topics() const noexcept {
  return topics_;
}

/**
 * @brief 统一执行解析/用户回调；标准异常转为话题警告并通知可选错误回调。
 * 错误回调本身的标准异常也会被记录，避免破坏ROS接收循环。
 */
void PublicReceiver::receive(const std::string& topic,
                             const std::function<void()>& operation) const {
  try {
    operation();
  } catch (const std::exception& error) {
    RCLCPP_WARN(rclcpp::get_logger("rmtc_public_receiver"), "%s: %s", topic.c_str(), error.what());
    if (callbacks_.on_error) {
      try {
        callbacks_.on_error(topic, error.what());
      } catch (const std::exception& callback_error) {
        RCLCPP_WARN(rclcpp::get_logger("rmtc_public_receiver"), "error callback: %s",
                    callback_error.what());
      }
    }
  }
}

}  // namespace rmtc
