/**
 * @file protocol.hpp
 * @brief 封装比赛公开JSON协议，以及策略使用的基础数据类型。
 *
 * 本文件不创建ROS节点，也不访问裁判网页、配置或内部数据。
 * 输入是ROS String消息里的JSON文本，输出是Perception、Position和Action。
 * ConnectionNode调用这里的解析/序列化函数，ProbeStrategy复用坐标转换。
 * 依赖nlohmann::json；头文件中的自由函数使用inline，避免多处包含时重复定义。
 *
 * @details 模块协作与维护约定
 * 坐标约定：比赛x向右、y向下；9×9局部实体图需以中心为原点换算后加本车位置。
 * 协议层只负责合法数据与动作的表示，不在这里作资源排序或事件战术判断。
 */
#pragma once

#include <nlohmann/json.hpp>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

namespace rmtc {

using Json = nlohmann::json;

/// 世界坐标或相对位移：x向右增加，y向下增加；具体含义由调用位置决定。
struct Position {
  int x{0};
  int y{0};
};

/**
 * @brief 将本车self.position转换成绝对坐标。
 * @param pos 形如[3,5]的JSON数组，必须恰好包含两个整数。
 * @return Position{x,y}；这里不检查坐标是否处于地图边界内。
 * @throws std::runtime_error 数组长度或元素类型不符合协议时抛出。
 *
 * 该函数只解析裁判给出的当前坐标，不累计移动，也不推算复活位置。
 * 地图尺寸与坐标范围由使用这些坐标的策略另行检查。
 */
inline Position parse_absolute_position(const Json& pos) {
  if (!pos.is_array() || pos.size() != 2 || !pos[0].is_number_integer() ||
      !pos[1].is_number_integer()) {
    throw std::runtime_error("position must be an integer [x,y] array");
  }
  return {pos[0].get<int>(), pos[1].get<int>()};
}

/**
 * @brief 从单个可见实体的9x9字符图中提取相对本车的位移。
 * @param field 实体的position字符串：81个格子字符、8个换行，共89字符。
 * @return Position{dx,dy}；与本车同格时图中没有2，返回{0,0}。
 * @throws std::runtime_error 字符串类型、长度、换行、字符或标记位置错误。
 *
 * 每张图只描述一个实体。行列从0计数时，中心为第4行第4列，
 * 因此位移为{2所在列-4,2所在行-4}，方向始终沿世界坐标轴。
 * 返回值仍是相对位移，调用方需加上self.position才能得到绝对坐标。
 */
inline Position parse_relative_position(const Json& field) {
  if (!field.is_string())
    throw std::runtime_error("entity.position must be a string");
  const auto s = field.get<std::string>();
  if (s.size() != 89)
    throw std::runtime_error("entity.position must have 89 characters");
  int twos = 0;
  int ones = 0;
  Position delta;
  for (int row = 0; row < 9; ++row) {
    for (int col = 0; col < 9; ++col) {
      // 前8行每行占10字符（9个格子加换行），最后一行没有末尾换行。
      const auto c = s[static_cast<std::size_t>(row * 10 + col)];
      if (c == '2') {
        ++twos;
        delta = {col - 4, row - 4};
      } else if (c == '1') {
        ++ones;
        if (row != 4 || col != 4)
          throw std::runtime_error("invalid entity origin");
      } else if (c != '0') {
        throw std::runtime_error("invalid entity cell");
      }
    }
    if (row < 8 && s[static_cast<std::size_t>(row * 10 + 9)] != '\n') {
      throw std::runtime_error("invalid entity newline");
    }
  }
  if (ones != 1 || twos > 1)
    throw std::runtime_error("invalid entity markers");
  return delta;
}

/**
 * @brief 一帧已经通过基础身份和定位检查的感知。
 * data保留原始公开JSON，任务、资源、血量等字段按后续策略需要读取；
 * tick/session_id/position/alive提取出来，供每帧通信流程直接使用。
 */
struct Perception {
  Json data;
  std::int64_t tick;
  std::string session_id;
  Position position;
  bool alive;

  /**
   * @brief 取得原始JSON中的本车状态对象，不复制整份JSON。
   * @return data["self"]的只读引用，有效期不超过当前Perception对象。
   * @throws nlohmann::json::exception data不含self或不是可索引对象时抛出。
   */
  const Json& self() const {
    return data.at("self");
  }

  /**
   * @brief 解析感知文本，核对场次格式、Tick、本车席位、存活标记和坐标。
   * @param text 本队perception话题中msg.data的原始JSON文本。
   * @param expected_team 节点启动时配置的席位，如A；必须与self.team一致。
   * @return 持有原始JSON及已提取基础字段的Perception对象。
   * @throws nlohmann::json::exception JSON语法错误、缺少必需字段或转换失败。
   * @throws std::runtime_error 字段类型、取值或席位不符合本程序要求。
   *
   * 这里只检查每帧必用的基础字段，地图和实体在ProbeStrategy中按需检查。
   * 不在这里判断是不是旧Tick或新场次，这些需要节点保存跨帧状态。
   * std::move(p)将JSON交给返回对象持有，避免再复制整份感知。
   */
  static Perception parse(const std::string& text, const std::string& expected_team) {
    auto p = Json::parse(text);
    if (!p.at("tick").is_number_integer() || !p.at("session_id").is_string()) {
      throw std::runtime_error("tick/session_id type mismatch");
    }
    const auto tick = p.at("tick").get<std::int64_t>();
    const auto session = p.at("session_id").get<std::string>();
    if (tick < 0 || session.empty())
      throw std::runtime_error("empty session_id or negative tick");
    const auto& self = p.at("self");
    if (!self.at("alive").is_boolean())
      throw std::runtime_error("alive must be boolean");
    if (self.at("team").get<std::string>() != expected_team) {
      throw std::runtime_error("wrong team perception");
    }
    const auto position = parse_absolute_position(self.at("position"));
    const bool alive = self.at("alive").get<bool>();
    return {std::move(p), tick, session, position, alive};
  }
};

/// 动作意图；策略负责选择type/direction，节点负责绑定当前帧的身份。
struct Action {
  std::string type{"wait"};
  std::string direction;

  /**
   * @brief 创建一个等待动作，不带方向，也不包含Tick或场次身份。
   * @return type为wait、direction为空的动作，供默认决策或异常降级使用。
   */
  static Action wait() {
    return {"wait", ""};
  }

  /**
   * @brief 创建一个移动意图，尚不向裁判发送。
   * @param direction 预期为up/down/left/right，合法性在to_json中检查。
   * @return type为move的动作；方向字符串通过移动语义保存到结果中。
   */
  static Action move(std::string direction) {
    return {"move", std::move(direction)};
  }

  /**
   * @brief 校验动作格式，并生成可以装入ROS String的动作JSON。
   * @param tick 当前感知的动作Tick，必须非负，不能使用上一帧的值。
   * @param session_id 当前感知中的非空场次ID，必须由调用方原样传入。
   * @return 包含tick、session_id、action的JSON；移动/攻击还包含direction。
   * @throws std::runtime_error 身份参数为空/非法，或动作和方向枚举非法。
   *
   * 此函数仅检查消息格式，不检查攻击保护、阶段限制或地图通行条件。
   * 等待/防御不输出direction。真正的ROS发布由ConnectionNode负责。
   */
  Json to_json(std::int64_t tick, const std::string& session_id) const {
    if (tick < 0 || session_id.empty())
      throw std::runtime_error("invalid action identity");
    if (type != "wait" && type != "move" && type != "attack" && type != "defend") {
      throw std::runtime_error("invalid action type");
    }
    Json result{{"tick", tick}, {"session_id", session_id}, {"action", type}};
    if (type == "move" || type == "attack") {
      if (direction != "up" && direction != "down" && direction != "left" && direction != "right") {
        throw std::runtime_error("invalid action direction");
      }
      result["direction"] = direction;
    }
    return result;
  }
};

}  // namespace rmtc
