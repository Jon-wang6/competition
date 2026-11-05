/**
 * @file protocol_test.cpp
 * @brief 不启动ROS或裁判的协议解析与移动策略测试。
 *
 * 使用人工构造的公开消息，验证坐标方向、坏消息拒绝、动作身份绑定，
 * 以及固定向下策略、下方受阻/边界等待、死亡等待和重置行为。
 * main独立编译为protocol_test，由CTest运行，不参与参赛节点的逐帧执行。
 *
 * @details 模块协作与维护约定
 * 验证对象：probe_strategy.hpp。
 * 运行约定：由独立测试可执行文件/CTest执行，断言失败用于定位回归，不是正式选手的逐帧逻辑。
 * 结果边界：通过表示这些构造场景满足约束，不证明所有随机地图均最优或稳定达到某个总分。
 */
#include "rmtc_connection/probe_strategy.hpp"

#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>

using rmtc::Json;

/**
 * @brief 测试断言：条件不满足时抛出带说明的异常。
 * @param condition 当前检查是否通过。
 * @param message 检查失败时交给测试入口打印的原因。
 * @throws std::runtime_error condition为false时抛出。
 * 使用函数而非assert，确保Release/NDEBUG构建下测试也不会被禁用。
 */
void require(bool condition, const std::string& message) {
  if (!condition)
    throw std::runtime_error(message);
}

/**
 * @brief 验证一项非法输入会被被测函数通过标准异常拒绝。
 * @param operation 待执行的操作，由调用处的Lambda封装参数。
 * @param message 操作没有抛出标准异常时使用的失败说明。
 * @throws std::runtime_error operation没有抛出std::exception时由require抛出。
 * 不限定异常的具体派生类型，以兼容JSON库异常和业务校验异常。
 */
void rejects(const std::function<void()>& operation, const std::string& message) {
  bool rejected = false;
  try {
    operation();
  } catch (const std::exception&) {
    rejected = true;
  }
  require(rejected, message);
}

/**
 * @brief 生成测试用的9x9实体位置图。
 * @param dx 实体相对中心的横向位移，调用方应传入[-4,4]。
 * @param dy 实体相对中心的纵向位移，调用方应传入[-4,4]。
 * @return 共89字符的字符串，中心为1；非同格实体位置标记2。
 * 此辅助函数不校验参数范围，也不保证越界输入能表达一个有效实体。
 */
std::string entity_map(int dx, int dy) {
  std::string result;
  for (int row = 0; row < 9; ++row) {
    if (row)
      result += '\n';
    for (int col = 0; col < 9; ++col) {
      result += row == 4 && col == 4 ? '1' : (row == dy + 4 && col == dx + 4 ? '2' : '0');
    }
  }
  return result;
}

/**
 * @brief 构造一帧满足当前解析器和移动策略要求的最小测试感知。
 * @return 独立JSON对象：A席位、Tick9、位置[1,1]、18x18地图、无可见实体。
 * 每次调用都重新构造，测试可自由修改字段而不影响其他用例。
 * 坐标、尺寸和场次仅为单元测试数据，不是参赛程序的地图配置。
 */
Json sample() {
  return {{"tick", 9},
          {"session_id", "test-match"},
          {"self", {{"team", "A"}, {"alive", true}, {"position", {1, 1}}}},
          {"map", {{"width", 18}, {"height", 18}}},
          {"visible_entities", Json::array()}};
}

/**
 * @brief 按组执行协议与策略检查，将任一失败转为CTest可识别的退出状态。
 * @return 全部检查通过返回0；捕获到标准异常并打印原因后返回1。
 * 不建立ROS连接，也不向任何比赛话题发布人工测试动作。
 */
int main() {
  try {
    // 第一组：世界坐标方向、同格实体及非法位置图的处理。
    const auto delta = rmtc::parse_relative_position(entity_map(1, -2));
    require(delta.x == 1 && delta.y == -2, "relative position axes changed");
    const auto overlap = rmtc::parse_relative_position(entity_map(0, 0));
    require(overlap.x == 0 && overlap.y == 0, "overlapping entity is not origin");
    rejects([] { rmtc::parse_relative_position("bad"); }, "short position accepted");
    auto broken = entity_map(1, 0);
    broken[9] = '0';
    rejects([&] { rmtc::parse_relative_position(broken); }, "bad newline accepted");
    broken = entity_map(1, 0);
    broken[0] = '2';
    rejects([&] { rmtc::parse_relative_position(broken); }, "multiple entities accepted");
    rejects([] { rmtc::parse_absolute_position(Json::array({true, 1})); },
            "boolean coordinate accepted");
    // 第二组：感知格式和身份校验，必须拒绝坏JSON、错误席位及错误字段类型。
    rejects([] { rmtc::Perception::parse("{", "A"); }, "invalid JSON accepted");
    rejects([] { rmtc::Perception::parse(sample().dump(), "B"); }, "wrong team accepted");
    auto input = sample();
    input["tick"] = "9";
    rejects([&] { rmtc::Perception::parse(input.dump(), "A"); }, "string tick accepted");
    input = sample();
    input["session_id"] = "";
    rejects([&] { rmtc::Perception::parse(input.dump(), "A"); }, "empty session accepted");

    // 第三组：正确感知的字段提取；重复决定/重置都保持向下，不再绕圈。
    const auto frame = rmtc::Perception::parse(sample().dump(), "A");
    require(frame.tick == 9 && frame.position.x == 1 && frame.position.y == 1,
            "perception identity or absolute position changed");
    rmtc::ProbeStrategy strategy;
    require(strategy.decide(frame).direction == "down", "first move is not down");
    require(strategy.decide(frame).direction == "down", "straight-down strategy changed direction");
    strategy.reset();
    require(strategy.decide(frame).direction == "down", "new session changed downward direction");

    // 第四组：侧面障碍不影响直行，下方障碍/车辆及底边只等待，资源不阻挡。
    input = sample();
    input["visible_entities"].push_back({{"type", "obstacle"}, {"position", entity_map(1, 0)}});
    strategy.reset();
    require(strategy.decide(rmtc::Perception::parse(input.dump(), "A")).direction == "down",
            "side obstacle changed downward direction");
    input["visible_entities"][0]["position"] = entity_map(0, 1);
    require(strategy.decide(rmtc::Perception::parse(input.dump(), "A")).type == "wait",
            "downward obstacle did not stop robot");
    input["visible_entities"][0]["type"] = "robot";
    require(strategy.decide(rmtc::Perception::parse(input.dump(), "A")).type == "wait",
            "downward robot did not stop robot");
    input["visible_entities"][0]["type"] = "resource_normal";
    require(strategy.decide(rmtc::Perception::parse(input.dump(), "A")).direction == "down",
            "resource incorrectly blocked downward movement");
    input["self"]["position"] = {17, 17};
    input["visible_entities"] = Json::array();
    strategy.reset();
    require(strategy.decide(rmtc::Perception::parse(input.dump(), "A")).type == "wait",
            "bottom edge caused a turn or out-of-bounds move");
    input["self"]["alive"] = false;
    require(strategy.decide(rmtc::Perception::parse(input.dump(), "A")).type == "wait",
            "dead robot attempted move");

    // 第五组：动作绑定当前帧身份，等待不带方向，非法方向不能序列化提交。
    const auto action = rmtc::Action::move("right").to_json(frame.tick, frame.session_id);
    require(action == Json{{"tick", 9},
                           {"session_id", "test-match"},
                           {"action", "move"},
                           {"direction", "right"}},
            "action did not preserve frame identity");
    require(!rmtc::Action::wait().to_json(9, "test-match").contains("direction"),
            "wait has direction");
    rejects([] { rmtc::Action::move("RIGHT").to_json(9, "test-match"); },
            "invalid direction accepted");
    std::cout << "protocol and probe strategy checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
