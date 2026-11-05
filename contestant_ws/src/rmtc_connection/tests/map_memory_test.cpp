/**
 * @file map_memory_test.cpp
 * @brief 验证实时建图、随机出生、资源/敌车观察记忆、死亡、场次和坐标翻转。
 *
 * 直接构造已解析的公开感知，不运行裁判、不向比赛发送动作。
 *
 * @details 模块协作与维护约定
 * 验证对象：map_memory.hpp。
 * 运行约定：由独立测试可执行文件/CTest执行，断言失败用于定位回归，不是正式选手的逐帧逻辑。
 * 结果边界：通过表示这些构造场景满足约束，不证明所有随机地图均最优或稳定达到某个总分。
 */
#include "rmtc_connection/map_memory.hpp"

#include <iostream>
#include <stdexcept>
#include <string>

/** @brief 在任何构建类型下检查条件，失败时打印有意义的原因。 */
void require(bool condition, const std::string& message) {
  if (!condition)
    throw std::runtime_error(message);
}

/** @brief 构造小尺寸、半径1的合法测试感知；只提供建图所需的公开字段。 */
rmtc::PublicPerception sample() {
  rmtc::PublicPerception frame;
  frame.session_id = "map-test-1";
  frame.tick = 0;
  frame.map = {7, 5};
  frame.vision_radius = 1;
  frame.self.team = "A";
  frame.self.alive = true;
  frame.self.position = {1, 1};
  rmtc::VisibleEntity wall;
  wall.type = "obstacle";
  wall.position = {2, 1};
  rmtc::VisibleEntity robot;
  robot.type = "robot";
  robot.id = "enemy-test";
  robot.team = "B";
  robot.robot_id = "B";
  robot.position = {1, 2};
  rmtc::VisibleEntity resource;
  resource.type = "resource_normal";
  resource.id = "resource-test";
  resource.position = {2, 0};
  resource.value = 10;
  frame.visible_entities = {wall, robot, resource};
  return frame;
}

/** @brief 按公开ID查找记忆；缺失时直接使测试失败，不依赖内部键的拼接格式。 */
const rmtc::EntityMemory& find_record(const rmtc::MapMemory& memory, const std::string& id) {
  for (const auto& [key, record] : memory.entities()) {
    (void)key;
    if (record.entity.id == id)
      return record;
  }
  throw std::runtime_error("missing entity memory: " + id);
}

/** @brief 验证视野外未知、原位置确认空、再次出现及同ID移动的语义区别。 */
void check_entity_history() {
  rmtc::MapMemory memory;
  auto frame = sample();
  frame.tick = 10;
  require(memory.update(frame) && memory.entities().size() == 2,
          "resources/robots not remembered separately");
  frame.tick = 11;
  frame.self.position = {5, 3};
  frame.visible_entities.clear();
  memory.update(frame);
  require(find_record(memory, "enemy-test").last_seen_tick == 10 &&
              !find_record(memory, "enemy-test").first_missing_tick &&
              !find_record(memory, "resource-test").first_missing_tick,
          "leaving vision was treated as confirmed disappearance");
  frame.tick = 12;
  frame.self.position = {1, 1};
  frame.self.alive = false;
  memory.update(frame);
  require(!find_record(memory, "enemy-test").first_missing_tick &&
              !find_record(memory, "resource-test").first_missing_tick,
          "dead robot used its empty perception to invalidate memories");
  frame.tick = 13;
  frame.self.alive = true;
  memory.update(frame);
  frame.tick = 14;
  memory.update(frame);
  require(find_record(memory, "enemy-test").first_missing_tick == 13 &&
              find_record(memory, "resource-test").first_missing_tick == 13 &&
              find_record(memory, "resource-test").last_seen_tick == 10,
          "first empty observation/last positive observation was overwritten");

  frame.tick = 15;
  frame.visible_entities = sample().visible_entities;
  frame.visible_entities[1].position = {2, 1};
  frame.visible_entities[2].type = "resource_high";
  frame.visible_entities[2].value = 20;
  memory.update(frame);
  require(memory.entities().size() == 2 &&
              find_record(memory, "enemy-test").entity.position.x == 2 &&
              find_record(memory, "enemy-test").last_seen_tick == 15 &&
              !find_record(memory, "enemy-test").first_missing_tick &&
              find_record(memory, "resource-test").first_seen_tick == 10 &&
              find_record(memory, "resource-test").entity.type == "resource_high" &&
              !find_record(memory, "resource-test").first_missing_tick,
          "reappearance/movement created duplicate memories or retained an absent state");
  frame.visible_entities.clear();
  require(!memory.update(frame) && !find_record(memory, "enemy-test").first_missing_tick,
          "duplicate Tick changed confirmed observations");
  frame.tick = 0;
  frame.session_id = "history-new-session";
  require(memory.update(frame) && memory.entities().empty(),
          "entity memories leaked into new match");
}

/** @brief 敌人复活换ID或从不完整身份补齐席位后，旧尸体/旧位置不得和新实例并存。 */
void check_enemy_reappearance() {
  auto frame = sample();
  frame.visible_entities = {sample().visible_entities[1]};
  frame.visible_entities.front().hp = 0;
  rmtc::MapMemory memory;
  memory.update(frame);
  frame.tick = 1;
  frame.visible_entities.clear();
  memory.update(frame);
  frame.tick = 2;
  auto enemy = sample().visible_entities[1];
  enemy.id = "new-life-instance";
  enemy.robot_id = "new-life-robot";
  enemy.position = {2, 1};
  enemy.hp = 100;
  frame.visible_entities = {enemy};
  memory.update(frame);
  require(memory.entities().size() == 1 &&
              find_record(memory, "new-life-instance").entity.hp == 100 &&
              !find_record(memory, "new-life-instance").first_missing_tick,
          "respawn with a new instance ID retained the previous corpse/position");

  rmtc::MapMemory partial;
  frame.tick = 0;
  frame.session_id = "partial-enemy-identity";
  frame.visible_entities.front().team.reset();
  partial.update(frame);
  frame.tick = 1;
  frame.visible_entities.front().team = "B";
  frame.visible_entities.front().position = {1, 2};
  partial.update(frame);
  require(partial.entities().size() == 1,
          "completing the seat identity left an old anonymous alias");
  auto other = frame.visible_entities.front();
  other.team = "C";
  frame.tick = 2;
  frame.visible_entities.push_back(other);
  partial.update(frame);
  require(partial.entities().size() == 2,
          "different teams with coincident entity IDs were merged");
}

/**
 * @brief 遍历两阶段和矩形地图的每个出生格、四个席位，检查视野与绝对坐标。
 * 每例使用新场次和非零首Tick，覆盖中途接入；视野外必须保持未知，轨迹从实测点开始。
 * 同场再跳到另一角模拟随机复活，要求旧地形保留且两段轨迹不画成瞬移连线。
 */
void check_arbitrary_spawn_positions() {
  std::size_t cases = 0;
  for (const rmtc::MapSize size : {rmtc::MapSize{20, 20}, {24, 24}, {7, 5}}) {
    for (const char team : std::string("ABCD")) {
      for (int y = 0; y < size.height; ++y) {
        for (int x = 0; x < size.width; ++x) {
          auto frame = sample();
          frame.session_id = "arbitrary-spawn-" + std::to_string(cases++);
          frame.tick = 17;
          frame.map = size;
          frame.vision_radius = 4;
          frame.self.team = std::string(1, team);
          frame.self.position = {x, y};
          const int dx = x + 1 < size.width ? 1 : -1;
          const int dy = y + 1 < size.height ? 1 : -1;
          auto wall = sample().visible_entities[0];
          wall.position = {x + dx, y};
          auto resource = sample().visible_entities[2];
          resource.position = {x, y + dy};
          frame.visible_entities = {wall, resource};
          rmtc::MapMemory memory;
          require(memory.update(frame), "arbitrary first position rejected");
          std::size_t observed = 0;
          for (int cy = 0; cy < size.height; ++cy) {
            for (int cx = 0; cx < size.width; ++cx) {
              const bool visible = std::abs(cx - x) <= 4 && std::abs(cy - y) <= 4;
              const bool obstacle = cx == wall.position.x && cy == wall.position.y;
              const int expected = visible ? (obstacle ? 100 : 0) : -1;
              require(memory.cells()[rmtc::MapMemory::index({cx, cy}, size)] == expected,
                      "spawn-dependent offset exposed or misplaced terrain");
              observed += visible;
            }
          }
          require(memory.observed_count() == observed && memory.paths().size() == 1 &&
                      memory.paths().front().size() == 1 &&
                      memory.paths().front().front().x == x &&
                      memory.paths().front().front().y == y,
                  "trajectory starts at an assumed spawn");
          const auto& record = find_record(memory, "resource-test");
          require(record.entity.position.x == resource.position.x &&
                      record.entity.position.y == resource.position.y &&
                      record.last_seen_tick == 17,
                  "resource memory shifted with birth position");
          frame.tick = 18;
          frame.self.position = {x < size.width / 2 ? size.width - 1 : 0,
                                 y < size.height / 2 ? size.height - 1 : 0};
          frame.self.position_reason = "respawn";
          frame.visible_entities.clear();
          memory.update(frame);
          const bool wall_seen_again = std::abs(wall.position.x - frame.self.position.x) <= 4 &&
                                       std::abs(wall.position.y - frame.self.position.y) <= 4;
          require(memory.paths().size() == 2 && memory.paths().back().size() == 1 &&
                      memory.cells()[rmtc::MapMemory::index(wall.position, size)] ==
                          (wall_seen_again ? 0 : 100),
                  "random respawn lost old terrain or drew a teleport trail");
        }
      }
    }
  }
  std::cout << "arbitrary birth/respawn mapping cases: " << cases << '\n';
}

/**
 * @brief 从看不到任何边界的地图中心正常行走到四边，逐帧核对累计地图和连续轨迹。
 * 固定地形只通过当帧视野内实体给出；预期可见域由所有历史位置的距离独立计算，
 * 保证后续发现边界时不重定原点、不遗忘老墙，也不提前公开未走到的内部区域。
 */
void check_center_to_boundary_mapping() {
  for (const int size : {20, 24}) {
    auto frame = sample();
    frame.map = {size, size};
    frame.vision_radius = 4;
    frame.session_id = "center-to-boundaries-" + std::to_string(size);
    frame.self.position = {size / 2, size / 2};
    std::vector<rmtc::Position> route{frame.self.position};
    for (const rmtc::Position goal : {rmtc::Position{0, size / 2}, {0, 0},
                                     {size - 1, 0}, {size - 1, size - 1}, {0, size - 1}}) {
      auto p = route.back();
      while (p.x != goal.x || p.y != goal.y) {
        if (p.x != goal.x) p.x += goal.x > p.x ? 1 : -1;
        else p.y += goal.y > p.y ? 1 : -1;
        route.push_back(p);
      }
    }
    const std::set<std::pair<int, int>> walls{{size - 2, 1}, {2, size - 2},
                                             {size / 2 + 1, size / 2 + 1}};
    rmtc::MapMemory memory;
    std::vector<rmtc::Position> visited;
    for (std::size_t step = 0; step < route.size(); ++step) {
      frame.tick = static_cast<std::int64_t>(step);
      frame.self.position = route[step];
      frame.visible_entities.clear();
      for (const auto& [x, y] : walls)
        if (std::abs(x - frame.self.position.x) <= 4 &&
            std::abs(y - frame.self.position.y) <= 4) {
          rmtc::VisibleEntity wall;
          wall.type = "obstacle";
          wall.position = {x, y};
          frame.visible_entities.push_back(wall);
        }
      require(memory.update(frame), "center-to-edge observation rejected");
      visited.push_back(frame.self.position);
      for (int y = 0; y < size; ++y)
        for (int x = 0; x < size; ++x) {
          const bool seen = std::any_of(visited.begin(), visited.end(), [&](rmtc::Position p) {
            return std::max(std::abs(x - p.x), std::abs(y - p.y)) <= 4;
          });
          const int expected = seen ? (walls.count({x, y}) ? 100 : 0) : -1;
          require(memory.cells()[rmtc::MapMemory::index({x, y}, frame.map)] == expected,
                  "discovering a boundary shifted, erased or leaked terrain");
        }
      require(memory.paths().size() == 1 && memory.paths().front().size() == visited.size(),
              "normal center-to-boundary walk broke its trajectory");
    }
  }
}

/** @brief 执行地图记忆检查，成功返回0，失败返回1；不需要ROS环境初始化。 */
int main() {
  try {
    rmtc::MapMemory memory;
    auto frame = sample();
    require(!memory.current() && memory.cells().empty(), "invented initial map");
    require(memory.update(frame), "first frame rejected");
    require(memory.cells().size() == 35 && memory.observed_count() == 9, "vision square incorrect");
    require(rmtc::MapMemory::index({0, 0}, frame.map) == 28 &&
                rmtc::MapMemory::index({6, 4}, frame.map) == 6,
            "ROS row order is upside down");
    require(memory.cells()[rmtc::MapMemory::index({2, 1}, frame.map)] == 100,
            "visible wall missing");
    require(memory.cells()[rmtc::MapMemory::index({1, 2}, frame.map)] == 0,
            "robot became permanent wall");
    require(memory.cells()[rmtc::MapMemory::index({6, 4}, frame.map)] == -1,
            "hidden terrain exposed");
    require(!memory.update(frame) && memory.paths().front().size() == 1,
            "duplicate Tick added history");

    frame.tick = 1;
    frame.self.position = {1, 0};
    frame.visible_entities.clear();
    require(memory.update(frame) && memory.observed_count() == 9,
            "map boundary clipping incorrect");
    require(memory.cells()[rmtc::MapMemory::index({2, 1}, frame.map)] == 0,
            "vanished wall stayed occupied");
    require(memory.paths().size() == 1 && memory.paths()[0].size() == 2,
            "normal move broke trajectory");

    frame.tick = 2;
    frame.self.position = {5, 3};
    frame.self.alive = false;
    require(memory.update(frame) && memory.observed_count() == 9, "death exposed unseen terrain");
    frame.tick = 3;
    frame.self.alive = true;
    frame.self.position_reason = "respawn";
    require(memory.update(frame) && memory.observed_count() == 18 && memory.paths().size() == 2,
            "respawn did not start a separate trajectory");
    require(memory.cells()[rmtc::MapMemory::index({0, 0}, frame.map)] == 0,
            "old explored map forgotten");
    frame.tick = 5;
    frame.self.position_reason.reset();
    frame.self.position = {6, 3};
    require(memory.update(frame) && memory.paths().size() == 3,
            "dropped frame created guessed trail");
    frame.tick = 4;
    require(!memory.update(frame) && memory.current()->tick == 5, "old Tick overwrote newer state");

    auto invalid = frame;
    invalid.tick = 6;
    invalid.map.width = 8;
    bool rejected = false;
    try {
      memory.update(invalid);
    } catch (const std::exception&) {
      rejected = true;
    }
    require(rejected && memory.current()->tick == 5, "same-session resize corrupted memory");

    const auto old_session = frame;
    frame.session_id = "map-test-2";
    frame.tick = 0;
    frame.map = {4, 3};
    frame.self.position = {0, 0};
    require(memory.update(frame) && memory.cells().size() == 12 && memory.observed_count() == 4 &&
                memory.entities().empty() && memory.paths().size() == 1,
            "new session retained old map or path");
    require(!memory.update(old_session) && memory.current()->session_id == "map-test-2",
            "late old-session frame restored previous match");
    check_entity_history();
    check_enemy_reappearance();
    check_arbitrary_spawn_positions();
    check_center_to_boundary_mapping();
    std::cout
        << "map memory: terrain, resource/enemy observation history, death and sessions passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
