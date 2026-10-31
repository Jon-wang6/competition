/**
 * @file mapping_node.cpp
 * @brief 将本队公开感知转换成RViz栅格地图和实时场景。
 *
 * RViz只是观察窗口，比赛仍由原裁判结算。本文件不控制车辆，不连接网页，
 * 不读取裁判配置/全图。地形累计记忆，敌车/资源区分当前观察与历史位置。
 * 比赛[x,y]的y向下，ROS的y向上；所有对象统一转换，避免地图上下颠倒。
 */
#include "rmtc_connection/mapping_node.hpp"
#include "rmtc_connection/ai_client.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace rmtc {
namespace {
using Marker = visualization_msgs::msg::Marker;

/** @brief 构造ROS空间点；z用于让轨迹、标记和文字处于地形上方。 */
geometry_msgs::msg::Point point(double x, double y, double z) {
  geometry_msgs::msg::Point result;
  result.x = x;
  result.y = y;
  result.z = z;
  return result;
}

/** @brief 将比赛格子中心转到ROS坐标，整个棋盘居中，左上角仍对应[0,0]。 */
geometry_msgs::msg::Point cell_center(Position p, MapSize map, double size, double z) {
  return point((p.x + 0.5 - map.width * 0.5) * size, (map.height * 0.5 - p.y - 0.5) * size, z);
}

/** @brief 设置标记RGBA；不依赖类型默认颜色，避免透明标记看不见。 */
void color(Marker& marker, float r, float g, float b, float a = 1.0F) {
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
}
}  // namespace

/**
 * @brief 校验席位和显示比例，创建独立建图订阅、地图/标记发布器及静态坐标系。
 * 地图和标记保留最新快照；事件来自本队AI解释，不发布车辆控制动作。
 */
MappingNode::MappingNode(const rclcpp::NodeOptions& options) : Node("mapping_node", options) {
  team_ = declare_parameter<std::string>("team", "A");
  cell_size_ = declare_parameter<double>("cell_size", 1.0);
  action_ticks_ = declare_parameter<std::int64_t>("action_ticks", 500);
  if (team_.size() != 1 || team_[0] < 'A' || team_[0] > 'D') {
    throw std::runtime_error("team must be A, B, C or D");
  }
  if (!std::isfinite(cell_size_) || cell_size_ < 0.01 || cell_size_ > 100.0) {
    throw std::runtime_error("cell_size must be finite and in [0.01, 100]");
  }
  if (action_ticks_ <= 0 || action_ticks_ > 20000)
    throw std::runtime_error("action_ticks must be in [1, 20000]");
  frame_id_ = "rmtc_" + team_ + "_map";
  const auto base = "/rmtc/team_" + team_;
  // 持久化保存最新完整快照，RViz稍后启动也能立即获取已经累计的地图。
  const auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(base + "/map", latched);
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(base + "/markers", latched);
  perception_sub_ = create_subscription<std_msgs::msg::String>(
      "/team_" + team_ + "/perception", rclcpp::QoS(1).reliable().durability_volatile(),
      [this](const std_msgs::msg::String::ConstSharedPtr message) { on_perception(*message); });
  ai_result_sub_ = create_subscription<std_msgs::msg::String>(
      base + "/ai_result", rclcpp::QoS(4).reliable().transient_local(),
      [this](const std_msgs::msg::String::ConstSharedPtr message) { on_ai_result(*message); });

  // 给RViz提供始终存在的地图坐标系；只定义显示坐标，不发布虚假的车辆里程计。
  frame_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = now();
  transform.header.frame_id = "rmtc_world";
  transform.child_frame_id = frame_id_;
  transform.transform.rotation.w = 1.0;
  frame_broadcaster_->sendTransform(transform);
  refresh_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { publish_markers(); });
  publish_markers();
  RCLCPP_INFO(get_logger(),
              "独立建图：订阅 /team_%s/perception 和本队ai_result；输出 %s/{map,markers}；等待比赛数据",
              team_.c_str(), base.c_str());
}

/**
 * @brief 解析本队感知并更新地图记忆；接受的新帧立即发布地图和标记。
 * 换场清空由MapMemory负责；旧帧不更新接收时间，非法消息只记录限频警告。
 */
void MappingNode::on_perception(const std_msgs::msg::String& message) {
  try {
    const auto frame = parse_public_perception(message.data, team_);
    const bool new_session =
        !memory_.current() || memory_.current()->session_id != frame.session_id;
    const auto previous_session = memory_.current() ? memory_.current()->session_id : "";
    if (!memory_.update(frame))
      return;
    if (new_session) {
      if (!previous_session.empty())
        retired_sessions_.insert(previous_session);
      events_.reset();
    }
    event_memory_.update(frame);
    events_.observe(frame, event_memory_);
    consume_ai_results();
    received_at_ = std::chrono::steady_clock::now();
    publish_map();
    publish_markers();
    if (new_session) {
      RCLCPP_INFO(get_logger(), "新场次 %s：%dx%d，首个收到的Tick=%ld，已清空上一场地图",
                  frame.session_id.c_str(), frame.map.width, frame.map.height,
                  static_cast<long>(frame.tick));
    }
  } catch (const std::exception& error) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "忽略非法感知：%s", error.what());
  }
}

/** @brief 接收本队内部AI消息；限制大小和缓存条数，不接触模型密钥或裁判私有配置。 */
void MappingNode::on_ai_result(const std_msgs::msg::String& message) {
  try {
    if (message.data.size() > 256 * 1024)
      throw std::runtime_error("AI IPC message too large");
    auto value = Json::parse(message.data);
    const auto session = value.at("session_id").get<std::string>();
    if (value.at("team") != team_ || retired_sessions_.count(session))
      return;
    pending_ai_results_.push_back(std::move(value));
    if (pending_ai_results_.size() > 8)
      pending_ai_results_.pop_front();
    consume_ai_results();
    if (memory_.current()) {
      publish_map();
      publish_markers();
    }
  } catch (const std::exception& error) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "忽略非法事件解释：%s", error.what());
  }
}

/** @brief 复用选手的结构校验与事件时间线，禁止mock、串场或与公开公告不匹配的计划。 */
void MappingNode::consume_ai_results() {
  if (!memory_.current())
    return;
  const auto& frame = *memory_.current();
  for (auto it = pending_ai_results_.begin(); it != pending_ai_results_.end();) {
    if (retired_sessions_.count(it->value("session_id", ""))) {
      it = pending_ai_results_.erase(it);
      continue;
    }
    if (it->value("session_id", "") != frame.session_id) {
      ++it;
      continue;
    }
    // 先取出待处理项，非法字段抛异常时也不会永久堵住队列。
    auto value = std::move(*it);
    it = pending_ai_results_.erase(it);
    try {
      const auto& raw_tick = value.at("source_tick");
      if (!raw_tick.is_number_integer() ||
          (raw_tick.is_number_unsigned() && raw_tick.get<std::uint64_t>() > 20000))
        throw std::runtime_error("invalid AI source tick");
      const auto source_tick = raw_tick.get<std::int64_t>();
      if (source_tick < 0 || source_tick > 20000)
        throw std::runtime_error("invalid AI source tick");
      if (source_tick > frame.tick) {
        it = pending_ai_results_.insert(it, std::move(value));
        ++it;
        continue;
      }
      if (!value.at("completion_received").get<bool>() || value.at("mock").get<bool>() ||
          !value.at("interpretation_valid").get<bool>() ||
          !value.at("event_understood").get<bool>() || !value.at("event_plan").is_object())
        continue;
      const auto source = value.at("source_event_text").get<std::string>();
      if (source != frame.event_text &&
          (source.empty() || frame.event_text.compare(0, source.size(), source) != 0))
        continue;
      auto plan = decode_ai_constraints(value.at("event_plan"), frame, source_tick, action_ticks_);
      plan.source_text = source;
      if (!plan.timeline && (value.value("event_kind", "") == "world_split" ||
                             value.value("event_kind", "") == "explosion")) {
        Stage2EventEffect effect;
        effect.kind = value.at("event_kind");
        effect.begin_tick = plan.begin_tick;
        effect.end_tick = plan.end_tick;
        effect.cells = plan.dangerous_cells;
        plan.events.push_back(std::move(effect));
      }
      events_.merge(plan, frame, event_memory_);
    } catch (const std::exception& error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "忽略非法事件计划：%s", error.what());
    }
  }
}

/**
 * @brief 将累计记忆发布为OccupancyGrid；未收到有效感知时不发布空地图。
 * 地图在ROS坐标中居中，cells已按自下向上的行序保存，无须再次翻转。
 */
void MappingNode::publish_map() {
  if (!memory_.current())
    return;
  const auto& frame = *memory_.current();
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp = now();
  grid.header.frame_id = frame_id_;
  grid.info.map_load_time = grid.header.stamp;
  grid.info.resolution = static_cast<float>(cell_size_);
  grid.info.width = static_cast<std::uint32_t>(frame.map.width);
  grid.info.height = static_cast<std::uint32_t>(frame.map.height);
  grid.info.origin.position.x = -frame.map.width * cell_size_ * 0.5;
  grid.info.origin.position.y = -frame.map.height * cell_size_ * 0.5;
  grid.info.origin.orientation.w = 1.0;
  grid.data = event_memory_.cells();
  map_pub_->publish(grid);
}

/**
 * @brief 发布完整RViz场景：坐标、车辆、资源、历史年龄、视野、轨迹和状态文字。
 * 每次先清理旧标记，防止消失实体和旧文字残留；定时调用时只刷新显示，不推进Tick。
 */
void MappingNode::publish_markers() {
  visualization_msgs::msg::MarkerArray output;
  std::map<std::string, int> next_ids;
  // 每个标记都使用同一坐标系和非零单位四元数；完整快照不依赖上个消息的ID。
  auto make = [&](const std::string& name, int type) {
    Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = name;
    // 各命名空间单独编号，状态文字的ID不随可见实体数量变化而漂移。
    marker.id = next_ids[name]++;
    marker.type = type;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    color(marker, 1, 1, 1);
    return marker;
  };
  // Humble中非空ns的DELETEALL只清理该命名空间；空ns才清理整个显示层。
  auto clear = make("", Marker::CUBE);
  clear.action = Marker::DELETEALL;
  output.markers.push_back(clear);
  // RViz默认字体对中文支持有限，画面标签用ASCII；源码和终端说明使用中文。
  auto text = [&](const std::string& ns, const std::string& value,
                  geometry_msgs::msg::Point position, double height, float r, float g, float b) {
    auto marker = make(ns, Marker::TEXT_VIEW_FACING);
    marker.pose.position = position;
    marker.scale.z = height;
    marker.text = value;
    color(marker, r, g, b);
    output.markers.push_back(std::move(marker));
  };
  if (!memory_.current()) {
    text("status", "Waiting for /team_" + team_ + "/perception", point(0, 0.7 * cell_size_, 0.1),
         0.72 * cell_size_, 1, 0.8F, 0.2F);
    text("status", "Start a match to receive data", point(0, -0.3 * cell_size_, 0.1),
         0.6 * cell_size_, 0.85F, 0.9F, 1);
    markers_pub_->publish(output);
    return;
  }

  const auto& frame = *memory_.current();
  const auto size = cell_size_;
  const double half_w = frame.map.width * size * 0.5;
  const double half_h = frame.map.height * size * 0.5;
  const double age =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - received_at_).count();
  const bool stale = age > 3.0;

  auto grid = make("grid", Marker::LINE_LIST);
  grid.scale.x = 0.012 * size;
  color(grid, 0.24F, 0.36F, 0.47F, 0.45F);
  for (int x = 0; x <= frame.map.width; ++x) {
    grid.points.push_back(point(-half_w + x * size, -half_h, 0.02 * size));
    grid.points.push_back(point(-half_w + x * size, half_h, 0.02 * size));
  }
  for (int y = 0; y <= frame.map.height; ++y) {
    grid.points.push_back(point(-half_w, half_h - y * size, 0.02 * size));
    grid.points.push_back(point(half_w, half_h - y * size, 0.02 * size));
  }
  output.markers.push_back(std::move(grid));
  const int label_step = std::max(1, std::max(frame.map.width, frame.map.height) / 32);
  for (int x = 0; x < frame.map.width; x += label_step) {
    text("coordinates", std::to_string(x),
         point(-half_w + (x + 0.5) * size, half_h + 0.5 * size, 0.1), 0.42 * size, 0.7F, 0.8F,
         0.9F);
  }
  for (int y = 0; y < frame.map.height; y += label_step) {
    text("coordinates", std::to_string(y),
         point(-half_w - 0.55 * size, half_h - (y + 0.5) * size, 0.1), 0.42 * size, 0.7F, 0.8F,
         0.9F);
  }

  // 历史轨迹保留；不同重生/漏帧片段分开发布，不产生横跨地图的连线。
  for (const auto& segment : memory_.paths()) {
    if (segment.size() < 2)
      continue;
    auto trail = make("trail", Marker::LINE_STRIP);
    trail.scale.x = 0.07 * size;
    color(trail, 0, 0.65F, 0.9F, 0.8F);
    for (const auto p : segment)
      trail.points.push_back(cell_center(p, frame.map, size, 0.07 * size));
    output.markers.push_back(std::move(trail));
  }

  if (frame.self.alive) {
    auto vision = make("vision", Marker::LINE_STRIP);
    vision.scale.x = 0.045 * size;
    color(vision, 0, 0.8F, 0.95F, stale ? 0.3F : 0.9F);
    const int r = frame.vision_radius;
    const double left = -half_w + std::max(0, frame.self.position.x - r) * size;
    const double right = -half_w + std::min(frame.map.width, frame.self.position.x + r + 1) * size;
    const double top = half_h - std::max(0, frame.self.position.y - r) * size;
    const double bottom = half_h - std::min(frame.map.height, frame.self.position.y + r + 1) * size;
    vision.points = {point(left, top, 0.08 * size), point(right, top, 0.08 * size),
                     point(right, bottom, 0.08 * size), point(left, bottom, 0.08 * size),
                     point(left, top, 0.08 * size)};
    output.markers.push_back(std::move(vision));
  }

  auto self = make("self", Marker::CYLINDER);
  self.pose.position = cell_center(frame.self.position, frame.map, size, 0.3 * size);
  self.scale.x = self.scale.y = 0.72 * size;
  self.scale.z = 0.5 * size;
  color(self, 0, 0.75F, 1, frame.self.alive ? (stale ? 0.35F : 1.0F) : 0.25F);
  output.markers.push_back(std::move(self));
  // 生命值在顶部状态栏显示；车旁只放席位，避免挤到相邻格子的资源标签。
  const auto self_label = team_ + (frame.self.alive ? "" : ":DEAD");
  auto self_label_position = cell_center(frame.self.position, frame.map, size, 0.9 * size);
  self_label_position.y += 0.62 * size;
  text("self_label", self_label, self_label_position, 0.55 * size, 0.02F, 0.3F, 0.55F);

  // 视野外记忆保留实体形状，以淡色半透明显示；确认原位置为空时仍画X。
  for (const auto& [key, record] : memory_.entities()) {
    (void)key;
    const auto& entity = record.entity;
    const bool robot = entity.type == "robot";
    const bool high = entity.type == "resource_high";
    const auto ticks_ago = frame.tick - record.last_seen_tick;
    const bool visible = frame.self.alive && ticks_ago == 0;
    const bool absent = record.first_missing_tick.has_value();
    const std::string group = robot ? "robots" : "resources";
    const auto ns = (visible ? "visible_" : (absent ? "absent_" : "remembered_")) + group;
    auto marker =
        make(ns, absent ? Marker::LINE_LIST : (robot ? Marker::CYLINDER : Marker::SPHERE));
    marker.pose.position =
        cell_center(entity.position, frame.map, size, (absent ? 0.12 : 0.3) * size);
    if (!absent) {
      marker.scale.x = marker.scale.y = (robot ? 0.68 : 0.42) * size;
      marker.scale.z = 0.5 * size;
    } else {
      marker.scale.x = 0.055 * size;
      const double radius = (robot ? 0.34 : 0.25) * size;
      marker.points = {point(-radius, -radius, 0), point(radius, radius, 0),
                       point(-radius, radius, 0), point(radius, -radius, 0)};
    }
    color(marker, robot ? 0.95F : 1.0F, robot ? 0.15F : (high ? 0.45F : 0.72F),
          robot ? 0.2F : 0.05F, (visible ? 1.0F : (absent ? 0.5F : 0.38F)) * (stale ? 0.6F : 1.0F));
    output.markers.push_back(std::move(marker));

    std::ostringstream label;
    if (robot) {
      label << entity.team.value_or(entity.robot_id.value_or("?"));
      // 旧生命值不标为当前生命值；历史只显示身份和最后看到距今的Tick数。
      if (visible && entity.hp)
        label << ':' << *entity.hp;
    } else {
      label << (high ? "H" : "R");
      if (entity.value)
        label << ':' << *entity.value;
    }
    // 第二行数字表示距最后一次确认存在的Tick数，例如10t就是10Tick前。
    if (!visible)
      label << '\n' << ticks_ago << 't';
    auto label_position = cell_center(entity.position, frame.map, size, 0.85 * size);
    label_position.y += (visible ? 0.62 : 0.85) * size;
    text(visible ? "entity_labels" : "memory_labels", label.str(), label_position,
         (visible ? 0.46 : 0.5) * size, robot ? 0.8F : 0.5F, robot ? 0.05F : 0.28F, 0.02F);
  }

  for (const auto& mission : frame.missions) {
    if (!mission.position || mission.position->x < 0 || mission.position->y < 0 ||
        mission.position->x >= frame.map.width || mission.position->y >= frame.map.height)
      continue;
    const auto& completed = frame.score.mission_completed_ids;
    const bool done = std::find(completed.begin(), completed.end(), mission.id) != completed.end();
    auto goal = make("mission_targets", Marker::CUBE);
    goal.pose.position = cell_center(*mission.position, frame.map, size, 0.09 * size);
    goal.scale.x = goal.scale.y = 0.85 * size;
    goal.scale.z = 0.12 * size;
    color(goal, 0.65F, done ? 0.65F : 0.1F, 0.85F, 0.65F);
    output.markers.push_back(std::move(goal));
    auto goal_label_position = cell_center(*mission.position, frame.map, size, 0.5 * size);
    goal_label_position.y -= 0.65 * size;
    text("mission_labels", done ? "GOAL OK" : "GOAL", goal_label_position, 0.44 * size, 0.8F, 0.15F,
         1);
  }

  std::ostringstream heading;
  heading << "Team " << team_ << " | Tick " << frame.tick << " | Explored " << std::fixed
          << std::setprecision(1) << 100.0 * memory_.observed_count() / memory_.cells().size()
          << '%';
  text("status", heading.str(), point(0, half_h + 3.2 * size, 0.1), 0.72 * size, 0.9F, 0.95F, 1);
  std::ostringstream details;
  details << "Pos [" << frame.self.position.x << ',' << frame.self.position.y << "] | HP "
          << frame.self.hp << " | Resource " << frame.score.resource_value;
  if (!frame.self.alive)
    details << " | Respawn " << frame.self.respawn_in;
  text("status", details.str(), point(0, half_h + 2.2 * size, 0.1), 0.6 * size, 0.75F, 0.85F,
       0.95F);
  std::ostringstream freshness;
  if (stale)
    freshness << "No new data for " << static_cast<int>(age) << "s - showing last received state";
  else
    freshness << "Receiving public perception";
  text("status", freshness.str(), point(0, half_h + 1.35 * size, 0.1), 0.48 * size,
       stale ? 1.0F : 0.4F, stale ? 0.6F : 0.85F, stale ? 0.3F : 0.7F);
  // 短行分开排版，避免整句图例在俯视窗口中被缩成难以阅读的小字。
  text("legend", "CYAN: self/view/trail   RED: robots   YELLOW/ORANGE: resources",
       point(0, -half_h - 0.9 * size, 0.1), 0.5 * size, 0.8F, 0.85F, 0.9F);
  text("legend", "WHITE: free   BLACK: obstacle   GRAY: unknown   PURPLE: goal",
       point(0, -half_h - 1.8 * size, 0.1), 0.5 * size, 0.8F, 0.85F, 0.9F);
  text("legend", "Faded + Nt: seen N ticks ago   X: old cell checked empty   Bright: visible now",
       point(0, -half_h - 2.7 * size, 0.1), 0.46 * size, 0.8F, 0.85F, 0.9F);
  append_event_markers(output);
  markers_pub_->publish(output);
}

}  // namespace rmtc
