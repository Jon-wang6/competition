/**
 * @file event_markers.cpp
 * @brief 将本队AI已校验的八类机制显示为区域、点位、倒计时和已确认Buff。
 *
 * 预告只显示半透明提示，不提前阻挡；生效的世界分裂同时写入地图障碍层。
 * 不分析广播句式、不读取裁判配置；只用最新感知Tick推进显示，断流不自行倒计时。
 */
#include "rmtc_connection/mapping_node.hpp"

#include <array>
#include <map>
#include <sstream>
#include <utility>

namespace rmtc {
namespace {
using Marker = visualization_msgs::msg::Marker;

struct EventStyle {
  const char* name;
  std::array<float, 3> rgb;
};

/** @brief 事件颜色与ASCII名称；避免RViz字体缺少中文字形导致方框。 */
EventStyle style(const std::string& kind) {
  if (kind == "explosion") return {"EXPLOSION", {1.0F, 0.15F, 0.1F}};
  if (kind == "illusion") return {"ILLUSION", {0.8F, 0.35F, 1.0F}};
  if (kind == "angel") return {"ANGEL", {0.15F, 1.0F, 0.45F}};
  if (kind == "unrivaled") return {"UNRIVALED", {1.0F, 0.85F, 0.1F}};
  if (kind == "world_split") return {"WORLD SPLIT", {1.0F, 0.35F, 0.05F}};
  if (kind == "pepper") return {"PEPPER", {1.0F, 0.5F, 0.3F}};
  if (kind == "purity") return {"PURITY", {0.15F, 0.95F, 1.0F}};
  return {"VULNERABILITY", {1.0F, 0.3F, 0.65F}};
}

/** @brief 用同一比例将比赛格子中心转换为居中、y向上的RViz坐标。 */
geometry_msgs::msg::Point center(Position p, MapSize size, double scale, double z) {
  geometry_msgs::msg::Point result;
  result.x = (p.x + 0.5 - size.width * 0.5) * scale;
  result.y = (size.height * 0.5 - p.y - 0.5) * scale;
  result.z = z * scale;
  return result;
}

/** @brief 展示模型解析出的机制参数；每项说明与当前解释一致，不硬编码伤害或持续期。 */
std::string mechanics(const Stage2EventEffect& effect) {
  std::ostringstream out;
  if (effect.kind == "explosion") out << '-' << effect.damage << " HP";
  if (effect.kind == "illusion") out << "reverse MOVE / " << effect.effect_ticks << 't';
  if (effect.kind == "angel")
    out << '+' << effect.heal_per_tick << " HP/t / " << effect.effect_ticks << 't';
  if (effect.kind == "unrivaled")
    out << "hold " << effect.stay_ticks << "t / hit " << effect.attack_damage;
  if (effect.kind == "world_split") out << "temporary walls";
  if (effect.kind == "pepper") out << "entry: cross -" << effect.damage << " HP";
  if (effect.kind == "purity") out << "defend: reflect " << effect.reflection_damage << " HP";
  if (effect.kind == "pan")
    out << "MOVE grants damage x" << effect.damage_multiplier;
  return out.str();
}
}  // namespace

/** @brief 添加独立命名空间的完整事件快照，结束/换局后由主快照DELETEALL自动清理。 */
void MappingNode::append_event_markers(visualization_msgs::msg::MarkerArray& output) {
  if (!memory_.current())
    return;
  const auto& frame = *memory_.current();
  std::map<std::string, int> ids;
  const auto make = [&](const std::string& name, int type, EventStyle appearance, float alpha) {
    Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = name;
    marker.id = ids[name]++;
    marker.type = type;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1;
    marker.color.r = appearance.rgb[0];
    marker.color.g = appearance.rgb[1];
    marker.color.b = appearance.rgb[2];
    marker.color.a = alpha;
    return marker;
  };
  const auto text = [&](const std::string& name, const std::string& value,
                        geometry_msgs::msg::Point position, EventStyle appearance, double height) {
    auto marker = make(name, Marker::TEXT_VIEW_FACING, appearance, 1);
    marker.text = value;
    marker.pose.position = position;
    marker.scale.z = height * cell_size_;
    output.markers.push_back(std::move(marker));
  };
  int status_row = 0;
  const auto panel = [&](const std::string& value, EventStyle appearance) {
    geometry_msgs::msg::Point p;
    p.x = -(frame.map.width * 0.5 + 4.3) * cell_size_;
    p.y = (frame.map.height * 0.5 - 0.8 - 1.5 * status_row++) * cell_size_;
    p.z = 0.2 * cell_size_;
    text("event_status", value, p, appearance, 0.4);
  };

  for (const auto& [id, effect] : events_.effects()) {
    (void)id;
    if (!effect.actor.empty() || frame.tick >= effect.end_tick)
      continue;
    const auto appearance = style(effect.kind);
    const bool preview = frame.tick < effect.begin_tick;
    std::ostringstream status;
    status << appearance.name << (preview ? " IN " : " ACTIVE ")
           << (preview ? effect.begin_tick - frame.tick : effect.end_tick - frame.tick) << "t\n"
           << mechanics(effect);
    if (effect.kind == "pan") {
      const auto window_end = effect.begin_tick + effect.movement_window_ticks;
      status << (frame.tick < window_end ? "\nMOVE window ends " : "\nMOVE window closed; until ")
             << (frame.tick < window_end ? window_end : effect.end_tick);
    }
    if (effect.kind == "unrivaled" && !preview)
      status << "\noccupy " << events_.progress(effect, team_) << '/' << effect.stay_ticks;
    panel(status.str(), appearance);

    if (effect.cells.empty() && effect.points.empty())
      continue;  // 反伤和易伤是全场规则，仅画状态说明，不盖住整张地图。
    const bool wall = effect.kind == "world_split";
    const auto& cells = effect.cells.empty() ? effect.points : effect.cells;
    auto area = make(preview ? "event_preview" : "event_areas",
                     effect.cells.empty() ? Marker::SPHERE_LIST : Marker::CUBE_LIST,
                     appearance, preview ? 0.18F : (wall ? 0.85F : 0.35F));
    area.scale.x = area.scale.y = (effect.cells.empty() ? 0.55 : 0.9) * cell_size_;
    area.scale.z = (wall && !preview ? 0.18 : 0.04) * cell_size_;
    geometry_msgs::msg::Point label;
    for (const auto p : cells) {
      const auto position = center(p, frame.map, cell_size_, 0.16);
      area.points.push_back(position);
      label.x += position.x;
      label.y += position.y;
    }
    output.markers.push_back(std::move(area));
    label.x /= cells.size();
    label.y = label.y / cells.size() + 0.7 * cell_size_;
    label.z = 0.6 * cell_size_;
    if (!effect.cells.empty())
      text("event_labels", std::string(appearance.name) + (preview ? " [PREVIEW]" : " [ACTIVE]"),
           label, appearance, 0.38);
    // 两个开关分别标名，不能只在两点的中间放一个标签。
    for (const auto p : effect.points) {
      auto at = center(p, frame.map, cell_size_, 0.6);
      at.y += 0.6 * cell_size_;
      text("event_point_labels", appearance.name, at, appearance, 0.34);
    }
  }

  const auto inferred=events_.inferred_wall_cells();
  if(!inferred.empty()) {
    auto walls=make("event_inferred_split",Marker::CUBE_LIST,style("world_split"),.75F);
    walls.scale.x=walls.scale.y=.85*cell_size_;walls.scale.z=.25*cell_size_;
    for(const auto p:inferred)walls.points.push_back(center(p,frame.map,cell_size_,.2));
    output.markers.push_back(std::move(walls));
    panel("SUSPECTED WORLD SPLIT\nObserved new walls / AI pending",style("world_split"));
  }
  // 已确认的个体效果可比区域存活更久；独立显示，避免区域消失就误认为Buff消失。
  std::map<std::string, int> buff_label_rows;
  for (const auto* kind : {"illusion", "angel", "unrivaled", "pan"}) {
    for (const auto* team : {"A", "B", "C", "D"}) {
      const auto* buff = events_.buff(team, kind, frame.tick);
      if (!buff)
        continue;
      std::ostringstream value;
      value << team << ": " << style(kind).name << " BUFF\n" << buff->until - frame.tick << "t left";
      if (team_ == team)
        panel(value.str(), style(kind));
      // 他车只在当前可见时标记位置；已知Buff不等于能获知它的视野外坐标。
      Position position = frame.self.position;
      bool visible = frame.self.team == team;
      for (const auto& entity : frame.visible_entities)
        if (entity.type == "robot" && entity.team && *entity.team == team) {
          position = entity.position;
          visible = true;
        }
      if (visible) {
        auto at = center(position, frame.map, cell_size_, 0.9);
        at.y -= (0.8 + buff_label_rows[team]++ * 0.75) * cell_size_;
        text("event_buff_labels", value.str(), at, style(kind), 0.32);
      }
    }
  }
  if (events_.effects().empty() && !frame.event_text.empty())
    panel("EVENTS\nAwaiting validated AI", {"", {0.85F, 0.85F, 0.85F}});
}
}  // namespace rmtc
