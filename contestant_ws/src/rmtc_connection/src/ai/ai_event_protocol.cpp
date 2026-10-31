/**
 * @file ai_event_protocol.cpp
 * @brief 八类模型事件的严格结构校验；区域只来自LLM给出的坐标，不解析公告句式。
 */
#include "rmtc_connection/ai_client.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace rmtc {
namespace {
/// 不接受字符串数字或小数Tick，所有范围限制针对结构化模型结果。
std::int64_t integral(const Json& value, std::int64_t low, std::int64_t high) {
  if (!value.is_number_integer() ||
      (value.is_number_unsigned() && value.get<std::uint64_t>() > static_cast<std::uint64_t>(high)))
    throw std::runtime_error("invalid_event_integer");
  const auto v = value.get<std::int64_t>();
  if (v < low || v > high)
    throw std::runtime_error("event_integer_range");
  return v;
}
/// 限制效果参数为有限非负数；不允许NaN、无限倍率或负伤害偷渡成奖励。
double numeric(const Json& value, double low = 0, double high = 10000) {
  if (!value.is_number())
    throw std::runtime_error("invalid_event_number");
  const auto v = value.get<double>();
  if (!std::isfinite(v) || v < low || v > high)
    throw std::runtime_error("event_number_range");
  return v;
}
/// 校验单个世界坐标，输出仍使用x向右、y向下的比赛坐标。
Position point(const Json& value, MapSize size) {
  if (!value.is_array() || value.size() != 2)
    throw std::runtime_error("event_point_shape");
  return {static_cast<int>(integral(value[0], 0, size.width - 1)),
          static_cast<int>(integral(value[1], 0, size.height - 1))};
}
/// 点与矩形分别校验，只有显式rectangles才展开矩形，绝不将两个points猜成矩形。
std::vector<Position> region(const Json& value, MapSize size, const char* field) {
  std::set<std::pair<int, int>> cells;
  if (value.contains(field)) {
    if (!value[field].is_array() || value[field].size() > 4096)
      throw std::runtime_error("event_region_size");
    for (const auto& v : value[field]) {
      const auto p = point(v, size);
      cells.emplace(p.x, p.y);
    }
  }
  if (std::string(field) == "cells" && value.contains("rectangles")) {
    if (!value["rectangles"].is_array() || value["rectangles"].size() > 32)
      throw std::runtime_error("event_rectangles_size");
    for (const auto& r : value["rectangles"]) {
      if (!r.is_array() || r.size() != 4)
        throw std::runtime_error("event_rectangle_shape");
      const int x0 = integral(r[0], 0, size.width - 1), y0 = integral(r[1], 0, size.height - 1);
      const int x1 = integral(r[2], x0, size.width - 1), y1 = integral(r[3], y0, size.height - 1);
      for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x)
          cells.emplace(x, y);
    }
  }
  if (cells.size() > 4096)
    throw std::runtime_error("event_region_too_large");
  std::vector<Position> out;
  for (auto [x, y] : cells)
    out.push_back({x, y});
  return out;
}
/// 内部IPC使用明确的格子数组，不携带模型原始响应、令牌或HTTP配置。
Json positions(const std::vector<Position>& cells) {
  Json out = Json::array();
  for (auto p : cells)
    out.push_back({p.x, p.y});
  return out;
}
}  // namespace

Stage2EventConstraints decode_ai_event_timeline(const Json& plan, const PublicPerception& frame,
                                                std::int64_t source_tick,
                                                std::int64_t action_ticks) {
  Stage2EventConstraints result;
  result.session_id = frame.session_id;
  result.source_text = frame.event_text;
  result.source_tick = integral(source_tick, 0, frame.tick);
  result.timeline = true;
  result.begin_tick = frame.tick;
  result.end_tick = frame.tick + 1;
  if (!plan.is_object() || !plan.at("events").is_array() || plan["events"].size() > 64)
    throw std::runtime_error("event_timeline_shape");
  const std::set<std::string> kinds{"explosion",   "illusion", "angel",  "unrivaled",
                                    "world_split", "pepper",   "purity", "pan"};
  const auto max_tick = std::min<std::int64_t>(20000, action_ticks + 5000);
  for (const auto& raw : plan["events"]) {
    Stage2EventEffect e;
    e.kind = raw.at("kind").get<std::string>();
    if (!kinds.count(e.kind))
      throw std::runtime_error("unknown_event_kind");
    e.begin_tick = integral(raw.at("begin_tick"), 0, max_tick);
    e.end_tick = integral(raw.at("end_tick"), e.begin_tick + 1, max_tick);
    e.actor = raw.value("actor", std::string{});
    // 两类事件均按规则瞬时施放；模型给的Buff持续时间不能扩大施放窗口。
    if (e.actor.empty() && (e.kind == "explosion" || e.kind == "illusion"))
      e.end_tick = e.begin_tick + 1;

    if (!e.actor.empty() &&
        (e.actor.size() != 1 || std::string("ABCD").find(e.actor) == std::string::npos ||
         (e.kind != "illusion" && e.kind != "angel" && e.kind != "unrivaled" && e.kind != "pan")))
      throw std::runtime_error("event_actor_invalid");
    e.cells = region(raw, frame.map, "cells");
    e.points = region(raw, frame.map, "points");
    if (e.kind == "world_split" && raw.contains("cross_point")) {
      const auto p = point(raw["cross_point"], frame.map);
      e.cells.clear();
      for (int y = 0; y < frame.map.height; ++y)
        for (int x = 0; x < frame.map.width; ++x)
          if (x == p.x || y == p.y)
            e.cells.push_back({x, y});
    }
    if (e.actor.empty() &&
        (e.kind == "explosion" || e.kind == "illusion" || e.kind == "unrivaled" ||
         e.kind == "world_split") &&
        e.cells.empty())
      throw std::runtime_error("missing_event_area");
    if (e.actor.empty() && (e.kind == "angel" || e.kind == "pepper") && e.points.empty())
      throw std::runtime_error("missing_event_points");
    if (e.kind == "explosion" || e.kind == "pepper")
      e.damage = numeric(raw.at("damage"), 1);
    if ((e.kind == "illusion" || e.kind == "angel") && e.actor.empty())
      e.effect_ticks = integral(raw.at("effect_ticks"), 1, 5000);
    if (e.kind == "angel") {
      e.heal_per_tick = numeric(raw.at("heal_per_tick"), 0.01);
      e.hp_cap = numeric(raw.value("hp_cap", Json(100)), 1);
    }
    if (e.kind == "unrivaled") {
      if (e.actor.empty())
        e.stay_ticks = integral(raw.at("stay_ticks"), 1, 5000);
      e.attack_radius = integral(raw.at("attack_radius"), 1, 32);
      e.attack_damage = numeric(raw.at("attack_damage"), 1);
      e.defended_damage = numeric(raw.at("defended_damage"), 0);
    }
    if (e.kind == "purity")
      e.reflection_damage = numeric(raw.at("reflection_damage"), 1);
    if (e.kind == "pan") {
      e.damage_multiplier = numeric(raw.at("damage_multiplier"), 1, 20);
      if (e.actor.empty())
        e.movement_window_ticks =
            integral(raw.at("movement_window_ticks"), 1, e.end_tick - e.begin_tick);
    }
    const auto relevant_until = e.kind == "illusion"
                                    ? std::max(e.end_tick, e.begin_tick + e.effect_ticks)
                                : e.kind == "angel" ? e.end_tick + e.effect_ticks
                                                    : e.end_tick;
    if (relevant_until <= frame.tick)
      continue;
    result.begin_tick = std::min(result.begin_tick, e.begin_tick);
    result.end_tick = std::max(result.end_tick, relevant_until);
    result.events.push_back(std::move(e));
  }
  return result;
}

Json encode_ai_event_timeline(const Stage2EventConstraints& constraints) {
  Json events = Json::array();
  for (const auto& e : constraints.events)
    events.push_back({{"kind", e.kind},
                      {"actor", e.actor},
                      {"begin_tick", e.begin_tick},
                      {"end_tick", e.end_tick},
                      {"cells", positions(e.cells)},
                      {"points", positions(e.points)},
                      {"damage", e.damage},
                      {"effect_ticks", e.effect_ticks},
                      {"heal_per_tick", e.heal_per_tick},
                      {"hp_cap", e.hp_cap},
                      {"stay_ticks", e.stay_ticks},
                      {"attack_radius", e.attack_radius},
                      {"attack_damage", e.attack_damage},
                      {"defended_damage", e.defended_damage},
                      {"reflection_damage", e.reflection_damage},
                      {"damage_multiplier", e.damage_multiplier},
                      {"movement_window_ticks", e.movement_window_ticks}});
  return {{"events", events}};
}
}  // namespace rmtc
