/**
 * @file map_memory.hpp
 * @brief 保存本队逐帧观察到的地图；不依赖ROS，不读取裁判内部地图。
 *
 * 依据公开协议的方形视野，将存活时视野内的地形刷新为可通行或障碍；
 * 尚未观察的格子保持-1。9x9实体字符串的0只是占位，不直接参与建图。
 * 地形是最近一次观察的记忆，视野外发生的变化只有再次看到后才能更新。
 * 敌车、资源单独保存最后一次观察及确认原位置为空的Tick，不写进障碍地图。
 * 历史记录不是实时位置，不推断谁采集了资源或敌人是否死亡。新场次全部清空。
 *
 * @details 模块协作与维护约定
 * 状态归属：每个策略/显示节点可各自持有记忆，不能假定RViz的缓存会自动共享给主控。
 * 地形与实体分开保存；当前可见证据优先，视野外的旧坐标仅代表最后目击。
 * index包含显示存储方向转换，不能与规划器常用的y*width+x数组下标直接混用。
 */
#pragma once

#include "rmtc_connection/public_data.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <vector>

namespace rmtc {

/**
 * @brief 一项资源或敌车的观察记忆；entity中的坐标/生命值均属于last_seen_tick。
 * first_missing_tick表示本轮缺失中首次确认原位置没有该实体的Tick，
 * 并非被采集/死亡的确定时间；视野外或本车阵亡时不生成“确认消失”。
 */
struct EntityMemory {
  VisibleEntity entity;
  std::int64_t first_seen_tick{0};
  std::int64_t last_seen_tick{0};
  std::optional<std::int64_t> first_missing_tick;
};

class MapMemory {
 public:
  /**
   * @brief 接收已经由parse_public_perception校验的一帧，更新地形、实体历史和轨迹。
   * @return 接受新帧时为true；重复Tick、旧Tick和已结束场次的迟到帧为false。
   * 新session_id自动清图；同一场次尺寸变化视为异常，不混合两张地图。
   * 阵亡帧只更新状态，不把空实体列表误认为整片视野没有障碍。
   */
  bool update(const PublicPerception& frame) {
    if (retired_sessions_.count(frame.session_id))
      return false;
    const bool new_session = !current_ || current_->session_id != frame.session_id;
    if (!new_session && frame.tick <= current_->tick)
      return false;
    const auto count = static_cast<std::int64_t>(frame.map.width) * frame.map.height;
    if (frame.map.width <= 0 || frame.map.height <= 0 || count > 1000000) {
      throw std::runtime_error("map dimensions exceed visualization limit (1000000 cells)");
    }
    if (!new_session &&
        (frame.map.width != current_->map.width || frame.map.height != current_->map.height)) {
      throw std::runtime_error("map dimensions changed without a new session_id");
    }
    if (new_session) {
      if (current_)
        retired_sessions_.insert(current_->session_id);
      cells_.assign(static_cast<std::size_t>(count), -1);
      paths_.clear();
      entities_.clear();
    }

    if (frame.self.alive) {
      const auto p = frame.self.position;
      const auto radius = frame.vision_radius;
      // 协议按横纵坐标差筛选实体；仅在地图内、当前方形视野内推断地形。
      for (int y = std::max(0, p.y - radius); y <= std::min(frame.map.height - 1, p.y + radius);
           ++y) {
        for (int x = std::max(0, p.x - radius); x <= std::min(frame.map.width - 1, p.x + radius);
             ++x) {
          cells_[index({x, y}, frame.map)] = 0;
        }
      }
      for (const auto& entity : frame.visible_entities) {
        if (entity.type == "obstacle")
          cells_[index(entity.position, frame.map)] = 100;
      }
      update_entities(frame);

      // 漏帧、死亡或复活导致的跳跃不连线，避免画出穿墙/瞬移的假轨迹。
      const bool continuous =
          !new_session && current_->self.alive && frame.tick == current_->tick + 1 &&
          frame.self.position_reason != "respawn" &&
          std::abs(p.x - current_->self.position.x) + std::abs(p.y - current_->self.position.y) <=
              1;
      if (!continuous || paths_.empty())
        paths_.push_back({p});
      else if (paths_.back().back().x != p.x || paths_.back().back().y != p.y)
        paths_.back().push_back(p);
    }
    current_ = frame;
    return true;
  }

  /**
   * @brief 将裁判左上角原点、y向下的格坐标转为OccupancyGrid数组下标。
   * ROS栅格数组从左下角逐行存储，因此必须翻转行；调用方须传合法坐标。
   */
  static std::size_t index(Position p, MapSize map) {
    return static_cast<std::size_t>(map.height - 1 - p.y) * map.width + p.x;
  }

  /// 最近接受的完整感知；尚未收到有效帧时为nullopt，不伪造出生点和尺寸。
  const std::optional<PublicPerception>& current() const noexcept {
    return current_;
  }

  /// 地形数组：-1未知、0最近观察为可通行、100最近观察为障碍，不含敌车。
  const std::vector<std::int8_t>& cells() const noexcept {
    return cells_;
  }

  /// 第二阶段临时墙恢复接口；调用方须持有公开事件依据，不改变默认观察更新逻辑。
  void restore_event_terrain(Position p, std::int8_t value) {
    if (!current_ || p.x < 0 || p.y < 0 || p.x >= current_->map.width ||
        p.y >= current_->map.height || (value != -1 && value != 0 && value != 100))
      throw std::invalid_argument("invalid event terrain restoration");
    cells_[index(p,current_->map)] = value;
  }

  /// 按连续存活片段划分的轨迹，不能把不同片段连接成同一条线。
  const std::vector<std::vector<Position>>& paths() const noexcept {
    return paths_;
  }

  /// 本场资源/敌车的记忆，键为类型分组后的实体标识；不会返回裁判未公开的实体。
  const std::map<std::string, EntityMemory>& entities() const noexcept {
    return entities_;
  }

  /// 返回本场已观察的格数；重新观察同一片区域不会重复计数。
  std::size_t observed_count() const {
    return static_cast<std::size_t>(
        std::count_if(cells_.begin(), cells_.end(), [](std::int8_t cell) { return cell >= 0; }));
  }

 private:
  /**
   * @brief 优先用协议标识关联同一个实体，避免敌人移动或资源类型改变后留下副本。
   * 每个席位只有一辆战车，故敌人优先按team关联，复活更换实例ID也覆盖旧尸体/旧位置。
   * 缺少席位时才使用实体ID；完全缺少标识时只记位置，不猜测不同观察属于同一辆车。
   */
  static std::string entity_key(const VisibleEntity& entity) {
    const bool robot = entity.type == "robot";
    const auto group = robot ? std::string("robot:") : std::string("resource:");
    if (robot && entity.team)
      return group + "seat:" + *entity.team;
    if (entity.id)
      return group + "id:" + *entity.id;
    if (robot && entity.robot_id)
      return group + "robot_id:" + *entity.robot_id;
    return group + "cell:" + entity.type + ":" + std::to_string(entity.position.x) + ":" +
           std::to_string(entity.position.y);
  }

  /**
   * @brief 在存活帧更新可见实体，并检查未再次出现的实体原位置是否仍在当前视野。
   * 再次看见同ID会更新位置、last_seen_tick并解除缺失标记；首次确认缺失的Tick
   * 在后续空视野中保持不变。资源保留历史；同席位敌人重现时覆盖上次实例和位置。
   */
  void update_entities(const PublicPerception& frame) {
    std::set<std::string> seen;
    for (const auto& entity : frame.visible_entities) {
      if (entity.type != "robot" && entity.type != "resource_normal" &&
          entity.type != "resource_high")
        continue;
      if (entity.type == "robot" && entity.team == frame.self.team)
        continue;
      const auto key = entity_key(entity);
      // 协议某帧可能缺team而仅有id，后续才给出完整身份。删去同一敌人的旧别名，
      // 不能让一次重现同时留下旧位置X与新位置实体；不同已知席位即使ID相同也不合并。
      if (entity.type == "robot") {
        for (auto previous = entities_.begin(); previous != entities_.end();) {
          const auto& old = previous->second.entity;
          const bool conflict = entity.team && old.team && entity.team != old.team;
          const bool same_identity = !conflict &&
              ((entity.team && old.team && entity.team == old.team) ||
               (entity.id && old.id && entity.id == old.id) ||
               (entity.robot_id && old.robot_id && entity.robot_id == old.robot_id));
          if (previous->first != key && old.type == "robot" && same_identity)
            previous = entities_.erase(previous);
          else
            ++previous;
        }
      }
      seen.insert(key);
      auto [it, inserted] = entities_.try_emplace(key);
      auto& record = it->second;
      if (inserted)
        record.first_seen_tick = frame.tick;
      record.entity = entity;
      record.last_seen_tick = frame.tick;
      record.first_missing_tick.reset();
    }
    for (auto& [key, record] : entities_) {
      if (seen.count(key) || record.first_missing_tick)
        continue;
      const auto p = record.entity.position;
      if (std::abs(p.x - frame.self.position.x) <= frame.vision_radius &&
          std::abs(p.y - frame.self.position.y) <= frame.vision_radius) {
        record.first_missing_tick = frame.tick;
      }
    }
  }

  std::optional<PublicPerception> current_;
  std::vector<std::int8_t> cells_;
  std::vector<std::vector<Position>> paths_;
  std::map<std::string, EntityMemory> entities_;
  std::set<std::string> retired_sessions_;
};

}  // namespace rmtc
