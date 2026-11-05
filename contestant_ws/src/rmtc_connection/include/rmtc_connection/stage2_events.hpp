/**
 * @file stage2_events.hpp
 * @brief 模型解释后的事件时间线、临时地形与Buff状态；从不解析广播自然语言。
 *
 * 只用本队公开观察确认入点、移动、驻留与区域命中；历史不足时不伪造已经获得Buff。
 * 可以接受模型从公开获得/刷新通知中提取的本车或他车状态。效果跨死亡保留，驻留
 * 进度死亡清零；所有期限均为绝对Tick，不随复活顺延。
 *
 * @details 模块协作与维护约定
 * 事件解释先经结构校验，再由这里维护Tick有效期、区域和临时状态，不能直接执行模型原文。
 * 临时墙与基础地形分开，事件到期需尊重最新观察；原来未知的格子不能凭恢复操作变成已知空地。
 */
#pragma once

#include "rmtc_connection/map_memory.hpp"
#include <deque>
#include <map>
#include <string>
#include <vector>

namespace rmtc {
/// 由LLM生成、经协议层严格校验的一项计划或已授予状态；cells和points不互相猜补。
struct Stage2EventEffect {
  std::string kind, id, actor;  ///< actor为空表示全局事件，A/B/C/D表示已授予的个体Buff。
  std::int64_t begin_tick{0}, end_tick{0};
  std::vector<Position> cells, points;
  double damage{0}, heal_per_tick{0}, hp_cap{100}, attack_damage{0}, defended_damage{0};
  double reflection_damage{0}, damage_multiplier{1};
  int effect_ticks{0}, stay_ticks{0}, attack_radius{0}, movement_window_ticks{0};
};

/// 一次有来源绑定的模型结果；保留旧危险区域格式用于历史回放，新增多事件时间线。
struct Stage2EventConstraints {
  std::string session_id, source_text;
  std::int64_t source_tick{0}, begin_tick{0}, end_tick{0};
  std::vector<Position> dangerous_cells;
  std::vector<Stage2EventEffect> events;
  bool timeline{false};  ///< true允许空时间线表示已理解的结束通知，不当成空危险区域执行。
};

struct Stage2EventBuff {
  Stage2EventEffect effect;
  std::int64_t until{0};
};

class Stage2Events {
 public:
  /// 新场次清空全部事件、临时墙下原地形、公开观察和已确认Buff。
  void reset();
  /// 记录当前公开观察及地形；同一Tick仅处理一次，不连接ROS，不读取裁判内部状态。
  void observe(const PublicPerception& frame, MapMemory& memory);
  /// 合并经协议校验的事件；晚到的解释利用本局历史补算，旧版本不能覆盖新解释。
  void merge(const Stage2EventConstraints& incoming, const PublicPerception& frame,
             MapMemory& memory);
  /// 仅在有效时返回已确认/公开授予的Buff；没有看到并不代表敌人一定没有Buff。
  const Stage2EventBuff* buff(const std::string& team, const std::string& kind,
                              std::int64_t tick) const;
  /// 查询全局规则事件是否正在生效，供攻防估计使用。
  const Stage2EventEffect* active(const std::string& kind, std::int64_t tick) const;
  /// 驻留计数只来自连续的真实公开帧；不会把规划中的站位算成已完成。
  int progress(const Stage2EventEffect& event, const std::string& team) const;
  /// 只读访问已校验计划，供目标收益和空间风险计算使用，不修改状态。
  const std::map<std::string, Stage2EventEffect>& effects() const {
    return effects_;
  }
  /// 用已知原地形和公告期限预测指定Tick的墙体；未知原地形仍返回-1，不制造空地。
  std::int8_t terrain_at(Position p, std::int64_t tick, const MapMemory& memory) const;
  /// 当前事件/Buff/临时墙数量的可解释日志，不包含私有状态或凭据。
  Json describe(const PublicPerception& frame) const;
  /// 已见空地后来变墙的疑似世界分裂格；仅来自实际观测，不扩张成未观察的完整区域。
  std::vector<Position> inferred_wall_cells() const;

  /// 判断格子是否属于模型给出的确切区域/点集，不根据事件名字猜测坐标。
  static bool contains(const std::vector<Position>& cells, Position p);

 private:
  struct Observation {
    PublicPerception frame;
    std::vector<std::int8_t> terrain;
  };
  std::deque<Observation> history_;
  std::map<std::string, Stage2EventEffect> effects_;
  std::map<std::string, std::int64_t> revisions_;
  std::map<std::string, Stage2EventBuff> buffs_;
  std::map<std::string, int> progress_;
  std::map<std::pair<int, int>, std::int8_t> wall_underlay_;
  struct InferredWall {
    std::int8_t base{0};
    std::int64_t first_seen{0},last_seen{0};
  };
  std::map<std::pair<int,int>,InferredWall> inferred_walls_;
  /// LLM缺失时，以“已知空地变墙”建立临时层；首次看到就是墙的格子不擅自归类。
  void observe_terrain_changes(const PublicPerception& frame);
  /// 按一次结算后的观察更新入点/驻留/易伤，按当前激活Tick授予区域幻术。
  void step(const PublicPerception& current, const PublicPerception* previous);
  /// 增量更新或新解释补算后重建临时墙，结束时只恢复原值，不把未知格变成空地。
  void terrain(const PublicPerception& frame, MapMemory& memory);
};
}  // namespace rmtc
