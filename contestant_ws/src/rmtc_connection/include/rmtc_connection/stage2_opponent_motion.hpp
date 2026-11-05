/**
 * @file stage2_opponent_motion.hpp
 * @brief 根据连续公开帧估计敌车运动，预测仅服务下一步风险与追击，不改变当前攻击判定。
 *
 * @details 模块协作与维护约定
 * 连续公开帧提供方向/交火线索，断帧、复活或实例变化时不能机械延续原速度。
 * 预测只服务风险与追击；别人上一轮在打第三方，不代表下一轮一定不会攻击本车。
 */
#pragma once
#include "rmtc_connection/map_memory.hpp"
#include <cmath>

namespace rmtc {
/// 保持与当前可见敌车相同的顺序；projected是一步直线外推，details记录依据与不确定性。
struct OpponentMotionEstimate {
  std::vector<VisibleEntity> projected;
  Json details = Json::array();
};
/**
 * @brief 只关联连续Tick、同场次、相同ID/队伍的敌车；失联/跳跃/未知落点取消外推。
 * 对手可以转向、攻击或等待，因此当前位置和外推位置后续按权重混合，不当作已知未来。
 * 当前ATTACK先于移动结算，必须继续瞄准当前坐标，不能瞄准这里的外推坐标。
 */
inline OpponentMotionEstimate estimate_opponent_motion(
    const PublicPerception& frame, const std::optional<PublicPerception>& previous,
    const std::vector<const VisibleEntity*>& visible, const MapMemory& memory) {
  OpponentMotionEstimate result;
  const auto manhattan = [](Position a, Position b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
  };
  for (const auto* enemy : visible) {
    auto projected = *enemy;
    Position velocity{0, 0};
    bool observed = false, extrapolated = false;
    std::string tendency = "unknown";
    if (previous && previous->session_id == frame.session_id && previous->tick + 1 == frame.tick) {
      for (const auto& old : previous->visible_entities) {
        if (old.type != "robot" || old.team != enemy->team || old.id != enemy->id ||
            (!old.id && !old.team) || (old.hp && *old.hp <= 0))
          continue;
        velocity = {enemy->position.x - old.position.x, enemy->position.y - old.position.y};
        if (std::abs(velocity.x) + std::abs(velocity.y) > 1)
          break;  // 复活/传送不属于普通移动，不能把位移当速度。
        observed = true;
        const int closing = manhattan(old.position, frame.self.position) -
                            manhattan(enemy->position, frame.self.position);
        tendency = velocity.x == 0 && velocity.y == 0 ? "stationary"
                   : closing > 0                      ? "approaching"
                   : closing < 0                      ? "receding"
                                                      : "lateral";
        const Position next{enemy->position.x + velocity.x, enemy->position.y + velocity.y};
        if (next.x >= 0 && next.y >= 0 && next.x < frame.map.width && next.y < frame.map.height &&
            memory.cells()[MapMemory::index(next, frame.map)] == 0) {
          projected.position = next;
          extrapolated = true;
        }
        break;
      }
    }
    result.projected.push_back(projected);
    result.details.push_back({{"id", enemy->id.value_or(enemy->team.value_or("enemy"))},
                              {"position", {enemy->position.x, enemy->position.y}},
                              {"velocity", {velocity.x, velocity.y}},
                              {"projected", {projected.position.x, projected.position.y}},
                              {"continuous_observation", observed},
                              {"extrapolated", extrapolated},
                              {"tendency", tendency}});
  }
  return result;
}
}  // namespace rmtc
