/**
 * @file map_generator.hpp
 * @brief 裁判练习地图生成接口；只供本地部署工具调用，不接入选手决策。
 *
 * 所有种子均按阶段生成：第一阶段20×20、第二阶段24×24；12个普通资源、4个高价值资源。
 * 初始自由区域连通，资源/出生点不重叠，任务目标及事件关键点保持可通行。
 * 障碍密度约10%是本地练习设置，不代表官方对障碍数量的额外规定。
 * seed=-1选择随机正整数实际种子，包括0在内的其余64位整数均生成可复现地图。
 *
 * @details 模块协作与维护约定
 * 仅用于本地练习部署，生成器可持有完整地图；该真值不得进入选手感知或特征。
 * 复现实验需记录阶段、实际种子和生成配置；随机种子入口应保留解析后的实际值。
 */
#pragma once

#include <nlohmann/json.hpp>
#include <cstdint>

namespace rmtc::practice {

/// 生成配置与复现信息分开保存，metadata不会作为机器人感知输入。
struct GeneratedPracticeMap {
  nlohmann::json config;
  nlohmann::json metadata;
  std::int64_t seed{0};
};

/**
 * @brief 基于官方阶段模板生成新的障碍和资源点位，保留第二阶段任务与事件定义。
 * @param base 官方模板JSON对象；只作为裁判配置的基础，不发给参赛程序。
 * @param stage 1或2，决定地图尺寸和A或A–D出生席位。
 * @param requested_seed -1随机，其余整数（含0）固定；记录实际seed便于重放。
 * @return 新配置和metadata；相同模板、阶段、种子、生成器版本得到相同结果。
 * 固定种子使用定义明确的随机引擎和自有洗牌，避免std::shuffle实现差异。
 * 不修改输入模板，不创建ROS对象；部署前仍须通过官方validate_config校验。
 * @throws std::exception 阶段/模板关键点非法或无法生成符合约束的地图。
 */
GeneratedPracticeMap generate_practice_map(const nlohmann::json& base, int stage,
                                           std::int64_t requested_seed = 0);

}  // namespace rmtc::practice
