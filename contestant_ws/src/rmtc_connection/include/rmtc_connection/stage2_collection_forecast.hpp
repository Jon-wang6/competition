/**
 * @file stage2_collection_forecast.hpp
 * @brief 资源竞争的共同情景与分区探索先验；只使用本队公开观察。
 *
 * 每个情景中每名敌人只有一个位置、一条采集路线。历史敌人位置只是猜测，
 * 到达时间不是确定事实；未知地形不当成空地，也不读取裁判资源配置。
 *
 * @details 模块协作与维护约定
 * 共同情景内每名对手只有一条采集路线，避免独立估计每个资源时把同一个对手当成能分身。
 * 输入为公开记忆和可通行地形，输出估计到达期限与区域供给先验；历史敌人位置随时可能失真。
 */
#pragma once
#include "rmtc_connection/map_memory.hpp"

namespace rmtc {
/// 单个共同情景；arrival以资源记忆键索引，缺失表示本情景没有预测到其被采集。
struct CollectionRaceScenario {
  std::string name;
  std::map<std::string,int> arrival;
};
struct CollectionRaceForecast {
  std::vector<CollectionRaceScenario> scenarios;
  int current_enemies{0}, historical_enemies{0};
  /// 输出假设与每块资源的预计到达Tick；不得将其写回实体记忆。
  Json describe() const;
};
/**
 * @brief 在四种有限情景中逐Tick模拟敌人沿已知空地依次采集。
 * @param previous 仅用连续公开帧提取移动方向；可为空。
 * @param memory_ticks 允许历史敌人参与假设的最大年龄；死亡记录不参与。
 * 最近/高值/延续方向/交战停留分别形成一个共同情景，不叠乘成敌人会分身。
 */
CollectionRaceForecast forecast_collection_race(const PublicPerception& frame,
    const MapMemory& memory,const PublicPerception* previous,int memory_ticks);

struct RegionalResourceSupply {
  std::vector<double> factors; ///< 按y向下排列；未知格均值保持1，只改变探索方向。
  Json regions=Json::array();
};
/**
 * @brief 以弱均匀先验融合分区资源发现密度和近期敌人经过的可能性。
 * 不假定真实地图有资源簇；证据少时收缩至均匀先验，所有区域保留非零机会。
 * 未完成高值任务时只温和调整发现方向，不凭空创建高值资源或保证任务可达。
 */
RegionalResourceSupply regional_collection_supply(const PublicPerception& frame,
    const MapMemory& memory,double strength,double initial_resource_value=200.0);
} // namespace rmtc
