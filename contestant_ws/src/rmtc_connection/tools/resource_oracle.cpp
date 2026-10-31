/**
 * @file resource_oracle.cpp
 * @brief 仅供离线可达性审计：假设完整地图已知，计算访问16资源所需的绝对最少步数。
 *
 * 使用经过独立全排列验证的A*距离表与精确开放终点TSP动态规划。
 * 这比实际选手拥有更多信息，结果只能作为100Tick全收集的理论上限检查；
 * 地图真值、最优顺序及路径不会发送给选手节点，也不会写入策略配置。
 */
#include "rmtc_connection/map_generator.hpp"
#include "rmtc_connection/resource_route.hpp"
#include <fstream>
#include <iostream>
#include <stdexcept>

/** @brief 按种子生成完整测试地图并计算精确最短访问路程，逐图落盘，错误时返回非零。 */
int main(int argc, char** argv) {
  try {
    if (argc != 4)
      throw std::invalid_argument("usage: resource_oracle template.json seeds.json results.jsonl");
    std::ifstream template_file(argv[1]), seed_file(argv[2]);
    if (!template_file || !seed_file)
      throw std::runtime_error("cannot open oracle input");
    nlohmann::json base, seeds;
    template_file >> base;
    seed_file >> seeds;
    if (!seeds.is_array() || seeds.empty())
      throw std::invalid_argument("oracle seeds must be nonempty array");
    std::ofstream output(argv[3]);
    if (!output)
      throw std::runtime_error("cannot write oracle output");
    std::size_t completed = 0, feasible = 0;
    for (const auto& seed_value : seeds) {
      const auto seed = seed_value.get<std::int64_t>();
      if (seed == -1)
        throw std::invalid_argument("oracle requires fixed seeds");
      const auto config = rmtc::practice::generate_practice_map(base, 1, seed).config;
      const auto& map = config.at("map");
      const int width = map.at("width").get<int>(), height = map.at("height").get<int>();
      rmtc::planning::GridMap grid{
          width, height, std::vector<std::int8_t>(static_cast<std::size_t>(width) * height, 0)};
      for (const auto& p : map.at("obstacles"))
        grid.cells[p.at(1).get<std::size_t>() * width + p.at(0).get<std::size_t>()] = 100;
      std::vector<rmtc::planning::ResourceGoal> goals;
      for (const auto& resource : map.at("resources"))
        goals.push_back(
            {resource.at("id").get<std::string>(),
             {resource.at("position").at(0).get<int>(), resource.at("position").at(1).get<int>()},
             resource.at("value").get<double>(),
             0});
      const auto& spawn = config.at("robots").at(0).at("spawn");
      rmtc::planning::ResourceRoutePlanner planner;
      const auto& tour =
          planner.plan(grid, {spawn.at(0).get<int>(), spawn.at(1).get<int>()}, goals);
      if (!tour.exact || tour.keys.size() != goals.size())
        throw std::runtime_error("oracle did not solve all resources exactly");
      const bool possible = tour.cost <= 100;
      feasible += possible;
      output << nlohmann::json{{"seed", seed},
                               {"min_steps_full_information", tour.cost},
                               {"feasible_within_100", possible},
                               {"resources", goals.size()}}
                    .dump()
             << '\n';
      output.flush();
      if (!output)
        throw std::runtime_error("cannot write oracle result");
      if (++completed % 50 == 0)
        std::cout << "oracle completed " << completed << '/' << seeds.size() << std::endl;
    }
    std::cout << "oracle feasible " << feasible << '/' << completed << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
