/**
 * @file stage2_exploration_value_test.cpp
 * @brief 验证开图奖励在旧封顶区间仍能区分观察面积，并保留关闭与旧策略回退。
 *
 * @details 模块协作与维护约定
 * 验证对象：stage2_exploration_value.hpp。
 * 运行约定：由独立测试可执行文件/CTest执行，断言失败用于定位回归，不是正式选手的逐帧逻辑。
 * 结果边界：通过表示这些构造场景满足约束，不证明所有随机地图均最优或稳定达到某个总分。
 */
#include "rmtc_connection/stage2_exploration_value.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

/** @brief 检查奖励单调、有限、上界与旧行为，不依赖隐藏地图或固定路径。 */
int main() {
  try {
    const auto value = rmtc::exploration_value;
    for (const double raw : {0.0, 0.01, 0.5, 1.5, 2.0, 4.0, 10.0}) {
      if (value(raw, 1.5, 0) != std::min(raw, 1.5) ||
          value(raw, 0, 1) != 0 || !std::isfinite(value(raw, 1.5, 1)) ||
          value(raw, 1.5, 1) > 1.5)
        throw std::runtime_error("disabled, legacy or bounded behavior changed");
    }
    if (!(value(2, 1.5, 1) < value(4, 1.5, 1)) ||
        value(2, 1.5, 0) != value(4, 1.5, 0) ||
        std::abs(value(2, 1.5, .5) - .5 * (value(2, 1.5, 0) + value(2, 1.5, 1))) > 1e-12)
      throw std::runtime_error("new information still loses its marginal value");
    std::cout << "Exploration reward checks passed\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
