/**
 * @file stage2_neural.hpp
 * @brief 第二阶段独立的小型残差网络；输入现有算法评分，输出动作评分修正。
 *
 * 特征只来自本队公开感知、显式记忆及现有规划，不读裁判内部地图。
 * 新特征64维，默认64→128(ReLU)→64(ReLU)→1；仍可读取旧32维模型。
 * 关闭时不加载模型；训练器使用同一参数排列，正式推理不依赖GPU或训练库。
 */
#pragma once
#include <array>
#include <cstddef>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace rmtc {
constexpr int kNeuralInputs = 64, kNeuralHidden1 = 128, kNeuralHidden2 = 64;
constexpr int kNeuralParameters = kNeuralInputs * kNeuralHidden1 + kNeuralHidden1 +
    kNeuralHidden1 * kNeuralHidden2 + kNeuralHidden2 + kNeuralHidden2 + 1;
using NeuralFeatures = std::array<float, kNeuralInputs>;

/// 神经网络与离线教师分别开关；采样路径仅用于显式开启的本地训练。
struct NeuralWeights {
  bool enabled{false}, collect{false};
  std::string model_file, sample_file, sample_group;
  double weight{0.25}, max_correction{1.0}, override_margin{0.03};
  int sample_stride{4}, teacher_horizon{6};
  double teacher_budget_ms{80};
  /// 拒绝无界网络修正与异常采样预算；启用推理必须明确提供模型。
  void validate(int maximum_horizon=7) const;
};

/// 固定、具名的可解释输入；顺序是模型格式的一部分，加载时逐项核对。
const std::array<const char*, kNeuralInputs>& neural_feature_names();
/// 采集网络使用独立特征域；同样64维也不能误加载战斗网络。
const std::array<const char*, kNeuralInputs>& collection_neural_feature_names();

class TinyNeuralNetwork {
 public:
  /// 固定随机种子初始化隐藏层，输出层为0，使未训练网络修正严格为0。
  explicit TinyNeuralNetwork(unsigned seed = 1, int hidden1 = kNeuralHidden1,
                            int hidden2 = kNeuralHidden2, int inputs = kNeuralInputs,
                            const std::string& domain="combat");
  /// 返回模型实际维度；旧模型保持32维和4225参数，不用新尺寸解释旧权重。
  int input_size() const noexcept { return inputs_; }
  int hidden1_size() const noexcept { return hidden1_; }
  int hidden2_size() const noexcept { return hidden2_; }
  /// 计算两层ReLU的前向传播；拒绝非有限输入，不更新任何局内状态。
  float predict(const NeuralFeatures& input) const;
  /// 严格读取版本、维度、特征名和有限权重，错误模型不能静默继续运行。
  void load(const std::string& path,const std::string& domain="combat");
  /// 扩容时逐层复制旧网络，新增通道初始贡献为0；同一输入的预测保持不变。
  void initialize_from(const TinyNeuralNetwork& source);
  /// 写入模型、训练来源与验证指标；正式进程只加载，不在线更新。
  void save(const std::string& path, const nlohmann::json& metadata = {}) const;
  /// 返回按输出神经元逐行排列的W1,b1,W2,b2,W3,b3，供C++训练器共享。
  std::vector<float>& parameters() noexcept {
    return parameters_;
  }
  const std::vector<float>& parameters() const noexcept {
    return parameters_;
  }

 private:
  int inputs_{kNeuralInputs}, hidden1_{kNeuralHidden1}, hidden2_{kNeuralHidden2};
  std::string domain_{"combat"};
  std::vector<float> parameters_;
};
}  // namespace rmtc
