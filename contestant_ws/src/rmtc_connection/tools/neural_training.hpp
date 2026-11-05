/**
 * @file neural_training.hpp
 * @brief 训练专用C++/Eigen前向、反向传播与Adam；正式选手不链接本文件。
 * 参数顺序与TinyNeuralNetwork一致，梯度由独立有限差分测试校验。
 *
 * @details 模块协作与维护约定
 * 训练时保存前向中间量、反向梯度及Adam状态，部署推理只需要兼容的模型参数。
 * 矩阵/数组顺序必须与TinyNeuralNetwork一致；梯度正确性依靠独立有限差分而非训练损失自证。
 */
#pragma once
#include "rmtc_connection/stage2_neural.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <vector>
namespace rmtc::training {
struct Sample {
  NeuralFeatures features{};
  float target{0}, baseline{0}, importance{1};
  std::string state, group;
  bool selected{false}, safety_ok{true};
};
struct Gradient {
  double loss{0};
  std::vector<float> values;
};
using Pair = std::pair<std::size_t, std::size_t>;
using ActionGroup = std::vector<std::size_t>;
using Matrix = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

/**
 * @brief 混合教师与旧模型标签，明确区分动作排序的部署增量和回归的原始输出。
 * listwise目标直接加在baseline上，旧模型项必须与实际部署的权重/限幅一致。
 * alpha=0时，排序目标应与旧模型自身一致，不产生改变策略的数据梯度。
 */
inline float blend_training_target(float teacher,float previous,float alpha,
                                  float policy_weight,bool listwise,float max_correction=1.F) {
  const float old_target=listwise?policy_weight*std::clamp(previous,-max_correction,max_correction):previous;
  return alpha*teacher+(1-alpha)*old_target;
}

/**
 * @brief 计算一个批次的均方误差、L2正则项和模型全部参数的解析梯度。
 * 行是神经元，列是样本；ReLU在0处导数取0，参数向量只读，不原地更新。
 */
inline Gradient gradient(const TinyNeuralNetwork& model, const std::vector<Sample>& data,
                         const std::vector<std::size_t>& order, std::size_t offset,
                         std::size_t count, float regularization = 0.0001F) {
  const auto& p = model.parameters();
  const int ni=model.input_size(),h1=model.hidden1_size(),h2=model.hidden2_size();
  const int b1 = ni*h1, w2 = b1+h1, b2 = w2+h1*h2, w3 = b2+h2;
  const Eigen::Map<const Matrix> W1(p.data(), h1, ni), W2(p.data()+w2, h2, h1), W3(p.data()+w3,1,h2);
  const Eigen::Map<const Eigen::VectorXf> B1(p.data()+b1,h1), B2(p.data()+b2,h2);
  Matrix x(ni, count), y(1, count), importance(1, count);
  double total_weight = 0;
  for (std::size_t j = 0; j < count; ++j) {
    const auto& s = data.at(order.at(offset + j));
    for (int i = 0; i < ni; ++i)
      x(i, j) = s.features[i];
    y(0, j) = s.target;
    importance(0, j) = s.importance;
    total_weight += s.importance;
  }
  const Matrix z1 = (W1 * x).colwise() + B1, a1 = z1.cwiseMax(0);
  const Matrix z2 = (W2 * a1).colwise() + B2, a2 = z2.cwiseMax(0);
  const Matrix error = (W3 * a2).array() + p.back() - y.array();
  const Matrix output = 2.F / static_cast<float>(total_weight) * error.array() * importance.array();
  const Matrix d2 = (W3.transpose() * output).array() * (z2.array() > 0).cast<float>();
  const Matrix d1 = (W2.transpose() * d2).array() * (z1.array() > 0).cast<float>();
  Gradient result;
  result.loss = (error.array().square() * importance.array()).sum() / total_weight;
  result.values.assign(p.size(), 0);
  Eigen::Map<Matrix>(result.values.data(), h1, ni) = d1 * x.transpose();
  Eigen::Map<Eigen::VectorXf>(result.values.data() + b1, h1) = d1.rowwise().sum();
  Eigen::Map<Matrix>(result.values.data() + w2, h2, h1) = d2 * a1.transpose();
  Eigen::Map<Eigen::VectorXf>(result.values.data() + b2, h2) = d2.rowwise().sum();
  Eigen::Map<Matrix>(result.values.data() + w3, 1, h2) = output * a2.transpose();
  result.values.back() = output.sum();
  for (std::size_t i = 0; i < p.size(); ++i) {
    result.loss += .5 * regularization * p[i] * p[i];
    result.values[i] += regularization * p[i];
  }
  return result;
}

/** @brief 学习同一局面中两个动作的残差差值，直接对应排序；公共分数平移不影响损失。 */
inline Gradient pairwise_gradient(const TinyNeuralNetwork& model, const std::vector<Sample>& data,
                                  const std::vector<Pair>& pairs,
                                  const std::vector<std::size_t>& order, std::size_t offset,
                                  std::size_t count) {
  const auto& p = model.parameters();
  const int ni=model.input_size(),h1=model.hidden1_size(),h2=model.hidden2_size();
  const int b1=ni*h1,w2=b1+h1,b2=w2+h1*h2,w3=b2+h2;
  const Eigen::Map<const Matrix> W1(p.data(),h1,ni),W2(p.data()+w2,h2,h1),W3(p.data()+w3,1,h2);
  const Eigen::Map<const Eigen::VectorXf> B1(p.data()+b1,h1),B2(p.data()+b2,h2);
  Matrix x(ni, count * 2);
  double total = 0;
  for (std::size_t j = 0; j < count; ++j) {
    const auto pair = pairs.at(order.at(offset + j));
    const auto& a = data[pair.first];
    const auto& b = data[pair.second];
    for (int i = 0; i < ni; ++i) {
      x(i, 2 * j) = a.features[i];
      x(i, 2 * j + 1) = b.features[i];
    }
    total += a.importance;
  }
  const Matrix z1 = (W1 * x).colwise() + B1, a1 = z1.cwiseMax(0), z2 = (W2 * a1).colwise() + B2,
               a2 = z2.cwiseMax(0);
  const Matrix prediction = (W3 * a2).array() + p.back();
  Matrix output = Matrix::Zero(1, count * 2);
  Gradient result;
  result.values.assign(p.size(), 0);
  for (std::size_t j = 0; j < count; ++j) {
    const auto pair = pairs.at(order.at(offset + j));
    const auto& a = data[pair.first];
    const auto& b = data[pair.second];
    const float error = prediction(0, 2 * j) - prediction(0, 2 * j + 1) - (a.target - b.target);
    result.loss += a.importance * error * error / total;
    output(0, 2 * j) = 2 * a.importance * error / total;
    output(0, 2 * j + 1) = -output(0, 2 * j);
  }
  const Matrix d2 = (W3.transpose() * output).array() * (z2.array() > 0).cast<float>();
  const Matrix d1 = (W2.transpose() * d2).array() * (z1.array() > 0).cast<float>();
  Eigen::Map<Matrix>(result.values.data(),h1,ni) = d1*x.transpose();
  Eigen::Map<Eigen::VectorXf>(result.values.data()+b1,h1) = d1.rowwise().sum();
  Eigen::Map<Matrix>(result.values.data()+w2,h2,h1) = d2*a1.transpose();
  Eigen::Map<Eigen::VectorXf>(result.values.data()+b2,h2) = d2.rowwise().sum();
  Eigen::Map<Matrix>(result.values.data()+w3,1,h2) = output*a2.transpose();
  result.values.back() = output.sum();
  for (std::size_t i = 0; i < p.size(); ++i) {
    result.loss += .00005 * p[i] * p[i];
    result.values[i] += .0001F * p[i];
  }
  return result;
}

/**
 * @brief 按同一局势的完整动作集合学习排序，softmax目标来自教师评分。
 * 正向使用部署时的残差权重和限幅；安全掩码剔除不能覆盖原动作的危险根节点。
 * 同一局势的概率和为1，避免为公共评分偏移花费容量；验证仍使用真实换动作门槛。
 */
inline Gradient listwise_gradient(const TinyNeuralNetwork& model,const std::vector<Sample>& data,
    const std::vector<ActionGroup>& groups,const std::vector<std::size_t>& order,std::size_t offset,
    std::size_t count,float policy_weight=.5F,float temperature=.05F,float max_correction=1.F) {
  const auto& p=model.parameters();
  const int ni=model.input_size(),h1=model.hidden1_size(),h2=model.hidden2_size();
  const int b1=ni*h1,w2=b1+h1,b2=w2+h1*h2,w3=b2+h2;
  std::vector<std::size_t> indices,ends;std::vector<float> weights;
  double total=0;
  for(std::size_t j=0;j<count;++j) {
    float weight=0;
    for(auto i:groups.at(order.at(offset+j)))if(data[i].safety_ok||data[i].selected) {
      indices.push_back(i);weight=std::max(weight,data[i].importance);
    }
    ends.push_back(indices.size());weights.push_back(weight);total+=weight;
  }
  if(indices.empty()||total<=0)throw std::runtime_error("empty listwise action batch");
  Matrix x(ni,indices.size());
  for(std::size_t j=0;j<indices.size();++j)
    for(int i=0;i<ni;++i)x(i,j)=data[indices[j]].features[i];
  const Eigen::Map<const Matrix> W1(p.data(),h1,ni),W2(p.data()+w2,h2,h1),W3(p.data()+w3,1,h2);
  const Eigen::Map<const Eigen::VectorXf> B1(p.data()+b1,h1),B2(p.data()+b2,h2);
  const Matrix z1=(W1*x).colwise()+B1,a1=z1.cwiseMax(0),z2=(W2*a1).colwise()+B2,a2=z2.cwiseMax(0);
  const Matrix prediction=(W3*a2).array()+p.back();
  Matrix output=Matrix::Zero(1,indices.size());Gradient result;
  std::size_t begin=0;
  for(std::size_t g=0;g<ends.size();++g) {
    const auto end=ends[g];double max_policy=-1e30,max_teacher=-1e30;
    for(auto j=begin;j<end;++j) {
      const auto& s=data[indices[j]];
      max_policy=std::max(max_policy,double((s.baseline+policy_weight*std::clamp(prediction(0,j),-max_correction,max_correction))/temperature));
      max_teacher=std::max(max_teacher,double((s.baseline+s.target)/temperature));
    }
    std::vector<double> policy(end-begin),teacher(end-begin);double psum=0,tsum=0;
    for(auto j=begin;j<end;++j) {
      const auto& s=data[indices[j]];
      psum+=policy[j-begin]=std::exp((s.baseline+policy_weight*std::clamp(prediction(0,j),-max_correction,max_correction))/temperature-max_policy);
      tsum+=teacher[j-begin]=std::exp((s.baseline+s.target)/temperature-max_teacher);
    }
    for(auto j=begin;j<end;++j) {
      const double actual=policy[j-begin]/psum,target=teacher[j-begin]/tsum,weight=weights[g]/total;
      result.loss-=weight*target*std::log(std::max(actual,1e-30));
      if(std::abs(prediction(0,j))<max_correction)
        output(0,j)=weight*(actual-target)*policy_weight/temperature;
    }
    begin=end;
  }
  const Matrix d2=(W3.transpose()*output).array()*(z2.array()>0).cast<float>();
  const Matrix d1=(W2.transpose()*d2).array()*(z1.array()>0).cast<float>();
  result.values.assign(p.size(),0);
  Eigen::Map<Matrix>(result.values.data(),h1,ni)=d1*x.transpose();
  Eigen::Map<Eigen::VectorXf>(result.values.data()+b1,h1)=d1.rowwise().sum();
  Eigen::Map<Matrix>(result.values.data()+w2,h2,h1)=d2*a1.transpose();
  Eigen::Map<Eigen::VectorXf>(result.values.data()+b2,h2)=d2.rowwise().sum();
  Eigen::Map<Matrix>(result.values.data()+w3,1,h2)=output*a2.transpose();result.values.back()=output.sum();
  for(std::size_t i=0;i<p.size();++i) {
    result.loss+=.00005*p[i]*p[i];result.values[i]+=.0001F*p[i];
  }
  return result;
}

/// Adam的一二阶矩和更新次数；可序列化，继续训练时保留优化器状态。
struct Adam {
  std::vector<float> first, second;
  int steps{0};
  /** @brief 将梯度范数限制为5后执行Adam更新，拒绝非有限损失与参数。 */
  void update(TinyNeuralNetwork& model, const Gradient& g, float rate) {
    double squared = 0;
    for (float v : g.values) {
      if (!std::isfinite(v))
        throw std::runtime_error("nonfinite gradient");
      squared += v * v;
    }
    const float scale = squared > 25 ? 5 / std::sqrt(squared) : 1;
    ++steps;
    const double c1 = 1 - std::pow(.9, steps), c2 = 1 - std::pow(.999, steps);
    auto& p = model.parameters();
    if(first.empty()&&steps==1){first.assign(p.size(),0);second=first;}
    if(first.size()!=p.size()||second.size()!=p.size()||g.values.size()!=p.size())
      throw std::runtime_error("optimizer/model dimension mismatch");
    for (std::size_t i = 0; i < p.size(); ++i) {
      const float v = g.values[i] * scale;
      first[i] = .9F * first[i] + .1F * v;
      second[i] = .999F * second[i] + .001F * v * v;
      p[i] -= rate * (first[i] / c1) / (std::sqrt(second[i] / c2) + 1e-8);
      if (!std::isfinite(p[i]))
        throw std::runtime_error("nonfinite trained parameter");
    }
  }
};
}  // namespace rmtc::training
