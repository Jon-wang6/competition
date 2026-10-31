/** @file stage2_neural.cpp
 * @brief 固定结构MLP的C++前向传播和模型校验；不连接HTTP，不修改原策略参数。
 */
#include "rmtc_connection/stage2_neural.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <random>
#include <stdexcept>

namespace rmtc {
void NeuralWeights::validate(int maximum_horizon) const {
  for (double v : {weight, max_correction, override_margin, teacher_budget_ms})
    if (!std::isfinite(v) || v < 0)
      throw std::invalid_argument("invalid neural weight");
  if (weight > 2 || max_correction > 3 || override_margin > 1 || sample_stride < 1 ||
      sample_stride > 100 || teacher_horizon < 2 || teacher_horizon > maximum_horizon || teacher_budget_ms < 1 ||
      teacher_budget_ms > 150 || (enabled && model_file.empty()) ||
      (collect && sample_file.empty()))
    throw std::invalid_argument("invalid neural paths/budget");
}

const std::array<const char*, kNeuralInputs>& neural_feature_names() {
  static const std::array<const char*, kNeuralInputs> names{{"tick_fraction",
                                                             "own_hp",
                                                             "damage_fraction",
                                                             "resource_fraction",
                                                             "map_coverage",
                                                             "resource_goal",
                                                             "explore_goal",
                                                             "mission_goal",
                                                             "combat_goal",
                                                             "evade_goal",
                                                             "local_action_score",
                                                             "baseline_action_score",
                                                             "goal_score",
                                                             "resource_score",
                                                             "mission_score",
                                                             "combat_score",
                                                             "information_score",
                                                             "patrol_score",
                                                             "visible_enemy_count",
                                                             "nearest_enemy_distance",
                                                             "incoming_damage_bound",
                                                             "destination_threat",
                                                             "destination_event_risk",
                                                             "goal_progress",
                                                             "blocked_streak",
                                                             "wait_action",
                                                             "move_action",
                                                             "attack_action",
                                                             "defend_action",
                                                             "move_dx",
                                                             "move_dy",
                                                             "spawn_protected",
    "nearest_enemy_hp_known", "nearest_enemy_hp", "nearest_enemy_defending",
    "nearest_enemy_protected", "nearest_enemy_unrivaled", "attack_target_present",
    "attack_target_hp_known", "attack_target_hp", "attack_target_defending",
    "attack_target_protected", "attack_target_unrivaled", "enemy_forward",
    "enemy_sideways", "enemy_recent_hp_loss", "own_recent_hp_change", "own_defending",
    "own_unrivaled_remaining", "own_illusion_remaining", "own_angel_remaining",
    "own_purity_remaining", "own_pepper_remaining", "pan_remaining", "world_split_remaining",
    "explosion_time_until", "mission_completion", "goal_path_distance",
    "landing_free_neighbors", "landing_unknown_neighbors", "landing_wall_neighbors",
    "landing_unknown_in_view", "nearest_enemy_distance_change", "landing_enemy_attackers"}};
  return names;
}

TinyNeuralNetwork::TinyNeuralNetwork(unsigned seed, int hidden1, int hidden2, int inputs,const std::string& domain)
    : inputs_(inputs), hidden1_(hidden1), hidden2_(hidden2),domain_(domain) {
  if(domain!="combat"&&domain!="collection")throw std::invalid_argument("unknown neural feature domain");
  if(domain=="collection"&&inputs!=kNeuralInputs)throw std::invalid_argument("collection features require 64 inputs");
  if ((inputs != 32 && inputs != kNeuralInputs) ||
      (inputs==32&&(hidden1!=64||hidden2!=32)) || hidden1 < 8 || hidden1 > 256 ||
      hidden2 < 8 || hidden2 > 128)
    throw std::invalid_argument("invalid neural architecture");
  parameters_.assign(inputs * hidden1 + hidden1 + hidden1 * hidden2 + hidden2 + hidden2 + 1, 0);
  std::mt19937 rng(seed);
  std::normal_distribution<float> first(0, std::sqrt(2.0F / inputs_));
  std::normal_distribution<float> second(0, std::sqrt(2.0F / hidden1_));
  for (int i = 0; i < inputs_ * hidden1_; ++i)
    parameters_[i] = first(rng);
  const int offset = inputs_ * hidden1_ + hidden1_;
  for (int i = 0; i < hidden1_ * hidden2_; ++i)
    parameters_[offset + i] = second(rng);
}

float TinyNeuralNetwork::predict(const NeuralFeatures& input) const {
  for (float v : input)
    if (!std::isfinite(v))
      throw std::runtime_error("nonfinite neural input");
  std::array<float, 256> h1{};
  std::array<float, 128> h2{};
  const int b1 = inputs_ * hidden1_;
  const int w2 = b1 + hidden1_, b2 = w2 + hidden1_ * hidden2_, w3 = b2 + hidden2_;
  for (int j = 0; j < hidden1_; ++j) {
    float v = parameters_[b1 + j];
    for (int i = 0; i < inputs_; ++i)
      v += parameters_[j * inputs_ + i] * input[i];
    h1[j] = std::max(0.0F, v);
  }
  for (int j = 0; j < hidden2_; ++j) {
    float v = parameters_[b2 + j];
    for (int i = 0; i < hidden1_; ++i)
      v += parameters_[w2 + j * hidden1_ + i] * h1[i];
    h2[j] = std::max(0.0F, v);
  }
  float out = parameters_.back();
  for (int i = 0; i < hidden2_; ++i)
    out += parameters_[w3 + i] * h2[i];
  if (!std::isfinite(out))
    throw std::runtime_error("nonfinite neural output");
  return out;
}

void TinyNeuralNetwork::initialize_from(const TinyNeuralNetwork& source) {
  const int ni=source.inputs_,h1=source.hidden1_,h2=source.hidden2_;
  if(inputs_<ni||hidden1_<h1||hidden2_<h2)
    throw std::invalid_argument("cannot shrink initialized neural model");
  const int ob1=ni*h1,ow2=ob1+h1,ob2=ow2+h1*h2,ow3=ob2+h2;
  const int b1=inputs_*hidden1_,w2=b1+hidden1_,b2=w2+hidden1_*hidden2_,w3=b2+hidden2_;
  for(int row=0;row<h1;++row) {
    for(int col=0;col<inputs_;++col)
      parameters_[row*inputs_+col]=col<ni?source.parameters_[row*ni+col]:0;
    parameters_[b1+row]=source.parameters_[ob1+row];
  }
  for(int row=0;row<h2;++row) {
    for(int col=0;col<hidden1_;++col)
      parameters_[w2+row*hidden1_+col]=col<h1?source.parameters_[ow2+row*h1+col]:0;
    parameters_[b2+row]=source.parameters_[ob2+row];
  }
  for(int col=0;col<hidden2_;++col)
    parameters_[w3+col]=col<h2?source.parameters_[ow3+col]:0;
  parameters_.back()=source.parameters_.back();
}

void TinyNeuralNetwork::load(const std::string& path,const std::string& domain) {
  if (std::filesystem::file_size(path) > 4 * 1024 * 1024)
    throw std::runtime_error("neural model too large");
  std::ifstream stream(path);
  nlohmann::json data;
  stream >> data;
  const auto shape = data.at("architecture").get<std::vector<int>>();
  if (shape.size() != 4 || shape[3] != 1 ||
      (data.at("format") != "rmtc_residual_mlp_v1" && data.at("format") != "rmtc_residual_mlp_v2"))
    throw std::runtime_error("incompatible neural model/features");
  if (data.at("format") == "rmtc_residual_mlp_v1" && shape != std::vector<int>{32,64,32,1})
    throw std::runtime_error("invalid legacy neural architecture");
  TinyNeuralNetwork checked(1, shape[1], shape[2], shape[0],domain);
  const auto& feature_names=domain=="collection"?collection_neural_feature_names():neural_feature_names();
  std::vector<std::string> names(feature_names.begin(), feature_names.begin()+shape[0]);
  if (data.at("features") != nlohmann::json(names))
    throw std::runtime_error("incompatible neural feature order");
  auto weights = data.at("parameters").get<std::vector<float>>();
  if (weights.size() != checked.parameters().size())
    throw std::runtime_error("invalid neural parameter count");
  for (float v : weights)
    if (!std::isfinite(v) || std::abs(v) > 100)
      throw std::runtime_error("invalid neural parameter");
  checked.parameters_ = std::move(weights);
  *this = std::move(checked);
}

void TinyNeuralNetwork::save(const std::string& path, const nlohmann::json& metadata) const {
  const std::filesystem::path target(path);
  if (!target.parent_path().empty())
    std::filesystem::create_directories(target.parent_path());
  std::ofstream stream(path);
  stream.exceptions(std::ios::failbit | std::ios::badbit);
  const auto& feature_names=domain_=="collection"?collection_neural_feature_names():neural_feature_names();
  const std::vector<std::string> names(feature_names.begin(),feature_names.begin()+inputs_);
  stream << nlohmann::json{{"format", inputs_ == 32 ? "rmtc_residual_mlp_v1" : "rmtc_residual_mlp_v2"},
                           {"architecture", {inputs_, hidden1_, hidden2_, 1}},
                           {"features", names},
                           {"parameters", parameters_},
                           {"training", metadata}}
                .dump(2)
         << '\n';
}
}  // namespace rmtc
