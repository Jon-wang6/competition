/** @file stage2_neural_policy_test.cpp
 * @brief 验证零网络/只采样时动作不变，以及网络不能新增当前已知致死动作。
 */
#include "rmtc_connection/stage2_strategy.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unistd.h>

/// 业务断言，不随Release优化而省略。
void require(bool value, const char* message) {
  if (!value)
    throw std::runtime_error(message);
}
/// 构造公开可见的一对一场景，不提供隐藏地形或对手指令。
rmtc::PublicPerception perception(int tick, double hp = 100) {
  rmtc::PublicPerception p;
  p.session_id = "neural-policy-test";
  p.tick = tick;
  p.map = {9, 9};
  p.vision_radius = 4;
  p.self.team = "A";
  p.self.alive = true;
  p.self.hp = hp;
  p.self.position = {4, 4};
  rmtc::VisibleEntity e;
  e.type = "robot";
  e.team = "B";
  e.id = "B";
  e.position = {4, 5};
  e.hp = 100;
  e.spawn_protected = false;
  e.defending = false;
  p.visible_entities = {e};
  return p;
}
/** @brief 使用真实策略流程检查模型开关、教师副作用与安全过滤；仅写本测试临时目录。 */
int main() {
  const auto dir =
      std::filesystem::temp_directory_path() / ("rmtc-neural-policy-" + std::to_string(getpid()));
  try {
    std::filesystem::create_directories(dir);
    const auto model = (dir / "zero.json").string();
    rmtc::TinyNeuralNetwork zero;
    zero.save(model);
    rmtc::Stage2Weights w;
    w.rollout.weight = 0;
    auto n = w;
    n.neural.enabled = true;
    n.neural.model_file = model;
    auto t = w;
    t.neural.collect = true;
    t.neural.sample_file = (dir / "samples.jsonl").string();
    t.neural.sample_group = "100";
    t.neural.teacher_horizon = 3;
    t.neural.sample_stride = 1;
    rmtc::Stage2Strategy base(w), neural(n), teacher(t);
    for (int tick = 100; tick < 108; ++tick) {
      const auto p = perception(tick);
      const auto a = base.decide(p), b = neural.decide(p), c = teacher.decide(p);
      require(a.type == b.type && a.direction == b.direction,
              "zero residual changed baseline action");
      require(a.type == c.type && a.direction == c.direction,
              "collecting labels changed executed action");
    }
    // 实际采样核对血量、连续掉血和攻击对象字段，避免新增维度只改名字没有接数据。
    auto features_cfg=t;
    features_cfg.neural.sample_file=(dir/"features.jsonl").string();
    {
      rmtc::Stage2Strategy collector(features_cfg);
      collector.decide(perception(100));
      auto p=perception(101,90);p.visible_entities.front().hp=80;
      collector.decide(p);
    }
    std::ifstream feature_file(features_cfg.neural.sample_file);
    std::string line;bool checked_features=false;
    while(std::getline(feature_file,line)) {
      const auto row=rmtc::Json::parse(line);
      if(row.at("tick")!=101)continue;
      for(const auto& sample:row.at("samples")) {
        const auto& x=sample.at("features");
        require(x.size()==64,"expanded input has wrong size");
        require(std::abs(x[33].get<double>()-.8)<1e-6&&std::abs(x[45].get<double>()-.2)<1e-6,
                "observed enemy health/loss was not encoded");
        require(std::abs(x[46].get<double>()+.1)<1e-6,"own hp change was not encoded");
        if(sample.at("action")=="attack"&&sample.at("direction")=="down") {
          require(x[37]==1&&std::abs(x[39].get<double>()-.8)<1e-6,
                  "attack direction did not identify its target");
          checked_features=true;
        }
      }
    }
    require(checked_features,"expanded feature collection did not complete");
    auto& values = zero.parameters();
    std::fill(values.begin(), values.end(), 0);
    values[27] = 1;
    const int w2=zero.input_size()*zero.hidden1_size()+zero.hidden1_size();
    const int w3=w2+zero.hidden1_size()*zero.hidden2_size()+zero.hidden2_size();
    values[w2] = 1;
    values[w3] = 50;
    const auto aggressive = (dir / "aggressive.json").string();
    zero.save(aggressive);
    n.neural.model_file = aggressive;
    n.neural.weight = 2;
    n.neural.max_correction = 3;
    rmtc::Stage2Strategy safe(n);
    const auto action = safe.decide(perception(120, 15));
    require(action.type == "defend",
            "neural correction overrode a surviving defense with a lethal action");
    std::filesystem::remove_all(dir);
    std::cout << "neural off/zero/teacher equivalence and lethal bound passed\n";
    return 0;
  } catch (const std::exception& e) {
    std::filesystem::remove_all(dir);
    std::cerr << e.what() << '\n';
    return 1;
  }
}
