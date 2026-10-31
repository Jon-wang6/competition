/** @file stage2_neural_test.cpp
 * @brief 校验MLP前向/文件、反向梯度有限差分、真实训练收敛和默认关闭行为。
 */
#include "../tools/neural_training.hpp"
#include <filesystem>
#include <iostream>
#include <numeric>
#include <stdexcept>

/// Release构建也执行的数值/业务断言，失败打印可定位原因。
void require(bool value, const std::string& reason) {
  if (!value)
    throw std::runtime_error(reason);
}

/** @brief 验证所有参数分组的解析梯度，随后学习独立的非线性样本关系。 */
int main() {
  try {
    rmtc::TinyNeuralNetwork model(7);
    // 同为64输入也不能互换战斗/采集模型，避免加载成功却解释成另一组特征。
    const auto collection_path=std::filesystem::temp_directory_path()/"rmtc-collection-domain-test.json";
    rmtc::TinyNeuralNetwork collector(7,32,16,64,"collection");collector.save(collection_path.string());
    rmtc::TinyNeuralNetwork checked;bool rejected=false;
    try{checked.load(collection_path.string());}catch(const std::exception&){rejected=true;}
    require(rejected,"combat inference accepted collection feature order");
    checked.load(collection_path.string(),"collection");
    require(checked.predict({})==0,"zero collection network changed after reload");
    std::filesystem::remove(collection_path);
    rmtc::NeuralFeatures x{};
    x[0] = .5F;
    require(model.predict(x) == 0, "new network must have exactly zero correction");
    auto& weights = model.parameters();
    for (auto& w : weights)
      w = .01F;
    std::vector<rmtc::training::Sample> samples(2);
    samples[0].features.fill(.2F);
    samples[0].target = .7F;
    samples[1].features.fill(.1F);
    samples[1].target = -.1F;
    std::vector<std::size_t> order{0, 1};
    const auto g = rmtc::training::gradient(model, samples, order, 0, 2, 0);
    const int b1=model.input_size()*model.hidden1_size(),w2=b1+model.hidden1_size(),
      b2=w2+model.hidden1_size()*model.hidden2_size(),w3=b2+model.hidden2_size();
    for (int index : {0, 63, b1, w2, b2, w3, int(weights.size()-1)}) {
      const float saved = weights[index], eps = .001F;
      weights[index] = saved + eps;
      const double plus = rmtc::training::gradient(model, samples, order, 0, 2, 0).loss;
      weights[index] = saved - eps;
      const double minus = rmtc::training::gradient(model, samples, order, 0, 2, 0).loss;
      weights[index] = saved;
      require(std::abs((plus - minus) / (2 * eps) - g.values[index]) < .002,
              "backprop disagrees with finite difference");
    }
    const std::vector<rmtc::training::Pair> pairs{{0, 1}};
    const std::vector<std::size_t> pair_order{0};
    const auto pg = rmtc::training::pairwise_gradient(model, samples, pairs, pair_order, 0, 1);
    for (int index : {0, b1, w2, b2, w3, int(weights.size()-1)}) {
      const float saved = weights[index], eps = .001F;
      weights[index] = saved + eps;
      const double plus =
          rmtc::training::pairwise_gradient(model, samples, pairs, pair_order, 0, 1).loss;
      weights[index] = saved - eps;
      const double minus =
          rmtc::training::pairwise_gradient(model, samples, pairs, pair_order, 0, 1).loss;
      weights[index] = saved;
      require(std::abs((plus - minus) / (2 * eps) - pg.values[index]) < .002,
              "pairwise gradient disagrees with finite difference");
    }
    model = rmtc::TinyNeuralNetwork(12);
    std::fill(model.parameters().begin(),model.parameters().end(),.01F);
    const std::vector<rmtc::training::ActionGroup> action_groups{{0,1}};
    const auto lg=rmtc::training::listwise_gradient(model,samples,action_groups,pair_order,0,1,.5F,.1F);
    auto& list_weights=model.parameters();
    for(int index:{0,b1,w2,b2,w3,int(list_weights.size()-1)}) {
      const float saved=list_weights[index],eps=.001F;
      list_weights[index]=saved+eps;
      const auto plus=rmtc::training::listwise_gradient(model,samples,action_groups,pair_order,0,1,.5F,.1F).loss;
      list_weights[index]=saved-eps;
      const auto minus=rmtc::training::listwise_gradient(model,samples,action_groups,pair_order,0,1,.5F,.1F).loss;
      list_weights[index]=saved;
      require(std::abs((plus-minus)/(2*eps)-lg.values[index])<.002,"listwise gradient differs from finite difference");
    }
    model=rmtc::TinyNeuralNetwork(12);
    samples.clear();
    // alpha=0必须保留部署后的旧策略；使用各不相同的输出避免公共偏移掩盖尺度错误。
    std::fill(model.parameters().begin(),model.parameters().end(),.01F);
    for(int i=0;i<3;++i) {
      rmtc::training::Sample s;s.features.fill(.1F*(i+1));
      s.target=rmtc::training::blend_training_target(.9F,model.predict(s.features),0,.5F,true);
      samples.push_back(s);
    }
    const std::vector<rmtc::training::ActionGroup> keep_groups{{0,1,2}};
    const auto keep=rmtc::training::listwise_gradient(model,samples,keep_groups,pair_order,0,1,.5F,.1F);
    for(std::size_t i=0;i<keep.values.size();++i)
      require(std::abs(keep.values[i]-.0001F*model.parameters()[i])<1e-6,
              "keeping old policy generated a data gradient beyond regularization");
    require(rmtc::training::blend_training_target(0,4,0,.5F,true)==.5F,
            "old deployment label did not apply clipping and weight");
    require(rmtc::training::blend_training_target(0,4,0,.5F,false)==4,
            "raw regression target was accidentally rescaled");
    require(rmtc::training::blend_training_target(0,4,0,.5F,true,.2F)==.1F,
            "custom deployment clip was ignored");
    samples[2].safety_ok=false;
    const auto masked=rmtc::training::listwise_gradient(model,samples,keep_groups,pair_order,0,1,.5F,.1F);
    samples[2].target=1000;
    const auto masked_again=rmtc::training::listwise_gradient(model,samples,keep_groups,pair_order,0,1,.5F,.1F);
    for(std::size_t i=0;i<masked.values.size();++i)
      require(masked.values[i]==masked_again.values[i],"unsafe teacher label affected training");
    samples.clear();model=rmtc::TinyNeuralNetwork(12);
    for (int i = 0; i < 96; ++i) {
      rmtc::training::Sample s;
      s.features[0] = (i % 12 - 6) / 6.F;
      s.features[1] = (i / 12 - 4) / 4.F;
      s.target = s.features[0] * s.features[1];
      samples.push_back(s);
    }
    order.resize(samples.size());
    std::iota(order.begin(), order.end(), 0);
    rmtc::training::Adam adam;
    const double before =
        rmtc::training::gradient(model, samples, order, 0, samples.size(), 0).loss;
    for (int step = 0; step < 400; ++step)
      adam.update(model, rmtc::training::gradient(model, samples, order, 0, samples.size(), 0),
                  .003F);
    const double after = rmtc::training::gradient(model, samples, order, 0, samples.size(), 0).loss;
    require(after < before * .05, "MLP failed to learn a nonlinear interaction");
    const auto path = std::filesystem::temp_directory_path() / "rmtc-neural-roundtrip-test.json";
    model.save(path.string());
    rmtc::TinyNeuralNetwork loaded;
    loaded.load(path.string());
    std::filesystem::remove(path);
    require(std::abs(model.predict(x) - loaded.predict(x)) < 1e-7,
            "serialized model changed inference");
    rmtc::TinyNeuralNetwork legacy(5,64,32,32);
    legacy.parameters().back()=.125F;
    legacy.save(path.string());loaded.load(path.string());std::filesystem::remove(path);
    require(loaded.input_size()==32&&loaded.parameters().size()==4225&&loaded.predict(x)==.125F,
            "legacy model changed architecture or inference");
    auto altered=x;altered[63]=.75F;
    require(loaded.predict(altered)==loaded.predict(x),"legacy model consumed new features");
    for(std::size_t i=0;i<legacy.parameters().size();++i)
      legacy.parameters()[i]=.03F*std::sin(float(i)*.17F);
    rmtc::TinyNeuralNetwork widened(19);widened.initialize_from(legacy);
    for(int row=0;row<20;++row) {
      for(int col=0;col<rmtc::kNeuralInputs;++col)altered[col]=std::sin(float(row*3+col));
      require(std::abs(widened.predict(altered)-legacy.predict(altered))<1e-7,
              "widening changed the existing policy before training");
    }
    std::cout << "neural gradients, nonlinear learning and model roundtrip passed; mse " << before
              << " -> " << after << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
