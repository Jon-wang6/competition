/**
 * @file train_stage2_neural.cpp
 * @brief 纯C++反向传播训练；按整张地图划分训练/验证，保存模型和逐轮真实指标。
 * 训练标签是更深公开信息推演的评分差，不是真实最优解或正式比赛分数。
 *
 * @details 模块协作与维护约定
 * 输入采样中的同局面候选组，按地图分割训练/验证；标签是教师差值，不是全局最优真值。
 * 训练损失下降不证明闭环提分，保存模型时应同时保留配置、基线签名和独立对局结果。
 */
#include "neural_training.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <random>
#include <set>

namespace {
using Json = nlohmann::json;
using Path = std::filesystem::path;
using namespace rmtc::training;
int split_modulus=5;
Json collection_baseline=Json::object();
double important_teacher_gap=.02;
/// 根据地图种子分组，同一图的四种出生和所有Tick只能进入同一数据分区。
bool validation_group(const std::string& group) {
  if (group.empty())
    throw std::runtime_error("sample missing map group");
  return std::stoll(group) % split_modulus == 0;
}
/// 读取C++采样文件，保留公开特征和教师残差，校验维度、有限值及取值范围。
void load(const Path& root, std::vector<Sample>& train, std::vector<Sample>& validation,
          std::set<std::string>& groups,const std::string& schema,double max_correction,Json& sources) {
  std::vector<Path> files;
  for (const auto& entry : std::filesystem::recursive_directory_iterator(root))
    if (entry.path().filename() == "teacher.jsonl" &&
        std::filesystem::exists(entry.path().parent_path() / "result.json"))
      files.push_back(entry.path());
  std::sort(files.begin(), files.end());
  for (const auto& path : files) {
    std::ifstream stream(path);
    std::string line;std::uint64_t fingerprint=1469598103934665603ULL;
    while (std::getline(stream, line)) {
      for(unsigned char c:line){fingerprint^=c;fingerprint*=1099511628211ULL;}
      if (line.empty())
        continue;
      const auto row = Json::parse(line);
      if(row.value("schema","combat-v1")!=schema)
        throw std::runtime_error("neural dataset baseline schema mismatch: "+path.string());
      if(schema=="planning-v3-collection") {
        if(!row.contains("collection_baseline"))throw std::runtime_error("collection baseline metadata missing");
        if(collection_baseline.empty())collection_baseline=row.at("collection_baseline");
        else if(collection_baseline!=row.at("collection_baseline"))throw std::runtime_error("mixed collection planning configurations");
      }
      if(std::abs(row.value("policy_clip",1.0)-max_correction)>1e-9)
        throw std::runtime_error("neural dataset deployment clipping mismatch");
      const auto group = row.at("group").get<std::string>();
      groups.insert(group);
      const auto state =
          row.at("session_id").get<std::string>() + ":" + std::to_string(row.at("tick").get<int>());
      double best_base = -1e30, best_teacher = -1e30, baseline_teacher = 0;
      for (const auto& value : row.at("samples")) {
        if(!value.value("safety_ok",true)&&!value.value("selected",false))continue;
        const double b = value.at("baseline"), t = b + value.at("target").get<double>();
        best_teacher = std::max(best_teacher, t);
        if (b > best_base) {
          best_base = b;
          baseline_teacher = t;
        }
      }
      for (const auto& value : row.at("samples")) {
        if(!value.value("safety_ok",true)&&!value.value("selected",false))continue;
        Sample s;
        const auto features=value.at("features").get<std::vector<float>>();
        if(features.size()!=32&&features.size()!=rmtc::kNeuralInputs)
          throw std::runtime_error("incompatible sample feature count");
        if(schema=="planning-v3-collection"&&features.size()!=rmtc::kNeuralInputs)
          throw std::runtime_error("collection samples require complete 64-feature schema");
        std::copy(features.begin(),features.end(),s.features.begin());
        s.target = value.at("target");
        s.baseline = value.at("baseline");
        s.state = state;
        s.group = group;
        s.selected=value.value("selected",false);
        s.safety_ok=value.value("safety_ok",true);
        s.importance = (s.baseline >= best_base - .5 ? 1.F : .2F) *
                       (best_teacher - baseline_teacher > important_teacher_gap ? 5.F : 1.F);
        for (float v : s.features)
          if (!std::isfinite(v) || std::abs(v) > 1.001)
            throw std::runtime_error("invalid normalized neural feature");
        if (!std::isfinite(s.target) || !std::isfinite(s.baseline))
          throw std::runtime_error("nonfinite sample label");
        (validation_group(group) ? validation : train).push_back(std::move(s));
      }
    }
    sources.push_back({{"path",path.string()},{"fnv1a64",std::to_string(fingerprint)}});
  }
}
/** @brief 测量未训练/已训练网络的误差及教师动作一致率；不把离线准确率称为胜率。 */
Json evaluate(const rmtc::TinyNeuralNetwork& model, const std::vector<Sample>& data,
              double policy_weight=.5,double policy_margin=.003,double max_correction=1.0) {
  double loss = 0, zero = 0, regret = 0;
  int states = 0, correct = 0, baseline_correct = 0;
  struct Choice {
    float teacher = -1e30F, base = -1e30F, pred = -1e30F;
    float chosen_teacher = 0, base_teacher = 0;
  };
  std::map<std::string, Choice> choices;
  std::map<std::string,std::vector<std::pair<const Sample*,float>>> policy_states;
  for (const auto& s : data) {
    const float p = model.predict(s.features), t = s.baseline + s.target;
    loss += (p - s.target) * (p - s.target);
    zero += s.target * s.target;
    auto& c = choices[s.state];
    policy_states[s.state].push_back({&s,p});
    if(s.safety_ok||s.selected)c.teacher = std::max(c.teacher, t);
    if (s.baseline > c.base) {
      c.base = s.baseline;
      c.base_teacher = t;
    }
    if ((s.safety_ok||s.selected) && s.baseline + p > c.pred) {
      c.pred = s.baseline + p;
      c.chosen_teacher = t;
    }
  }
  for (const auto& [key, c] : choices) {
    (void)key;
    ++states;
    correct += c.chosen_teacher >= c.teacher - .001F;
    baseline_correct += c.base_teacher >= c.teacher - .001F;
    regret += c.teacher - c.chosen_teacher;
  }
  double policy_regret=0,original_regret=0;int policy_correct=0,policy_changes=0,original_correct=0;
  for(const auto& [key,rows]:policy_states) {
    (void)key;std::size_t original=0;
    bool selected=false;
    for(std::size_t i=0;i<rows.size();++i) {
      if(rows[i].first->selected){original=i;selected=true;break;}
      if(rows[i].first->baseline>rows[original].first->baseline)original=i;
    }
    (void)selected;
    const auto correction=[&](std::size_t i){return policy_weight*std::clamp<double>(rows[i].second,-max_correction,max_correction);};
    const double initial=correction(original);
    double best_value=rows[original].first->baseline+initial+policy_margin,teacher=-1e30;
    std::size_t chosen=original;
    for(std::size_t i=0;i<rows.size();++i) {
      const auto& s=*rows[i].first;if(s.safety_ok||s.selected)teacher=std::max(teacher,double(s.baseline+s.target));
      const double delta=correction(i),value=s.baseline+delta;
      if(s.safety_ok&&delta>initial+policy_margin&&value>best_value+1e-9) {
        best_value=value;chosen=i;
      }
    }
    const auto& c=*rows[chosen].first;const auto& o=*rows[original].first;
    policy_regret+=teacher-c.baseline-c.target;original_regret+=teacher-o.baseline-o.target;
    policy_correct+=c.baseline+c.target>=teacher-.001;
    original_correct+=o.baseline+o.target>=teacher-.001;policy_changes+=chosen!=original;
  }
  return {{"samples", data.size()},
          {"states", states},
          {"mse", loss / data.size()},
          {"zero_network_mse", zero / data.size()},
          {"teacher_action_agreement", double(correct) / states},
          {"baseline_teacher_agreement", double(baseline_correct) / states},
          {"teacher_regret", regret / states},
          {"policy_teacher_regret",policy_regret/states},{"original_policy_regret",original_regret/states},
          {"policy_teacher_agreement",double(policy_correct)/states},
          {"original_policy_agreement",double(original_correct)/states},
          {"policy_changes",policy_changes},{"policy_weight",policy_weight},{"policy_margin",policy_margin}};
}
}  // namespace

/** @brief 加载样本、执行小批次Adam、每轮输出验证变化，保留最佳和各5轮检查点。 */
int main(int argc, char** argv) {
  try {
    std::map<std::string, std::string> args;
    for (int i = 1; i + 1 < argc; i += 2)
      args[argv[i]] = argv[i + 1];
    if (args.count("--zero-output")) {
      rmtc::TinyNeuralNetwork{}.save(args["--zero-output"]);
      return 0;
    }
    const Path input = args.at("--data"), output = args.at("--output");
    const int epochs = args.count("--epochs") ? std::stoi(args["--epochs"]) : 30;
    const int seed = args.count("--seed") ? std::stoi(args["--seed"]) : 42,
              batch = args.count("--batch") ? std::stoi(args["--batch"]) : 256;
    const float rate = args.count("--rate") ? std::stof(args["--rate"]) : .001F;
    const double policy_weight=args.count("--policy-weight")?std::stod(args["--policy-weight"]):.5;
    const double policy_margin=args.count("--policy-margin")?std::stod(args["--policy-margin"]):.003;
    const float anchor=args.count("--anchor")?std::stof(args["--anchor"]):.15F;
    const int hidden1=args.count("--hidden1")?std::stoi(args["--hidden1"]):rmtc::kNeuralHidden1;
    const int hidden2=args.count("--hidden2")?std::stoi(args["--hidden2"]):rmtc::kNeuralHidden2;
    if(policy_weight<0||policy_weight>2||policy_margin<0||policy_margin>1||anchor<0||anchor>1)
      throw std::runtime_error("invalid runtime policy calibration");
    const bool pairwise = args.count("--pairwise") && args["--pairwise"] == "true";
    const bool listwise=args.count("--listwise")&&args["--listwise"]=="true";
    const float temperature=args.count("--temperature")?std::stof(args["--temperature"]):.05F;
    if((pairwise&&listwise)||temperature<.005||temperature>1)
      throw std::runtime_error("invalid listwise configuration");
    const bool invariant =
        args.count("--rotation-invariant") && args["--rotation-invariant"] == "true";
    if (epochs < 1 || epochs > 1000 || batch < 2 || batch > 4096 || rate <= 0 || rate > .1)
      throw std::runtime_error("invalid training limits");
    std::filesystem::create_directories(output.parent_path());
    Eigen::setNbThreads(1);
    std::vector<Sample> train, validation;
    std::set<std::string> groups;
    const double max_correction=args.count("--max-correction")?std::stod(args["--max-correction"]):1.0;
    if(!std::isfinite(max_correction)||max_correction<=0||max_correction>3)
      throw std::runtime_error("invalid neural deployment clipping");
    const std::string schema=args.count("--schema")?args.at("--schema"):"combat-v1";
    important_teacher_gap=args.count("--important-teacher-gap")?std::stod(args.at("--important-teacher-gap")):.02;
    if(!std::isfinite(important_teacher_gap)||important_teacher_gap<0||important_teacher_gap>1)
      throw std::runtime_error("invalid important teacher gap");
    split_modulus=args.count("--validation-modulus")?std::stoi(args.at("--validation-modulus")):5;
    if(split_modulus<3||split_modulus>20)throw std::runtime_error("invalid map split modulus");
    Json dataset_sources=Json::array();
    load(input, train, validation, groups,schema,max_correction,dataset_sources);
    if (train.size() < 100 || validation.size() < 50)
      throw std::runtime_error("insufficient independent training/validation samples");
    const std::string domain=schema=="planning-v3-collection"?"collection":"combat";
    if(domain=="collection"&&invariant)throw std::runtime_error("combat rotation mask cannot be applied to collection features");
    rmtc::TinyNeuralNetwork model(seed,hidden1,hidden2,rmtc::kNeuralInputs,domain);
    Adam adam;
    const float target_blend=args.count("--target-blend")?std::stof(args["--target-blend"]):1.F;
    if(target_blend<0||target_blend>1)throw std::runtime_error("invalid target blend");
    if(args.count("--initialize")) {
      rmtc::TinyNeuralNetwork previous;previous.load(args.at("--initialize"),domain);
      model.initialize_from(previous);
      // 仅训练标签采用温和更新；验证仍使用原始教师标签，不能改标签制造指标提升。

    }
    if (args.count("--resume")) {
      model.load(args["--resume"],domain);
      if(domain=="collection") {
        std::ifstream stream(args["--resume"]);Json saved;stream>>saved;
        if(saved.at("training").value("collection_baseline",Json::object())!=collection_baseline)
          throw std::runtime_error("resume collection baseline differs; use explicit initialization for transfer learning");
      }
      const auto state = Path(args["--resume"] + ".optimizer.json");
      if (std::filesystem::exists(state)) {
        std::ifstream s(state);
        Json j;
        s >> j;
        adam.first = j.at("first").get<std::vector<float>>();
        adam.second = j.at("second").get<std::vector<float>>();
        adam.steps = j.at("steps");
        if (adam.first.size() != model.parameters().size() ||
            adam.second.size() != model.parameters().size())
          throw std::runtime_error("incompatible optimizer");
      }
    }
    // 使用同一独立验证分区校准部署权重；不读取闭环测试地图的结果来训练。
    if(args.count("--evaluate")) {
      model.load(args.at("--evaluate"),domain);
      if(domain=="collection") {
        std::ifstream stream(args.at("--evaluate"));Json saved;stream>>saved;
        if(saved.at("training").value("collection_baseline",Json::object())!=collection_baseline)
          throw std::runtime_error("evaluation collection baseline differs from dataset");
      }
      Json calibration=Json::array();
      for(double weight:{.125,.25,.5,.75,1.0})
        for(double margin:{.003,.015})
          calibration.push_back(evaluate(model,validation,weight,margin,max_correction));
      std::ofstream stream(output);stream<<calibration.dump(2)<<'\n';
      std::cout<<calibration.dump()<<std::endl;return 0;
    }

    if(target_blend<1) {
      if(!args.count("--initialize")&&!args.count("--resume"))
        throw std::runtime_error("blending requires an initialized or resumed model");
      for(auto& sample:train)sample.target=blend_training_target(sample.target,model.predict(sample.features),
          target_blend,static_cast<float>(policy_weight),listwise,static_cast<float>(max_correction));
    }
    std::vector<Pair> pairs;
    std::vector<ActionGroup> action_groups;
    if(listwise) {
      std::map<std::string,ActionGroup> states;
      for(std::size_t i=0;i<train.size();++i)states[train[i].state].push_back(i);
      for(auto& [key,group]:states){(void)key;action_groups.push_back(std::move(group));}
    }
    if (pairwise) {
      std::map<std::string, std::vector<std::size_t>> states;
      for (std::size_t i = 0; i < train.size(); ++i)
        if(train[i].safety_ok||train[i].selected)states[train[i].state].push_back(i);
      for (const auto& [key, indices] : states) {
        (void)key;
        auto good = indices.front();
        float best_base = -1e30;
        for (auto i : indices) {
          best_base = std::max(best_base, train[i].baseline);
          if (train[i].baseline + train[i].target > train[good].baseline + train[good].target)
            good = i;
        }
        for (auto i : indices)
          if (i != good && train[i].baseline >= best_base - .5)
            pairs.push_back({good, i});
      }
    }
    if (invariant)
      for (int row = 0; row < model.hidden1_size(); ++row)
        for (int col : {29, 30})
          model.parameters()[row * model.input_size() + col] = 0;
    std::vector<std::size_t> order(listwise?action_groups.size():pairwise ? pairs.size() : train.size());
    std::iota(order.begin(), order.end(), 0);
    std::mt19937 rng(seed);
    Json history = Json::array();
    const auto before = evaluate(model, validation,policy_weight,policy_margin,max_correction);
    // 把第0轮也纳入选择；继续训练变差时必须保留训练前模型。
    double best=before.at("mse"),best_policy=before.at("policy_teacher_regret");
    const Json initial_metadata{{"epoch",0},{"seed",seed},{"map_groups",groups},
      {"schema",schema},{"label_version",2},{"max_correction",max_correction},
      {"collection_baseline",collection_baseline},
      {"important_teacher_gap",important_teacher_gap},
      {"train_samples",train.size()},{"validation_samples",validation.size()},
      {"initialized_from",args.count("--initialize")?args.at("--initialize"):""},
      {"training_target_blend",target_blend},{"metrics",{{"validation",before}}},
      {"official_ai_qualification",false}};
    if(adam.first.empty()){adam.first.assign(model.parameters().size(),0);adam.second=adam.first;}
    for(const auto& path:{output.string(),output.string()+".policy.json"}) {
      model.save(path,initial_metadata);
      std::ofstream state(path+".optimizer.json");
      state<<Json{{"steps",adam.steps},{"first",adam.first},{"second",adam.second}}.dump();
    }
    const auto began = std::chrono::steady_clock::now();
    std::cout << Json{{"epoch", 0},
                      {"train_samples", train.size()},
                      {"validation", before},
                      {"map_groups", groups.size()},
                      {"parameter_count", model.parameters().size()}}
                     .dump()
              << std::endl;
    for (int epoch = 1; epoch <= epochs; ++epoch) {
      std::shuffle(order.begin(), order.end(), rng);
      for (std::size_t offset = 0; offset < order.size(); offset += batch) {
        const auto count = std::min<std::size_t>(batch, order.size() - offset);
        auto g = listwise?listwise_gradient(model,train,action_groups,order,offset,count,policy_weight,temperature,static_cast<float>(max_correction)):
                 pairwise ? pairwise_gradient(model, train, pairs, order, offset, count)
                          : gradient(model, train, order, offset, count);
        if(pairwise&&anchor>0) {
          std::vector<std::size_t> point_order;point_order.reserve(count*2);
          for(std::size_t i=0;i<count;++i) {
            const auto pair=pairs.at(order.at(offset+i));
            point_order.push_back(pair.first);point_order.push_back(pair.second);
          }
          const auto anchored=gradient(model,train,point_order,0,point_order.size());
          g.loss+=anchor*anchored.loss;
          for(std::size_t i=0;i<g.values.size();++i)g.values[i]+=anchor*anchored.values[i];
        }
        if (invariant)
          for (int row = 0; row < model.hidden1_size(); ++row)
            for (int col : {29, 30})
              g.values[row * model.input_size() + col] = 0;
        adam.update(model, g, rate);
        if (invariant)
          for (int row = 0; row < model.hidden1_size(); ++row)
            for (int col : {29, 30})
              model.parameters()[row * model.input_size() + col] = 0;
      }
      const auto checked = evaluate(model, validation,policy_weight,policy_margin,max_correction);
      Json row{{"epoch", epoch},
               {"validation", checked},
               {"train", evaluate(model, train,policy_weight,policy_margin,max_correction)},
               {"seconds",
                std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count()}};
      history.push_back(row);
      std::cout << row.dump() << std::endl;
      const Json metadata{
          {"method", listwise?"listwise teacher policy distribution; runtime-scaled clipped residual; Adam":
                     pairwise ? "pairwise deeper-rollout residual; Adam backpropagation"
                              : "supervised deeper-rollout residual; Adam backpropagation"},
          {"dataset_sources",dataset_sources},{"schema",schema},{"label_version",2},{"max_correction",max_correction},
          {"collection_baseline",collection_baseline},
          {"important_teacher_gap",important_teacher_gap},
          {"resumed_from",args.count("--resume")?args.at("--resume"):""},
          {"rotation_invariant", invariant},
          {"pointwise_anchor",anchor},{"policy_weight",policy_weight},{"policy_margin",policy_margin},
          {"initialized_from",args.count("--initialize")?args.at("--initialize"):""},
          {"training_target_blend",target_blend},
          {"listwise_temperature",temperature},{"listwise_action_groups",action_groups.size()},
          {"training_pairs", pairs.size()},
          {"seed", seed},
          {"epoch", epoch},
          {"dataset", std::filesystem::absolute(input).string()},
          {"map_groups", groups},
          {"train_samples", train.size()},
          {"validation_samples", validation.size()},
          {"split", "map seed modulo "+std::to_string(split_modulus)+"; remainder 0 held out"},
          {"metrics", row},
          {"official_ai_qualification", false}};
      if (checked.at("mse").get<double>() < best) {
        best = checked.at("mse");
        model.save(output.string(), metadata);
        std::ofstream state(output.string() + ".optimizer.json");
        state << Json{{"steps", adam.steps}, {"first", adam.first}, {"second", adam.second}}.dump();
      }
      if (checked.at("policy_teacher_regret").get<double>() < best_policy) {
        best_policy = checked.at("policy_teacher_regret");
        model.save(output.string() + ".policy.json", metadata);
        std::ofstream state(output.string() + ".policy.json.optimizer.json");
        state << Json{{"steps", adam.steps}, {"first", adam.first}, {"second", adam.second}}.dump();
      }
      if (epoch % 5 == 0)
        model.save(output.string() + ".epoch-" + std::to_string(epoch) + ".json", metadata);
      std::ofstream report(output.string() + ".training.json");
      report << Json{{"before", before}, {"epochs", history}, {"best_validation_mse", best}}.dump(2)
             << '\n';
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
