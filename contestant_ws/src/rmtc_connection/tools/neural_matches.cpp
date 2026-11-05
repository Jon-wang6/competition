/**
 * @file neural_matches.cpp
 * @brief C++离线闭环对局编排：随机出生、冻结对手、逐Tick验收与教师数据采集。
 *
 * 只启动隔离训练域200..230，端口23000起；控制算法仍使用公开感知。
 * 原裁判结算完整500Tick，缓存仅解释事件，不提供真实AI资格，不读取任何密钥。
 *
 * @details 模块协作与维护约定
 * 编排器负责隔离对局、采样和结果汇总，不是正式选手的在线决策模块。
 * 复现需要共同地图/出生/对手设置；模型解释缓存只能用于实验，不代表真实网关资格。
 */
#include "rmtc_connection/map_generator.hpp"
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <set>
#include <spawn.h>
#include <stdexcept>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <fcntl.h>
#include <sstream>
extern char** environ;
namespace {
using Json = nlohmann::json;
using Path = std::filesystem::path;
using Clock = std::chrono::steady_clock;
const Path root = "/home/jon/robot/competition";
std::mutex output_mutex;
bool compress_logs=false;
std::chrono::system_clock::time_point stop_at=std::chrono::system_clock::time_point::max();
/// 显式批次截止时间；到点停止接新局，并让当前局走原有子进程清理路径。
bool deadline_reached(){return std::chrono::system_clock::now()>=stop_at;}

/** @brief 对实际文件内容做缓存指纹；按路径/大小/修改时间复用，不用于安全认证。 */
std::string fingerprint(const Path& path) {
  static std::mutex mutex;
  static std::map<std::string,std::pair<std::string,std::string>> cache;
  const auto key=std::filesystem::absolute(path).string();
  const auto stamp=std::to_string(std::filesystem::file_size(path))+":"+
      std::to_string(std::filesystem::last_write_time(path).time_since_epoch().count());
  std::lock_guard<std::mutex> lock(mutex);
  if(cache.count(key)&&cache.at(key).first==stamp)return cache.at(key).second;
  std::ifstream stream(path,std::ios::binary);if(!stream)throw std::runtime_error("cannot fingerprint "+key);
  std::uint64_t hash=1469598103934665603ULL;char bytes[65536];
  while(stream.read(bytes,sizeof(bytes))||stream.gcount())
    for(std::streamsize i=0;i<stream.gcount();++i){hash^=static_cast<unsigned char>(bytes[i]);hash*=1099511628211ULL;}
  const auto value=std::to_string(hash);cache[key]={stamp,value};return value;
}

/** @brief 绑定参数、地图、控制器、模型、对手及裁判内容，杜绝同目录误读旧结果。 */
struct Job;
Json cache_identity(const Job& job,const Json& pool);

/// 读取本轮明确指定的JSON，不接触密钥文件或私有实时比赛数据。
Json read(const Path& path) {
  std::ifstream s(path);
  if (!s)
    throw std::runtime_error("cannot read " + path.string());
  Json j;
  s >> j;
  return j;
}
/// 将配置/指标完整写入测试目录；父目录由本工具创建。
void save(const Path& path, const Json& value) {
  std::filesystem::create_directories(path.parent_path());
  const auto temporary = Path(path.string() + ".tmp-" + std::to_string(getpid()));
  std::ofstream s(temporary);
  s.exceptions(std::ios::failbit | std::ios::badbit);
  s << value.dump(2) << '\n';
  s.close();
  std::filesystem::rename(temporary, path);
}
/// 将独立argv交给posix_spawn，设置进程组和日志；析构时清理本工具创建的子进程。
class Process {
 public:
  Process(std::vector<std::string> args, const Path& log,
          const std::map<std::string, std::string>& env, const Path& input = {}) {
    std::vector<std::string> values;
    for (const auto& [k, v] : env)
      values.push_back(k + "=" + v);
    std::vector<char*> argv, envp;
    for (auto& a : args)
      argv.push_back(a.data());
    argv.push_back(nullptr);
    for (auto& e : values)
      envp.push_back(e.data());
    envp.push_back(nullptr);
    posix_spawn_file_actions_t files;
    posix_spawn_file_actions_init(&files);
    posix_spawn_file_actions_addopen(&files, STDOUT_FILENO, log.c_str(),
                                     O_WRONLY | O_CREAT | O_TRUNC, 0600);
    posix_spawn_file_actions_adddup2(&files, STDOUT_FILENO, STDERR_FILENO);
    if (!input.empty())
      posix_spawn_file_actions_addopen(&files, STDIN_FILENO, input.c_str(), O_RDONLY, 0);
    posix_spawnattr_t attr;
    posix_spawnattr_init(&attr);
    posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETPGROUP);
    posix_spawnattr_setpgroup(&attr, 0);
    const int error = posix_spawnp(&pid_, argv.front(), &files, &attr, argv.data(), envp.data());
    posix_spawnattr_destroy(&attr);
    posix_spawn_file_actions_destroy(&files);
    if (error)
      throw std::runtime_error("spawn failed: " + args.front() + " error=" + std::to_string(error));
  }
  Process(const Process&) = delete;
  Process& operator=(const Process&) = delete;
  ~Process() {
    stop();
  }
  /// 非阻塞检查退出状态，运行中返回-1；正常退出保留退出码。
  int poll() {
    if (done_)
      return code_;
    int status = 0;
    if (waitpid(pid_, &status, WNOHANG) == pid_) {
      done_ = true;
      code_ = WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status);
    }
    return done_ ? code_ : -1;
  }
  /// 先SIGINT让ROS关闭并刷新训练文件；超时才终止该进程组，不影响用户程序。
  void stop() {
    if (pid_ <= 0 || poll() != -1)
      return;
    kill(-pid_, SIGINT);
    for (int i = 0; i < 100 && poll() == -1; ++i)
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (poll() == -1) {
      kill(-pid_, SIGKILL);
      int status;
      waitpid(pid_, &status, 0);
      done_ = true;
    }
  }

 private:
  pid_t pid_{-1};
  bool done_{false};
  int code_{-1};
};
/// 生成隔离子进程环境，主动去掉上游模型凭据，禁止离线对局意外触发付费调用。
std::map<std::string, std::string> environment(int domain, const Path& out) {
  std::map<std::string, std::string> env;
  for (char** p = environ; *p; ++p) {
    std::string e = *p;
    const auto split = e.find('=');
    env[e.substr(0, split)] = e.substr(split + 1);
  }
  for (const char* name :
       {"PRACTICE_LLM_KEY", "PRACTICE_LLM_URL", "PRACTICE_MODEL", "DEEPSEEK_API_KEY"})
    env.erase(name);
  env["PRACTICE_REAL_AI_TEAMS"] = "";
  env["ROS_DOMAIN_ID"] = std::to_string(domain);
  env["ROS_LOCALHOST_ONLY"] = "0";
  env["RMTC_TRAINING_GATEWAY"] = "1";
  env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp";
  env["ROS_LOG_DIR"] = (out / "ros").string();
  return env;
}
/// libcurl只访问本机训练端口，响应限制16MiB，避免无界缓存。
std::size_t receive(char* data, std::size_t size, std::size_t n, void* context) {
  auto& text = *static_cast<std::string*>(context);
  if (text.size() + size * n > 16 * 1024 * 1024)
    return 0;
  text.append(data, size * n);
  return size * n;
}
/// 请求本地裁判状态或停止命令；失败由调用方决定是否重试，只读状态可重试。
Json request(int port, bool stop = false) {
  CURL* handle = curl_easy_init();
  if (!handle)
    throw std::runtime_error("curl init failed");
  std::string body;
  const std::string url =
      "http://127.0.0.1:" + std::to_string(port) + "/api/" + (stop ? "stop" : "state");
  curl_easy_setopt(handle, CURLOPT_URL, url.c_str());
  curl_easy_setopt(handle, CURLOPT_PROXY, "");
  curl_easy_setopt(handle, CURLOPT_TIMEOUT_MS, 3000L);
  curl_easy_setopt(handle, CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(handle, CURLOPT_WRITEFUNCTION, receive);
  curl_easy_setopt(handle, CURLOPT_WRITEDATA, &body);
  if (stop) {
    curl_easy_setopt(handle, CURLOPT_POST, 1L);
    curl_easy_setopt(handle, CURLOPT_POSTFIELDS, "{}");
  }
  const auto code = curl_easy_perform(handle);
  long status = 0;
  curl_easy_getinfo(handle, CURLINFO_RESPONSE_CODE, &status);
  curl_easy_cleanup(handle);
  if (code != CURLE_OK || status != 200)
    throw std::runtime_error("local referee transport " + std::to_string(status));
  return Json::parse(body);
}
/// 加入ROS参数，严格以独立argv传递，路径中的空格不会交给shell解释。
void parameters(std::vector<std::string>& args, const Json& params) {
  for (auto it = params.begin(); it != params.end(); ++it) {
    args.push_back("-p");
    args.push_back(it.key() + ":=" + it.value().dump());
  }
}
/// 构造四种出生配置；布局、资源和事件保持相同，密集出生使用四个不同的相邻格。
Json map_for(const Json& base, const std::string& mode, int seed) {
  Json out = base;
  const int n = base.at("map").at("width");
  std::mt19937_64 rng(static_cast<unsigned>(seed) * 31 + 7);
  std::vector<Json> positions;
  for (const auto& r : base.at("robots"))
    positions.push_back(r.at("spawn"));
  if (mode == "corner_shuffle")
    std::shuffle(positions.begin(), positions.end(), rng);
  else if (mode != "fixed") {
    std::set<std::pair<int, int>> blocked;
    for (const auto& p : base.at("map").at("obstacles"))
      blocked.emplace(p[0], p[1]);
    for (const auto& r : base.at("map").at("resources"))
      blocked.emplace(r["position"][0], r["position"][1]);
    const auto split = base.at("events").at("definitions").at("world_split").at("point");
    std::vector<std::pair<int, int>> free;
    for (int y = 0; y < n; ++y)
      for (int x = 0; x < n; ++x)
        if (!blocked.count({x, y}) && x != split[0] && y != split[1])
          free.emplace_back(x, y);
    positions.clear();
    if (mode == "uniform_free") {
      std::shuffle(free.begin(), free.end(), rng);
      for (int i = 0; i < 4; ++i)
        positions.push_back({free[i].first, free[i].second});
    } else {
      const std::set<std::pair<int, int>> valid(free.begin(), free.end());
      std::vector<std::pair<int, int>> anchors;
      for (auto p : free)
        if (valid.count({p.first + 1, p.second}) && valid.count({p.first, p.second + 1}) &&
            valid.count({p.first + 1, p.second + 1}))
          anchors.push_back(p);
      if (anchors.empty())
        throw std::runtime_error("no valid clustered birth");
      const auto p = anchors[rng() % anchors.size()];
      positions = {{p.first, p.second},
                   {p.first + 1, p.second},
                   {p.first, p.second + 1},
                   {p.first + 1, p.second + 1}};
      std::shuffle(positions.begin(), positions.end(), rng);
    }
  }
  for (int i = 0; i < 4; ++i)
    out["robots"][i]["spawn"] = positions[i];
  return out;
}
struct Job {
  int seed;
  std::string spawn, profile;
  Path config;
  Json definition;
};

Json cache_identity(const Job& job,const Json& pool) {
  Json identity{{"schema",2},{"definition",job.definition},{"map",fingerprint(job.config)},
      {"binary",fingerprint(job.definition.at("binary").get<std::string>())},{"pool",pool}};
  const auto add_model=[&](const Json& definition) {
    const auto p=definition.value("parameters",Json::object());
    return p.value("neural_enabled",false)?fingerprint(p.at("neural_model_file").get<std::string>()):std::string{};
  };
  identity["model"]=add_model(job.definition);
  const auto add_collector=[&](const Json& definition) {
    const auto p=definition.value("parameters",Json::object());
    return p.value("collection_neural_enabled",false)?fingerprint(p.at("collection_neural_model_file").get<std::string>()):std::string{};
  };
  identity["collection_model"]=add_collector(job.definition);
  for(auto p=pool.begin();p!=pool.end();++p)
    identity["opponent_content"][p.key()]={{"binary",fingerprint(p.value().at("binary").get<std::string>())},
        {"model",add_model(p.value())},{"collection_model",add_collector(p.value())}};
  for(const auto& relative:{"referee/local_v1.2/rmtc_practice/bin/referee_node",
      "referee/local_v1.2/rmtc_practice/bin/simulator_node","referee/local_v1.2/rmtc_practice/bin/robot_node",
      "install/rmtc_connection/lib/rmtc_connection/practice_web",
      "install/rmtc_connection/lib/rmtc_connection/practice_scoring",
      "runtime/stage2-total-joint-20261030-1503/event-anchors.jsonl",
      "build/rmtc_connection/ai_cache_replay",
      "runtime/stage2-total-joint-20261030-1503/build/lockstep_driver"})
    identity["infrastructure"][relative]=fingerprint(root/relative);
  return identity;
}

/** @brief 仅审计裁判已记录的本队公开感知，确认出生格被占时的就近重生；不提供给策略。 */
bool confirmed_respawn(const Path& out,const Json& session,int tick,const Json& position) {
  for(const auto& file:std::filesystem::recursive_directory_iterator(out/"referee")) {
    if(file.path().filename()!="communication.jsonl")continue;
    std::ifstream stream(file.path());std::string line;
    while(std::getline(stream,line)) {
      const auto record=Json::parse(line);
      if(record.value("kind","")!="perception"||record.value("seat","")!="A")continue;
      const auto& p=record.at("data");
      if(p.at("session_id")!=session||p.at("tick")!=tick)continue;
      const auto& self=p.at("self");
      return self.at("position")==position&&self.value("position_reason","")=="respawn"&&
          self.value("alive",false)&&self.value("protected",false)&&self.value("hp",0.)==100;
    }
  }
  return false;
}

/** @brief 汇总真实计分与条件任务分，按地图成组计算配对置信区间，避免同图四出生伪增样本量。 */
Json report(const Path& output) {
  std::vector<Json> rows;
  for (const auto& item : std::filesystem::directory_iterator(output))
    if (item.is_directory() && std::filesystem::exists(item.path() / "result.json"))
      rows.push_back(read(item.path() / "result.json"));
  Json result;
  std::set<std::string> profiles;
  for (const auto& r : rows)
    profiles.insert(r.at("profile").get<std::string>());
  for (const std::string environment : {"all", "official", "mixed"})
    for (const auto& profile : profiles) {
      std::vector<Json> selected;
      for (const auto& r : rows)
        if (r["profile"] == profile &&
            (environment == "all" || r["opponent_environment"] == environment))
          selected.push_back(r);
      if (selected.empty())
        continue;
      Json s{{"n", selected.size()}};
      for (const char* key : {"score45", "conditional_score60", "resource_final", "resource40",
                              "resource80", "combat_score", "resource_score", "raw_tasks",
                              "neural_changed", "neural_inference_mean_ms"}) {
        double total = 0;
        for (const auto& r : selected)
          total += r.at(key).get<double>();
        s[key] = total / selected.size();
      }
      double reach = 0, tasks = 0, time = 0, maximum = 0;
      for (const auto& r : selected) {
        reach += r.at("reach_tick").get<int>() >= 0;
        tasks += r.at("raw_tasks") == 3;
        time += r["decision_ms"]["mean"].get<double>();
        maximum = std::max(maximum, r["decision_ms"]["max"].get<double>());
      }
      s["reach_rate"] = reach / selected.size();
      s["all_tasks_rate"] = tasks / selected.size();
      s["decision_mean_ms"] = time / selected.size();
      s["decision_max_ms"] = maximum;
      if(std::all_of(selected.begin(),selected.end(),[](const auto& r){return r.contains("longest_unproductive_two_cycle");})) {
        double frames=0,longest=0,games=0;
        for(const auto& r:selected) {
          frames+=r.at("two_cycle_frames").template get<double>();
          const auto length=r.at("longest_unproductive_two_cycle").template get<int>();
          longest=std::max(longest,double(length));games+=length>=8;
        }
        s["two_cycle_frames_mean"]=frames/selected.size();s["longest_unproductive_two_cycle"]=longest;
        s["games_with_two_cycle_at_least8_rate"]=games/selected.size();
      }
      for (const std::string mode : {"fixed", "corner_shuffle", "uniform_free", "clustered"}) {
        double score = 0, res = 0;
        int count = 0;
        for (const auto& r : selected)
          if (r["spawn"] == mode) {
            ++count;
            score += r.at("conditional_score60").get<double>();
            res += r.at("resource_final").get<double>();
          }
        if (count)
          s["by_spawn"][mode] = {
              {"n", count}, {"conditional", score / count}, {"resource", res / count}};
      }
      result[environment][profile] = s;
      for(const std::string reference:{"baseline","feedback","neural"})
      if (profiles.count(reference) && profile != reference) {
        std::map<std::string, Json> base;
        for (const auto& r : rows)
          if (r["profile"] == reference)
            base[std::to_string(r.at("seed").get<int>()) + r.at("spawn").get<std::string>()] = r;
        std::map<int, std::vector<double>> differences;
        std::map<int,std::vector<double>> actual_differences;
        double resource_delta = 0, actual_delta = 0;
        int pairs = 0;
        for (const auto& r : selected) {
          const auto k = std::to_string(r.at("seed").get<int>()) + r.at("spawn").get<std::string>();
          if (!base.count(k))
            continue;
          const auto& b = base.at(k);
          if (b.at("opponent_roster") != r.at("opponent_roster"))
            throw std::runtime_error("unpaired opponent roster");
          differences[r.at("seed")].push_back(r.at("conditional_score60").get<double>() -
                                              b.at("conditional_score60").get<double>());
          actual_differences[r.at("seed")].push_back(r.at("score45").get<double>()-b.at("score45").get<double>());
          resource_delta +=
              r.at("resource_final").get<double>() - b.at("resource_final").get<double>();
          actual_delta += r.at("score45").get<double>() - b.at("score45").get<double>();
          ++pairs;
        }
        if (pairs) {
          std::vector<double> delta;
          std::vector<double> actual_map_delta;
          double sum = 0;
          for (const auto& [seed, values] : differences) {
            (void)seed;
            double value = 0;
            for (double v : values)
              value += v;
            value /= values.size();
            delta.push_back(value);
            const auto& a=actual_differences.at(seed);
            double actual=0;for(double v:a)actual+=v;
            actual_map_delta.push_back(actual/a.size());
            sum += value;
          }
          std::mt19937 rng(20261030);
          std::vector<double> boot;
          std::vector<double> actual_boot;
          for (int i = 0; i < 4000; ++i) {
            double mean = 0;
            double actual=0;
            for (std::size_t j = 0; j < delta.size(); ++j) {
              const auto index=rng()%delta.size();mean+=delta[index];actual+=actual_map_delta[index];
            }
            boot.push_back(mean / delta.size());
            actual_boot.push_back(actual/delta.size());
          }
          std::sort(boot.begin(), boot.end());
          std::sort(actual_boot.begin(),actual_boot.end());
          result[environment][profile]["vs_"+reference] = {{"paired_maps", delta.size()},
                                                         {"conditional_delta", sum / delta.size()},
                                                         {"resource_delta", resource_delta / pairs},
                                                         {"actual_delta", actual_delta / pairs},
                                                         {"bootstrap95", {boot[99], boot[3899]}},
                                                         {"actual_bootstrap95",{actual_boot[99],actual_boot[3899]}}};
        }
      }
    }
  save(output / "summary.json", result);
  return result;
}

/// 运行一局完整500Tick，控制器仅接公开域；训练驱动只负责同步原裁判结算。
Json match(const Job& job, int slot, const Path& output, const Json& pool) {
  if(deadline_reached())throw std::runtime_error("batch deadline reached");
  const auto out = output / (job.profile + "-" + job.spawn + "-seed" + std::to_string(job.seed));
  const auto identity=cache_identity(job,pool);
  if (std::filesystem::exists(out / "result.json")) {
    const auto cached=read(out/"result.json");
    if(cached.value("cache_identity",Json::object())==identity)return cached;
  }
  if (std::filesystem::exists(out / "controller.jsonl")||std::filesystem::exists(out/"result.json")) {
    const auto archive =
        output / "failures" /
        (out.filename().string() + "-" +
         std::to_string(std::chrono::system_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(archive.parent_path());
    std::filesystem::rename(out, archive);
  }
  std::filesystem::create_directories(out);
  const int domain = 200 + slot * 2, port = 23000 + slot * 2;
  const auto env = environment(domain, out);
  std::vector<std::unique_ptr<Process>> children;
  const auto start = Clock::now();
  const auto launch = [&](const std::string& label, std::vector<std::string> args) {
    children.push_back(std::make_unique<Process>(std::move(args), out / (label + ".log"), env));
    return children.back().get();
  };
  Json params = job.definition.at("parameters");
  if (params.value("neural_collect", false)) {
    params["neural_sample_file"] = (out / "teacher.jsonl").string();
    params["neural_sample_group"] = std::to_string(job.seed);
  }
  if(params.value("collection_neural_collect",false)) {
    params["collection_neural_sample_file"]=(out/"teacher.jsonl").string();
    params["collection_neural_sample_group"]=std::to_string(job.seed);
  }
  Json roster = Json::object();
  if (job.seed % 2) {
    std::mt19937 rng(job.seed + 20261028);
    std::vector<std::string> names;
    for (auto it = pool.begin(); it != pool.end(); ++it)
      names.push_back(it.key());
    std::shuffle(names.begin(), names.end(), rng);
    std::vector<std::string> teams{"B", "C", "D"};
    std::shuffle(teams.begin(), teams.end(), rng);
    const int count = 1 + rng() % 3;
    for (int i = 0; i < count; ++i)
      roster[teams[i]] = pool.at(names[i]);
  }
  const auto controller = [&](const std::string& team, const std::string& binary, const Json& p) {
    std::vector<std::string> args{
        binary,
        "--ros-args",
        "-p",
        "team:=" + team,
        "-p",
        "mode:=stage2",
        "-p",
        "action_ticks:=500",
        "-p",
        "log_file:=" +
            (out / (team == "A" ? "controller.jsonl" : "opponent-" + team + ".jsonl")).string()};
    parameters(args, p);
    return launch("controller-" + team, std::move(args));
  };
  const auto cache = [&](const std::string& team) {
    return launch(
        "cache-" + team,
        {(root / "build/rmtc_connection/ai_cache_replay").string(), "--ros-args", "-p",
         "team:=" + team, "-p",
         "cache_file:=" +
             (root / "runtime/stage2-total-joint-20261030-1503/event-anchors.jsonl").string(),
         "-p", "report_file:=" + (out / ("cache-" + team + ".json")).string(), "-p",
         "training_gateway_url:=http://127.0.0.1:" + std::to_string(port) +
             "/api/training/event-json"});
  };
  Json state;
  try {
    controller("A", job.definition.at("binary"), params);
    cache("A");
    std::string bots;
    for (const std::string team : {"B", "C", "D"})
      if (roster.contains(team)) {
        controller(team, roster[team]["binary"], roster[team]["parameters"]);
        cache(team);
      } else {
        if (!bots.empty())
          bots += ",";
        bots += team;
      }
    std::vector<std::string> referee_args{"ros2",
                                          "launch",
                                          "rmtc_connection",
                                          "referee.launch.py",
                                          "stage:=2",
                                          "tick_ms:=5000",
                                          "max_ticks:=501",
                                          "ai_mode:=mock",
                                          "port:=" + std::to_string(port),
                                          "domain_id:=" + std::to_string(domain),
                                          "internal_domain_id:=" + std::to_string(domain + 1),
                                          "config_file:=" + job.config.string(),
                                          "data_dir:=" + (out / "referee").string(),
                                          "ros_log_dir:=" + (out / "ros").string()};
    if (!bots.empty())
      referee_args.push_back("bots:=" + bots);
    launch("referee", std::move(referee_args));
    const auto deadline = Clock::now() + std::chrono::seconds(75);
    while (true) {
      if(deadline_reached())throw std::runtime_error("batch deadline reached while preparing");
      for (auto& p : children)
        if (p->poll() != -1)
          throw std::runtime_error("child exited before ready; inspect logs");
      try {
        state = request(port);
        if (state.value("status", Json::object()).value("ready", false) &&
            state.value("board", Json::object()).value("tick", -1) == 0)
          break;
      } catch (const std::exception&) {
      }
      if (Clock::now() > deadline)
        throw std::runtime_error("ready timeout");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::this_thread::sleep_for(std::chrono::seconds(3));
    Process* driver =
        launch("driver",
               {(root / "runtime/stage2-total-joint-20261030-1503/build/lockstep_driver").string(),
                std::to_string(domain), "500", (out / "driver.json").string()});
    const auto end = Clock::now() + std::chrono::seconds(240);
    while (driver->poll() == -1) {
      if(deadline_reached())throw std::runtime_error("batch deadline reached during match");
      if (Clock::now() > end)
        throw std::runtime_error("match timeout");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (driver->poll() != 0)
      throw std::runtime_error("driver failed; inspect driver.log");
    const auto terminal_deadline = Clock::now() + std::chrono::seconds(5);
    do {
      state = request(port);
      if (state.at("board").at("tick") == 500)
        break;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } while (Clock::now() < terminal_deadline);
    if (state.at("board").at("tick") != 500)
      throw std::runtime_error("missing final tick");
    try {
      request(port, true);
    } catch (const std::exception&) {
    }
  } catch (...) {
    try {
      request(port, true);
    } catch (const std::exception&) {
    }
    for (auto it = children.rbegin(); it != children.rend(); ++it)
      (*it)->stop();
    throw;
  }
  for (auto it = children.rbegin(); it != children.rend(); ++it)
    (*it)->stop();
  std::ifstream log(out / "controller.jsonl");
  std::string line;
  Json summary, first, final;
  std::vector<Json> rx, acks;
  int tx = 0, nn_changed = 0, nn_eligible = 0,collection_changed=0,collection_samples=0;
  double inference = 0;
  int inferred = 0;
  while (std::getline(log, line)) {
    if (line.empty())
      continue;
    const auto row = Json::parse(line);
    const auto kind = row.value("event", "");
    if (kind == "rx")
      rx.push_back(row);
    if (kind == "tx")
      ++tx;
    if (kind == "ack")
      acks.push_back(row);
    if (kind == "summary")
      summary = row;
    if (kind == "stage2_plan" && row.contains("neural")) {
      const auto& n = row["neural"];
      nn_changed += n.value("changed", false);
      nn_eligible += n.value("eligible", false);
      if (n.contains("inference_ms")) {
        inference += n["inference_ms"].get<double>();
        ++inferred;
      }
    }
    if(kind=="stage2_plan"&&row.contains("planning")) {
      const auto n=row.at("planning").value("collection_neural",Json::object());
      collection_changed+=n.value("changed",false);collection_samples+=n.value("sample_written",false);
    }
  }
  if (rx.size() != 501 || tx != 500 || acks.size() != 500 || summary.value("parse_errors", 1) != 0)
    throw std::runtime_error("incomplete frames/actions/receipts");
  int replacements = 0, death_canceled_moves = 0;
  const auto deployed = read(job.config);
  if (rx.front().at("position") != deployed.at("robots")[0].at("spawn"))
    throw std::runtime_error("deployed birth differs from paired test configuration");
  const auto cross = deployed.at("events").at("definitions").at("world_split").at("point");
  for (const auto& ack : acks)
    if (!ack.value("matched", false)) {
      const int t = ack.at("tick");
      const auto& a = rx.at(t);
      const auto& b = rx.at(t + 1);
      const auto pa = a.at("position"), pb = b.at("position");
      const bool waited = ack.at("receipt").value("action", "") == "wait";
      // 攻击先结算：本Tick阵亡时模拟器把未执行MOVE归为WAIT，实际死亡与不变位置可核实。
      if (waited && a.at("alive").get<bool>() && !b.at("alive").get<bool>() && pa == pb &&
          a["score"]["resource_value"] == b["score"]["resource_value"]) {
        ++death_canceled_moves;
        continue;
      }
      const bool split = t == 135 && (pa[0] == cross[0] || pa[1] == cross[1]) &&
          (pb==deployed["robots"][0]["spawn"]||confirmed_respawn(out,b.at("session_id"),t+1,pb));
      if (!waited || !split || !a.at("alive").get<bool>() || !b.at("alive").get<bool>() ||
          a["score"]["resource_value"] != b["score"]["resource_value"] ||
          std::abs(b["score"]["effective_damage"].get<double>() -
                   std::max(0., a["score"]["effective_damage"].get<double>() - 60)) > 1e-7)
        throw std::runtime_error("unexplained action receipt mismatch");
      ++replacements;
    }
  Json own;
  for (const auto& team : state.at("board").at("teams"))
    if (team.at("team") == "A")
      own = team;
  if (own.at("invalid_actions") != 0)
    throw std::runtime_error("invalid actions in match");
  const auto cache_report = read(out / "cache-A.json");
  if (!cache_report.value("training_json_gateway", false) ||
      cache_report.value("http_calls", 0) <= 0 ||
      cache_report.at("cached_event_kinds").size() != 8 ||
      cache_report.value("formal_ai_qualification", true) || own.value("ai_eligible", true))
    throw std::runtime_error("incomplete/unmarked training JSON event chain");
  int reach = -1;
  int alternating=0,longest_alternating=0,two_cycle_frames=0;
  // 仅用本队公开位置和得分识别真正无收益的两格往返；死亡/传送/得分变化会中断。
  for(std::size_t i=2;i<rx.size();++i) {
    const auto& a=rx[i-2];const auto& b=rx[i-1];const auto& c=rx[i];
    const auto pa=a.at("position"),pb=b.at("position");
    const int distance=std::abs(pa[0].get<int>()-pb[0].get<int>())+std::abs(pa[1].get<int>()-pb[1].get<int>());
    const bool repeated=a.at("alive").get<bool>()&&b.at("alive").get<bool>()&&c.at("alive").get<bool>()&&
        c.at("position")==pa&&pa!=pb&&distance==1&&a.at("score")==b.at("score")&&b.at("score")==c.at("score");
    if(repeated){alternating=alternating?alternating+1:3;++two_cycle_frames;longest_alternating=std::max(longest_alternating,alternating);}
    else alternating=0;
  }
  for (const auto& row : rx)
    for (const auto& id : row["score"]["mission_completed_ids"])
      if (id == "reach_target" && reach < 0)
        reach = row.at("tick");
  const double score = own.at("score");
  const Json result{{"seed", job.seed},
                    {"cache_identity",identity},
                    {"spawn", job.spawn},
                    {"profile", job.profile},
                    {"parameters", params},
                    {"opponent_roster", roster},
                    {"opponent_environment", roster.empty() ? "official" : "mixed"},
                    {"score45", score},
                    {"conditional_score60", score + 5 * own.at("mission_completed_raw").get<int>()},
                    {"resource40", rx[40]["score"]["resource_value"]},
                    {"resource80", rx[80]["score"]["resource_value"]},
                    {"resource_final", own.at("resource_value")},
                    {"combat_score", own["score_breakdown"]["combat"]},
                    {"resource_score", own["score_breakdown"]["resource"]},
                    {"raw_tasks", own.at("mission_completed_raw")},
                    {"reach_tick", reach},
                    {"two_cycle_frames",two_cycle_frames},{"longest_unproductive_two_cycle",longest_alternating},
                    {"neural_changed", nn_changed},
                    {"collection_neural_changed",collection_changed},{"collection_training_states",collection_samples},
                    {"neural_eligible", nn_eligible},
                    {"neural_inference_mean_ms", inferred ? inference / inferred : 0},
                    {"decision_ms", summary.at("stage2_decision_ms")},
                    {"frames", rx.size()},
                    {"actions", tx},
                    {"acks", acks.size()},
                    {"world_split_replacements", replacements},
                    {"death_canceled_actions", death_canceled_moves},
                    {"training_gateway_http_calls", cache_report.at("http_calls")},
                    {"event_kinds", cache_report.at("cached_event_kinds")},
                    {"real_ai_eligible", own.at("ai_eligible")},
                    {"wall_seconds", std::chrono::duration<double>(Clock::now() - start).count()}};
  save(out / "result.json", result);
  if(compress_logs) {
    // 仅压缩本工具本局已关闭的裁判日志；保留控制轨迹和教师样本，方便诊断与续训。
    std::vector<std::string> args{"gzip","-1","--"};
    for(const auto& file:std::filesystem::recursive_directory_iterator(out/"referee"))
      if(file.is_regular_file()&&file.path().extension()==".jsonl")args.push_back(file.path().string());
    if(args.size()>3) {
      Process gzip(std::move(args),out/"compression.log",env);
      while(gzip.poll()==-1)std::this_thread::sleep_for(std::chrono::milliseconds(20));
      if(gzip.poll()!=0)throw std::runtime_error("log compression failed");
    }
  }
  {
    std::lock_guard<std::mutex> lock(output_mutex);
    std::cout << Json{{"profile", job.profile},
                      {"spawn", job.spawn},
                      {"seed", job.seed},
                      {"score45", score},
                      {"conditional", result["conditional_score60"]},
                      {"resource", result["resource_final"]},
                      {"nn_changed", nn_changed},
                      {"max_ms", result["decision_ms"]["max"]}}
                     .dump()
              << std::endl;
  }
  return result;
}
}  // namespace

/** @brief 读取批次参数，预生成同图出生配置，使用有界C++线程并行执行完整对局。 */
int main(int argc, char** argv) {
  try {
    if (argc == 3 && std::string(argv[1]) == "--report") {
      std::cout << report(argv[2]).dump(2) << std::endl;
      return 0;
    }
    std::map<std::string, std::string> args;
    for (int i = 1; i + 1 < argc; i += 2)
      args[argv[i]] = argv[i + 1];
    const Path output = args.at("--out");
    compress_logs=args.count("--compress-logs")&&args.at("--compress-logs")=="true";
    if(args.count("--stop-at-epoch"))
      stop_at=std::chrono::system_clock::time_point(std::chrono::seconds(std::stoll(args.at("--stop-at-epoch"))));
    auto profiles = read(args.at("--profiles"));
    for (auto p = profiles.begin(); p != profiles.end(); ++p)
      if (p.value().contains("checkpoint")) {
        auto values = read(p.value().at("checkpoint").get<std::string>()).at("parameters");
        values.update(p.value().value("overrides", Json::object()));
        p.value()["parameters"] = std::move(values);
      }
    const int seed_start = std::stoi(args.at("--seed")), maps = std::stoi(args.at("--maps")),
              jobs = std::stoi(args.at("--jobs"));
    const int slot_offset=args.count("--slot-offset")?std::stoi(args.at("--slot-offset")):0;
    if (jobs < 1 || jobs > 16 || slot_offset<0||slot_offset+jobs>16||maps < 1 || maps > 10000)
      throw std::runtime_error("invalid maps/jobs");
    std::filesystem::create_directories(output);
    save(output / "profiles.json", profiles);
    std::ofstream(output / "COLCON_IGNORE");
    const auto base = read(root / "referee/local_v1.2/rmtc_practice/config/stage2.json");
    const auto pool = read(root / "runtime/stage2-total-joint-20261030-1503/opponent-pool.json");
    std::vector<Job> work;
    for (int seed = seed_start; seed < seed_start + maps; ++seed) {
      const auto generated = rmtc::practice::generate_practice_map(base, 2, seed);
      for (const std::string mode : {"fixed", "corner_shuffle", "uniform_free", "clustered"}) {
        const auto config = map_for(generated.config, mode, seed);
        const auto path = output / "maps" / (mode + "-" + std::to_string(seed) + ".json");
        save(path, config);
        for (auto p = profiles.begin(); p != profiles.end(); ++p)
          work.push_back({seed, mode, p.key(), path, p.value()});
      }
    }
    curl_global_init(CURL_GLOBAL_DEFAULT);
    // 每批固定打乱版本/出生顺序，减少某版总在更高机器负载下执行的偏差。
    std::mt19937 scheduling_rng(20261031);
    std::shuffle(work.begin(),work.end(),scheduling_rng);
    std::atomic<std::size_t> next{0};
    std::atomic<int> failures{0};
    std::vector<std::thread> workers;
    for (int slot = 0; slot < jobs; ++slot)
      workers.emplace_back([&, slot]() {
        while (true) {
          if(deadline_reached())break;
          const auto index = next.fetch_add(1);
          if (index >= work.size())
            break;
          const auto& job = work[index];
          bool passed = false;
          for (int attempt = 0; attempt < 2 && !passed && !deadline_reached(); ++attempt)
            try {
              match(job, slot+slot_offset, output, pool);
              passed = true;
            } catch (const std::exception& e) {
              std::lock_guard<std::mutex> lock(output_mutex);
              save(output / (job.profile + "-" + job.spawn + "-seed" + std::to_string(job.seed)) /
                       "failure.json",
                   {{"error", e.what()}, {"attempt", attempt}});
              std::cerr << "RETRY " << job.profile << ' ' << job.seed << ' ' << e.what()
                        << std::endl;
            }
          if (!passed&&!deadline_reached())
            ++failures;
        }
      });
    for (auto& thread : workers)
      thread.join();
    curl_global_cleanup();
    const auto completed=report(output);
    std::size_t completed_games=0;
    if(completed.contains("all"))for(const auto& profile:completed.at("all"))
      completed_games+=profile.at("n").get<std::size_t>();
    std::cout
        << Json{{"games",completed_games},{"planned_games",work.size()},
                {"deadline_reached",deadline_reached()},{"failures", failures.load()}, {"paid_api_calls", 0}}.dump()
        << std::endl;
    return deadline_reached()?3:failures ? 1 : 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
