/**
 * @file simulator_probe.cpp
 * @brief 仅在隔离域内直接驱动裁判模拟器，读取完整结算状态以审计规则。
 * 输入每行是一个动作批次，输出对应状态；此程序不属于参赛选手，禁止连接比赛域。
 */
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <fstream>
#include <filesystem>
#include <sys/wait.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>

/** @brief 审计子进程的所有权；无论断言是否失败都停止本次模拟器。 */
struct Child {
  pid_t pid{-1};
  ~Child() { if (pid > 0) {kill(pid, SIGTERM); waitpid(pid, nullptr, 0);} }
};

/** @brief 连接隔离测试域，逐批次发送并等待单次结算，超时明确失败。 */
int main(int argc, char** argv) {
  const char* domain = std::getenv("ROS_DOMAIN_ID");
  if (!domain || std::string(domain) != "192") return 2;
  nlohmann::json fixture;
  Child child;
  std::ofstream history;
  if (argc == 4) {
    fixture = nlohmann::json::parse(std::ifstream(argv[1]));
    const std::string stem = argv[3];
    std::ofstream(stem + ".config.json") << fixture.at("config").dump();
    child.pid = fork();
    if (child.pid == 0) {
      setenv("ROS_DOMAIN_ID", "191", 1);
      setenv("RMTC_INTERNAL_DOMAIN_ID", "192", 1);
      const int log = open((stem + ".log").c_str(), O_CREAT|O_WRONLY|O_TRUNC, 0644);
      dup2(log, 1); dup2(log, 2); close(log);
      execl(argv[2], argv[2], "--ros-args", "-p", ("config:=" + stem + ".config.json").c_str(),
            "-p", "session_id:=rule-audit", "-p", ("event_log:=" + stem + ".events.jsonl").c_str(), nullptr);
      _exit(127);
    }
    if (child.pid < 0) return 5;
    history.open(stem + ".states.jsonl");
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("referee_rule_probe");
  auto pub = node->create_publisher<std_msgs::msg::String>("/referee/validated_actions", 10);
  nlohmann::json state;
  auto sub = node->create_subscription<std_msgs::msg::String>(
      "/simulator/state", rclcpp::QoS(10).transient_local(),
      [&](std_msgs::msg::String::SharedPtr m) {state = nlohmann::json::parse(m->data);});
  auto wait = [&](int tick) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node);
      if (!state.is_null() && state.value("tick", -1) == tick) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
  };
  if (!wait(0)) {std::cerr << "initial state timeout\n"; return 3;}
  if (fixture.is_null()) std::cout << state.dump() << std::endl;
  else history << state.dump() << '\n';
  const auto discovery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
  while (pub->get_subscription_count() == 0 && std::chrono::steady_clock::now() < discovery_deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (pub->get_subscription_count() == 0) {std::cerr << "action discovery timeout\n";return 6;}
  std::string line;
  std::size_t index = 0;
  int failed = 0, checked = 0;
  while (fixture.is_null() ? static_cast<bool>(std::getline(std::cin, line)) : index < fixture.at("steps").size()) {
    auto step = fixture.is_null() ? nlohmann::json::parse(line) : fixture.at("steps").at(index++);
    auto command = nlohmann::json{{"actions", step.value("actions", nlohmann::json::object())}};
    command["tick"] = state.at("tick");
    command["session_id"] = state.at("session_id");
    for (auto& item : command["actions"].items()) {
      item.value()["tick"] = state.at("tick");
      item.value()["session_id"] = state.at("session_id");
    }
    const int tick = state.at("tick");
    std_msgs::msg::String msg;
    msg.data = command.dump(); pub->publish(msg);
    if (!wait(tick + 1)) {std::cerr << "step timeout " << tick << '\n'; return 4;}
    if (fixture.is_null()) std::cout << state.dump() << std::endl;
    else {
      history << state.dump() << '\n';
      for (const auto& check : step.value("checks", nlohmann::json::array())) {
        ++checked;
        const std::string pointer = check.at("path");
        const auto& expected = check.at("value");
        const auto path = nlohmann::json::json_pointer(pointer);
        bool good = state.contains(path) && state.at(path) == expected;
        if (!good && state.contains(path) && expected.is_number() && state.at(path).is_number())
          good = std::abs(expected.get<double>() - state.at(path).get<double>()) < 1e-8;
        if (!good) {
          ++failed;
          std::cerr << fixture.at("name") << " tick=" << tick << " " << pointer
                    << " expected=" << expected << " actual=" << (state.contains(path) ? state.at(path) : nlohmann::json()) << '\n';
        }
      }
    }
  }
  rclcpp::shutdown();
  if (!fixture.is_null()) std::cout << nlohmann::json{{"name", fixture.at("name")}, {"checks", checked}, {"failed", failed}}.dump() << std::endl;
  return failed ? 1 : 0;
}
