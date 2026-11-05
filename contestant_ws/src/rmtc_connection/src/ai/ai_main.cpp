/**
 * @file ai_main.cpp
 * @brief 独立C++ AI进程：订阅本队感知→调用网关→发布受校验结果，不直接发动作。
 *
 * HTTP即使等待数秒，也只占用本进程；connection_node按原来的ROS回调节奏决策。
 * 开局和任务/事件文本变化时调用；去重、节流、有限重试、额度错误停用均按本局管理。
 * /rmtc/team_X/ai_result是我们自有进程间通信话题，不是裁判的内部话题。
 *
 * @details 模块协作与维护约定
 * 独立进程内执行同步HTTP调用，相对于主控是进程间异步；慢请求不占主控执行器回调。
 * 请求去重、重试与次数限制属于调用生命周期；发布结果仍需主控检查场次、来源和时效。
 */
#include "rmtc_connection/ai_client.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <iostream>
#include <set>

namespace rmtc {

class AiNode final : public rclcpp::Node {
 public:
  /// 配置HTTP、席位与日志；正式令牌只从指定环境变量读取，不作为ROS参数暴露。
  AiNode() : Node("rmtc_ai") {
    team_ = declare_parameter<std::string>("team", "A");
    config_.url =
        declare_parameter<std::string>("ai_url", "http://127.0.0.1:8080/v1/chat/completions");
    config_.model = declare_parameter<std::string>("ai_model", "");
    config_.max_tokens = declare_parameter<int>("ai_max_tokens", 4096);
    config_.service_mode = declare_parameter<std::string>("ai_service_mode", "mock");
    config_.json_mode = declare_parameter<bool>("ai_json_mode", false);
    config_.disable_thinking = declare_parameter<bool>("ai_disable_thinking", true);
    config_.timeout_ms = declare_parameter<int>("ai_timeout_ms", 10000);
    config_.connect_timeout_ms = std::min(1000L, config_.timeout_ms);
    max_calls_ = declare_parameter<int>("ai_max_calls", 40);
    budget_cny_ = declare_parameter<double>("ai_budget_cny", 0.0);
    action_ticks_ = declare_parameter<std::int64_t>("action_ticks", 500);
    cooldown_ms_ = declare_parameter<int>("ai_cooldown_ms", 500);
    const auto token_env = declare_parameter<std::string>("ai_token_env", "RMTC_AI_TOKEN");
    const auto log_path = declare_parameter<std::string>("ai_log_file", "");
    if (team_.size() != 1 || std::string("ABCD").find(team_) == std::string::npos ||
        (config_.service_mode != "mock" && config_.service_mode != "real" &&
         config_.service_mode != "official") ||
        config_.timeout_ms < 100 || config_.timeout_ms > 30000 || max_calls_ < 1 ||
        max_calls_ > 1000 || config_.max_tokens < 1 || config_.max_tokens > 32768 ||
        action_ticks_ <= 0 || cooldown_ms_ < 100 ||
        !std::isfinite(budget_cny_) || budget_cny_ < 0 ||
        (budget_cny_ > 0 && (!config_.disable_thinking || config_.model != "deepseek-flash")))
      throw std::runtime_error("invalid AI node configuration");
    if (config_.service_mode != "official") {
      config_.token = "practice-" + team_;
    } else {
      const auto* token = std::getenv(token_env.c_str());
      if (!token || !*token)
        throw std::runtime_error("official AI token environment variable is missing");
      config_.token = token;
    }
    if (config_.token.find_first_of("\r\n") != std::string::npos)
      throw std::runtime_error("invalid AI token format");
    if (!log_path.empty()) {
      log_.open(log_path, std::ios::app);
      if (!log_)
        throw std::runtime_error("cannot open AI log file");
    }
    publisher_ = create_publisher<std_msgs::msg::String>(
        "/rmtc/team_" + team_ + "/ai_result",
        rclcpp::QoS(rclcpp::KeepLast(4)).reliable().transient_local());
    subscription_ = create_subscription<std_msgs::msg::String>(
        "/team_" + team_ + "/perception",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
        [this](const std_msgs::msg::String::SharedPtr msg) { receive(msg->data); });
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { maybe_call(); });
    write({{"event", "ai_ready"},
           {"team", team_},
           {"service_mode", config_.service_mode},
           {"json_mode", config_.json_mode},
           {"disable_thinking", config_.disable_thinking},
           {"timeout_ms", config_.timeout_ms},
           {"budget_cny", budget_cny_},
           {"max_calls", max_calls_}});
    RCLCPP_INFO(get_logger(),
                "AI process ready team=%s mode=%s; timeout=%ldms; never publishes actions",
                team_.c_str(), config_.service_mode.c_str(), config_.timeout_ms);
  }

  /// 输出本进程请求数、成功数和HTTP耗时；不把本地记录称为官方资格确认。
  void summary() {
    Json result{{"event", "ai_summary"},
                {"requests", requests_},
                {"completions", successes_},
                {"mock_completions", mock_successes_},
                {"sessions", sessions_},
                {"http_mean_ms", requests_ ? elapsed_total_ / requests_ : 0},
                {"http_max_ms", elapsed_max_},
                {"budget_cny", budget_cny_},
                {"reserved_upper_cny", reserved_cny_},
                {"estimated_peak_cny", estimated_cny_},
                {"process_runtime_ms",
                 std::chrono::duration<double, std::milli>(Clock::now() - began_).count()}};
    write(result);
    RCLCPP_INFO(get_logger(), "AI SUMMARY %s", result.dump().c_str());
  }

 private:
  using Clock = std::chrono::steady_clock;

  /// 仅写本队结构化结果及统计，不记录凭据、URL、请求头或未经检查的HTTP正文。
  void write(Json record) {
    if (latest_)
      record["session_id"] = latest_->session_id;
    if (log_) {
      log_ << record.dump() << '\n';
      log_.flush();
    }
  }

  /// 只接受新场次/新Tick；换局清空去重和退避记录，迟到旧局不发起请求。
  void receive(const std::string& data) {
    try {
      auto frame = parse_public_perception(data, team_);
      if (retired_.count(frame.session_id))
        return;
      if (latest_ && latest_->session_id == frame.session_id && frame.tick <= latest_->tick)
        return;
      if (!latest_ || frame.session_id != latest_->session_id) {
        if (latest_)
          retired_.insert(latest_->session_id);
        done_signature_.clear();
        pending_signature_.clear();
        attempts_ = calls_this_session_ = 0;
        disabled_ = false;
        next_call_ = Clock::time_point{};
        ++sessions_;
      }
      latest_ = std::move(frame);
      maybe_call();
    } catch (const std::exception&) {
      RCLCPP_WARN(get_logger(), "AI ignored invalid public perception");
    }
  }

  /// 每份任务/公告最多成功请求一次；失败最多重试3次，限额/认证失败等到新场次。
  void maybe_call() {
    if (!latest_ || latest_->tick >= action_ticks_ || disabled_ ||
        calls_this_session_ >= max_calls_ || Clock::now() < next_call_ || !rclcpp::ok())
      return;
    const auto& frame = *latest_;
    const auto signature = Json{
        {"mission_text", frame.mission_text},
        {"missions", frame.raw.value("missions", Json::array())},
        {"event_text",
         frame.event_text}}.dump();
    if (signature == done_signature_)
      return;
    if (signature != pending_signature_) {
      pending_signature_ = signature;
      attempts_ = 0;
    }
    if (attempts_ >= 3)
      return;
    double reserve = 0;
    if (budget_cny_ > 0) {
      auto request=build_ai_request(frame,action_ticks_,config_.model,config_.json_mode,config_.disable_thinking);
      request["max_tokens"]=config_.max_tokens;
      reserve = ai_request_reserved_cny(request);
      if (reserved_cny_ + reserve > budget_cny_) {
        disabled_ = true;
        write({{"event", "ai_budget_stop"}, {"budget_cny", budget_cny_},
                {"reserved_upper_cny", reserved_cny_}, {"requests", requests_}});
        return;
      }
      reserved_cny_ += reserve;  // 包含失败和超时；换场也不退还本进程已预留额度。
    }
    ++attempts_;
    ++calls_this_session_;
    ++requests_;
    auto result = call_ai(config_, frame, action_ticks_);
    if (result.usage.contains("prompt_tokens") && result.usage.contains("completion_tokens"))
      estimated_cny_ += (result.usage.at("prompt_tokens").get<double>() * 2.0 +
                        result.usage.at("completion_tokens").get<double>() * 8.0) / 1e6;
    else estimated_cny_ += reserve;
    elapsed_total_ += result.elapsed_ms;
    elapsed_max_ = std::max(elapsed_max_, result.elapsed_ms);
    if (result.completion_received) {
      if (result.mock || result.interpretation_valid)
        done_signature_ = signature;
      ++successes_;
      if (result.mock)
        ++mock_successes_;
    }
    if (result.http_status == 401 || result.http_status == 403 ||
        result.error_code == "quota_exceeded")
      disabled_ = true;
    next_call_ =
        Clock::now() + std::chrono::milliseconds((result.completion_received &&
                                                  (result.mock || result.interpretation_valid))
                                                     ? cooldown_ms_
                                                     : std::max(cooldown_ms_, 1000 * attempts_));
    auto reply = result.to_json();
    std_msgs::msg::String msg;
    msg.data = reply.dump();
    publisher_->publish(msg);
    reply["event"] = "ai_reply";
    reply["attempt"] = attempts_;
    write(reply);
    RCLCPP_INFO(get_logger(), "AI tick=%ld HTTP=%ld completion=%s mock=%s elapsed=%.2fms error=%s",
                static_cast<long>(frame.tick), result.http_status,
                result.completion_received ? "true" : "false", result.mock ? "true" : "false",
                result.elapsed_ms, result.error_code.c_str());
  }

  AiClientConfig config_;
  std::string team_, done_signature_, pending_signature_;
  std::optional<PublicPerception> latest_;
  std::set<std::string> retired_;
  int max_calls_{40}, cooldown_ms_{500}, attempts_{0}, calls_this_session_{0};
  std::int64_t action_ticks_{500};
  bool disabled_{false};
  std::uint64_t requests_{0}, successes_{0}, mock_successes_{0}, sessions_{0};
  double elapsed_total_{0}, elapsed_max_{0};
  double budget_cny_{0}, reserved_cny_{0}, estimated_cny_{0};
  Clock::time_point began_{Clock::now()}, next_call_{};
  std::ofstream log_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rmtc

/// 在独立进程内运行AI节点；其异常只导致AI进程退出，主控继续使用本地策略。
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    const auto node = std::make_shared<rmtc::AiNode>();
    rclcpp::spin(node);
    node->summary();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "ai_node: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
