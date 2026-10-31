/**
 * @file ai_cache_replay.cpp
 * @brief 批量评估专用：复用真实模型解析过的同一公共公告，不调用模型、不授予AI计分资格。
 *
 * 只允许隔离训练域100..130、200..230的偶数域；后者供并行正常节奏验证。
 * 仅复用全局事件，不复制上一局个体Buff；
 * 原事件公告必须逐字出现在当前公开event_text中，源Tick不得晚于当前Tick。
 * 不安装到参赛入口、不解析自然语言、不读取裁判内部状态或密钥。
 */
#include "rmtc_connection/ai_client.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <set>
#include <curl/curl.h>

namespace {
using rmtc::Json;

/** @brief 一次加载逐字公告及其已有解析结果，只允许复用全局事件。 */
class CacheReplay : public rclcpp::Node {
 public:
  CacheReplay() : Node("training_event_cache") {
    team_ = declare_parameter<std::string>("team", "A");
    const auto path = declare_parameter<std::string>("cache_file", "");
    report_ = declare_parameter<std::string>("report_file", "");
    training_url_ = declare_parameter<std::string>("training_gateway_url", "");
    if (!training_url_.empty() && training_url_.find("http://127.0.0.1:") != 0)
      throw std::runtime_error("training JSON gateway must be local");
    std::ifstream input(path);
    if (!input || team_.size() != 1 || std::string("ABCD").find(team_) == std::string::npos)
      throw std::runtime_error("invalid training cache configuration");
    for (std::string line; std::getline(input, line);) {
      if (line.empty())
        continue;
      auto row = Json::parse(line);
      if (!row.at("event").value("actor", "").empty())
        throw std::runtime_error("individual effect in cache");
      cache_.push_back(std::move(row));
    }
    publisher_ = create_publisher<std_msgs::msg::String>(
        "/rmtc/team_" + team_ + "/ai_result", rclcpp::QoS(4).reliable().transient_local());
    subscription_ = create_subscription<std_msgs::msg::String>(
        "/team_" + team_ + "/perception", rclcpp::QoS(1).reliable(),
        [this](std_msgs::msg::String::ConstSharedPtr msg) { receive(msg->data); });
  }

  /** @brief 只记录覆盖率；缓存回复从不写入裁判ai.jsonl，不伪造成功HTTP调用。 */
  ~CacheReplay() override {
    Json result{{"frames", frames_},
                {"announcement_frames", announced_},
                {"matched_frames", matched_},
                {"replays", replays_},
                {"http_calls", http_calls_},
                {"formal_ai_qualification", false},
                {"training_json_gateway", !training_url_.empty()},
                {"max_cached_source_tick", max_cached_tick_},
                {"cached_event_kinds", kinds_}};
    if (!report_.empty())
      std::ofstream(report_) << result.dump(2) << '\n';
  }

 private:
  /// 有界接收本机训练接口的chat/completions JSON；绝不访问真实模型服务。
  static std::size_t append(char* data, std::size_t size, std::size_t count, void* context) {
    auto& out = *static_cast<std::string*>(context);
    if (out.size() + size * count > 512 * 1024)
      return 0;
    out.append(data, size * count);
    return size * count;
  }
  /** @brief 将已有结构化答案交给裁判训练接口，再使用与真实模型相同的解析器处理响应。 */
  rmtc::AiReply training_reply(const rmtc::PublicPerception& frame, const Json& plan) {
    const auto payload = Json{
        {"session_id", frame.session_id},
        {"team", team_},
        {"source_tick", frame.tick},
        {"source_event_text", frame.event_text},
        {"answer",
         {{"summary", "本地离线训练事件"},
          {"event_understood", true},
          {"event_kind", "multiple"},
          {"event_plan",
           plan}}}}.dump();
    CURL* curl = curl_easy_init();
    if (!curl)
      throw std::runtime_error("training curl init failed");
    std::string body;
    auto* headers = curl_slist_append(nullptr, "Content-Type: application/json");
    curl_easy_setopt(curl, CURLOPT_URL, training_url_.c_str());
    curl_easy_setopt(curl, CURLOPT_PROXY, "");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload.c_str());
    curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 1500L);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, append);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &body);
    const auto code = curl_easy_perform(curl);
    long status = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    ++http_calls_;
    if (code != CURLE_OK || status != 200)
      throw std::runtime_error("training JSON gateway unavailable: " + std::to_string(status));
    const auto completion = Json::parse(body);
    if (!completion.value("training_only", false) ||
        completion.value("formal_ai_qualification", true))
      throw std::runtime_error("unmarked training JSON response");
    auto reply = rmtc::decode_ai_completion(completion, frame, 500, false);
    reply.http_status = status;
    if (!reply.interpretation_valid || !reply.event_understood || !reply.constraints)
      throw std::runtime_error("training JSON interpretation rejected");
    return reply;
  }
  /** @brief 收集当前公告中精确出现且已到解析时刻的缓存；不推测任何新公告。 */
  void receive(const std::string& text) {
    try {
      const auto frame = rmtc::parse_public_perception(text, team_);
      ++frames_;
      if (frame.event_text.empty())
        return;
      ++announced_;
      Json plan{{"events", Json::array()}};
      for (const auto& cached : cache_) {
        const auto& anchor = cached.at("anchor").get_ref<const std::string&>();
        if (cached.at("source_tick").get<std::int64_t>() > frame.tick ||
            frame.event_text.find(anchor) == std::string::npos)
          continue;
        max_cached_tick_ = std::max(max_cached_tick_, cached.at("source_tick").get<std::int64_t>());
        plan["events"].push_back(cached.at("event"));
      }
      if (plan["events"].empty())
        return;
      ++matched_;
      const auto signature = plan.dump();
      if (session_ == frame.session_id && previous_ == signature)
        return;
      const auto constraints = rmtc::decode_ai_constraints(plan, frame, frame.tick, 500);
      for (const auto& effect : constraints.events)
        kinds_.insert(effect.kind);
      rmtc::AiReply reply;
      reply.session_id = frame.session_id;
      reply.team = team_;
      reply.source_tick = frame.tick;
      reply.source_event_text = frame.event_text;
      reply.mock = false;
      reply.completion_received = true;
      reply.interpretation_valid = true;
      reply.event_understood = true;
      reply.event_kind = "multiple";
      reply.constraints = constraints;
      if (!training_url_.empty())
        reply = training_reply(frame, plan);
      auto value = reply.to_json();
      value["training_cache_only"] = true;
      std_msgs::msg::String message;
      message.data = value.dump();
      publisher_->publish(message);
      session_ = frame.session_id;
      previous_ = signature;
      ++replays_;
    } catch (const std::exception& error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "cache rejected: %s", error.what());
    }
  }
  std::string team_, report_, session_, previous_, training_url_;
  std::size_t http_calls_{0};
  std::vector<Json> cache_;
  std::size_t frames_{0}, announced_{0}, matched_{0}, replays_{0};
  std::int64_t max_cached_tick_{0};
  std::set<std::string> kinds_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
}  // namespace

/** @brief 限定在本机隔离训练域内，拒绝与常用比赛域27混用。 */
int main(int argc, char** argv) {
  try {
    const char* raw = std::getenv("ROS_DOMAIN_ID");
    const int domain = raw ? std::stoi(raw) : -1;
    const bool isolated = (domain >= 100 && domain <= 130) || (domain >= 200 && domain <= 230);
    if (!isolated || domain % 2)
      throw std::runtime_error("training domain required");
    rclcpp::init(argc, argv);
    {
      auto node = std::make_shared<CacheReplay>();
      rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
