/**
 * @file practice_gateway.cpp
 * @brief C++练习AI代理，成功HTTP响应消耗配额；模型内容是否能用由选手自行判断。
 * 密钥只从父进程环境读取，不写入请求审计、配额文件或诊断日志。
 *
 * @details 模块协作与维护约定
 * 练习代理维护令牌/额度与转发结果；配额记录与正式赛事网关的状态不是同一份数据。
 * HTTP转发成功并不保证回复符合选手事件结构；密钥和令牌不应写入调试输出。
 */
#include "rmtc_connection/practice_gateway.hpp"
#include <curl/curl.h>
#include <cstdlib>
#include <fstream>
#include <stdexcept>

namespace rmtc::practice {
GatewayReply training_json_completion(const nlohmann::json& input,const nlohmann::json& state,
                                      bool enabled) {
  using Json=nlohmann::json;
  if(!enabled)return {403,Json{{"error","training_endpoint_disabled"}}.dump()};
  try {
    if(input.dump().size()>256*1024||input.at("session_id")!=state.at("session_id")||
        !input.at("source_tick").is_number_integer()||input.at("source_tick")<0||
        input.at("source_tick")>state.at("board").at("tick")||
        !input.at("source_event_text").is_string()||input.at("source_event_text").get_ref<const std::string&>().empty())
      throw std::invalid_argument("invalid training session/tick/broadcast");
    bool found=false;for(const auto& robot:state.at("settings").at("config").at("robots"))if(robot.at("team")==input.at("team"))found=true;
    const auto& answer=input.at("answer");
    if(!found||!answer.at("summary").is_string()||!answer.at("event_understood").is_boolean()||
        !answer.at("event_kind").is_string()||!answer.at("event_plan").is_object())
      throw std::invalid_argument("invalid training interpretation");
    return {200,Json{{"id","chatcmpl-training-json"},{"object","chat.completion"},
      {"model","practice-training-json"},{"training_only",true},{"formal_ai_qualification",false},
      {"session_id",input.at("session_id")},{"source_tick",input.at("source_tick")},
      {"choices",Json::array({{{"index",0},{"message",{{"role","assistant"},{"content",answer.dump()}}},{"finish_reason","stop"}}})},
      {"usage",{{"prompt_tokens",0},{"completion_tokens",0},{"total_tokens",0}}}}.dump()};
  }catch(const std::exception& e){return {400,Json{{"error",e.what()}}.dump()};}
}
namespace {
using Json = nlohmann::json;
/** @brief 构造兼容网关错误，区分鉴权、配额及上游故障。 */
GatewayReply failure(int status, const char* code, const char* message) {
  return {status, Json{{"error", {{"code", code}, {"type", "gateway_error"}, {"message", message}}}}.dump()};
}
/** @brief 读取非密钥运行选项；不输出完整进程环境。 */
std::string env(const char* key) {const char* value = std::getenv(key); return value ? value : "";}
/** @brief 原子持久化配额，写盘失败必须停止确认成功，防止额度回退。 */
void save(const std::filesystem::path& path, const Json& value) {
  std::ofstream out; out.exceptions(std::ios::badbit | std::ios::failbit);
  out.open(path.string()+".tmp"); out << value.dump() << '\n'; out.close();
  std::filesystem::rename(path.string()+".tmp", path);
}
/** @brief 接收有限大小的完整上游响应；不进行逐token转发。 */
std::size_t receive(char* bytes, std::size_t size, std::size_t count, void* opaque) {
  auto& output = *static_cast<std::string*>(opaque);
  const auto length = size * count;
  if (output.size() + length > 16*1024*1024) return 0;
  output.append(bytes, length); return length;
}
/** @brief 单次真实转发，不自动重试；超时504、连接故障502、HTTP状态原样返回。 */
GatewayReply upstream(Json body) {
  const std::string url = env("PRACTICE_LLM_URL"), key = env("PRACTICE_LLM_KEY"), model = env("PRACTICE_MODEL");
  if (url.empty() || key.empty()) return failure(503, "upstream_unconfigured", "真实模型服务尚未配置");
  if (!body.contains("model") || body["model"] == "") body["model"] = model;
  if (!body["model"].is_string() || body["model"] == "") return failure(400, "invalid_model", "须提供模型名");
  if (!model.empty() && body["model"] != model) return failure(400, "invalid_model", "请使用本场公布的模型");
  CURL* curl = curl_easy_init();
  if (!curl) return failure(502, "upstream_error", "无法初始化上游请求");
  auto* headers = curl_slist_append(nullptr, "Content-Type: application/json");
  headers = curl_slist_append(headers, ("Authorization: Bearer " + key).c_str());
  headers = curl_slist_append(headers, "Accept-Encoding: identity");
  const auto payload = body.dump();
  std::string output;
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload.c_str());
  curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE_LARGE, static_cast<curl_off_t>(payload.size()));
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, 5000L);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 30000L);
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 0L);
  curl_easy_setopt(curl, CURLOPT_PROTOCOLS, CURLPROTO_HTTP | CURLPROTO_HTTPS);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, receive);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &output);
  const auto code = curl_easy_perform(curl);
  long status = 0; curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
  curl_slist_free_all(headers); curl_easy_cleanup(curl);
  if (code == CURLE_OPERATION_TIMEDOUT) return failure(504, "upstream_timeout", "上游模型调用超时");
  if (code != CURLE_OK || status == 0) return failure(502, "upstream_error", "上游模型连接失败");
  if (status >= 300 && status < 400) return failure(502, "upstream_redirect", "上游重定向被拒绝，请配置最终地址");
  return {static_cast<int>(status), std::move(output)};
}
}

/** @brief 启动时恢复令牌生命周期配额；环境变量0代表不限制，默认100。 */
PracticeGateway::PracticeGateway(std::filesystem::path data) : data_(std::move(data)), consumed_(Json::object()) {
  std::filesystem::create_directories(data_);
  const auto teams=env("PRACTICE_REAL_AI_TEAMS");
  if(!teams.empty()) {
    if(teams.find_first_not_of("ABCD")!=std::string::npos)
      throw std::invalid_argument("PRACTICE_REAL_AI_TEAMS must contain only seat letters");
    real_teams_=teams;
  }
  const auto value = env("PRACTICE_AI_QUOTA");
  if (!value.empty()) {
    std::size_t used = 0; quota_ = std::stoi(value, &used);
    if (used != value.size() || quota_ < 0) throw std::invalid_argument("PRACTICE_AI_QUOTA must be nonnegative");
  }
  if (std::filesystem::exists(data_/"gateway-quota.json")) {
    consumed_ = Json::parse(std::ifstream(data_/"gateway-quota.json"));
    if (!consumed_.is_object()) throw std::runtime_error("invalid quota ledger");
    for (const auto& item : consumed_.items())
      if (!item.value().is_number_unsigned() && (!item.value().is_number_integer() || item.value().get<long long>() < 0))
        throw std::runtime_error("invalid quota counter");
  }
  if (curl_global_init(CURL_GLOBAL_DEFAULT) != CURLE_OK) throw std::runtime_error("curl initialization failed");
}

/** @brief 从本场裁判审计文件汇总网页调用状态，排除其他场次，绝不相信选手自报。 */
nlohmann::json PracticeGateway::records(const Json& state) const {
  Json result = Json::object();
  std::ifstream in(std::filesystem::path(state.value("run_dir", ""))/"ai.jsonl");
  std::string line;
  while (std::getline(in, line)) {
    const auto record = Json::parse(line, nullptr, false);
    if (record.is_object() && record.value("session_id", "") == state.value("session_id", "")) result[record.at("seat").get<std::string>()] = record;
  }
  return result;
}

/** @brief 验证本地席位令牌并独占该席位转发；成功先落盘，随后才向客户端返回。 */
GatewayReply PracticeGateway::call(const std::string& auth, const Json& body, const Json& state) {
  if (auth.empty() || auth == "Bearer ") return failure(401, "missing_token", "缺少席位令牌");
  const std::string session = state.value("session_id", "");
  std::string seat;
  for (char value : std::string("ABCD"))
    if (auth == "Bearer " + session + "-" + value || auth == "Bearer practice-" + std::string(1,value)) seat = value;
  if (seat.empty()) return failure(401, "invalid_token", "无效的本地练习令牌");
  if (!body.is_object() || !body.contains("messages") || !body["messages"].is_array() || body["messages"].empty()) return failure(400, "invalid_request", "messages必须是非空数组");
  if (body.contains("stream") && (!body["stream"].is_boolean() || body["stream"].get<bool>())) return failure(400, "stream_unsupported", "不支持流式请求");
  if (state.value("stopped", true) || state.value("session_id", "").empty()) return failure(503, "no_session", "尚未部署练习");
  bool present = false;
  for (const auto& robot : state.at("settings").at("config").at("robots")) if (robot.at("team") == seat) present = true;
  if (!present) return failure(401, "invalid_token", "该席位不属于当前场次");
  if(state.at("ai_mode")=="real"&&real_teams_.find(seat)==std::string::npos)
    return failure(403,"training_upstream_disabled","本机测试未为该席位开启付费上游；没有真实调用资格");
  // practice-A等仅是本地兼容别名，映射到本场新签发令牌，不能额外获得第二份额度。
  const std::string token = session + "-" + seat;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (quota_ && consumed_.value(token, 0) >= quota_) return failure(429, "quota_exceeded", "本令牌成功调用额度耗尽");
    if (!in_flight_.insert(token).second) return failure(429, "concurrent_request", "本席位已有请求进行中");
  }
  GatewayReply reply;
  try {
    if (state.at("ai_mode") == "mock") {
      const Json content{{"goals", {"resource_high","resource_normal","reach_target"}},
          {"event_plan",nullptr}, {"summary","模拟响应，仅验证通信，未理解事件"}};
      reply = {200, Json{{"id", "chatcmpl-practice-mock"}, {"object", "chat.completion"}, {"model", "practice-mock"},
          {"choices", Json::array({{{"index",0}, {"finish_reason","stop"}, {"message",{{"role","assistant"}, {"content",content.dump()}}}}})},
          {"usage",{{"prompt_tokens",0},{"completion_tokens",0},{"total_tokens",0}}}}.dump()};
    } else reply = upstream(body);
    const bool success = reply.status >= 200 && reply.status < 300;
    std::lock_guard<std::mutex> lock(mutex_);
    // 槽位在日志落盘完成前保持占用，避免同席位并发写入破坏计数。
    const auto previous = records(state).value(seat, Json::object());
    const Json record{{"seat",seat}, {"session_id",state.at("session_id")}, {"mode",state.at("ai_mode")},
        {"status",reply.status}, {"success",success}, {"success_count",previous.value("success_count",0)+int(success)},
        {"attempt_count",previous.value("attempt_count",0)+1}};
    std::ofstream out; out.exceptions(std::ios::badbit | std::ios::failbit);
    out.open(std::filesystem::path(state.at("run_dir").get<std::string>())/"ai.jsonl", std::ios::app);
    if (success) {consumed_[token] = consumed_.value(token,0)+1; save(data_/"gateway-quota.json", consumed_);}
    out << record.dump() << '\n'; out.close();
    in_flight_.erase(token);
    return reply;
  } catch (...) {
    std::lock_guard<std::mutex> lock(mutex_); in_flight_.erase(token); throw;
  }
}
}
