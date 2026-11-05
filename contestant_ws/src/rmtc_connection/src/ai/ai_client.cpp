/**
 * @file ai_client.cpp
 * @brief 通过libcurl调用统一聊天网关；限制超时、响应大小、协议和重定向。
 *
 * 提示词要求模型解释事件，仅将支持且校验通过的危险区域用于当前基础策略。
 * 不使用关键词/正则解析公告，不从模拟响应中提取事件规则，不读取裁判内部数据。
 *
 * @details 模块协作与维护约定
 * HTTP输入是公开感知、网关配置与请求上限，输出AiReply状态；不直接发送机器人动作。
 * CURL句柄与头部用RAII清理，响应回调捕获异常；body和接收缓冲必须活到同步请求结束。
 * HTTP成功、JSON可解析、业务结构有效与裁判AI资格是不同层级，不可相互替代。
 */
#include "rmtc_connection/ai_client.hpp"

#include <curl/curl.h>

#include <chrono>
#include <memory>
#include <stdexcept>

namespace rmtc {
namespace {

constexpr std::size_t kResponseLimit = 256 * 1024;

/// 接收有界HTTP正文；超限时终止传输，异常不能跨越libcurl的C回调边界。
std::size_t append_body(char* data, std::size_t size, std::size_t count, void* context) noexcept {
  try {
    auto& output = *static_cast<std::string*>(context);
    const auto bytes = size * count;
    if (bytes > kResponseLimit - output.size())
      return 0;
    output.append(data, bytes);
    return bytes;
  } catch (...) {
    return 0;
  }
}

/// 在当前AI进程首次使用时初始化libcurl；此进程没有共享主控的网络线程或状态。
void initialize_curl() {
  static const auto initialized = curl_global_init(CURL_GLOBAL_DEFAULT);
  if (initialized != CURLE_OK)
    throw std::runtime_error("curl_initialization_failed");
}

}  // namespace

AiReply call_ai(const AiClientConfig& config, const PublicPerception& frame,
                std::int64_t action_ticks) {
  const auto started = std::chrono::steady_clock::now();
  AiReply result;
  result.session_id = frame.session_id;
  result.team = frame.self.team;
  result.source_tick = frame.tick;
  result.source_event_text = frame.event_text;
  result.mock = config.service_mode == "mock";
  try {
    initialize_curl();
    auto request = build_ai_request(frame, action_ticks, config.model, config.json_mode,
                                   config.disable_thinking);
    request["max_tokens"]=config.max_tokens;
    const auto body=request.dump();
    if (body.size() > kResponseLimit || config.max_tokens < 1 || config.max_tokens > 32768 ||
        config.token.empty() ||
        (config.url.rfind("http://", 0) != 0 && config.url.rfind("https://", 0) != 0))
      throw std::runtime_error("invalid_request_configuration");
    std::unique_ptr<CURL, decltype(&curl_easy_cleanup)> handle(curl_easy_init(), curl_easy_cleanup);
    if (!handle)
      throw std::runtime_error("curl_handle_failed");
    curl_slist* raw_headers = nullptr;
    raw_headers = curl_slist_append(raw_headers, "Content-Type: application/json");
    raw_headers = curl_slist_append(raw_headers, ("Authorization: Bearer " + config.token).c_str());
    std::unique_ptr<curl_slist, decltype(&curl_slist_free_all)> headers(raw_headers,
                                                                        curl_slist_free_all);
    std::string response;
    curl_easy_setopt(handle.get(), CURLOPT_URL, config.url.c_str());
    curl_easy_setopt(handle.get(), CURLOPT_POST, 1L);
    curl_easy_setopt(handle.get(), CURLOPT_HTTPHEADER, headers.get());
    curl_easy_setopt(handle.get(), CURLOPT_POSTFIELDS, body.c_str());
    curl_easy_setopt(handle.get(), CURLOPT_POSTFIELDSIZE, static_cast<long>(body.size()));
    curl_easy_setopt(handle.get(), CURLOPT_CONNECTTIMEOUT_MS, config.connect_timeout_ms);
    curl_easy_setopt(handle.get(), CURLOPT_TIMEOUT_MS, config.timeout_ms);
    curl_easy_setopt(handle.get(), CURLOPT_NOSIGNAL, 1L);
    curl_easy_setopt(handle.get(), CURLOPT_FOLLOWLOCATION, 0L);
    curl_easy_setopt(handle.get(), CURLOPT_PROTOCOLS, CURLPROTO_HTTP | CURLPROTO_HTTPS);
    curl_easy_setopt(handle.get(), CURLOPT_WRITEFUNCTION, append_body);
    curl_easy_setopt(handle.get(), CURLOPT_WRITEDATA, &response);
    const auto code = curl_easy_perform(handle.get());
    curl_easy_getinfo(handle.get(), CURLINFO_RESPONSE_CODE, &result.http_status);
    if (code != CURLE_OK) {
      result.error_code = "curl_" + std::to_string(static_cast<int>(code));
    } else if (result.http_status < 200 || result.http_status >= 300) {
      result.error_code = "http_" + std::to_string(result.http_status);
      const auto error = Json::parse(response, nullptr, false);
      if (error.is_object() && error.contains("error") && error["error"].is_object() &&
          error["error"].value("code", "") == "quota_exceeded")
        result.error_code = "quota_exceeded";
    } else {
      result = decode_ai_completion(Json::parse(response), frame, action_ticks, result.mock);
      curl_easy_getinfo(handle.get(), CURLINFO_RESPONSE_CODE, &result.http_status);
    }
  } catch (...) {
    result.error_code = "invalid_ai_response_or_configuration";
  }
  result.elapsed_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
  return result;
}

}  // namespace rmtc
