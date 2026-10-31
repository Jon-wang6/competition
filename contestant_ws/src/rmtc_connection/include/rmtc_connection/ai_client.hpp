/**
 * @file ai_client.hpp
 * @brief 独立AI进程的HTTP客户端及受校验的结构化回复；主控不链接网络客户端。
 *
 * 仅发送本队公开感知。令牌来自环境变量或本地练习令牌，不进入ROS消息和日志。
 * mock只确认请求链路，不用于解释事件。真实回复也必须通过结构、场次和时间校验。
 */
#pragma once

#include "rmtc_connection/stage2_strategy.hpp"
#include <stdexcept>

namespace rmtc {

/// AI进程配置；url为完整聊天端点，token只能留在AI进程内。
struct AiClientConfig {
  std::string url;
  std::string token;
  std::string model;                 ///< 留空使用网关默认模型。
  int max_tokens{4096};              ///< 输出预算由选手设置并转发，须遵守赛前公布限制。
  std::string service_mode{"mock"};  ///< mock、本地真实代理real、正式网关official。
  bool json_mode{false};             ///< 支持时要求模型输出JSON对象，默认不改变官方网关请求。
  bool disable_thinking{true};       ///< 本赛DeepSeek网关默认发送顶层thinking.type=disabled，可显式关闭此选项。
  long timeout_ms{2000};
  long connect_timeout_ms{500};
};

/// 一次回复的传输统计和解释结果；不把成功响应直接等同于官方计分资格。
struct AiReply {
  std::string session_id, team, source_event_text;
  std::int64_t source_tick{0};
  long http_status{0};
  double elapsed_ms{0};
  bool completion_received{false};
  bool mock{true};
  bool interpretation_valid{false};
  bool event_understood{false};
  std::string summary, error_code;
  std::string event_kind{"unknown"};  ///< 由模型分类，支持八类事件及多事件时间线。
  Json usage = Json::object();        ///< 上游返回的非负token计数，用于费用审计；不保留其他响应字段。
  std::optional<Stage2EventConstraints> constraints;

  /// 编码本队内部AI结果消息；永远不包含token、请求头或完整HTTP响应。
  Json to_json() const;
};

/// 从官方兼容回复中解析八类事件；保留旧危险格/矩形结构用于回放兼容。
AiReply decode_ai_completion(const Json& response, const PublicPerception& frame,
                             std::int64_t action_ticks, bool configured_mock);
/// 将模型理解后的危险区域转换成策略约束；只校验结构化JSON，不分析自然语言。
Stage2EventConstraints decode_ai_constraints(const Json& plan, const PublicPerception& frame,
                                             std::int64_t source_tick, std::int64_t action_ticks);
/// 八类事件时间线的数值/区域校验及内部IPC编码；不读取或匹配任何自然语言。
Stage2EventConstraints decode_ai_event_timeline(const Json& plan,const PublicPerception& frame,
                                               std::int64_t source_tick,std::int64_t action_ticks);
Json encode_ai_event_timeline(const Stage2EventConstraints& constraints);
/// 构建包含公开状态、任务和事件原文的提示词；所有坐标来自本队合法感知。
Json build_ai_request(const PublicPerception& frame, std::int64_t action_ticks,
                      const std::string& model = "", bool json_mode = false,
                      bool disable_thinking = false);
/// DeepSeek Flash审计预留：按2026-10-28高峰单价、保守输入字节估计和最大输出计算。
/// 仅支持明确关闭思考、没有工具的Flash请求；预留不是实际账单，不支持其他模型价格。
inline double ai_request_reserved_cny(const Json& request) {
  if (request.value("model", "") != "deepseek-flash" || request.contains("tools") ||
      !request.contains("thinking") || request.at("thinking").value("type", "") != "disabled" ||
      !request.contains("max_tokens") || !request.at("max_tokens").is_number_integer() ||
      request.at("max_tokens") <= 0 || request.at("max_tokens") > 32768)
    throw std::invalid_argument("unsupported request for bounded Flash cost audit");
  return ((2.0 * request.dump().size() + 1024) * 2.0 +
          request.at("max_tokens").get<double>() * 8.0) / 1e6;
}
/// 有界同步HTTP调用，只能由独立ai_node进程执行，禁止放进主控感知回调。
AiReply call_ai(const AiClientConfig& config, const PublicPerception& frame,
                std::int64_t action_ticks);

}  // namespace rmtc
