/** @file practice_gateway.hpp
 * @brief 裁判专用AI网关：每令牌成功配额、真实/模拟审计及上游错误映射。
 *
 * @details 模块协作与维护约定
 * 练习代理维护令牌/额度与转发结果；配额记录与正式赛事网关的状态不是同一份数据。
 * HTTP转发成功并不保证回复符合选手事件结构；密钥和令牌不应写入调试输出。
 */
#pragma once
#include <nlohmann/json.hpp>
#include <filesystem>
#include <mutex>
#include <set>
#include <string>

namespace rmtc::practice {
struct GatewayReply { int status; std::string body; };

/** @brief 离线注入结构化答案并包装为标准chat/completions格式，永不写正式AI配额账本。 */
GatewayReply training_json_completion(const nlohmann::json& input,const nlohmann::json& state,
                                      bool isolated_training_enabled);

/** @brief 每场签发独立席位令牌；同一令牌的配额在网页进程重启后也不恢复。 */
class PracticeGateway {
 public:
  /** @brief 恢复本地配额账本，初始化用于真实转发的HTTP运行库。 */
  explicit PracticeGateway(std::filesystem::path data);
  /** @brief 校验当前场次席位及配额，转发一次请求并持久化真实/模拟审计。 */
  GatewayReply call(const std::string& auth, const nlohmann::json& body,
                    const nlohmann::json& state);
  /** @brief 只返回当前场次最新审计汇总，供观赛网页展示调用次数。 */
  nlohmann::json records(const nlohmann::json& state) const;
 private:
  std::filesystem::path data_;
  std::mutex mutex_;
  nlohmann::json consumed_;
  std::set<std::string> in_flight_;
  int quota_{100};
  std::string real_teams_{"ABCD"};  ///< 本机付费转发席位白名单；默认保持四队均可调用。
};
}
