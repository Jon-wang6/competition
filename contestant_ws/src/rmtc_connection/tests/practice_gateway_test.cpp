/** @file practice_gateway_test.cpp
 * @brief 离线测试练习网关的零额度含义、持久化配额、令牌换场及兼容别名；不访问外网。
 *
 * @details 模块协作与维护约定
 * 验证对象：practice_gateway.hpp。
 * 运行约定：由独立测试可执行文件/CTest执行，断言失败用于定位回归，不是正式选手的逐帧逻辑。
 * 结果边界：通过表示这些构造场景满足约束，不证明所有随机地图均最优或稳定达到某个总分。
 */
#include "rmtc_connection/practice_gateway.hpp"
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <unistd.h>
using Json = nlohmann::json;
/** @brief 发布构建也执行的断言，失败直接标明违反的网关规则。 */
void check(bool value, const char* message) {if (!value) throw std::runtime_error(message);}
/** @brief 独立临时目录的所有权，退出时仅清理本测试创建的记录。 */
struct Temporary {
  std::filesystem::path path = std::filesystem::temp_directory_path()/("rmtc-gateway-test-"+std::to_string(getpid()));
  ~Temporary(){std::filesystem::remove_all(path);}
};
/** @brief 构造仅A席位的本地模拟场次，不需要模型密钥，也不生成真实调用资格。 */
Json state(const std::filesystem::path& root, const std::string& session) {
  const auto run = root/session;std::filesystem::create_directories(run);
  return {{"session_id",session},{"run_dir",run.string()},{"stopped",false},{"ai_mode","mock"},
      {"settings",{{"config",{{"robots",Json::array({{{"team","A"}}})}}}}}};
}
/** @brief 覆盖配额生命周期、跨场隔离和本地别名共用额度，确认模拟分支不会访问外部服务。 */
int main() {
  Temporary temp;
  try {
    const auto current = state(temp.path,"first");
    auto training_state=current;training_state["board"]={{"tick",10}};
    const Json injected{{"session_id","first"},{"team","A"},{"source_tick",10},{"source_event_text","公开事件"},
      {"answer",{{"summary","训练数据"},{"event_understood",true},{"event_kind","multiple"},{"event_plan",{{"events",Json::array()}}}}}};
    check(rmtc::practice::training_json_completion(injected,training_state,false).status==403,"training gateway enabled without isolation");
    const auto completion=rmtc::practice::training_json_completion(injected,training_state,true);
    check(completion.status==200&&Json::parse(completion.body).at("training_only")==true,"training gateway did not return marked completion");
    auto future=injected;future["source_tick"]=11;
    check(rmtc::practice::training_json_completion(future,training_state,true).status==400,"future training event accepted");
    auto stale=injected;stale["session_id"]="other";
    check(rmtc::practice::training_json_completion(stale,training_state,true).status==400,"cross-session training event accepted");
    const Json request{{"messages",Json::array({{{"role","user"},{"content","test"}}})}};
    setenv("PRACTICE_AI_QUOTA","0",1);
    {rmtc::practice::PracticeGateway unlimited(temp.path);
      for(int i=0;i<5;++i) check(unlimited.call("Bearer first-A",request,current).status==200,"quota=0 must be unlimited");}
    setenv("PRACTICE_AI_QUOTA","2",1);
    rmtc::practice::PracticeGateway limited(temp.path);
    check(limited.call("Bearer first-A",request,current).status==429,"restart must preserve old token consumption");
    const auto next = state(temp.path,"second");
    check(limited.call("Bearer first-A",request,next).status==401,"previous session token must be rejected");
    check(limited.call("Bearer second-A",request,next).status==200,"new token has its own quota");
    check(limited.call("Bearer practice-A",request,next).status==200,"local alias resolves current token");
    check(limited.call("Bearer practice-A",request,next).status==429,"alias cannot obtain extra quota");
    check(limited.call("Bearer second-D",request,next).status==401,"absent seat must not obtain token access");
    const auto records = limited.records(next);
    check(records["A"]["mode"]=="mock" && records["A"]["success_count"]==2,"mock audit keeps transport success separate");
    setenv("PRACTICE_REAL_AI_TEAMS","A",1);
    rmtc::practice::PracticeGateway only_a(temp.path);
    auto seats=state(temp.path,"cost-control");
    seats["settings"]["config"]["robots"].push_back({{"team","B"}});
    seats["ai_mode"]="real";
    check(only_a.call("Bearer cost-control-B",request,seats).status==403,"disabled bot must never reach paid upstream");
    check(only_a.records(seats).empty(),"blocked bot must not gain AI qualification");
    seats["ai_mode"]="mock";
    check(only_a.call("Bearer cost-control-B",request,seats).status==200,"paid allowlist must not disable mock testing");
    unsetenv("PRACTICE_REAL_AI_TEAMS");
    std::cout << "practice gateway: all checks passed\n";
  } catch(const std::exception& error) {std::cerr << error.what() << '\n';return 1;}
}
