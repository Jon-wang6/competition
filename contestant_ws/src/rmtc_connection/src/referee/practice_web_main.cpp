/**
 * @file practice_web_main.cpp
 * @brief 本地练习网页入口：保留当前地图部署，提供C++种子生成接口，转发官方裁判API。
 *
 * 官方practice.py及裁判二进制不修改，运行在独立回环端口。本程序只增加网页与
 * 地图配置管理，不接入选手策略、不向公开ROS感知注入全图信息。-1随机，
 * 其余整数（含0）可复现；C++代理补全v1.2 AI鉴权、额度与审计。
 * /api/training/event-json仅在显式开启的隔离训练域使用，返回标准JSON但不授予AI资格。
 *
 * @details 模块协作与维护约定
 * 本地扩展服务连接网页、地图生成与官方裁判接口；与参赛策略进程分离。
 * 部署参数、地图生成和比赛开始是不同操作，定位地图变化时应核对请求与所保存配置。
 */
#include "rmtc_connection/map_generator.hpp"
#include "rmtc_connection/practice_gateway.hpp"

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <charconv>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
namespace net = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
using tcp = net::ip::tcp;
using Json = nlohmann::json;
using Request = http::request<http::string_body>;
using Response = http::response<http::string_body>;
using Path = std::filesystem::path;
using namespace std::chrono_literals;

/** @brief 精确解析64位整数，种子经过字符串传输以避免浏览器丢失大整数精度。 */
std::int64_t integer(const std::string& value) {
  std::int64_t result = 0;
  const auto parsed = std::from_chars(value.data(), value.data() + value.size(), result);
  if (value.empty() || parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size())
    throw std::invalid_argument("参数须为64位整数");
  return result;
}

/** @brief 读取UTF-8文件；失败时报告文件名，不退回不匹配的地图模板。 */
std::string read_file(const Path& path) {
  std::ifstream stream(path);
  if (!stream)
    throw std::runtime_error("无法读取文件：" + path.string());
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

/** @brief 写入独立练习生成记录，官方目录始终只读；IO错误向网页报告。 */
void write_json(const Path& path, const Json& value) {
  std::ofstream stream;
  stream.exceptions(std::ios::failbit | std::ios::badbit);
  stream.open(path);
  stream << value.dump(2) << '\n';
}

/** @brief 构造不缓存、单次连接的HTTP响应，JSON和网页均明确声明内容类型。 */
Response response(http::status status, std::string body,
                  const char* mime = "application/json; charset=utf-8") {
  Response result{status, 11};
  result.set(http::field::content_type, mime);
  result.set(http::field::cache_control, "no-store");
  result.keep_alive(false);
  result.body() = std::move(body);
  result.prepare_payload();
  return result;
}

/** @brief 只向固定回环裁判端口转发；连接/写入/响应分别限时，保留AI鉴权及官方错误。 */
Response forward(Request request, unsigned short port) {
  net::io_context io;
  beast::tcp_stream stream(io);
  beast::flat_buffer buffer;
  http::response_parser<http::string_body> parser;
  parser.body_limit(16 * 1024 * 1024);
  beast::error_code failure;
  request.keep_alive(false);
  request.set(http::field::accept_encoding, "identity");
  request.prepare_payload();
  stream.expires_after(3s);
  stream.async_connect(
      tcp::endpoint{net::ip::make_address("127.0.0.1"), port}, [&](beast::error_code error) {
        if (error) {
          failure = error;
          return;
        }
        stream.expires_after(5s);
        http::async_write(stream, request, [&](beast::error_code write_error, std::size_t) {
          if (write_error) {
            failure = write_error;
            return;
          }
          stream.expires_after(40s);
          http::async_read(stream, buffer, parser, [&](beast::error_code read_error, std::size_t) {
            failure = read_error;
          });
        });
      });
  io.run();
  if (failure)
    throw std::runtime_error("官方裁判暂时不可用：" + failure.message());
  auto result = parser.release();
  result.keep_alive(false);
  result.set(http::field::cache_control, "no-store");
  return result;
}

/** @brief 使用地图、阶段与出生点识别地图；修改Tick或任务不丢失地图种子记录。 */
std::string map_key(const Json& config) {
  return Json{
      {"stage", config.at("stage")}, {"map", config.at("map")}, {"robots", config.at("robots")}}
      .dump();
}

/** @brief 生成器摘要转换成浏览器安全格式，64位种子永不经过JavaScript Number。 */
Json web_metadata(const Json& metadata) {
  auto result = metadata;
  result["seed"] = std::to_string(metadata.at("seed").get<std::int64_t>());
  result["requested_seed"] = std::to_string(metadata.at("requested_seed").get<std::int64_t>());
  return result;
}

/// 只管理网页与配置；注册表记录种子对应地图，不保存机器人控制状态。
class PracticeWeb {
 public:
  /** @brief 载入两阶段官方模板及启动地图元信息，固定后端端口并创建自有记录目录。 */
  PracticeWeb(Path official, Path web, Path data, Path initial_config, unsigned short port,
              unsigned short backend)
      : web_(std::move(web)), data_(std::move(data)), port_(port), backend_(backend), gateway_(data_) {
    for (int stage : {1, 2}) {
      templates_[stage] =
          Json::parse(read_file(official / "config" / ("stage" + std::to_string(stage) + ".json")));
      const auto default_map =
          rmtc::practice::generate_practice_map(templates_.at(stage), stage, 0);
      remember(default_map.config, default_map.metadata);
    }
    auto metadata_path = initial_config;
    metadata_path.replace_extension("metadata.json");
    if (std::filesystem::is_regular_file(metadata_path)) {
      const auto metadata = Json::parse(read_file(metadata_path));
      // 兼容旧运行记录，只按元信息标记原配置；不把旧18×18模板认作新版种子0。
      if (metadata.value("generator_version", 0) >= 2 || metadata.at("seed") != 0)
        remember(Json::parse(read_file(initial_config)), metadata);
    }
    std::filesystem::create_directories(data_ / "generated-maps");
  }

  /** @brief 服务自有静态网页、生成预览及原始裁判API；拒绝跨来源控制请求。 */
  Response handle(Request request) {
    const std::string target(request.target());
    try {
      if (request.method() == http::verb::get && target == "/")
        return response(http::status::ok, read_file(web_ / "practice.html"),
                        "text/html; charset=utf-8");
      if (request.method() == http::verb::get && target == "/practice.js")
        return response(http::status::ok, read_file(web_ / "practice.js"),
                        "text/javascript; charset=utf-8");
      if (request.method() == http::verb::get && target == "/healthz")
        return response(http::status::ok, Json{{"status","ok"}, {"rules_version","1.2-local"}}.dump());
      if (request.method() == http::verb::post && target == "/v1/chat/completions") {
        if (request.chunked() || request.find(http::field::transfer_encoding) != request.end())
          return response(http::status::not_implemented, Json{{"error",{{"code","transfer_encoding_unsupported"}}}}.dump());
        const std::string content_type(request[http::field::content_type]);
        if (content_type.substr(0,16) != "application/json")
          return response(http::status::bad_request, Json{{"error",{{"code","invalid_content_type"}}}}.dump());
        const auto state_reply = forward(Request{http::verb::get, "/api/state", 11}, backend_);
        if (state_reply.result() != http::status::ok) return state_reply;
        const auto reply = gateway_.call(std::string(request[http::field::authorization]),
            Json::parse(request.body()), Json::parse(state_reply.body()));
        return response(static_cast<http::status>(reply.status), reply.body);
      }
      if (request.method() != http::verb::get && request.method() != http::verb::post)
        return response(http::status::method_not_allowed, Json{{"error", "只接受GET/POST"}}.dump());
      const std::string origin(request[http::field::origin]), host(request[http::field::host]);
      if (request.method() == http::verb::post && !origin.empty() && origin != "http://" + host &&
          origin != "https://" + host)
        return response(http::status::forbidden, Json{{"error", "拒绝跨域控制"}}.dump());
      if (request.method() == http::verb::post && target == "/api/map")
        return generate(request);
      if(request.method()==http::verb::post&&target=="/api/training/event-json") {
        const char* flag=std::getenv("RMTC_TRAINING_GATEWAY");
        const char* raw=std::getenv("ROS_DOMAIN_ID");const int domain=raw?std::stoi(raw):-1;
        const bool enabled=flag&&std::string(flag)=="1"&&domain%2==0&&
            ((domain>=100&&domain<=130)||(domain>=200&&domain<=230));
        if(!enabled)return response(http::status::forbidden,Json{{"error","training_endpoint_disabled"}}.dump());
        const auto current=forward(Request{http::verb::get,"/api/state",11},backend_);
        if(current.result()!=http::status::ok)return current;
        const auto value=Json::parse(request.body());
        const auto reply=rmtc::practice::training_json_completion(value,Json::parse(current.body()),enabled);
        if(reply.status==200){std::lock_guard<std::mutex> lock(mutex_);
          std::ofstream(data_/"training-json-audit.jsonl",std::ios::app)<<Json{{"session_id",value.at("session_id")},
            {"team",value.at("team")},{"source_tick",value.at("source_tick")},{"training_only",true},
            {"formal_ai_qualification",false}}.dump()<<'\n';}
        return response(static_cast<http::status>(reply.status),reply.body);
      }
      if (request.method() == http::verb::post && target == "/api/start") {
        // API调用方未传config时也沿用当前地图，彻底避免官方的缺省模板回退。
        auto body = Json::parse(request.body());
        if (!body.is_object())
          throw std::invalid_argument("请求体须为JSON对象");
        if (!body.contains("config")) {
          const auto state_response = forward(Request{http::verb::get, "/api/state", 11}, backend_);
          if (state_response.result() != http::status::ok)
            return state_response;
          const auto state = Json::parse(state_response.body());
          if (!state.at("settings").is_object())
            throw std::invalid_argument("请先生成或导入地图");
          const auto& current = state.at("settings");
          if (body.value("stage", current.at("stage").get<int>()) != current.at("stage").get<int>())
            throw std::invalid_argument("切换阶段请先生成对应阶段地图");
          body["stage"] = current.at("stage");
          body["config"] = current.at("config");
        }
        request.body() = body.dump();
      }
      auto result = forward(std::move(request), backend_);
      if (target == "/api/state" && result.result() == http::status::ok) {
        auto state = Json::parse(result.body());
        state["ai"] = gateway_.records(state);
        state["rules_version"] = "1.2-local";
        state["ai_tokens"] = Json::object();
        if (state.contains("settings") && state["settings"].is_object())
          for (const auto& robot : state["settings"].at("config").at("robots")) {
            const std::string seat = robot.at("team");
            state["ai_tokens"][seat] = state.value("session_id", "") + "-" + seat;
          }
        state["gateway"] = "http://127.0.0.1:" + std::to_string(port_) + "/v1/chat/completions";
        state["map_seed"] = nullptr;
        if (state.contains("settings") && state["settings"].is_object()) {
          const auto key = map_key(state["settings"].at("config"));
          std::lock_guard<std::mutex> lock(mutex_);
          const auto found = seeds_.find(key);
          if (found != seeds_.end())
            state["map_seed"] = found->second;
        }
        result.body() = state.dump();
        result.prepare_payload();
      }
      return result;
    } catch (const Json::exception&) {
      return response(http::status::bad_request,
                      Json{{"error", "JSON格式或必需字段不正确"}}.dump());
    } catch (const std::invalid_argument& error) {
      return response(http::status::bad_request, Json{{"error", error.what()}}.dump());
    } catch (const std::exception& error) {
      std::cerr << "practice web: " << error.what() << '\n';
      return response(http::status::bad_gateway, Json{{"error", error.what()}}.dump());
    }
  }

 private:
  /** @brief 将预览地图和元信息关联；有界缓存避免长时间测试无限累计内存。 */
  void remember(const Json& config, const Json& metadata) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (seeds_.size() >= 256)
      seeds_.erase(seeds_.begin());
    seeds_[map_key(config)] = web_metadata(metadata);
  }

  /** @brief 只生成并保存待部署配置，不调用start，不改变当前比赛和暂停状态。 */
  Response generate(const Request& request) {
    const auto body = Json::parse(request.body());
    if (!body.at("stage").is_number_integer())
      throw std::invalid_argument("stage须为整数1或2");
    const auto stage_value = body.at("stage").get<std::int64_t>();
    if (stage_value != 1 && stage_value != 2)
      throw std::invalid_argument("stage须为1或2");
    const auto stage = static_cast<int>(stage_value);
    if (!body.at("seed").is_string())
      throw std::invalid_argument("种子须为整数字符串");
    const auto generated = rmtc::practice::generate_practice_map(
        templates_.at(stage), stage, integer(body.at("seed").get<std::string>()));
    auto metadata = generated.metadata;
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    const auto stem = "web-stage" + std::to_string(stage) + "-seed" +
                      std::to_string(generated.seed) + "-" + std::to_string(stamp);
    const auto config_file = data_ / "generated-maps" / (stem + ".json");
    metadata["config_file"] = config_file.string();
    write_json(config_file, generated.config);
    write_json(data_ / "generated-maps" / (stem + ".metadata.json"), metadata);
    remember(generated.config, metadata);
    std::cout << "地图预览 stage=" << stage << " seed=" << generated.seed
              << " config=" << config_file << std::endl;
    return response(
        http::status::ok,
        Json{{"config", generated.config}, {"metadata", web_metadata(metadata)}}.dump());
  }

  Path web_, data_;
  unsigned short port_, backend_;
  std::map<int, Json> templates_;
  rmtc::practice::PracticeGateway gateway_;
  std::mutex mutex_;
  std::map<std::string, Json> seeds_;
};

/// 每次HTTP连接持有独立请求/响应；长时间裁判操作不会破坏并发状态轮询。
class Session : public std::enable_shared_from_this<Session> {
 public:
  /** @brief 接管已接收的回环连接，设置2MiB请求上限及自有服务引用。 */
  Session(tcp::socket socket, PracticeWeb& app) : stream_(std::move(socket)), app_(app) {
    parser_.body_limit(2 * 1024 * 1024);
  }

  /** @brief 限时读取单次请求、处理并回复；共享所有权确保异步操作期间对象存活。 */
  void run() {
    stream_.expires_after(10s);
    http::async_read(stream_, buffer_, parser_,
                     [self = shared_from_this()](beast::error_code error, std::size_t) {
                       if (error) {
                         if (error != http::error::body_limit) return;
                         self->reply_ = response(http::status::payload_too_large,
                             Json{{"error",{{"code","request_too_large"}}}}.dump());
                       } else self->reply_ = self->app_.handle(self->parser_.release());
                       self->stream_.expires_after(10s);
                       http::async_write(
                           self->stream_, self->reply_, [self](beast::error_code, std::size_t) {
                             beast::error_code ignored;
                             self->stream_.socket().shutdown(tcp::socket::shutdown_both, ignored);
                           });
                     });
  }

 private:
  beast::tcp_stream stream_;
  beast::flat_buffer buffer_;
  http::request_parser<http::string_body> parser_;
  Response reply_;
  PracticeWeb& app_;
};

/** @brief 持续接受连接；每个会话放到strand避免多线程执行同一连接的回调。 */
void accept_next(tcp::acceptor& acceptor, net::io_context& io, PracticeWeb& app) {
  acceptor.async_accept(net::make_strand(io), [&](beast::error_code error, tcp::socket socket) {
    if (!error)
      std::make_shared<Session>(std::move(socket), app)->run();
    if (acceptor.is_open())
      accept_next(acceptor, io, app);
  });
}

}  // namespace

/** @brief 校验命令行并运行回环网页服务；SIGINT/SIGTERM关闭自有服务，裁判由launch管理。 */
int main(int argc, char** argv) {
  try {
    std::map<std::string, std::string> args;
    for (int i = 1; i < argc; ++i) {
      const std::string name = argv[i];
      if (i + 1 >= argc)
        throw std::invalid_argument("参数缺少值：" + name);
      args[name] = argv[++i];
    }
    const auto port = integer(args.at("--port")), backend = integer(args.at("--backend-port"));
    if (port < 1 || port > 65535 || backend < 1 || backend > 65535 || port == backend)
      throw std::invalid_argument("网页端口与裁判端口须不同且在1..65535内");
    PracticeWeb app(args.at("--official-dir"), args.at("--web-dir"), args.at("--data-dir"),
                    args.at("--config"), static_cast<unsigned short>(port),
                    static_cast<unsigned short>(backend));
    net::io_context io(12);
    // 接收下一连接与信号关闭必须串行，避免多线程同时访问acceptor。
    tcp::acceptor acceptor(net::make_strand(io),
                           {net::ip::make_address("127.0.0.1"), static_cast<unsigned short>(port)});
    net::signal_set signals(acceptor.get_executor(), SIGINT, SIGTERM);
    signals.async_wait([&](beast::error_code, int) {
      acceptor.close();
      io.stop();
    });
    accept_next(acceptor, io, app);
    std::cout << "练习网页：http://127.0.0.1:" << port
              << "；第一阶段20×20、第二阶段24×24；-1随机，其余整数（含0）固定" << std::endl;
    std::vector<std::thread> workers;
    for (int i = 0; i < 11; ++i)
      workers.emplace_back([&] { io.run(); });
    io.run();
    for (auto& worker : workers)
      worker.join();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "练习网页启动失败：" << error.what() << '\n';
    return 1;
  }
}
