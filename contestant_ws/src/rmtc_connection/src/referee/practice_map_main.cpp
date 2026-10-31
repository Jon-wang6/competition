/**
 * @file practice_map_main.cpp
 * @brief C++练习地图生成工具：读取官方模板，输出配置文件、种子记录和JSON摘要。
 *
 * 由referee.launch.py调用，不随选手程序启动。不会覆盖已有输出文件，
 * 官方模板只读；metadata单独保存，不加入公开感知或参赛程序参数。
 */
#include "rmtc_connection/map_generator.hpp"
#include <charconv>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

/** @brief 严格解析64位整数，拒绝尾随字符、小数或越界种子。 */
std::int64_t integer(const std::string& text) {
  std::int64_t value = 0;
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
  if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size())
    throw std::invalid_argument("invalid 64-bit integer: " + text);
  return value;
}

/** @brief 写入一个新的JSON文件；已有文件拒绝覆盖，失败通过异常报告。 */
void write_json(const std::filesystem::path& path, const nlohmann::json& value) {
  if (std::filesystem::exists(path))
    throw std::runtime_error("refusing to overwrite: " + path.string());
  if (!path.parent_path().empty())
    std::filesystem::create_directories(path.parent_path());
  std::ofstream stream;
  stream.exceptions(std::ios::failbit | std::ios::badbit);
  stream.open(path);
  stream << value.dump(2) << '\n';
  stream.close();
}

}  // namespace

/** @brief 解析命令行、生成并保存地图；成功输出机器可读摘要，错误输出stderr并返回1。 */
int main(int argc, char** argv) {
  try {
    std::filesystem::path source, output;
    std::int64_t stage = 1, seed = 0;
    for (int i = 1; i < argc; ++i) {
      const std::string option = argv[i];
      if (option == "--help") {
        std::cout << "generate_practice_map --template FILE --output FILE [--stage 1|2] [--seed "
                     "INTEGER]\n";
        return 0;
      }
      if (i + 1 >= argc)
        throw std::invalid_argument("missing value for " + option);
      const std::string value = argv[++i];
      if (option == "--template")
        source = value;
      else if (option == "--output")
        output = value;
      else if (option == "--stage")
        stage = integer(value);
      else if (option == "--seed")
        seed = integer(value);
      else
        throw std::invalid_argument("unknown option " + option);
    }
    if (source.empty() || output.empty() || (stage != 1 && stage != 2))
      throw std::invalid_argument("template/output required; stage must be 1 or 2");
    std::ifstream stream(source);
    if (!stream)
      throw std::runtime_error("cannot read official template " + source.string());
    nlohmann::json base;
    stream >> base;
    const auto result = rmtc::practice::generate_practice_map(base, static_cast<int>(stage), seed);
    auto metadata = result.metadata;
    auto metadata_path = output;
    metadata_path.replace_extension("metadata.json");
    if (std::filesystem::exists(output) || std::filesystem::exists(metadata_path))
      throw std::runtime_error("output or metadata already exists");
    metadata["template"] = std::filesystem::absolute(source).string();
    metadata["config_file"] = std::filesystem::absolute(output).string();
    metadata["metadata_file"] = std::filesystem::absolute(metadata_path).string();
    write_json(output, result.config);
    write_json(metadata_path, metadata);
    std::cout << metadata.dump() << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
