# 构建链：colcon发现本包，ament/CMake配置目标，编译器生成对象后链接各节点。
# 静态策略库不单独启动进程；只有add_executable定义的程序由launch/run启动。
# install目录是ros2按包名查找的运行产物，源码/构建/安装位置需保持对应。
# 测试工具与真实API审计分开，不能在普通CTest中隐式花费模型额度。
# 文件：CMakeLists.txt
# 用途：配置本队C++ ROS 2包的依赖、编译选项、安装位置及独立测试。
# src按core/planning/stage1/stage2/ai/referee/visualization分组；公共头文件在include中。
# 由colcon调用本文件；compile_commands.json同时供VS Code读取真实编译参数。
cmake_minimum_required(VERSION 3.16)
project(rmtc_connection)

# 让编辑器使用与实际构建一致的头文件路径、宏和编译选项。
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)
find_package(libcurl_vendor REQUIRED)
find_package(CURL REQUIRED)

# 裁判部署专用地图生成工具；不链接选手策略，也不创建ROS节点。
add_library(rmtc_practice_maps STATIC src/referee/map_generator.cpp)
target_include_directories(rmtc_practice_maps PUBLIC include)
target_compile_features(rmtc_practice_maps PUBLIC cxx_std_17)
target_compile_options(rmtc_practice_maps PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(rmtc_practice_maps PUBLIC nlohmann_json::nlohmann_json)
add_executable(generate_practice_map src/referee/practice_map_main.cpp)
target_link_libraries(generate_practice_map rmtc_practice_maps)
target_compile_options(generate_practice_map PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS generate_practice_map DESTINATION lib/${PROJECT_NAME})

# 自有裁判扩展：地图、网页、AI代理与计分各自独立；默认使用本地v1.2副本。
add_executable(practice_web src/referee/practice_web_main.cpp src/referee/practice_gateway.cpp)
target_link_libraries(practice_web rmtc_practice_maps Boost::system Threads::Threads CURL::libcurl)
target_compile_options(practice_web PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS practice_web DESTINATION lib/${PROJECT_NAME})
add_executable(practice_scoring src/referee/practice_scoring_main.cpp)
target_include_directories(practice_scoring PRIVATE include)
target_compile_features(practice_scoring PRIVATE cxx_std_17)
ament_target_dependencies(practice_scoring rclcpp std_msgs)
target_link_libraries(practice_scoring nlohmann_json::nlohmann_json)
target_compile_options(practice_scoring PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS practice_scoring DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY web/ DESTINATION share/${PROJECT_NAME}/web)

# 独立标准C++ A*库：参考本机ws11的二维实现，适配比赛四方向网格。
# 由资源策略显式链接；库本身不创建节点，独立RViz建图不调用规划器。
add_library(rmtc_astar STATIC src/planning/astar.cpp)
target_include_directories(rmtc_astar PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(rmtc_astar PUBLIC cxx_std_17)
target_compile_options(rmtc_astar PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS rmtc_astar EXPORT rmtc_astar_targets ARCHIVE DESTINATION lib)
ament_export_targets(rmtc_astar_targets HAS_LIBRARY_TARGET)

# 完整公开数据解析独立编译；控制节点复用解析，不启动额外PublicReceiver订阅。
add_library(rmtc_public_interface STATIC src/core/public_interface.cpp)
target_include_directories(rmtc_public_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(rmtc_public_interface PUBLIC cxx_std_17)
target_compile_options(rmtc_public_interface PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(rmtc_public_interface rclcpp std_msgs)
target_link_libraries(rmtc_public_interface nlohmann_json::nlohmann_json)
install(TARGETS rmtc_public_interface ARCHIVE DESTINATION lib)

# 资源决策与ROS收发分离；旧策略使用A*，Joint使用公开障碍上的BFS距离估算。
add_library(rmtc_resource_strategy STATIC
  src/stage1/resource_strategy.cpp
  src/planning/resource_route.cpp
  src/planning/weighted_resource.cpp
  src/planning/frontier_exploration.cpp
  src/planning/resource_peek.cpp
  src/planning/coverage_route.cpp)
target_link_libraries(rmtc_resource_strategy rmtc_public_interface rmtc_astar)
target_compile_options(rmtc_resource_strategy PRIVATE -Wall -Wextra -Wpedantic)

# 第二阶段策略复用第一阶段纯规划库；参数与场次缓存各自独立。
add_library(rmtc_stage2_strategy STATIC src/stage2/stage2_strategy.cpp src/stage2/stage2_neural.cpp src/stage2/stage2_joint_planner.cpp
  src/stage2/stage2_events.cpp src/stage2/stage2_rollout.cpp src/stage2/stage2_collection_search.cpp
  src/stage2/stage2_resource_race.cpp src/stage2/stage2_planning.cpp src/stage2/stage2_collection_forecast.cpp
  src/stage2/stage2_collection_neural.cpp)
target_link_libraries(rmtc_stage2_strategy rmtc_resource_strategy rmtc_public_interface rmtc_astar)
target_compile_options(rmtc_stage2_strategy PRIVATE -Wall -Wextra -Wpedantic)

# 主控只链接纯结构化校验；HTTP库和libcurl只进入独立ai_node进程。
add_library(rmtc_ai_protocol STATIC src/ai/ai_protocol.cpp src/ai/ai_event_protocol.cpp)
target_link_libraries(rmtc_ai_protocol rmtc_stage2_strategy)
target_compile_options(rmtc_ai_protocol PRIVATE -Wall -Wextra -Wpedantic)
add_library(rmtc_ai_http STATIC src/ai/ai_client.cpp)
target_link_libraries(rmtc_ai_http rmtc_ai_protocol CURL::libcurl)
target_compile_options(rmtc_ai_http PRIVATE -Wall -Wextra -Wpedantic)
add_executable(ai_node src/ai/ai_main.cpp)
target_link_libraries(ai_node rmtc_ai_http)
target_compile_options(ai_node PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS ai_node DESTINATION lib/${PROJECT_NAME})

add_executable(connection_node src/core/connection_main.cpp src/core/connection_node.cpp)
target_include_directories(connection_node PRIVATE include)
target_compile_features(connection_node PRIVATE cxx_std_17)
target_compile_options(connection_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(connection_node rclcpp std_msgs)
target_link_libraries(connection_node rmtc_resource_strategy rmtc_stage2_strategy rmtc_ai_protocol)
install(TARGETS connection_node DESTINATION lib/${PROJECT_NAME})

# 独立进程只负责显示；复用AI结构校验与事件地形，不运行规划器、不发动作。
add_library(rmtc_mapping STATIC src/visualization/mapping_node.cpp src/visualization/event_markers.cpp)
target_link_libraries(rmtc_mapping rmtc_public_interface rmtc_ai_protocol)
target_compile_options(rmtc_mapping PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(rmtc_mapping nav_msgs geometry_msgs visualization_msgs tf2_ros)
add_executable(mapping_node src/visualization/mapping_main.cpp)
target_link_libraries(mapping_node rmtc_mapping)
target_compile_options(mapping_node PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS mapping_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
# 安装Python launch和RViz配置，供ros2 launch按包名查找。
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "__pycache__" EXCLUDE)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

include(CTest)
if(BUILD_TESTING)
  find_package(Eigen3 REQUIRED NO_MODULE)
  add_executable(train_stage2_neural tools/train_stage2_neural.cpp)
  add_executable(train_resource_presence tools/train_resource_presence.cpp)
  target_link_libraries(train_resource_presence rmtc_stage2_strategy)
  target_compile_options(train_resource_presence PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(train_stage2_neural rmtc_stage2_strategy Eigen3::Eigen)
  target_compile_options(train_stage2_neural PRIVATE -Wall -Wextra -Wpedantic)
  add_executable(neural_matches tools/neural_matches.cpp)
  target_link_libraries(neural_matches rmtc_practice_maps CURL::libcurl Threads::Threads)
  target_compile_options(neural_matches PRIVATE -Wall -Wextra -Wpedantic)
  # 注册普通离线测试，统一警告选项；额外参数为所需库，不改变各库原有依赖。
  function(rmtc_add_test target source)
    add_executable(${target} ${source})
    target_link_libraries(${target} ${ARGN})
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME ${target} COMMAND ${target})
  endfunction()
  rmtc_add_test(stage2_neural_test tests/stage2_neural_test.cpp rmtc_stage2_strategy Eigen3::Eigen)
  rmtc_add_test(stage2_neural_policy_test tests/stage2_neural_policy_test.cpp rmtc_stage2_strategy)
  rmtc_add_test(stage2_planning_test tests/stage2_planning_test.cpp rmtc_stage2_strategy)

  rmtc_add_test(stage2_collection_transfer_test tests/stage2_collection_transfer_test.cpp
    rmtc_stage2_strategy)

  # 离线参数搜索工具，显式运行；不把整批调参加入日常ctest，也不连接比赛ROS域。
  add_executable(resource_tuning tools/resource_tuning.cpp)
  target_link_libraries(resource_tuning rmtc_resource_strategy rmtc_practice_maps)
  target_compile_options(resource_tuning PRIVATE -Wall -Wextra -Wpedantic)
  # 完整信息下的最短路仅用于审计时限上限，绝不接入选手节点。
  add_executable(resource_oracle tools/resource_oracle.cpp)
  target_link_libraries(resource_oracle rmtc_resource_strategy rmtc_practice_maps)
  target_compile_options(resource_oracle PRIVATE -Wall -Wextra -Wpedantic)

  # 仅回放本队公开感知，诊断真实对局的后半程决策；固定轨迹不能作为闭环胜率。
  add_executable(stage2_replay tools/stage2_replay.cpp)
  target_link_libraries(stage2_replay rmtc_stage2_strategy)
  target_compile_options(stage2_replay PRIVATE -Wall -Wextra -Wpedantic)
  # 训练专用公告缓存；不安装、不注册为CTest，不与正式AI调用资格混合。
  add_executable(ai_cache_replay tools/ai_cache_replay.cpp)
  target_link_libraries(ai_cache_replay rmtc_ai_protocol CURL::libcurl)
  ament_target_dependencies(ai_cache_replay rclcpp std_msgs)
  target_compile_options(ai_cache_replay PRIVATE -Wall -Wextra -Wpedantic)

  # 离线CUDA参数筛选：动态加载系统驱动，不给正式选手增加CUDA依赖。
  add_executable(gpu_parameter_screen tools/gpu_parameter_screen.cpp)
  target_link_libraries(gpu_parameter_screen ${CMAKE_DL_LIBS})
  target_compile_options(gpu_parameter_screen PRIVATE -Wall -Wextra -Wpedantic)

  rmtc_add_test(coverage_route_test tests/coverage_route_test.cpp
    rmtc_resource_strategy)

  rmtc_add_test(frontier_exploration_test tests/frontier_exploration_test.cpp
    rmtc_resource_strategy rmtc_practice_maps)

  rmtc_add_test(map_generator_test tests/map_generator_test.cpp
    rmtc_practice_maps)
  rmtc_add_test(practice_rules_test tests/practice_rules_test.cpp
    nlohmann_json::nlohmann_json)
  target_include_directories(practice_rules_test PRIVATE include)
  target_compile_features(practice_rules_test PRIVATE cxx_std_17)
  add_executable(practice_gateway_test tests/practice_gateway_test.cpp src/referee/practice_gateway.cpp)
  target_include_directories(practice_gateway_test PRIVATE include)
  target_compile_features(practice_gateway_test PRIVATE cxx_std_17)
  target_compile_options(practice_gateway_test PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(practice_gateway_test nlohmann_json::nlohmann_json CURL::libcurl)
  add_test(NAME practice_gateway_test COMMAND practice_gateway_test)

  rmtc_add_test(weighted_resource_test tests/weighted_resource_test.cpp
    rmtc_resource_strategy)

  rmtc_add_test(resource_route_test tests/resource_route_test.cpp
    rmtc_resource_strategy)

  rmtc_add_test(resource_strategy_test tests/resource_strategy_test.cpp
    rmtc_resource_strategy)

  rmtc_add_test(stage2_duel_stalemate_test tests/stage2_duel_stalemate_test.cpp rmtc_stage2_strategy)
  rmtc_add_test(stage2_strategy_test tests/stage2_strategy_test.cpp
    rmtc_stage2_strategy)
  rmtc_add_test(ai_protocol_test tests/ai_protocol_test.cpp rmtc_ai_protocol)
  rmtc_add_test(stage2_events_test tests/stage2_events_test.cpp rmtc_ai_protocol)
  rmtc_add_test(stage2_frontier_value_test tests/stage2_frontier_value_test.cpp rmtc_public_interface)
  rmtc_add_test(stage2_exploration_value_test tests/stage2_exploration_value_test.cpp rmtc_public_interface)
  rmtc_add_test(stage2_resource_sweep_test tests/stage2_resource_sweep_test.cpp rmtc_stage2_strategy)
  rmtc_add_test(stage2_collection_search_test tests/stage2_collection_search_test.cpp rmtc_stage2_strategy)
  rmtc_add_test(stage2_battle_evidence_test tests/stage2_battle_evidence_test.cpp rmtc_stage2_strategy)
  rmtc_add_test(stage2_navigation_progress_test tests/stage2_navigation_progress_test.cpp rmtc_public_interface)
  rmtc_add_test(stage2_enemy_belief_test tests/stage2_enemy_belief_test.cpp rmtc_public_interface)
  rmtc_add_test(stage2_interference_test tests/stage2_interference_test.cpp rmtc_stage2_strategy)
  rmtc_add_test(stage2_resource_race_test tests/stage2_resource_race_test.cpp rmtc_stage2_strategy)
  rmtc_add_test(stage2_acquisition_curve_test tests/stage2_acquisition_curve_test.cpp rmtc_stage2_strategy)
  rmtc_add_test(stage2_purpose_test tests/stage2_purpose_test.cpp rmtc_stage2_strategy)
  rmtc_add_test(stage2_battle_curve_test tests/stage2_battle_curve_test.cpp rmtc_stage2_strategy)
  # 同一前瞻实现另编译测试入口，检查固定联合动作的结算；正式库不包含测试接口。
  add_executable(stage2_rollout_test tests/stage2_rollout_test.cpp src/stage2/stage2_rollout.cpp src/stage2/stage2_events.cpp)
  target_compile_definitions(stage2_rollout_test PRIVATE RMTC_ROLLOUT_TESTING)
  target_link_libraries(stage2_rollout_test rmtc_public_interface rmtc_astar)
  target_compile_options(stage2_rollout_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME stage2_rollout_test COMMAND stage2_rollout_test)

  # 手动广播审计含真实网络调用，必须显式--live；不放入自动CTest。
  add_executable(ai_broadcast_check tools/ai_broadcast_check.cpp)
  target_link_libraries(ai_broadcast_check rmtc_ai_http)
  target_compile_options(ai_broadcast_check PRIVATE -Wall -Wextra -Wpedantic)

  rmtc_add_test(astar_test tests/astar_test.cpp
    rmtc_astar)

  rmtc_add_test(protocol_test tests/protocol_test.cpp
    nlohmann_json::nlohmann_json)
  target_include_directories(protocol_test PRIVATE include)
  target_compile_features(protocol_test PRIVATE cxx_std_17)

  rmtc_add_test(public_interface_test tests/public_interface_test.cpp
    rmtc_public_interface)

  rmtc_add_test(map_memory_test tests/map_memory_test.cpp
    rmtc_public_interface)

  rmtc_add_test(mapping_node_test tests/mapping_node_test.cpp
    rmtc_mapping)
  rmtc_add_test(mapping_events_test tests/mapping_events_test.cpp rmtc_mapping)
  # 测试在独立ROS域内发布构造数据，禁止向实际比赛域27注入感知。
  set_tests_properties(mapping_node_test PROPERTIES TIMEOUT 20
    ENVIRONMENT "ROS_DOMAIN_ID=89;ROS_LOCALHOST_ONLY=1;RMW_IMPLEMENTATION=rmw_fastrtps_cpp")
  set_tests_properties(mapping_events_test PROPERTIES TIMEOUT 30
    ENVIRONMENT "ROS_DOMAIN_ID=90;ROS_LOCALHOST_ONLY=1;RMW_IMPLEMENTATION=rmw_fastrtps_cpp")
endif()
ament_package()
