"""启动我们自己的C++选手控制节点，不启动裁判。

文件：contestant.launch.py
用途：统一设置ROS通信环境，并向connection_node传入席位、模式和总Tick。
默认A席位、resource模式、100步；mode:=stage2自动使用第二阶段策略和500步。
默认算法和权重统一由resource_config.hpp指定；launch参数留空时不覆盖C++配置。
resource_order:=nearest可切回旧版最近资源策略，便于比较折返距离。
resource_order:=weighted统一比较采集与探索：路程＋边缘－资源奖励－新增未知格奖励。
resource_order:=joint把采集与观察点放进同一条路线，是当前默认算法。
各策略参数在resource_config.hpp中分组；k_edge/k_value只调整Weighted。
修改C++参数后重新编译并重启选手端，不会实时改变已运行的程序。
第二阶段默认另起ai_node进程；主控不等待HTTP，第一阶段不启动AI。
strategy_params_file留空时，stage2加载config/stage2.yaml，stage2_planning加载config/stage2_planning.yaml；
要切换历史组，显式指定对应检查点parameters.yaml。第一阶段仍沿用C++默认权重。

维护说明：
    选手基础入口；mode决定策略分支，strategy_params_file决定配置来源，Node参数与环境配置共同形成有效设置。
    launch负责创建进程和传递参数，实际决策由C++节点完成。
    修改源码后先构建并source对应install；运行中节点不会因源文件改动自动更新。
"""

import math
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _start_contestant(context):
    """解析参数并创建主控节点，第二阶段可另起独立AI节点。

    context由launch提供；将action_ticks转成整数，两个权重转成浮点数，
    避免k_value:=2被ROS推断成整数而与C++ double参数类型冲突。
    ros_log_dir自动创建；log_file留空时只使用节点原有的终端/ROS日志，
    指定路径时创建父目录并启用节点自带的JSONL记录。
    此处不编译代码，运行的是最近一次colcon build安装的connection_node。
    """
    domain = int(LaunchConfiguration("domain_id").perform(context))
    mode = LaunchConfiguration("mode").perform(context)
    ticks_text = LaunchConfiguration("action_ticks").perform(context)
    action_ticks = int(ticks_text) if ticks_text else (500 if mode in ("stage2", "stage2_planning") else 100)
    if domain < 0 or action_ticks <= 0:
        raise ValueError("domain_id须非负，action_ticks须为正整数")
    overrides = {}
    for name in ("phase_enabled", "separate_goal_switch_bonus", "resource_race_enabled", "acquisition_curve_enabled", "temporal_policy_enabled", "purpose_enabled"):
        enabled = LaunchConfiguration(name).perform(context).strip()
        if enabled:
            overrides[name] = enabled == "true"
    resource_order = LaunchConfiguration("resource_order").perform(context)
    if resource_order:
        overrides["resource_order"] = resource_order
    for name in ("k_edge", "k_value"):
        text = LaunchConfiguration(name).perform(context).strip()
        if text:
            value = float(text)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{name}须为有限非负数")
            overrides[name] = value
    ros_log_dir = Path(LaunchConfiguration("ros_log_dir").perform(context)).expanduser().resolve()
    ros_log_dir.mkdir(parents=True, exist_ok=True)
    log_file = LaunchConfiguration("log_file").perform(context)
    if log_file:
        log_path = Path(log_file).expanduser().resolve()
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_file = str(log_path)

    strategy_file = LaunchConfiguration("strategy_params_file").perform(context).strip()
    # 第二阶段按mode选择配置；启用实验网络不代表已证实提分，显式文件用于复现实验。
    # C++权重结构保留通用默认值，历史参数文件不会被额外合并进新的实验开关。
    if not strategy_file and mode in ("stage2", "stage2_planning"):
        strategy_file = PathJoinSubstitution(
            [FindPackageShare("rmtc_connection"), "config",
             "stage2_planning.yaml" if mode == "stage2_planning" else "stage2.yaml"]
        ).perform(context)
    parameter_files = []
    if strategy_file:
        strategy_path = Path(strategy_file).expanduser().resolve()
        if not strategy_path.is_file():
            raise ValueError(f"找不到策略参数文件：{strategy_path}")
        parameter_files.append(str(strategy_path))
    actions = [
        Node(
            package="rmtc_connection",
            executable="connection_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                *parameter_files,
                {
                    "team": LaunchConfiguration("team").perform(context),
                    # 显式保持字符串，队名如123、true或含冒号时也不按YAML推断类型。
                    "team_name": ParameterValue(
                        LaunchConfiguration("team_name").perform(context), value_type=str
                    ),
                    "mode": mode,
                    **overrides,
                    "action_ticks": action_ticks,
                    "log_file": ParameterValue(log_file, value_type=str),
                }
            ],
            additional_env={
                "ROS_DOMAIN_ID": str(domain),
                "ROS_LOCALHOST_ONLY": "0",
                "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
                "ROS_LOG_DIR": str(ros_log_dir),
            },
            on_exit=[Shutdown(reason="选手程序已结束")],
        )
    ]
    if mode in ("stage2", "stage2_planning") and LaunchConfiguration("ai_enabled").perform(context) == "true":
        ai_log = LaunchConfiguration("ai_log_file").perform(context)
        ai_log_path = Path(ai_log).expanduser().resolve() if ai_log else (
            ros_log_dir.parent / ("ai-team_" + LaunchConfiguration("team").perform(context) + ".jsonl")
        )
        ai_log_path.parent.mkdir(parents=True, exist_ok=True)
        params = {
            "team": LaunchConfiguration("team").perform(context),
            "action_ticks": action_ticks,
            "ai_log_file": ParameterValue(str(ai_log_path), value_type=str),
        }
        for name in ("ai_url", "ai_model", "ai_service_mode", "ai_token_env"):
            params[name] = ParameterValue(LaunchConfiguration(name).perform(context), value_type=str)
        for name in ("ai_timeout_ms", "ai_max_calls", "ai_cooldown_ms", "ai_max_tokens"):
            params[name] = int(LaunchConfiguration(name).perform(context))
        for name in ("ai_json_mode", "ai_disable_thinking"):
            params[name] = LaunchConfiguration(name).perform(context) == "true"
        actions.append(
            Node(
                package="rmtc_connection", executable="ai_node", output="screen", emulate_tty=True,
                parameters=[params],
                additional_env={
                    "ROS_DOMAIN_ID": str(domain), "ROS_LOCALHOST_ONLY": "0",
                    "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp", "ROS_LOG_DIR": str(ros_log_dir),
                },
                # AI退出不关闭主控；主控继续按本地策略决策。
            )
        )
    return actions


def generate_launch_description():
    """声明选手节点参数，返回由ROS 2执行的启动描述。

    team与实际分配席位一致，domain_id与裁判公开域一致。
    action_ticks留空随模式选择100/500；自定义长度时必须与裁判保持一致。
    参数通过命令行的参数名:=值覆盖，无须改动C++源码。
    """
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "project_dir",
                default_value="/home/jon/robot/competition",
                description="项目根目录，用于默认日志位置",
            ),
            DeclareLaunchArgument(
                "team",
                default_value="A",
                choices=["A", "B", "C", "D"],
                description="本程序控制的席位",
            ),
            DeclareLaunchArgument(
                "team_name", default_value="奶龙做什么都队", description="显示队名"
            ),
            DeclareLaunchArgument(
                "mode",
                default_value="resource",
                choices=["wait", "probe", "resource", "stage2", "stage2_planning"],
                description="resource第一阶段；stage2第二阶段；stage2_planning预测规划；probe直行；wait等待",
            ),
            DeclareLaunchArgument(
                "resource_order",
                default_value="",
                choices=["", "tour", "nearest", "weighted", "joint"],
                description="留空采用resource_config.hpp默认；可选tour、nearest、weighted、joint",
            ),
            DeclareLaunchArgument(
                "k_edge", default_value="", description="留空采用C++默认；weighted边缘距离权重"
            ),
            DeclareLaunchArgument(
                "k_value",
                default_value="",
                description="留空采用C++默认；weighted每10点价值抵扣代价",
            ),
            DeclareLaunchArgument(
                "action_ticks", default_value="", description="留空随模式选择100/500；须与裁判一致"
            ),
            DeclareLaunchArgument(
                "phase_enabled", default_value="", choices=["", "true", "false"],
                description="第二阶段显式分期实验开关；留空采用C++配置，false可作旧策略对照",
            ),
            DeclareLaunchArgument(
                "separate_goal_switch_bonus", default_value="", choices=["", "true", "false"],
                description="目标惯性只用于选目标；留空采用C++配置",
            ),
            DeclareLaunchArgument(
                "resource_race_enabled", default_value="", choices=["", "true", "false"],
                description="整局一次性资源优先与对手到达期限规划；留空读取C++配置",
            ),
            DeclareLaunchArgument(
                "acquisition_curve_enabled", default_value="", choices=["", "true", "false"],
                description="固定采集偏好与探索时间衰减；留空读取C++配置",
            ),
            DeclareLaunchArgument(
                "temporal_policy_enabled", default_value="", choices=["", "true", "false"],
                description="按参数含义独立做前后期拟合；留空读取C++配置",
            ),
            DeclareLaunchArgument("purpose_enabled", default_value="", choices=["", "true", "false"],
                                  description="可选覆盖资源/战斗目的与短期截击开关"),
            DeclareLaunchArgument("strategy_params_file", default_value="",
                                  description="策略参数YAML；第二阶段留空加载config/stage2.yaml，显式指定可复现历史组"),
            DeclareLaunchArgument(
                "domain_id", default_value="27", description="与裁判公开ROS域保持一致"
            ),
            DeclareLaunchArgument(
                "log_file", default_value="", description="可选的本程序JSONL日志路径"
            ),
            DeclareLaunchArgument("ai_enabled", default_value="true", choices=["true", "false"],
                                  description="第二阶段是否另起AI进程；第一阶段始终不启动"),
            DeclareLaunchArgument("ai_url", default_value="http://127.0.0.1:8080/v1/chat/completions",
                                  description="完整AI网关聊天地址"),
            DeclareLaunchArgument("ai_service_mode", default_value="mock", choices=["mock", "real", "official"],
                                  description="本地模拟/本地真实代理/正式统一网关"),
            DeclareLaunchArgument("ai_model", default_value="", description="留空使用网关默认模型"),
            DeclareLaunchArgument("ai_max_tokens", default_value="4096", description="模型输出token上限，按官方公布限制设置"),
            DeclareLaunchArgument("ai_json_mode", default_value="false", choices=["true", "false"],
                                  description="上游支持时要求JSON对象输出"),
            DeclareLaunchArgument("ai_disable_thinking", default_value="true", choices=["true", "false"],
                                  description="DeepSeek扩展：关闭思考模式，其他服务默认不发送"),
            DeclareLaunchArgument("ai_token_env", default_value="RMTC_AI_TOKEN",
                                  description="正式令牌环境变量名；本地练习固定使用practice-席位"),
            DeclareLaunchArgument("ai_timeout_ms", default_value="10000", description="AI进程HTTP超时毫秒"),
            DeclareLaunchArgument("ai_max_calls", default_value="40", description="本程序每局请求预算，不代表官方配额"),
            DeclareLaunchArgument("ai_cooldown_ms", default_value="500", description="AI请求最小间隔毫秒"),
            DeclareLaunchArgument("ai_log_file", default_value="", description="默认在ROS日志目录旁保存AI JSONL"),
            DeclareLaunchArgument(
                "ros_log_dir",
                default_value=PathJoinSubstitution(
                    [LaunchConfiguration("project_dir"), "runtime", "ros"]
                ),
                description="ROS日志目录",
            ),
            OpaqueFunction(function=_start_contestant),
        ]
    )
