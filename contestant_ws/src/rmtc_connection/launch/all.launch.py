"""一键更新并启动裁判、选手、建图及RViz。

文件：all.launch.py
用途：先在/home/jon/robot/competition根目录编译本队包，成功后再启动三个现有launch。
update默认true；update:=false跳过编译。源码入口也能直接ros2 launch绝对路径运行，
无需先安装这个新launch。每个子进程重新加载根目录install/local_setup.bash，避免
旧终端或ament索引缓存选中contestant_ws/install中的旧程序。只负责启动编排，
不修改官方裁判、不修改策略参数、不自动开始比赛、不自动终止其他已运行程序。

维护说明：
    本地全套编排入口；更新流程与子launch参数在此汇合，切勿与单独启动的同席位主控重复运行。
    launch负责创建进程和传递参数，实际决策由C++节点完成。
    修改源码后先构建并source对应install；运行中节点不会因源文件改动自动更新。
"""

import os
from pathlib import Path
import socket

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


# shell脚本文本固定；项目路径及ROS参数以独立argv传入，不拼接进shell代码。
_BUILD_SCRIPT = """set -e
source /opt/ros/humble/setup.bash
exec colcon build --packages-select rmtc_connection
"""
_RUN_SCRIPT = """set -e
source /opt/ros/humble/setup.bash
source "$1/install/local_setup.bash"
expected="$1/install/rmtc_connection"
actual="$(ros2 pkg prefix rmtc_connection)"
if [ "$actual" != "$expected" ]; then
    echo "启动停止：实际包路径为 $actual，预期为 $expected" >&2
    exit 2
fi
echo "使用根目录安装版本：$actual"
shift
exec ros2 launch rmtc_connection "$@"
"""


def _value(context, name):
    """读取launch参数的最终字符串值，保留空字符串作为子launch使用C++默认值的标记。"""
    return LaunchConfiguration(name).perform(context)


def _clean_environment(context):
    """保留launch设置的上游环境，移除旧工作区前缀；不输出环境中的密钥。"""
    environment = dict(context.environment)
    for name in ("AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH"):
        environment.pop(name, None)
    return environment


def _check_conflicts(project, domain, port):
    """检查网页端口及同域本项目旧节点；仅报告冲突，不杀死用户的进程或比赛。"""
    with socket.socket() as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            probe.bind(("127.0.0.1", port))
        except OSError as error:
            raise RuntimeError(
                f"网页端口{port}已占用，请先在旧裁判端终端按Ctrl+C，或指定其他port。"
            ) from error
    conflicts = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdecimal():
            continue
        try:
            command = (entry / "cmdline").read_bytes().split(b"\0")
            executable = Path(os.fsdecode(command[0]))
            if executable.name not in (
                "connection_node",
                "ai_node",
                "mapping_node",
            ) or not executable.is_relative_to(project):
                continue
            # 只读取ROS域字段，不输出或保存其他进程环境（其中可能含令牌）。
            process_domain = next(
                (
                    item.split(b"=", 1)[1].decode()
                    for item in (entry / "environ").read_bytes().split(b"\0")
                    if item.startswith(b"ROS_DOMAIN_ID=")
                ),
                "0",
            )
            if process_domain == str(domain):
                conflicts.append(f"{executable.name}(PID {entry.name})")
        except (OSError, ValueError, IndexError):
            continue
    if conflicts:
        raise RuntimeError(
            "同一ROS域仍有旧节点：" + ", ".join(conflicts) + "；请先在旧选手端/RViz终端按Ctrl+C。"
        )


def _start_stack(context):
    """编译成功后从根目录安装位置启动三个原有launch；任一退出时统一关闭整套进程。"""
    project = Path(_value(context, "project_dir")).expanduser().resolve()
    prefix = project / "install" / "rmtc_connection"
    required = [project / "install" / "local_setup.bash"]
    required += [
        prefix / "lib" / "rmtc_connection" / name
        for name in ("connection_node", "ai_node", "mapping_node", "practice_web", "practice_scoring", "generate_practice_map")
    ]
    required += [
        prefix / "share" / "rmtc_connection" / "launch" / name
        for name in ("referee.launch.py", "contestant.launch.py", "rviz.launch.py")
    ]
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise RuntimeError("根目录安装不完整，请使用update:=true重新编译：" + ", ".join(missing))

    stage = int(_value(context, "stage"))
    total_ticks = _value(context, "max_ticks") or ("100" if stage == 1 else "500")
    common = {
        "project_dir": str(project),
        "domain_id": _value(context, "domain_id"),
        "ros_log_dir": _value(context, "ros_log_dir"),
    }
    referee = {
        **common,
        **{
            name: _value(context, name)
            for name in (
                "stage",
                "seed",
                "port",
                "bots",
                "internal_domain_id",
                "tick_ms",
                "config_file",
                "ai_mode",
                "referee_dir",
                "data_dir",
            )
        },
        "max_ticks": total_ticks,
    }
    contestant = {
        **common,
        **{
            name: _value(context, name)
            for name in (
                "team",
                "team_name",
                "mode",
                "resource_order",
                "k_edge",
                "k_value",
                "log_file",
            )
        },
        "action_ticks": total_ticks,
    }
    contestant["mode"] = _value(context, "mode") or ("stage2" if stage == 2 else "resource")
    contestant["phase_enabled"] = _value(context, "phase_enabled")
    contestant["separate_goal_switch_bonus"] = _value(context, "separate_goal_switch_bonus")
    contestant["resource_race_enabled"] = _value(context, "resource_race_enabled")
    contestant["acquisition_curve_enabled"] = _value(context, "acquisition_curve_enabled")
    contestant["temporal_policy_enabled"] = _value(context, "temporal_policy_enabled")
    contestant["purpose_enabled"] = _value(context, "purpose_enabled")
    contestant["strategy_params_file"] = _value(context, "strategy_params_file")
    contestant.update({
        name: _value(context, name) for name in (
            "ai_enabled", "ai_model", "ai_token_env", "ai_timeout_ms", "ai_max_calls",
            "ai_cooldown_ms", "ai_log_file",
            "ai_json_mode", "ai_disable_thinking",
        )
    })
    contestant["ai_url"] = _value(context, "ai_url") or (
        "http://127.0.0.1:" + _value(context, "port") + "/v1/chat/completions"
    )
    contestant["ai_service_mode"] = _value(context, "ai_service_mode") or _value(context, "ai_mode")
    visualization = {
        **common,
        "action_ticks": total_ticks,
        "team": _value(context, "team"),
        "use_rviz": _value(context, "use_rviz"),
    }
    get_logger("rmtc.all").info(
        f"启动根目录版本：{prefix}；stage={stage}；总Tick={total_ticks}；裁判与选手长度自动同步。"
    )
    if stage == 2:
        ai_status = "本队AI进程已关闭。"
        if _value(context, "ai_enabled") == "true":
            detail = (
                "模拟接口只验证通信，不解释广播、任务不计分；第二阶段实得总分上限45。"
                if contestant["ai_service_mode"] == "mock"
                else "独立AI进程调用真实模型，主控不等待HTTP。"
            )
            ai_status = f"本队AI模式：{contestant['ai_service_mode']}；{detail}"
        get_logger("rmtc.all").info(ai_status)
    actions = []
    for filename, arguments in (
        ("referee.launch.py", referee),
        ("contestant.launch.py", contestant),
        ("rviz.launch.py", visualization),
    ):
        # 空参数不覆盖子launch默认；例如不传resource_order时直接读取最新C++配置。
        command = [
            "bash",
            "--noprofile",
            "--norc",
            "-c",
            _RUN_SCRIPT,
            "rmtc-all",
            str(project),
            filename,
        ]
        command += [f"{name}:={value}" for name, value in arguments.items() if value != ""]
        actions.append(
            ExecuteProcess(
                cmd=command,
                cwd=str(project),
                env=_clean_environment(context),
                output="screen",
                emulate_tty=True,
                sigterm_timeout="15",
                sigkill_timeout="5",
                on_exit=[Shutdown(reason=f"{filename}已退出，关闭整套练习程序")],
            )
        )
    return actions


def _build_finished(event, context):
    """只允许编译成功后启动；编译失败抛错退出，Ctrl+C取消时不再创建子进程。"""
    if context.is_shutdown:
        return []
    if event.returncode != 0:
        raise RuntimeError(
            f"编译失败（退出码{event.returncode}），已停止启动；请修复上方编译错误，未运行旧程序。"
        )
    get_logger("rmtc.all").info("更新编译完成，开始启动裁判、选手和RViz。")
    return _start_stack(context)


def _prepare(context):
    """统一校验参数与旧进程冲突，并异步编译；更新过程中可随时Ctrl+C取消。"""
    project = Path(_value(context, "project_dir")).expanduser().resolve()
    if not (project / "contestant_ws" / "src" / "rmtc_connection" / "package.xml").is_file():
        raise RuntimeError("找不到项目源码，请检查project_dir：" + str(project))
    stage, domain = int(_value(context, "stage")), int(_value(context, "domain_id"))
    internal_text = _value(context, "internal_domain_id")
    internal_domain = int(internal_text) if internal_text else domain + 1
    port, seed = int(_value(context, "port")), int(_value(context, "seed"))
    ticks = _value(context, "max_ticks")
    period = _value(context, "tick_ms")
    if not 0 <= domain <= 232 or not 0 <= internal_domain <= 232 or domain == internal_domain:
        raise ValueError("公开ROS域与内部域须为不同的0..232整数")
    if not 1 <= port <= 65535 or not -(1 << 63) <= seed < (1 << 63):
        raise ValueError("port须为1..65535；seed须为64位整数")
    if ticks and not 1 <= int(ticks) <= 5000:
        raise ValueError("max_ticks须为1..5000")
    if period and not 50 <= int(period) <= 5000:
        raise ValueError("tick_ms须为50..5000毫秒")
    if stage == 1 and _value(context, "team") != "A":
        raise ValueError("第一阶段只使用A席位")
    if _value(context, "team") in _value(context, "bots").split(","):
        raise ValueError("选手席位不能同时设置为陪练")
    _check_conflicts(project, domain, port)
    if _value(context, "update") == "false":
        get_logger("rmtc.all").info("跳过更新，使用根目录已编译版本。")
        return _start_stack(context)
    get_logger("rmtc.all").info("先更新编译，工作目录固定为：" + str(project))
    builder = ExecuteProcess(
        cmd=["bash", "--noprofile", "--norc", "-c", _BUILD_SCRIPT],
        cwd=str(project),
        env=_clean_environment(context),
        output="screen",
        emulate_tty=True,
    )
    return [
        RegisterEventHandler(OnProcessExit(target_action=builder, on_exit=_build_finished)),
        builder,
    ]


def generate_launch_description():
    """提供总启动参数；默认更新后打开三端，固定种子0，周期按阶段采用100/500ms。"""
    project = LaunchConfiguration("project_dir")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "project_dir",
                default_value="/home/jon/robot/competition",
                description="唯一编译及安装根目录",
            ),
            DeclareLaunchArgument(
                "update",
                default_value="true",
                choices=["true", "false"],
                description="启动前重新编译最新代码",
            ),
            DeclareLaunchArgument(
                "stage",
                default_value="1",
                choices=["1", "2"],
                description="比赛阶段，总Tick自动对应100/500",
            ),
            DeclareLaunchArgument(
                "seed",
                default_value="0",
                description="-1随机；其余整数（含0）固定；尺寸按阶段20×20/24×24",
            ),
            DeclareLaunchArgument(
                "tick_ms", default_value="", description="留空按阶段采用100/500ms；1000可慢速观察"
            ),
            DeclareLaunchArgument(
                "max_ticks", default_value="", description="留空按阶段100/500，同时传给裁判与选手"
            ),
            DeclareLaunchArgument(
                "team", default_value="A", choices=["A", "B", "C", "D"], description="本队席位"
            ),
            DeclareLaunchArgument(
                "team_name", default_value="奶龙做什么都队", description="本队显示名称"
            ),
            DeclareLaunchArgument("bots", default_value="", description="陪练席位，不得包含本队"),
            DeclareLaunchArgument(
                "domain_id", default_value="27", description="三端共用的公开ROS域"
            ),
            DeclareLaunchArgument(
                "internal_domain_id", default_value="", description="官方内部域，留空为公开域加1"
            ),
            DeclareLaunchArgument("port", default_value="8080", description="裁判网页端口"),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                choices=["true", "false"],
                description="是否打开RViz；false仍启动建图",
            ),
            DeclareLaunchArgument(
                "mode",
                default_value="",
                choices=["", "resource", "stage2", "stage2_planning", "probe", "wait"],
                description="留空按stage选择第一/第二阶段策略；也可显式覆盖",
            ),
            DeclareLaunchArgument(
                "resource_order",
                default_value="",
                choices=["", "tour", "nearest", "weighted", "joint"],
                description="留空读取最新C++策略配置",
            ),
            DeclareLaunchArgument(
                "phase_enabled", default_value="", choices=["", "true", "false"],
                description="第二阶段分期实验开关；留空读取C++配置",
            ),
            DeclareLaunchArgument(
                "separate_goal_switch_bonus", default_value="", choices=["", "true", "false"],
                description="目标惯性与动作收益分离；留空读取C++配置",
            ),
            DeclareLaunchArgument(
                "resource_race_enabled", default_value="", choices=["", "true", "false"],
                description="整局一次性资源优先与争抢路线；留空读取C++配置",
            ),
            DeclareLaunchArgument(
                "acquisition_curve_enabled", default_value="", choices=["", "true", "false"],
                description="固定采集偏好与探索时间衰减；留空读取C++配置",
            ),
            DeclareLaunchArgument(
                "temporal_policy_enabled", default_value="", choices=["", "true", "false"],
                description="按参数含义拟合前后期曲线；留空读取C++配置",
            ),
            DeclareLaunchArgument("purpose_enabled", default_value="", choices=["", "true", "false"],
                                  description="可选覆盖目的切换与截击预测开关"),
            DeclareLaunchArgument("strategy_params_file", default_value="",
                                  description="可选保存的ROS策略参数YAML路径"),
            DeclareLaunchArgument("k_edge", default_value="", description="留空读取C++边缘系数"),
            DeclareLaunchArgument("k_value", default_value="", description="留空读取C++价值系数"),
            DeclareLaunchArgument(
                "config_file", default_value="", description="显式完整地图配置，优先于seed"
            ),
            DeclareLaunchArgument(
                "ai_mode", default_value="mock", choices=["mock", "real"], description="练习AI模式"
            ),
            DeclareLaunchArgument("ai_enabled", default_value="true", choices=["true", "false"],
                                  description="第二阶段另起AI进程，第一阶段忽略"),
            DeclareLaunchArgument("ai_url", default_value="", description="留空使用本次网页端口的聊天接口"),
            DeclareLaunchArgument("ai_service_mode", default_value="", choices=["", "mock", "real", "official"],
                                  description="留空与练习ai_mode同步；正式单独启动可选official"),
            DeclareLaunchArgument("ai_model", default_value="", description="留空使用网关默认模型"),
            DeclareLaunchArgument("ai_json_mode", default_value="false", choices=["true", "false"],
                                  description="上游支持时要求JSON对象输出"),
            DeclareLaunchArgument("ai_disable_thinking", default_value="true", choices=["true", "false"],
                                  description="DeepSeek扩展：关闭思考模式"),
            DeclareLaunchArgument("ai_token_env", default_value="RMTC_AI_TOKEN", description="令牌环境变量名"),
            DeclareLaunchArgument("ai_timeout_ms", default_value="10000", description="独立AI进程HTTP超时毫秒"),
            DeclareLaunchArgument("ai_max_calls", default_value="40", description="本程序每局请求预算"),
            DeclareLaunchArgument("ai_cooldown_ms", default_value="500", description="AI请求最小间隔毫秒"),
            DeclareLaunchArgument("ai_log_file", default_value="", description="可选独立AI JSONL路径"),
            DeclareLaunchArgument(
                "referee_dir",
                default_value=PathJoinSubstitution([project, "referee", "local_v1.2", "rmtc_practice"]),
                description="本地v1.2修正版；官方原包保留",
            ),
            DeclareLaunchArgument(
                "data_dir",
                default_value=PathJoinSubstitution([project, "runtime", "referee"]),
                description="裁判运行记录",
            ),
            DeclareLaunchArgument(
                "ros_log_dir",
                default_value=PathJoinSubstitution([project, "runtime", "ros"]),
                description="三端ROS日志目录",
            ),
            DeclareLaunchArgument("log_file", default_value="", description="可选选手JSONL日志"),
            OpaqueFunction(function=_prepare),
        ]
    )
