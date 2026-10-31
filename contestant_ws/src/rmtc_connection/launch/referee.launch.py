"""启动本地v1.2修正版练习裁判，保留原始官方包。

文件：referee.launch.py
用途：启动独立C++网页入口，并通过ExecuteProcess调用本地副本start_referee.sh。
默认启动第一阶段，周期采用规则值100/500ms，网页端口8080，部署后暂停。
不启动选手控制程序；正式节奏可显式设置tick_ms:=100或500。
可配置阶段、陪练、速度、总Tick、端口、AI模式以及自定义练习配置文件。
按阶段生成地图：第一阶段20×20、第二阶段24×24；-1随机，其余整数（含0）固定。
地图部署前交官方校验器确认，不直接采用官方默认布局。
网页生成只准备下一场配置，部署沿用当前配置，不因重复点击而重新随机。
显式config_file优先使用原配置，不重新生成，便于重放手工测试场景。
默认使用referee/local_v1.2/rmtc_practice；官方v1.1与旧版分别保留，
需要回归旧版本时，可显式指定referee_dir，不必移动或修改官方文件。
本文件只用于本地练习部署，不向参赛决策提供地图或事件配置数据。
"""

import json
from pathlib import Path
import socket
import subprocess
import uuid

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def _generate_map(referee_dir, stage, seed, data_dir):
    """调用C++生成器，随后运行官方validate_config；成功才返回可部署的配置路径。

    每次启动使用独立文件名，同种子地图内容相同但不会覆盖其他进程的文件。
    实际随机种子和生成器版本保存在相邻metadata.json，便于复现。
    此处仅编排工具与打印部署信息，地图生成算法全部在C++中实现。
    """
    executable = (
        Path(get_package_prefix("rmtc_connection"))
        / "lib"
        / "rmtc_connection"
        / "generate_practice_map"
    )
    if not executable.is_file():
        raise RuntimeError("找不到C++地图生成器，请先在/home/jon/robot/competition执行colcon build")
    output = data_dir / "generated-maps" / f"stage{stage}-request{seed}-{uuid.uuid4().hex}.json"
    generation = subprocess.run(
        [
            str(executable),
            "--template",
            str(referee_dir / "config" / f"stage{stage}.json"),
            "--stage",
            stage,
            "--seed",
            str(seed),
            "--output",
            str(output),
        ],
        capture_output=True,
        text=True,
        timeout=15,
    )
    if generation.returncode:
        raise RuntimeError(f"地图生成失败：{generation.stderr.strip()}")
    metadata = json.loads(generation.stdout)
    validation = subprocess.run(
        [str(referee_dir / "bin" / "validate_config")],
        input=output.read_text(),
        capture_output=True,
        text=True,
        timeout=10,
    )
    if validation.returncode:
        raise RuntimeError(f"官方校验器拒绝生成地图：{validation.stderr.strip()}")
    get_logger("rmtc.seeded_map").info(
        f"地图已通过官方校验：stage={stage} seed={metadata['seed']} "
        f"尺寸={metadata['width']}×{metadata['height']} 障碍={metadata['obstacle_count']} "
        f"资源={metadata['normal_resources']}普通+{metadata['high_resources']}高价值；配置={output}"
    )
    return output


def _start_referee(context):
    """读取参数，校验地图，创建官方裁判与独立C++网页两个进程。

    context由ROS 2 launch传入，包含命令行覆盖后的全部启动参数。
    本函数在launch实际运行时执行，返回ExecuteProcess动作列表。
    不复制或修改官方脚本；日志保存目录通过官方--data-dir参数传入。
    可选tick_ms/max_ticks为空时不传对应选项，采用官方各阶段默认值。
    路径无效或数字参数不合法时抛出异常，使launch明确报告启动失败。
    """
    referee_dir = Path(LaunchConfiguration("referee_dir").perform(context)).expanduser().resolve()
    script = referee_dir / "start_referee.sh"
    if not script.is_file():
        raise RuntimeError(f"找不到官方裁判启动脚本：{script}")

    stage = LaunchConfiguration("stage").perform(context)
    seed = int(LaunchConfiguration("seed").perform(context))
    if not -(1 << 63) <= seed < (1 << 63):
        raise ValueError("seed须为64位整数；-1表示随机")
    port = int(LaunchConfiguration("port").perform(context))
    domain = int(LaunchConfiguration("domain_id").perform(context))
    internal_text = LaunchConfiguration("internal_domain_id").perform(context)
    internal_domain = int(internal_text) if internal_text else domain + 1
    if not 1 <= port <= 65535:
        raise ValueError("port必须在1到65535之间")
    backend_text = LaunchConfiguration("backend_port").perform(context)
    if backend_text:
        backend_port = int(backend_text)
    else:
        # 网页仍使用port；为未修改的官方HTTP服务选取一个独立回环端口。
        with socket.socket() as probe:
            probe.bind(("127.0.0.1", 0))
            backend_port = probe.getsockname()[1]
    if not 1 <= backend_port <= 65535 or backend_port == port:
        raise ValueError("backend_port须在1到65535之间且不能等于网页端口")
    if domain < 0 or internal_domain < 0 or internal_domain == domain:
        raise ValueError("公开域和内部域须为不同的非负整数")

    data_dir = Path(LaunchConfiguration("data_dir").perform(context)).expanduser().resolve()
    ros_log_dir = Path(LaunchConfiguration("ros_log_dir").perform(context)).expanduser().resolve()
    data_dir.mkdir(parents=True, exist_ok=True)
    ros_log_dir.mkdir(parents=True, exist_ok=True)

    # 参数使用列表逐项传递，不经过shell字符串拼接，路径中有空格也能使用。
    command = [
        "bash",
        str(script),
        "--stage",
        stage,
        "--port",
        str(backend_port),
        "--bots",
        LaunchConfiguration("bots").perform(context),
        "--ai-mode",
        LaunchConfiguration("ai_mode").perform(context),
        "--data-dir",
        str(data_dir),
    ]
    for argument, option in (("tick_ms", "--tick-ms"), ("max_ticks", "--max-ticks")):
        value = LaunchConfiguration(argument).perform(context)
        if value:
            if int(value) <= 0:
                raise ValueError(f"{argument}必须为正整数")
            command.extend([option, value])
    config_file = LaunchConfiguration("config_file").perform(context)
    if config_file:
        config_path = Path(config_file).expanduser().resolve()
        if not config_path.is_file():
            raise RuntimeError(f"找不到练习配置：{config_path}")
        get_logger("rmtc.seeded_map").info("使用显式config_file保留原地图，seed不参与本次生成。")
    else:
        config_path = _generate_map(referee_dir, stage, seed, data_dir)
    command.extend(["--config", str(config_path)])

    tick_text = LaunchConfiguration("tick_ms").perform(context)
    effective_tick_ms = int(tick_text) if tick_text else (100 if stage == "1" else 500)
    get_logger("rmtc.referee").info(
        f"本次部署周期：{effective_tick_ms}ms/Tick；网页表单修改须点“部署新练习”才生效。"
    )

    # 实际模型地址和密钥仍按官方要求从父进程环境继承，不放入launch参数。
    web_executable = (
        Path(get_package_prefix("rmtc_connection")) / "lib" / "rmtc_connection" / "practice_web"
    )
    web_dir = Path(get_package_share_directory("rmtc_connection")) / "web"
    if not web_executable.is_file():
        raise RuntimeError("找不到C++练习网页程序，请先编译rmtc_connection")
    get_logger("rmtc.referee").info(
        f"练习网页：http://127.0.0.1:{port}；第一阶段20×20、第二阶段24×24；-1随机，其余整数（含0）固定。"
    )
    return [
        ExecuteProcess(
            cmd=command,
            cwd=str(referee_dir),
            output="screen",
            emulate_tty=True,
            additional_env={
                "ROS_DOMAIN_ID": str(domain),
                "RMTC_INTERNAL_DOMAIN_ID": str(internal_domain),
                "ROS_LOCALHOST_ONLY": "0",
                "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
                "ROS_LOG_DIR": str(ros_log_dir),
                "PRACTICE_PUBLIC_PORT": str(port),
                "PRACTICE_BACKEND_ONLY": "1",
                "PYTHONUNBUFFERED": "1",
            },
            sigterm_timeout="10",
            sigkill_timeout="5",
            on_exit=[Shutdown(reason="练习裁判进程已结束")],
        ),
        ExecuteProcess(
            cmd=[
                str(web_executable),
                "--port",
                str(port),
                "--backend-port",
                str(backend_port),
                "--official-dir",
                str(referee_dir),
                "--web-dir",
                str(web_dir),
                "--data-dir",
                str(data_dir),
                "--config",
                str(config_path),
            ],
            output="screen",
            sigterm_timeout="10",
            sigkill_timeout="5",
            on_exit=[Shutdown(reason="练习网页进程已结束")],
        ),
    ]


def generate_launch_description():
    """声明可覆盖的参数，并返回启动官方裁判所需的LaunchDescription。

    ROS 2通过此约定入口加载文件。这里只声明启动步骤，不立即运行裁判。
    project_dir默认指向当前电脑的项目目录，迁移目录时可在命令行覆盖。
    """
    project = LaunchConfiguration("project_dir")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "project_dir", default_value="/home/jon/robot/competition", description="项目根目录"
            ),
            DeclareLaunchArgument(
                "referee_dir",
                default_value=PathJoinSubstitution([project, "referee", "local_v1.2", "rmtc_practice"]),
                description="本地v1.2修正版；官方v1.1原包另存",
            ),
            DeclareLaunchArgument(
                "stage", default_value="1", choices=["1", "2"], description="1为采集，2为对抗"
            ),
            DeclareLaunchArgument(
                "seed",
                default_value="0",
                description="地图种子：-1每次随机，其余整数（含0）固定；config_file优先",
            ),
            DeclareLaunchArgument("port", default_value="8080", description="网页和练习AI代理端口"),
            DeclareLaunchArgument(
                "backend_port", default_value="", description="官方HTTP内部端口，默认自动选择"
            ),
            DeclareLaunchArgument(
                "bots", default_value="", description="陪练席位，例如B,C,D；默认无陪练"
            ),
            DeclareLaunchArgument(
                "domain_id", default_value="27", description="公开ROS域，须与选手端一致"
            ),
            DeclareLaunchArgument(
                "internal_domain_id", default_value="", description="裁判内部域，留空为公开域加1"
            ),
            DeclareLaunchArgument(
                "tick_ms",
                default_value="",
                description="留空采用规则100/500毫秒；1000可慢速观察",
            ),
            DeclareLaunchArgument(
                "max_ticks", default_value="", description="留空采用阶段默认长度：100或500步"
            ),
            DeclareLaunchArgument(
                "ai_mode",
                default_value="mock",
                choices=["mock", "real"],
                description="默认模拟AI；real须预先配置官方要求的上游环境变量",
            ),
            DeclareLaunchArgument(
                "config_file", default_value="", description="可选的完整练习配置JSON路径"
            ),
            DeclareLaunchArgument(
                "data_dir",
                default_value=PathJoinSubstitution([project, "runtime", "referee"]),
                description="裁判运行数据保存目录",
            ),
            DeclareLaunchArgument(
                "ros_log_dir",
                default_value=PathJoinSubstitution([project, "runtime", "ros"]),
                description="裁判ROS节点日志目录",
            ),
            OpaqueFunction(function=_start_referee),
        ]
    )
