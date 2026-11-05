"""启动独立C++建图节点和RViz，不重复启动裁判或选手控制程序。

文件：rviz.launch.py
用途：接收本队公开感知及AI已校验解释，实时显示累计地图和八类事件。
默认A队、公开ROS域27；team/domain_id必须与选手启动参数一致。
use_rviz:=false时仅运行建图进程，便于无桌面环境检查地图话题。

维护说明：
    独立显示入口；team与domain_id应匹配感知来源，Fixed Frame与地图话题应对应同一队伍。
    launch负责创建进程和传递参数，实际决策由C++节点完成。
    修改源码后先构建并source对应install；运行中节点不会因源文件改动自动更新。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _start_visualization(context):
    """校验启动参数，创建独立建图节点，并按需打开加载预设配置的RViz。

    RViz默认配置使用A队话题，通过重映射切换到所选席位，-f同步固定坐标系。
    两个子进程使用与裁判相同的公开ROS域；日志保存在项目runtime/ros下。
    关闭RViz会一并停止本launch创建的建图节点，不会停止另外启动的裁判/选手。
    """
    team = LaunchConfiguration("team").perform(context)
    domain = int(LaunchConfiguration("domain_id").perform(context))
    cell_size = float(LaunchConfiguration("cell_size").perform(context))
    action_ticks = int(LaunchConfiguration("action_ticks").perform(context))
    if not 0 <= domain <= 232:
        raise ValueError("domain_id须在0到232之间")
    if not 0.01 <= cell_size <= 100:
        raise ValueError("cell_size须在0.01到100之间")
    ros_log_dir = Path(LaunchConfiguration("ros_log_dir").perform(context)).expanduser().resolve()
    ros_log_dir.mkdir(parents=True, exist_ok=True)
    environment = {
        "ROS_DOMAIN_ID": str(domain),
        "ROS_LOCALHOST_ONLY": "0",
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
        "ROS_LOG_DIR": str(ros_log_dir),
    }
    namespace = "/rmtc/team_" + team
    actions = [
        Node(
            package="rmtc_connection",
            executable="mapping_node",
            namespace=namespace,
            parameters=[{"team": team, "cell_size": cell_size, "action_ticks": action_ticks}],
            additional_env=environment,
            output="screen",
            emulate_tty=True,
            on_exit=[Shutdown(reason="独立建图节点已结束")],
        )
    ]
    if LaunchConfiguration("use_rviz").perform(context) == "true":
        config = LaunchConfiguration("rviz_config").perform(context)
        if not Path(config).is_file():
            raise ValueError("RViz配置文件不存在：" + config)
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="map_viewer",
                namespace=namespace,
                arguments=["-d", config, "-f", "rmtc_" + team + "_map"],
                remappings=[
                    ("/rmtc/team_A/map", namespace + "/map"),
                    ("/rmtc/team_A/markers", namespace + "/markers"),
                ],
                additional_env=environment,
                output="screen",
                on_exit=[Shutdown(reason="RViz窗口已关闭")],
            )
        )
    return actions


def generate_launch_description():
    """声明Python launch参数；默认一条命令同时启动建图和RViz窗口。

    cell_size只改变每格在显示中的米数，不改变比赛坐标或移动距离。
    rviz_config可替换为用户另存的视角配置；默认配置为俯视地图。
    """
    default_config = str(
        Path(get_package_share_directory("rmtc_connection")) / "rviz" / "competition.rviz"
    )
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
                description="与选手一致的席位，只接收该队公开感知",
            ),
            DeclareLaunchArgument("domain_id", default_value="27", description="裁判公开ROS域"),
            DeclareLaunchArgument(
                "cell_size", default_value="1.0", description="每格对应的显示米数"
            ),
            DeclareLaunchArgument(
                "action_ticks", default_value="500", description="事件时间线校验使用的总Tick"
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                choices=["true", "false"],
                description="是否同时打开RViz窗口",
            ),
            DeclareLaunchArgument(
                "rviz_config", default_value=default_config, description="RViz配置文件"
            ),
            DeclareLaunchArgument(
                "ros_log_dir",
                default_value=PathJoinSubstitution(
                    [LaunchConfiguration("project_dir"), "runtime", "ros"]
                ),
                description="ROS日志目录",
            ),
            OpaqueFunction(function=_start_visualization),
        ]
    )
