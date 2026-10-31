"""第一阶段完整启动入口：更新编译后，启动裁判、C++选手、建图和RViz。

默认20×20种子0地图、100 Tick、100ms/Tick，选手使用现有第一阶段策略。
update:=false可跳过编译，tick_ms:=1000可慢速观察；其他参数沿用all.launch.py。
比赛需在裁判网页手动开始，退出时只关闭本次启动的进程。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """固定第一阶段与资源策略，复用三端联启器及其编译、参数同步和退出管理。"""
    return LaunchDescription(
        [
            DeclareLaunchArgument("stage", default_value="1", choices=["1"], description="第一阶段"),
            DeclareLaunchArgument(
                "mode", default_value="resource", choices=["resource"], description="第一阶段策略"
            ),
            DeclareLaunchArgument(
                "tick_ms", default_value="100", description="每Tick毫秒数；1000便于慢速观察"
            ),
            DeclareLaunchArgument("bots", default_value="", description="第一阶段默认无陪练"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("rmtc_connection"), "launch", "all.launch.py"]
                    )
                ),
                launch_arguments={
                    "stage": "1",
                    "mode": "resource",
                    "tick_ms": LaunchConfiguration("tick_ms"),
                    "bots": LaunchConfiguration("bots"),
                }.items(),
            ),
        ]
    )
