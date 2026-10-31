"""第二阶段预测规划模式：更新编译后联启裁判、选手、建图与RViz。

默认500 Tick、500ms/Tick、A席位及B/C/D陪练；比赛在网页手动开始。
参数集中于config/stage2_planning.yaml，旧stage2.launch.py仍使用原策略。
默认mock AI只验证通信，不解释广播、不取得正式任务计分资格。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """复用原三端联启与更新流程，仅选择新的模式和其独立参数文件。"""
    return LaunchDescription([
        DeclareLaunchArgument("stage", default_value="2", choices=["2"]),
        DeclareLaunchArgument("mode", default_value="stage2_planning", choices=["stage2_planning"]),
        DeclareLaunchArgument("tick_ms", default_value="500"),
        DeclareLaunchArgument("bots", default_value="B,C,D"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare("rmtc_connection"), "launch", "all.launch.py"])),
            launch_arguments={
                "stage": "2", "mode": "stage2_planning",
                "tick_ms": LaunchConfiguration("tick_ms"),
                "bots": LaunchConfiguration("bots"),
            }.items()),
    ])
