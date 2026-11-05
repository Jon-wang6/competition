"""第一阶段Joint选手、独立建图和RViz；不启动裁判或AI进程。

默认A席位、公开ROS域27、100Tick。修改C++后须先重新colcon build。

维护说明：
    外部裁判接入入口；只启动第一阶段选手与显示，地图和Tick由外部感知驱动。
    launch负责创建进程和传递参数，实际决策由C++节点完成。
    修改源码后先构建并source对应install；运行中节点不会因源文件改动自动更新。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """复用选手和显示启动入口，固定Joint并统一席位、ROS域和比赛长度。"""
    launch_dir = PathJoinSubstitution([FindPackageShare("rmtc_connection"), "launch"])
    common = {
        "project_dir": LaunchConfiguration("project_dir"),
        "team": LaunchConfiguration("team"),
        "domain_id": LaunchConfiguration("domain_id"),
        "action_ticks": "100",
    }
    return LaunchDescription([
        DeclareLaunchArgument("project_dir", default_value="/home/jon/robot/competition"),
        DeclareLaunchArgument("team", default_value="A", choices=["A"]),
        DeclareLaunchArgument("team_name", default_value="奶龙做什么都队"),
        DeclareLaunchArgument("domain_id", default_value="27"),
        DeclareLaunchArgument("use_rviz", default_value="true", choices=["true", "false"]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([launch_dir, "contestant.launch.py"])),
            launch_arguments={**common, "mode": "resource", "resource_order": "joint",
                              "team_name": LaunchConfiguration("team_name"),
                              "ai_enabled": "false"}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([launch_dir, "rviz.launch.py"])),
            launch_arguments={**common, "use_rviz": LaunchConfiguration("use_rviz")}.items()),
    ])
