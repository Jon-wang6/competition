"""第一阶段Joint选手、独立建图和RViz；不启动裁判或AI进程。

默认A席位、公开ROS域27、100Tick。修改C++后须先重新colcon build。
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
