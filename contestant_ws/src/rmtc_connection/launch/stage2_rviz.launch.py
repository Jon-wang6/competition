"""第二阶段当前预测规划选手、独立建图和RViz；不启动裁判或模拟AI服务。

默认启用独立正式网关客户端，须设置RMTC_AI_URL及本场RMTC_AI_TOKEN。
请求使用POST、Bearer令牌、deepseek-flash、JSON输出及关闭思考。
离线只看策略和地图时可显式传ai_enabled:=false；AI不会阻塞主控动作回调。
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _check_gateway(context):
    """仅在明确开启AI时检查正式端点和本场令牌，不读取或打印令牌内容。"""
    if LaunchConfiguration("ai_enabled").perform(context) != "true":
        return []
    url = LaunchConfiguration("ai_url").perform(context).strip()
    if not url.startswith(("http://", "https://")) or not url.endswith("/v1/chat/completions"):
        raise ValueError("请设置RMTC_AI_URL或ai_url为正式网关完整/v1/chat/completions地址")
    if not os.environ.get("RMTC_AI_TOKEN", "").strip():
        raise ValueError("启用正式AI前，请在本机设置本场令牌环境变量RMTC_AI_TOKEN")
    return []


def generate_launch_description():
    """固定使用stage2_planning及其当前参数，两端共享席位、ROS域和500Tick。"""
    launch_dir = PathJoinSubstitution([FindPackageShare("rmtc_connection"), "launch"])
    common = {
        "project_dir": LaunchConfiguration("project_dir"),
        "team": LaunchConfiguration("team"),
        "domain_id": LaunchConfiguration("domain_id"),
        "action_ticks": "500",
    }
    return LaunchDescription([
        DeclareLaunchArgument("project_dir", default_value="/home/jon/robot/competition"),
        DeclareLaunchArgument("team", default_value="A", choices=["A", "B", "C", "D"]),
        DeclareLaunchArgument("team_name", default_value="奶龙做什么都队"),
        DeclareLaunchArgument("domain_id", default_value="27"),
        DeclareLaunchArgument("use_rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("ai_enabled", default_value="true", choices=["true", "false"],
                              description="默认正式网关；离线可显式关闭，比赛任务分要求本场真实成功调用"),
        DeclareLaunchArgument("ai_url", default_value=EnvironmentVariable("RMTC_AI_URL", default_value="")),
        DeclareLaunchArgument("ai_model", default_value="deepseek-flash"),
        DeclareLaunchArgument("ai_max_tokens", default_value="4096", description="多事件JSON输出上限，按官方公告调整；可设512"),
        DeclareLaunchArgument("ai_timeout_ms", default_value="10000"),
        DeclareLaunchArgument("ai_max_calls", default_value="40", description="按官方本场配额设置"),
        OpaqueFunction(function=_check_gateway),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([launch_dir, "contestant.launch.py"])),
            launch_arguments={**common, "mode": "stage2_planning",
                              "team_name": LaunchConfiguration("team_name"),
                              "ai_enabled": LaunchConfiguration("ai_enabled"),
                              "ai_url": LaunchConfiguration("ai_url"),
                              "ai_model": LaunchConfiguration("ai_model"),
                              "ai_max_tokens": LaunchConfiguration("ai_max_tokens"),
                              "ai_service_mode": "official", "ai_token_env": "RMTC_AI_TOKEN",
                              "ai_json_mode": "true", "ai_disable_thinking": "true",
                              "ai_timeout_ms": LaunchConfiguration("ai_timeout_ms"),
                              "ai_max_calls": LaunchConfiguration("ai_max_calls")}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([launch_dir, "rviz.launch.py"])),
            launch_arguments={**common, "use_rviz": LaunchConfiguration("use_rviz")}.items()),
    ])
