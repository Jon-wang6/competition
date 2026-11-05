"""第二阶段真实AI入口：独立C++ AI进程经本地v1.2网关调用DeepSeek。

优先从本机.secrets/deepseek_api_key.txt读取密钥，文件为空/不存在时兼容
DEEPSEEK_API_KEY环境变量，映射到裁判要求的PRACTICE_LLM_KEY；
不作为launch参数、命令行参数或日志输出。与普通第二阶段使用相同策略参数。
默认500ms/Tick；tick_ms:=1000可慢速观察。完整调用超时30秒，主控独立运行。
默认无陪练，只验证A队；可传bots:=B,C,D，但陪练AI请求也会调用同一真实上游。
真实模型调用会消耗用户DeepSeek API额度；练习记录不代表正式赛事资格。

维护说明：
    本地真实模型练习入口；会使用配置的真实服务额度，测试策略本身时无需为此启用网络请求。
    launch负责创建进程和传递参数，实际决策由C++节点完成。
    修改源码后先构建并source对应install；运行中节点不会因源文件改动自动更新。
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _configure(context):
    """优先读取用户的单行密钥文件；仅回传环境，不把文件正文放进参数或日志。"""
    key_file = Path(LaunchConfiguration("deepseek_key_file").perform(context)).expanduser()
    try:
        if key_file.stat().st_size > 8192:
            raise RuntimeError("DeepSeek密钥文件过大，请只保留密钥本身。")
        key = key_file.read_text(encoding="utf-8-sig").strip()
    except FileNotFoundError:
        key = ""
    except OSError:
        raise RuntimeError("无法读取DeepSeek密钥文件，请检查文件路径及读取权限。") from None
    if not key:
        key = EnvironmentVariable("DEEPSEEK_API_KEY", default_value="").perform(context)
    if not key or any(char.isspace() for char in key):
        raise RuntimeError("请在DeepSeek密钥文件中填写一行密钥本身，或设置DEEPSEEK_API_KEY。")
    return [
        SetEnvironmentVariable("PRACTICE_LLM_KEY", key),
        SetEnvironmentVariable("PRACTICE_LLM_URL", "https://api.deepseek.com/chat/completions"),
        SetEnvironmentVariable("PRACTICE_MODEL", LaunchConfiguration("deepseek_model")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("rmtc_connection"), "launch", "all.launch.py"])
            ),
            launch_arguments={
                "stage": "2",
                "mode": LaunchConfiguration("mode"),
                "ai_enabled": "true",
                "ai_mode": "real",
                "ai_service_mode": "real",
                "ai_url": "",  # 沿用本次裁判端口，真实密钥由裁判转发时添加。
                "ai_model": LaunchConfiguration("deepseek_model"),
                "ai_json_mode": "true",
                "ai_disable_thinking": "true",
                "ai_timeout_ms": "30000",
                "tick_ms": LaunchConfiguration("tick_ms"),
            }.items(),
        ),
    ]


def generate_launch_description():
    """声明模型和比赛节奏；其余地图、端口、更新、RViz参数沿用all.launch.py。"""
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="stage2", choices=["stage2", "stage2_planning"],
                                  description="stage2原策略；stage2_planning独立预测规划"),
            DeclareLaunchArgument(
                "deepseek_key_file",
                default_value="/home/jon/robot/competition/.secrets/deepseek_api_key.txt",
                description="本地单行密钥文件路径；文件内容不作为launch参数传递",
            ),
            DeclareLaunchArgument("deepseek_model", default_value="deepseek-flash",
                                  description="DeepSeek官方模型名"),
            DeclareLaunchArgument("tick_ms", default_value="500",
                                  description="规则周期500ms；1000可慢速观察事件解释"),
            OpaqueFunction(function=_configure),
        ]
    )
