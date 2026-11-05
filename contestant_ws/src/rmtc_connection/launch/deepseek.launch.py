"""兼容原DeepSeek启动命令；新入口统一命名为stage2_deepseek.launch.py。

两个入口执行相同的启动流程，模型、密钥路径、地图及更新参数均沿用新入口。

维护说明：
    兼容入口；维护时优先检查被包含的stage2_deepseek，避免两份启动行为产生差异。
    launch负责创建进程和传递参数，实际决策由C++节点完成。
    修改源码后先构建并source对应install；运行中节点不会因源文件改动自动更新。
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """转发至唯一的真实AI入口，避免维护两份模型配置和密钥读取逻辑。"""
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("rmtc_connection"), "launch", "stage2_deepseek.launch.py"]
                    )
                )
            )
        ]
    )
