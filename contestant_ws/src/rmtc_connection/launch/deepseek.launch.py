"""兼容原DeepSeek启动命令；新入口统一命名为stage2_deepseek.launch.py。

两个入口执行相同的启动流程，模型、密钥路径、地图及更新参数均沿用新入口。
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
