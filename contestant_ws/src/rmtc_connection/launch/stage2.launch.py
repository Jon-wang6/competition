"""第二阶段完整启动入口：更新编译后，启动裁判、C++选手、建图和RViz。

默认24×24种子0地图、500 Tick、500ms/Tick，本队A席位，B/C/D为官方陪练。
更换team时须同时调整bots，陪练不能包含本队；其他参数沿用all.launch.py。
比赛需在网页手动开始。选手单独启动可用contestant.launch.py mode:=stage2。
默认另起ai_node调用mock接口，只用于验证通信，不解释广播、不提供真实AI计分资格。
此模式第二阶段任务不计分，实得总分上限45；不能据此检验是否达到50分。
真实DeepSeek入口为stage2_deepseek.launch.py；八类事件经模型解析并校验后交给策略。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """固定第二阶段策略和默认比赛节奏，复用三端联启器并同步裁判/选手总Tick。"""
    return LaunchDescription(
        [
            DeclareLaunchArgument("stage", default_value="2", choices=["2"], description="第二阶段"),
            DeclareLaunchArgument(
                "mode", default_value="stage2", choices=["stage2"], description="第二阶段基础策略"
            ),
            DeclareLaunchArgument(
                "tick_ms", default_value="500", description="每Tick毫秒数；1000便于慢速观察"
            ),
            DeclareLaunchArgument(
                "bots", default_value="B,C,D", description="默认A为本队；陪练席位不得包含本队"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("rmtc_connection"), "launch", "all.launch.py"]
                    )
                ),
                launch_arguments={
                    "stage": "2",
                    "mode": "stage2",
                    "tick_ms": LaunchConfiguration("tick_ms"),
                    "bots": LaunchConfiguration("bots"),
                }.items(),
            ),
        ]
    )
