# RMTC C++ 参赛项目

本项目使用 C++17 与 ROS 2，包含第一阶段 Joint 资源采集、第二阶段预测规划、本地残差神经网络、独立 AI 网关客户端和 RViz 建图显示。

以下命令按本机目录 `/home/jon/robot/competition` 编写。所有程序均通过 Python 格式的 launch 文件启动。

## 编译与环境

首次使用或修改 C++ 后，在项目根目录编译：

```bash
cd /home/jon/robot/competition
colcon build --packages-select rmtc_connection --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

**每个新终端**先加载环境：

```bash
cd /home/jon/robot/competition
source install/setup.bash
```

`setup.bash` 只加载 ROS 工作区环境，不是启动程序的脚本。修改参数配置后也应更新安装目录并重启节点。

## 第一阶段：选手程序与 RViz

```bash
ros2 launch rmtc_connection stage1_rviz.launch.py domain_id:=27 team:=A
```

启动 Joint 资源采集、独立建图与 RViz，使用 100 Tick；**不启动裁判端或 AI 客户端**。该入口当前仅接受 A 席位。

本地练习时，另开终端启动第一阶段裁判：

```bash
ros2 launch rmtc_connection referee.launch.py stage:=1 domain_id:=27 seed:=0
```

打开 <http://127.0.0.1:8080/>，在选手就绪后点击“开始 / 继续”。已有外部裁判时无需启动本地裁判。

## 第二阶段：本地选手程序与 RViz

终端一启动裁判及 B/C/D 陪练：

```bash
ros2 launch rmtc_connection referee.launch.py stage:=2 domain_id:=27 bots:=B,C,D seed:=0 tick_ms:=500
```

终端二启动当前预测规划选手与显示：

```bash
ros2 launch rmtc_connection stage2_rviz.launch.py domain_id:=27 team:=A ai_enabled:=false
```

在网页开始比赛。这一选手入口使用 500 Tick，**不包含裁判端**。

`ai_enabled:=false` 关闭外部 HTTP 模型调用，不消耗真实模型额度；它不关闭本地采集神经网络。关闭网关客户端后，不应期待它提供新的自然语言事件解释或真实调用资格。

当前 `config/stage2_planning.yaml` 启用了采集残差网络，战斗网络仍关闭。该网络属于本地实验配置，尚未证实稳定提高比赛总分；正式使用前需核对赛事允许的策略与本场配置。

## 第二阶段：接入指定网关

如果需要启用独立 AI 客户端，先在启动选手的终端设置完整网关地址和本场令牌。下面的地址与令牌是占位符，必须替换：

```bash
export RMTC_AI_URL='http://裁判IP:端口/v1/chat/completions'
export RMTC_AI_TOKEN='本场令牌'
ros2 launch rmtc_connection stage2_rviz.launch.py domain_id:=29 team:=B ai_enabled:=true
```

`29` 和 `B` 只是示例，按现场分配替换；第二阶段入口支持 A/B/C/D。不要把真实令牌提交到仓库。

客户端使用 POST `/v1/chat/completions`、Bearer 令牌，默认模型为 `deepseek-flash`，要求 JSON 输出并关闭思考。启用客户端不等于调用必然成功；还需检查地址、网络、本场令牌、额度和回复校验。

已有外部裁判时，只启动选手与显示，不再启动本地裁判。模型及策略是否可用于正式比赛，应以当场规则为准。

## 其他 launch 入口

| 文件 | 用途 |
|---|---|
| `stage1_rviz.launch.py` | 第一阶段选手 + 建图 + RViz，不含裁判 |
| `stage2_rviz.launch.py` | 当前第二阶段选手 + 建图 + RViz，可启用正式网关客户端，不含裁判 |
| `referee.launch.py` | 单独启动本地练习裁判及相关服务 |
| `contestant.launch.py` | 单独启动选手，可指定模式与参数文件 |
| `rviz.launch.py` | 单独启动建图与 RViz，不启动选手和裁判 |
| `stage1.launch.py` | 第一阶段本地完整联启，包含更新流程和裁判 |
| `stage2_planning.launch.py` | 当前预测规划模式的本地完整联启，包含更新流程和裁判 |
| `stage2.launch.py` | 原第二阶段策略的本地完整联启，参数来自 `stage2.yaml` |
| `stage2_deepseek.launch.py` | 本地真实模型练习入口，可能消耗 API 额度 |

例如，单独启动第二阶段选手，不开 RViz 和 HTTP AI：

```bash
ros2 launch rmtc_connection contestant.launch.py mode:=stage2_planning domain_id:=27 team:=A ai_enabled:=false
```

单独打开同队伍的显示：

```bash
ros2 launch rmtc_connection rviz.launch.py domain_id:=27 team:=A
```

本地一键启动第一阶段或第二阶段预测规划，分别使用：

```bash
ros2 launch rmtc_connection stage1.launch.py
```

```bash
ros2 launch rmtc_connection stage2_planning.launch.py
```

一键入口已经包含裁判和选手，不要同时再运行对应的单独入口。同一场次、同一席位只运行一个主控。

## 参数与文件位置

- `contestant_ws/src/rmtc_connection/include/rmtc_connection/resource_config.hpp`：第一阶段编译期默认参数。
- `contestant_ws/src/rmtc_connection/config/stage2_planning.yaml`：当前第二阶段预测规划参数。
- `contestant_ws/src/rmtc_connection/config/stage2_planning_baseline.yaml`：启用采集网络前的基线参数。
- `contestant_ws/src/rmtc_connection/config/models/collection_small64.json`：当前采集残差模型。
- `contestant_ws/src/rmtc_connection/config/MODEL_NOTES.md`：模型文件说明。
- `contestant_ws/src/rmtc_connection/launch/`：启动文件。
- `referee/`：官方归档及本地练习副本；本地扩展默认使用 `local_v1.2`。
- `docs/`：赛事规则与教程。
- `checkpoints/`：历史模型、参数与验证结果。
- `runtime/`：运行日志及练习记录，Git 忽略，不作为选手的隐藏状态输入。

模型路径中包含本机绝对安装路径；迁移到另一台电脑时，应调整为新安装目录中的对应模型文件。

## 运行检查

- `READY`：节点就绪，等待感知。
- `SESSION`：收到新的比赛场次。
- `RX/TX`：接收感知、发送动作。
- `ACK`：核对动作回执；`matched=true`仅表示检查到的动作类型匹配，移动是否成功还需看`moved`与坐标。
- `SUMMARY`：退出时汇总帧数、动作与异常统计。

ROS_DOMAIN_ID、席位、话题和 RViz 固定坐标系需要对应。RViz 地图异常不等于主控必然没收到数据，应分别查看主控和建图日志。

详细 JSONL 决策日志仅在配置 `log_file` 时写入，不是默认一定存在。算法性能与得分需通过闭环对局验证，不能仅凭节点启动成功或模型文件加载成功判断。
