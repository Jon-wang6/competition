# RMTC C++参赛项目：第一步接入

代码与官方裁判均位于 `/home/jon/robot/competition`。

## 目录

- `referee/rmtc_practice/`：官方练习裁判原件，不修改。
- `referee/rmtc_practice.tar.gz`、`.sha256`：官方归档与校验文件。
- `contestant_ws/src/rmtc_connection/`：我们自己的C++17 ROS 2接入节点。
- `scripts/`：我们自己的构建、启动脚本，不修改官方启动脚本。
- `docs/`：赛事规则和练习教程PDF副本。
- `runtime/`：运行时日志与裁判产生的练习记录，不作为参赛策略输入。

## 运行

在项目目录执行：

```bash
cd /home/jon/robot/competition
bash scripts/build.sh
```

终端1启动裁判，初始暂停：

```bash
cd /home/jon/robot/competition
bash scripts/run_referee.sh --stage 1
```

浏览器打开 `http://127.0.0.1:8080`。

终端2启动自己的C++程序：

```bash
cd /home/jon/robot/competition
bash scripts/run_bot.sh A wait
```

在网页点“开始 / 继续”。`wait`只验证等待动作；停止节点后可改为移动探测：

```bash
bash scripts/run_bot.sh A probe
```

`probe`根据当前公开感知检查相邻障碍和车辆，轮换方向移动，仅用于接口验证，尚不包含采集策略或事件AI。不要同时运行两个控制A席位的程序，也不要给A勾选陪练。

第三个机器人启动参数是本场总动作Tick数，默认100。例如第二阶段裁判用 `bash scripts/run_referee.sh --stage 2 --bots B,C,D`，机器人用 `bash scripts/run_bot.sh A wait 500`。如果网页修改总Tick，机器人也需设置相同值，以便最终状态只处理回执、不再提交动作。当前接入节点尚不处理随机事件，不能作为完整第二阶段参赛程序。

## 核对日志

- `READY`：节点启动，开始等感知。
- `SESSION`：新场次或中途加入，以感知中的当前位置定位。
- `RX/TX`：收到本Tick感知并提交动作。
- `ACK`：核对裁判上一Tick回执；`matched=true`说明动作类型一致，`moved=true`说明移动成功。
- `SUMMARY`：退出时统计收到帧数、发送数、匹配回执与解析/回执错误。

日志存到 `runtime/bot/*.jsonl`，网页接入诊断还应同步出现“动作Tick”和“执行Tick”。仅收到感知、或默认WAIT不能单独证明程序接入成功，需结合裁判接收记录与回执。

同场旧Tick丢弃；同Tick重发缓存动作而不重新决策；新session清空动作与移动测试状态；忽略已离开场次的迟到消息。死亡时等待。感知异常不使进程退出，可靠定位下规划失败会退回WAIT。

所有参赛动作仅通过本队公开ROS话题生成。程序不读取网页、全图、官方配置或日志。AI、采集和战斗在后续步骤实现。
