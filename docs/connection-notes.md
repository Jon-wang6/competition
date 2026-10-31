# 第一步：裁判接口分析

本说明基于官方练习包中的 `CONTESTANT_PROTOCOL.md`、`README.md` 和测试教程。

## 两条通信链路

1. ROS 2负责感知与动作。本步骤只接这条链路。
2. HTTP负责LLM请求。当前不调用AI，后续单独接入。

本机练习裁判网页为 `http://127.0.0.1:8080`，是供人调试的控制台，不是机器人输入源。练习管理操作与参赛决策分开：测试过程中可用网页部署、暂停或重开，机器人只订阅自己席位的公开感知。

## ROS配置

| 配置 | 本机默认值 |
|---|---|
| ROS发行版 | Humble |
| 中间件 | rmw_fastrtps_cpp |
| ROS_DOMAIN_ID | 27 |
| ROS_LOCALHOST_ONLY | 0 |
| 感知订阅 | /team_A/perception |
| 动作发布 | /team_A/action |
| 名称注册 | /team_A/registration |
| 消息类型 | std_msgs/msg/String，data内部为JSON文本 |
| 感知与动作QoS | RELIABLE、VOLATILE、KEEP_LAST(1) |
| 注册QoS | RELIABLE、TRANSIENT_LOCAL、KEEP_LAST(1) |

席位可通过启动参数切换。正式现场使用主办方指定的ROS域和席位。

## 一帧的处理顺序

收到JSON后，先检查 `tick`、`session_id`、`self.team`、`self.alive`、`self.position` 的类型。

新场次先清空旧动作和移动测试状态。旧Tick忽略；相同Tick重新发布缓存动作，不能重复推进决策状态。

每帧直接读取 `self.position=[x,y]`。不靠移动指令累加位置，重连或重启也从当前感知重新定位。

接着处理 `self.last_action`：回执中的tick对应上一动作窗口，不能拿它当本Tick动作。只有曾提交过的Tick才能与自己的动作进行核对。

最后选择 `wait` 或合法相邻格的 `move`，原样回填当前感知的tick和session_id，然后向本队action话题发布。

当前代码通过 `action_ticks` 参数知道本场总动作数。第一阶段默认100；第二阶段应设置500；练习网页修改长度后需同步设置。终态帧仍接收并核对最后一个动作回执，不提交新动作。公开协议没有要求每帧提供比赛结束标志，因此不能从缺失字段推测结束。

## 两种position不能混用

- `self.position`：自己的绝对整数坐标数组。
- `visible_entities[i].position`：单个实体的9行字符串，89个字符，中心1、实体2，换行8个。行列按0开始时，偏移为 `[col-4,row-4]`，再加自己的绝对坐标。
- 没有2且中心保留1，表示同格；0只是占位，不能当作地图可通行标记。
- 坐标轴固定：x向右、y向下，不随车辆朝向旋转。

## 接通的证据

不能只看Tick增长，因为没接入也会自动WAIT。需要同时满足：

1. 自己日志中收到感知并记录发送动作。
2. 裁判接入诊断显示本席位“动作Tick”。
3. 后续感知中last_action的Tick和动作与提交相符。
4. 移动测试出现moved=true，self.position真实变化。
5. 裁判“执行Tick”更新、invalid_actions无异常增加。

## 代码入口

`contestant_ws/src/rmtc_connection/src/core/connection_node.cpp` 中：

- 构造函数：声明参数，建立本队话题，注册名称。
- `on_perception()`：解析、场次切换、去重、回执处理与动作提交。
- `inspect_receipt()`：核对上一Tick裁判回执。
- `relative_position()`：校验并转换实体位置图。
- `decide()`：当前只有等待或轮换方向的移动探测；以后在这里连接正式策略模块。
- `record()`：写自己的JSONL日志，不读取裁判日志。

这是接口基础程序。采集、地图探索、战斗和事件AI仍是后续工作。
