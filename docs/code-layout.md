# 当前代码与启动入口

编译、安装统一在 `/home/jon/robot/competition`。全部 launch 及 shell 入口使用这里的 `install`。
业务程序为 C++17，Python 负责 ROS 2 启动及离线测试编排。源码目录整理不会改变当前策略或权重。

## 文件位置

包根目录：`/home/jon/robot/competition/contestant_ws/src/rmtc_connection`。

| 位置（相对于包根目录） | 内容 |
| --- | --- |
| `src/core/` | ROS 通信、公开数据解析；主控入口为 `connection_main.cpp` |
| `src/planning/` | A*、资源路线、前沿探索、覆盖路线等共用算法 |
| `src/stage1/` | 第一阶段资源策略 |
| `src/stage2/` | 第二阶段决策、联合路线、事件处理和前瞻搜索 |
| `src/ai/` | 独立 AI 进程、HTTP 调用、提示词与结构化回复校验 |
| `src/referee/` | 本地裁判扩展：地图生成、网页、AI 网关、计分 |
| `src/visualization/` | 独立建图节点、实体及八类事件 RViz 标记 |
| `include/rmtc_connection/` | 公共头文件和可调参数，保留原有 include 名称 |
| `tests/` | 自动测试，使用 CTest 运行 |
| `tools/` | 离线调参、最短路审计、回放、广播检查；不加入自动测试 |
| `launch/` | `.launch.py` 启动入口 |
| `web/`、`rviz/` | 裁判网页和 RViz 配置 |

项目根目录下：

- `referee/local_v1.2/rmtc_practice/`：当前默认裁判；官方原始包仍保留在 `referee/v1.1/rmtc_practice/`。
- `runtime/`：对局、调参、测试记录和本轮整理前的压缩备份。
- `docs/`：规则资料及说明；`rules-v1.2.pdf` 是最新提供的规则。
- `.secrets/deepseek_api_key.txt`：本机密钥，不提交、不打印。
- `build/`、`install/`、`log/`：根目录唯一的当前构建产物。
- `contestant_ws/build/`、`contestant_ws/install/`、`contestant_ws/log/`：历史构建产物，当前入口均不使用。

文件名约定：小写下划线；头文件 `.hpp`，实现 `.cpp`，程序入口 `*_main.cpp`，测试 `*_test.cpp`。
ROS 包名、可执行文件名、话题和公共头文件路径保持不变。

## 日常运行

新终端先执行：

```bash
cd /home/jon/robot/competition
source install/setup.bash
```

然后选一个入口；不要同时启动三套：

```bash
# 第一阶段：裁判 + 选手 + 建图 + RViz；20×20，100 Tick，100ms/Tick
ros2 launch rmtc_connection stage1.launch.py

# 第二阶段：同上，带 B/C/D 陪练；24×24，500 Tick，500ms/Tick；AI 为 mock
ros2 launch rmtc_connection stage2.launch.py

# 第二阶段：真实 DeepSeek；默认仅 A 队，读取本机密钥；500ms/Tick
ros2 launch rmtc_connection stage2_deepseek.launch.py

# 真实 DeepSeek，加入 B/C/D 陪练；陪练也会调用同一上游并消耗额度
ros2 launch rmtc_connection stage2_deepseek.launch.py bots:=B,C,D
```

三个完整入口默认 `update:=true`，启动前编译最新 C++ 源码，编译失败就不启动旧程序。
编译成功后打开 `http://127.0.0.1:8080/`，手动开始比赛。Ctrl+C 关闭本次启动的整套程序。

常用后缀：`seed:=-1` 随机地图，`seed:=50` 固定地图，`tick_ms:=1000` 慢速观察，
`use_rviz:=false` 只运行建图，`update:=false` 跳过编译、使用当前已安装版本。
未显式传入权重参数时，沿用 C++ 配置中的值。

`deepseek.launch.py` 保留为 `stage2_deepseek.launch.py` 的兼容入口。
真实入口默认模型沿用 `deepseek-flash`，可通过 `deepseek_model:=模型名` 修改。
mock 返回成功只表示通信通畅，不解释广播，也不提供真实 AI 任务计分资格。
本地真实调用通过不代表正式比赛资格。

## 单独运行与修改参数

```bash
# 单独启动裁判（第二阶段、模拟模式）
ros2 launch rmtc_connection referee.launch.py stage:=2 bots:=B,C,D

# 单独启动第二阶段主控和 AI（默认使用本地模拟接口）
ros2 launch rmtc_connection contestant.launch.py mode:=stage2

# 单独启动建图及 RViz
ros2 launch rmtc_connection rviz.launch.py
```

单独入口不自动编译；修改后先在项目根目录执行 `colcon build --packages-select rmtc_connection`，
再执行 `source install/setup.bash` 并重启相应程序。完整阶段入口会自动完成编译。

调参位置均在公共头文件目录：

- `resource_config.hpp`：第一阶段算法选择与参数。
- `stage2_config.hpp`：第二阶段资源、探索、攻击、保命等参数。
- `stage2_rollout_config.hpp`：第二阶段前瞻搜索与计算预算。
- `stage2_event_config.hpp`：事件处理权重。

## 验证命令

```bash
cd /home/jon/robot/competition
source /opt/ros/humble/setup.bash
colcon list --names-only
colcon build --packages-select rmtc_connection
ctest --test-dir build/rmtc_connection --output-on-failure
```

自动测试不调用付费模型。`tools/ai_broadcast_check.cpp` 的真实广播测试需显式运行。
整理记录与验证结果保存于 `runtime/code-organization-20261028/`。

## RViz 特殊机制显示

建图节点同时订阅本队 `perception` 与 `/rmtc/team_席位/ai_result`。事件只能来自
同一场比赛、与公开广播匹配且经结构校验的真实 AI 解释；mock 不提供机制区域。
不读取裁判全图，也不在显示层用字符串匹配代替模型理解。

| 机制 | 画面名称与颜色 | 显示内容 |
| --- | --- | --- |
| 爆炸 | EXPLOSION，红色 | 预告/施放区域、伤害、倒计时 |
| 幻术 | ILLUSION，紫色 | 施放区域、反向移动持续时间、已确认的个体效果 |
| 天使恩泽 | ANGEL，绿色 | 回血点、每 Tick 回血量、已确认回血效果剩余时间 |
| 天下无双 | UNRIVALED，金色 | 堡垒范围、本车连续驻留进度、已确认无双效果 |
| 世界分裂 | WORLD SPLIT，橙红色 | 完整临时墙区域，包括视野外格子；到期恢复原地形 |
| 火爆辣椒 | PEPPER，橙粉色 | 开关点、进入时触发十字伤害的说明 |
| 洁身自好 | PURITY，青色 | 全场反伤规则、伤害与剩余时间 |
| 潘潘的神 | VULNERABILITY，粉红色 | 移动触发易伤的窗口、伤害倍率、已确认易伤剩余时间 |

半透明 PREVIEW 为预告，ACTIVE 为当前生效。机制说明位于地图左侧，
个体效果同时标在当前可见车辆附近；未知位置的敌人不会因获得 Buff 而暴露位置。
倒计时按比赛 Tick 推进，暂停或断流时不会自行减少。临时事件墙不增加亲眼探索率。

## 第二阶段：先采集，后补任务与交战

参数集中在 `include/rmtc_connection/stage2_config.hpp`；修改后重新启动完整阶段入口，
默认的 `update:=true` 会重新编译。

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `opening_resource_weight` | 1.3 | 开局资源与采集任务偏好 |
| `resource_weight_end` | 0.95 | 资源偏好终值，80Tick内线性过渡 |
| `opening_reach_weight` | 0.15 | 开局到达GOAL的偏好 |
| `reach_weight_end` | 1.0 | 后期到达GOAL的偏好 |
| `reach_ramp_ticks` | 240 | GOAL偏好线性渐增的时长 |
| `reach_deadline_buffer` | 24 | 路程之外为任务预留的Tick |
| `reach_deadline_weight` | 2.0 | 接近最晚出发时间时的任务偏好 |
| `visible_resource_weight` | 1.4 | 当前亲眼看到的资源额外偏好；不作用于旧记忆 |
| `opening_combat_weight` | 0.6 | 开局攻击与主动交战偏好 |
| `combat_weight_end` | 1.15 | 后期交战偏好 |
| `combat_ramp_ticks` | 200 | 攻击偏好线性渐增的时长 |

GOAL与攻击使用各自时间线，不会因为资源开局窗口80Tick结束而一起突然恢复。
资源已确认耗尽时提前恢复GOAL正常偏好；剩余时间接近“估计路程+24Tick”时提高任务紧迫性。
未知区域未连通时只能用路程下界提醒提前出发，不能保证一定赶得及。所有目标仍比较风险，
不会为保证吃资源或到达任务而绕过防御、死亡与事件约束。

资源争抢复用已知地形上的四邻接最短路，比较本车与最快的当前可见敌车；
每车一次BFS距离场可复用到所有资源，与相同网格上的A*最短步数一致。
对手更早到会降低该资源预期收益，步数相近时连续折扣，不直接宣称一定抢不到。
敌人视野外位置和未知路线都不视作确定事实。

选手JSONL的 `stage2_plan` 中新增 `reach_weight`、`resource_races`；
每块资源记录是否当前可见、记忆年龄、本车步数、最快可见敌车步数及争抢折扣。
这些系数只改变决策偏好，不改变裁判真实的资源、伤害或任务计分。


## 离线批量调参与 GPU

`tools/gpu_parameter_screen.cpp` 是独立 C++ 数值筛选工具，通过本机 CUDA 驱动计算
候选参数与真实对局样本的高斯核加权预测。它不安装到参赛入口，不是本地语言模型；
预测高分只决定下一批测什么参数，最终结果来自完整裁判对局。正常 launch 不依赖 GPU。

`tools/ai_cache_replay.cpp` 只在隔离训练 ROS 域 100～130 的偶数域运行。
它复用过去真实模型解析过的逐字相同的全局事件公告，不复制上一局的个体 Buff，
不解释新广播、不调用 HTTP，也不授予正式 AI 任务计分资格。该缓存假设解释及时到达，
不复现真实 API 延迟，因此不能替代正式模型端到端测试。

本轮参数、完整对局、GPU/CPU 核对和统计保存在 `runtime/stage2-batch-20261028/`。
每场有效记录必须包含 501 个感知帧、500 个动作和回执，且裁判无无效动作。
拥堵导致的丢帧或启动失败保留日志并补跑；正常撞车回执不等同于协议无效动作。
