# RMTC 选手练习裁判

本包用于 Ubuntu 22.04、ROS 2 Humble、x86_64 Linux 的本机开发及可信局域网联调。公开 ROS 接口与比赛接入一致；地图、任务目标、事件参数及日程为独立练习数据，不代表正式赛场。包内含裁判与参考车的可执行文件、启动和显示脚本，不含裁判 C++ 源码、正式赛事配置、档案及凭据。

详细操作步骤见 [测试使用教程](测试使用教程.md)，竞赛要求见 [赛事规则](rules.txt)，公开消息格式见 [通信协议](CONTESTANT_PROTOCOL.md)。本包测试结果不替代正式现场检录；上场须更新席位、网络、网关地址和本场令牌。

## 安装与启动

安装 ROS 2 Humble 后补齐运行依赖：

```bash
sudo apt install ros-humble-rclpy ros-humble-std-msgs ros-humble-rmw-fastrtps-cpp python3-requests libcurl4
sha256sum -c rmtc_practice.sha256
tar xzf rmtc_practice.tar.gz
cd rmtc_practice
./start_referee.sh
```

打开 http://127.0.0.1:8080 。默认部署第二阶段并暂停，先启动自己的程序，再点击“开始 / 继续”。没有接入的车辆默认等待，不会阻止练习；选手进程退出后其车辆同样等待。无需凑齐四人。

```bash
./start_referee.sh --stage 1
./start_referee.sh --stage 2 --bots B,C,D
./start_referee.sh --port 8081 --config my-practice.json
./start_bot.sh A "我的参考车"
```

使用其它端口时，参考车设置 `AI_GATEWAY=http://127.0.0.1:8081/v1/chat/completions`。陪练与自己不得占用同一席位。各陪练独立运行参考策略，其行为不代表参赛策略要求。第一阶段默认只使用 A 席位；第二阶段使用 A–D，自己的席位不要勾选为陪练。

默认第一阶段 100 ms/Tick、100 Tick，第二阶段 500 ms/Tick、500 Tick。每 Tick 先结算攻击伤害及死亡，再执行存活车辆移动。对抗伤害计分在有效伤害达到 300 时满分（15 分），累计有效伤害可继续增长。

网页可暂停、恢复、重开、选择阶段、添加 0–3 台陪练，编辑并导入/导出完整练习配置。切换阶段重新加载该阶段模板；刷新页面会丢失未保存的编辑。新配置部署前校验，错误配置不会结束现有场次。API 仅在未提供 `config` 键时使用模板；显式提交空对象、空数组、null 或其他非对象值均被拒绝。重开保存旧日志并生成新的 `session_id`。到达总 Tick 后停止推进，控制台保留结果，可再次部署。

## 自己的选手程序

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=27
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMTC_TEAM_TOKEN=practice-A
# 启动自己的程序：席位 A，网关 http://127.0.0.1:8080/v1/chat/completions
```

具体启动参数由选手程序决定，上述变量不会自动替所有程序配置参数。ROS 话题、QoS、感知字段、动作校验及自然语言事件格式遵循随包 `CONTESTANT_PROTOCOL.md`。动作必须回填当前感知中的 `tick` 和 `session_id`。每帧使用本车绝对坐标，视野实体使用协议规定的相对位置。不要解析结构化事件参数或读观赛真值作为策略输入。

诊断页的感知 Tick 表示测试端发布可见，不证明选手收到。动作 Tick 表示诊断器看到发布；执行 Tick 才表示裁判已接受并结算。选手还应在自己日志中确认收到感知，核对 `self.last_action`。页面展示公开感知中的 `last_action` 和接入错误；公开感知及动作保存在 `communication.jsonl`。可测试过期 Tick、错场次、非法动作、重启恢复、断连和四车并行。缺失或错误场次 ID 的动作记为 `session_mismatch`，写入通信日志并显示诊断，不能算作已接受动作。

上场保留相同程序，按主办方通知更换席位、ROS 网络配置、AI 网关地址和令牌。不要硬编码本地 `practice-*` 令牌、场次 ID、地图、事件时间或任务坐标。测试端与正式端接口一致不等于任何网络环境都免检，赛前须完成实际双向联调。

## AI 接入

默认 `--ai-mode mock`：返回固定 OpenAI 兼容响应，令牌为 `practice-A` 至 `practice-D`，仅检查 HTTP 链路，不理解事件，也不产生正式 AI 资格。

真实 LLM 测试由选手提供自己的 OpenAI 兼容服务及凭据：

```bash
export PRACTICE_LLM_URL='https://你的服务/v1/chat/completions'
read -rsp '自己的模型 API Key: ' PRACTICE_LLM_KEY; echo
export PRACTICE_LLM_KEY
export PRACTICE_MODEL='你的模型名称'
./start_referee.sh --ai-mode real
```

选手程序请求本地 `/v1/chat/completions`，携带 `Authorization: Bearer practice-A`；代理将该令牌替换为选手自备的上游凭据。上游地址必须是最终请求地址，代理不跟随重定向；收到重定向返回 502，请修正地址。支持非流式 `messages` 请求；不提供正式额度、正式令牌生命周期或 AI 计分资格判定。队伍旁和诊断表按本场、席位累计成功次数，失败请求不增加成功次数；重开归零。真实模式显示“LLM 成功 N 次”，模拟模式显示“模拟成功 N 次”，不能用模拟成功证明模型理解了事件。诊断只记录模式、席位、次数和请求结果，不记录密钥及提示词。真实调用会使用自己的模型额度，陪练也可能请求模型。随机事件理解和决策必须用真实 LLM 验证。

## 网络与日志

网页及模型代理只绑定 `127.0.0.1`。同机可直接使用。异机 ROS 选手须设置相同域及局域网发现环境；远程使用网页/代理可从选手机执行 `ssh -L 8080:127.0.0.1:8080 用户@测试裁判IP`。参赛程序只使用通信手册列出的公开话题。

本地多个测试实例须分别选择不重叠的公开域与内部域、端口。例如 `ROS_DOMAIN_ID=40 RMTC_INTERNAL_DOMAIN_ID=41 ./start_referee.sh --port 8081`。默认内部域为公开域加一。不要与正在运行的正式裁判使用相同域。

日志默认保存在 `~/.rmtc-practice/practice-<随机场次ID>/`，可用 `--data-dir` 指定：

- `config.json`：本场练习配置。
- `communication.jsonl`：诊断器收到的感知及动作。
- `frames.jsonl`：练习比分与观赛记录。
- `score.json`：本地练习运行数据，不得作为参赛程序输入。
- `ai.jsonl`：模型请求结果摘要。
- `*.log`、`ros-logs/`：进程日志。

节点启动期间单独等待就绪；启动成功后开始心跳检查。停止后页面明确显示“已停止”，并拒绝延迟心跳恢复运行状态。进程异常、裁判心跳超时或关键日志写入失败会显示故障并尝试暂停。检查日志后重开。退出使用 Ctrl+C。练习地图与参数均为测试数据；参赛程序不得读取本地运行文件或网页观赛接口获取策略输入。
