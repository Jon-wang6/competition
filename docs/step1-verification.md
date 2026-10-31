# 第一步实际验证记录（2026-10-21）

环境：当前电脑 Ubuntu 22.04 / x86_64 / ROS 2 Humble，Fast DDS，公开域27。

## 已通过

| 项目 | 实际观察 |
|---|---|
| 官方包校验 | tar.gz的SHA-256通过；manifest列出的全部16个文件通过 |
| 官方程序保持原样 | 不修改裁判源码、脚本、地图或配置；运行数据保存到runtime |
| 裁判启动 | 本机8080端口就绪，第一阶段默认暂停，100 ms/Tick、100 Tick |
| 官方参考车验证 | A车成功移动与采集，裁判动作Tick、执行Tick更新；随后停止参考车 |
| C++构建 | colcon构建成功，C++17，启用Wall/Wextra/Wpedantic |
| 等待链路 | 完整100步WAIT均收到匹配回执，解析错误、回执错误、无效动作均为0 |
| 移动链路 | 场次practice-a5cf54cd8cfd47df8b8bd45688aa31fa中提交100个新Tick动作，匹配回执100个，成功移动100次 |
| 暂停恢复 | Tick12暂停并保持不变；恢复时重复帧重发缓存动作1次，未重复推进移动状态 |
| 最终状态 | 移动场次收到101帧（Tick0至100），只提交100个新Tick动作（0至99）；第100帧只处理最终回执 |
| 重开场次 | 同一C++进程收到新session后从Tick0重新接入，无需人工修改场次ID |
| 选手进程重启 | 场次practice-8c7970e1ca354a29870d85878906c4d1暂停后重启C++节点，首帧为Tick9、实际位置[2,1]，随后Tick9移动成功至[3,1] |

最初WAIT验证时曾多提交终态Tick100的动作（裁判忽略，没有增加无效动作）。随后新增action_ticks参数，在上述完整移动场次中验证已停止终态提交。

移动验证只是轮换四个方向，在附近绕圈；途中自动拾取资源不代表采集策略已经完成。WAIT回执本身可能来自默认行为，因此还结合裁判action_tick、executed_tick及真实移动回执确认接入。

## 对应证据

- `runtime/bot/*.jsonl`：我们自己的rx、tx、replay、ack、finished日志。
- `runtime/probe-console.log`：完整移动和重开过程。
- `runtime/restart-console.log`：进程重启后恢复过程。
- `runtime/build-step1.log`：最终版本构建记录。
- `runtime/referee/practice-<session>/communication.jsonl`：裁判的接收和感知记录，仅用于人工核对。

## 本步骤范围

已完成基础公开ROS接入及恢复流程。本步骤没有实现AI请求、事件理解、正式采集规划、战斗策略或正式现场网络检录。这些是后续工作。
