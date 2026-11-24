# 从 30 砖堆到地面墙

[主视频 · 138.64 秒](media/main.mp4) · [首块 B028 完整闭环 · 47.52 秒](media/loop_verified.mp4) · [B026 实际换位再抓 · 58.80 秒](media/mobile_cycle.mp4) · [下载答辩包](defense.zip) · [同一 B026，全程固定世界机位](media/fixed_cycle.mp4) · [侧立 B012 的窄面实抓与铺墙](media/edge_cycle.mp4) · [逐砖数据](evidence/full/audit/per_brick.csv) · [连续积分末态](evidence/full/snapshots/30_011_complete.npz)

从原始砖堆实抓 **30** 块，后臂独持验收 **30** 块，墙上稳定 **30/30** 块。按从低到高的顺序，各皮完成 [10, 8, 6, 6] 块，前三皮顺砖，第四皮丁砖。校验程序从已保存的有效末态继续积分 0.6 秒，逐个检查重力方向的实际承重接触、落点、砖体转角和松爪后的位移。最大落点误差 1.094 mm，0.6 秒内最大漂移 0.000015 mm。逐砖支撑实体和力以 N 记录在 CSV 中。

![实际墙体，取自末态积分数据](media/main_wall.jpg)

## 场景与控制

场景保留 A 初始快照的 30 个独立自由刚体、台面、砖尺寸、质量、位姿和速度。原始快照 ID 为 `c1481c381f2d6d4d8bda84fd7220519416e921d6454de68f8c65870a4e42ed0c`。初始位姿、源文件和状态哈希见 [source.json](evidence/full/source.json)。车型为四轮独立转向和驱动的 Ranger Mini V3，单一自由底盘携带两套 Gen3、双腕相机、前 Robotiq 双指和后 Schmalz Cobot Pump 顶吸机构。原始网格从服务器持久资产目录加载。当前输入使用 MuJoCo 真值位姿与接触，未运行视觉网络。

1. A 的增量三维支撑图以“下砖 → 上砖”为有向边。只在准许端点中按宽面朝上程度、高度和共同双侧条带筛选。候选砖 ID 对两个底盘位姿的一致性在同一初始砖堆实测。锁定后才检查当前车位与备选站位。正式记录中 13 个目标先驱车换位再抓。
2. 前爪靠接触闭合并短试抬。HeldBrick 直接读实抓刚体 ID、两侧接触、前臂关节、工具到砖的 SE(3) 和砖尺寸。后臂根据实测指爪投影为完整密封圈选择吸点，求双臂交接姿态。吸附使用有效直径 25 mm、假设压差 70 kPa 的有限吸附模型。法向额定保持力 34.36 N，剪切、偏心倾覆及扭转力矩分别设上限。预载确认后卸载前爪。每块都记录后臂独持运动，实测最大相对滑移 0.390 mm。
3. 底盘持砖行驶，实测运输路径各为 0.579–2.381 m。墙体坐标固定于世界系。达到承重门槛后泄压、竖直撤离，后臂退出后按实际位置、姿态和支撑计数。末态核验再次检查第四皮每块丁砖至少由两块第三皮砖承重。
4. 每个选中、试抬、实抓输入、交接、行车、承重、泄压、落墙事件都带仿真时刻和砖号。[变换记录](evidence/full/audit/transforms.jsonl) 可交叉检查世界、底盘、前后工具和砖体的位姿。[连续性核验](evidence/full/audit/chain.json) 检查每块砖的自由关节、绑定对象、回放时刻、双指与吸盘阶段以及车辆真实位移。轨迹最大采样间隔 40.000 ms。每次续跑恢复 MuJoCo 全积分状态、执行器、求解器热启动、实际密封锚与控制目标；原始失败日志和检查点另存。

B010 的两侧通道受邻砖限制。前臂指垫沿短端面实际推移 18.80 mm，监测到的单个指垫接触峰值为 3.83 N，其他砖最大位移 0.0001 mm。重新观测后才以双指抓起。

## 时间、镜头与干预

首块含返程的现实用时 172.02 s，仿真 40.402 s。首三次完整循环中位现实用时 122.45 s，最近三次完整循环中位 84.82 s。记录到的最长单项运动为 brick_026 的 `reposition_waypoint`，现实 16.51 s。逐砖表区分完整循环、恢复后的实测部分和包含排错停顿的服务端经过时间。规划日志给出寻找站位的墙钟时间，各个运动阶段单列仿真时间与现实时间。实抓至最后一趟返程累计 80.84 分钟，含诊断停顿。含恢复最慢的砖是 brick_012，耗时 895.1 s；单砖累计站位规划最长为 brick_026 的 31.0 s。未单独计时的计算、I/O 和进程停顿合并列示。首块以后移除了吸附状态下的重复约束求解，180 步一致性与耗时见 [测量](evidence/full/step_benchmark.json)。

物理与 IK/拓扑在 CPU 上运行。离线渲染实际设备为 **NVIDIA GeForce RTX 4090/PCIe/SSE2**，主视频渲染和 H.264 编码流水线合计 119.2 s。主片 B026 的取砖、交接、运输、铺砖和返程按 1× 仿真时间播放，遇到受阻砖时的实体疏理动作同样按 1× 播放，其余施工过程为 22×。机位切换与静帧发生在同一条已记录时间线上。右侧三维图沿用 A 原始三维渲染样式，选砖和移除后显示相应仿真时刻的支撑图。末帧斜俯视为实际施工结果。

保留路径中有 21 次恢复日志，保留轨迹中吸附超限脱开 0 次，计入已归档的失败试验后合计 3 个不同的超限事件。共有 24 份失败尝试历史归档。通过服务端检查点续跑；试验过程中有人修改控制程序、选择恢复点。[恢复记录及失败原因](evidence/full/audit/interventions.json) · [完整事件](evidence/full/events.jsonl) · [全部验收](evidence/full/audit/verification.json) · [模型和依赖版本](provenance.json)。

另以最终候选代码从原始砖堆冷启动，独立核验了首块 B028 实抓、交接、落墙及返程，期间恢复 0 次。[独立首块验收](evidence/fresh_start/verification.json)。该复跑单独计数。

## 在实验服务器复现

```bash
cd /home/cjx/disassembly/abc_course_final
source env.sh
bash run.sh status runs/main01
python audit.py runs/main01 runs/main01/audit_recheck
python chain_audit.py runs/main01 runs/main01/audit_recheck/chain.json
python transform_audit.py runs/main01 runs/main01/audit_recheck/transforms.jsonl
python film.py runs/main01 replay_main --until 30 --feature-slot 5 --speed 22 --hd
bash run.sh start "runs/repro_$(date +%Y%m%d_%H%M%S)"
```

从已有现场继续时先复制整个运行目录，然后执行 `bash run.sh resume <运行副本> <运行副本>/snapshots/30_011_complete.npz`。运行目录包含 MJCF、连续轨迹段、控制器与积分快照、原始事件和接触变化记录。正式运行使用多个调试修订，最新代码可运行完整流程；精确复核已交付实验可直接恢复快照或回放保存轨迹。后期工作进程的逐文件 SHA-256 及控制器源码在 evidence/full/controller_versions，早期版本可查阶段提交 01476b5（双指参考）、d174e5e（首块集成）及 75f6179（第一皮证据）。服务器环境 Python 3.10.19，MuJoCo 3.1.6。原始堆的 MuJoCo 积分状态、XML 与哈希收据在 evidence/full/original_pile。答辩包包括视频、末态、报告、代码和逐砖证据，完整历史轨迹可从同目录 evidence/full/segments 获取。
