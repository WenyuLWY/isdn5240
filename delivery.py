"""Assemble the audited run and its controller, without scratch frames or other experiments."""
from pathlib import Path
from collections import Counter
from statistics import median
import argparse,subprocess,json,shutil,sys,hashlib,csv
root=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('out',type=Path);p.add_argument('--movie',default='main');args=p.parse_args()
run,out=args.run,args.out
def cp(src,dst):
    dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
def read(src):return json.loads(src.read_text())
def log(src):return [json.loads(l) for l in src.read_text().splitlines() if l.strip()]
def data(dst,v):dst.parent.mkdir(parents=True,exist_ok=True);dst.write_text(json.dumps(v,indent=1,ensure_ascii=False))
def command(*a):return subprocess.check_output(list(map(str,a)),text=True).strip()
code=['integrated.py','scene.py','rig.py','demo.py','baseline_input.py','axes.py','exact_strips.py','declutter_native.py','vacuum.py','build_pump.py','scan.py','joint_path.py','clearance_check.py','restore.py','audit.py','chain_audit.py','transform_audit.py','film.py','graph_view.py','egl.py','env.sh','run.sh','wall_targets.py','vehicle_margin.py','source_version.py','status.py','timing_capture.py','pedigree.py','prepare_dependency.py','delivery.py','ik_seeds.json','a_reference.json','layout30.json','prototype.xml','provenance.json','targets_world.json','benchmark_step.py','pin_cpu.py','pack.py','fresh_first.py','inspect_movie.py','wall_figure.py','passage_audit.py']
for name in code:cp(root/name,out/name)
shutil.copytree(root/'upstream',out/'upstream',dirs_exist_ok=True)
full=out/'evidence/full';media=out/'media'
for name in ['source.json','scene.xml','parked.xml','step_benchmark.json','cpu_tuning.json','events.jsonl','contact_changes.jsonl','checkpoint_clock.json','status.json']:
    cp(run/name,full/name)
cp(Path(read(root/'provenance.json')['config']),full/'oracle73_config.yaml')
original=Path(read(run/'source.json')['snapshot']);reference=read(original)
cp(original,full/'original_pile/snapshot.json')
for key,name in [('state_path','state.npz'),('xml_path','scene.xml')]:
    src=Path(reference[key])
    assert hashlib.sha256(src.read_bytes()).hexdigest()==reference[key.replace('_path','_sha256')]
    cp(src,full/'original_pile'/name)
for sub in ['segments','snapshots']:
    for f in (run/sub).glob('*.npz'):cp(f,full/sub/f.name)
versions=run.parent/(run.name+'_versions')
if versions.is_dir():shutil.copytree(versions,full/'controller_versions',dirs_exist_ok=True)
for suffix in ['.json','.jpg','.mp4']:
    for prefix in ['loop_verified','mobile_cycle','fixed_cycle','edge_cycle',args.movie]:
        for f in (run/'media').glob(prefix+'*'+suffix):cp(f,media/f.name)
for n in ['wall_as_built.jpg','wall_plan.png']:cp(run/'media'/n,media/n)
for name in ['verification.json','per_brick.csv','chain.json','transforms.jsonl','render_resource.csv','passage.json']:cp(run/'audit_live'/name,full/'audit'/name)
if (run/'fresh_first/verification.json').exists():shutil.copytree(run/'fresh_first',out/'evidence/fresh_start',dirs_exist_ok=True)
v=read(full/'audit/verification.json');chain=read(full/'audit/chain.json');events=log(full/'events.jsonl');rr=v['rows'];n=len(rr)
count=lambda k:len({e['brick_id'] for e in events if e['phase']==k})
good=[r['measured_cycle_wall_s'] for r in rr if r['time_scope']=='full' and r['measured_cycle_wall_s']]
travel=[r['drive_m'] for r in rr];duration=lambda file:float(command('ffprobe','-v','error','-show_entries','format=duration','-of','csv=p=0',file))
movie=read(media/(args.movie+'.json'));frames=movie['frames'];length=duration(media/(args.movie+'.mp4'));assert abs(frames/25-length)<.041
recover=[];all_trips={}
for path in sorted((run/'attempts').glob('*/events.jsonl')):
    source=log(path)
    if not source:continue
    for x in source:
        if x['phase']=='overload_trip':all_trips[(x['brick_id'],round(x['time'],6),x['stage'])]=x
    e=next((e for e in reversed(source) if e['phase']=='blocked'),None)
    if e:
        recover.append(dict(archive=path.parent.name,target=e['brick_id'],sim_time=e['time'],phase=e.get('phase_failed'),error=e['error']))
        cp(path,full/'attempts'/path.parent.name/'events.jsonl')
        for f in path.parent.glob('*_blocked.npz'):cp(f,full/'attempts'/path.parent.name/f.name)
for x in events:
    if x['phase']=='overload_trip':all_trips[(x['brick_id'],round(x['time'],6),x['stage'])]=x
data(full/'audit/interventions.json',dict(overload_across_attempts=list(all_trips.values()),discarded_attempts=recover,restore_events=[x for x in events if x['phase']=='restore']))
final=read(full/'status.json');stage=final['stage'];latest=Path(final['snapshot']).name
reposition={x['brick_id'] for x in events if x['phase']=='reposition' and x['distance']>.05}
summ=dict(picked=count('picked'),handover=count('handover'),placed=n,verified_courses=v['actual_course_counts'],last_sim_s=final['sim_time'],first_cycle_real_s=good[0],median_uninterrupted_wall_s=median(good),median_last_three_wall_s=median(good[-3:]),peak_wall_error_mm=max(r['wall_error_mm'] for r in rr),peak_wall_drift_mm=max(r['drift_mm'] for r in rr),max_gap_s=chain['max_sample_gap_s'],repositioned_bricks=sorted(reposition),peak_solo_slip_mm=max(r['rear_slip_mm'] for r in rr),overload_trips=sum(e['phase']=='overload_trip' for e in events),resume_events=sum(e['phase']=='restore' for e in events),source_snapshot=read(full/'source.json')['snapshot_id'],movie_s=length,first_movie_s=duration(media/'loop_verified.mp4'),mobile_movie_s=duration(media/'mobile_cycle.mp4'),render_s=movie['render_and_encode_s'],gpu=movie['renderer'],latest=latest,stage=stage)
elapsed=[r['elapsed_including_recovery_s'] for r in rr if r['elapsed_including_recovery_s'] is not None];summ['elapsed_through_return_s']=sum(elapsed)
slow=max((r['max_stage_wall_s'],r['max_measured_motion_stage'],r['brick']) for r in rr if r['max_stage_wall_s'] is not None)
summ['slowest_measured_motion_s'],summ['slowest_measured_motion'],summ['slowest_measured_brick']=slow
slowest_elapsed=max((r['elapsed_including_recovery_s'],r['brick']) for r in rr if r['elapsed_including_recovery_s'] is not None)
slowest_station=max((r['station_planning_s'],r['brick']) for r in rr)
summ['slowest_including_recovery']=slowest_elapsed;summ['slowest_station_plan']=slowest_station
summ['encode_feed_and_flush_wall_s']=movie.get('encoder_feed_and_flush_wall_s')
summ['course_freezes']=movie.get('course_freezes',[])
summ['overloads_across_all_attempts']=len(all_trips);summ['archived_attempts']=len(recover)
nudges=[x for x in events if x['phase']=='decluttered']
summ['physical_clearance_moves']=nudges
corner=next((x for x in events if x['phase']=='physical_corner_recovery'),None)
corner_text=(f"B{corner['brick_id']:03} 换位途中底盘触及工作台角，最深接触 {abs(corner['deepest_contact_m'])*1000:.3f} mm。控制器停止，车轮实际后退，净空恢复至 {corner['clearance_m']*1000:.1f} mm 后继续同一个目标。" if corner else "")
passage=read(full/'audit/passage.json')
nudge_text='未触发实体疏理。'
if nudges:
    from math import dist
    nudge_text=' '.join(f"B{x['brick_id']:03} 的两侧通道受邻砖限制。前臂指垫沿短端面实际推移 {1000*dist(x['translation'],[0,0,0]):.2f} mm，监测到的单个指垫接触峰值为 {x['peak_pad_force_N']:.2f} N，其他砖最大位移 {1000*x['max_neighbor_move_m']:.4f} mm。重新观测后才以双指抓起。"
                          for x in nudges)
data(full/'audit/summary.json',summ)
readme=f"""# 从 30 砖堆到地面墙

[主视频 · {length:.2f} 秒](media/{args.movie}.mp4) · [首块 B028 完整闭环 · {summ['first_movie_s']:.2f} 秒](media/loop_verified.mp4) · [B026 实际换位再抓 · {summ['mobile_movie_s']:.2f} 秒](media/mobile_cycle.mp4) · [下载答辩包](defense.zip) · [同一 B026，全程固定世界机位](media/fixed_cycle.mp4) · [侧立 B012 的窄面实抓与铺墙](media/edge_cycle.mp4) · [逐砖数据](evidence/full/audit/per_brick.csv) · [连续积分末态](evidence/full/snapshots/{latest})

从原始砖堆实抓 **{summ['picked']}** 块，后臂独持验收 **{summ['handover']}** 块，墙上稳定 **{n}/30** 块。按从低到高的顺序，各皮完成 {v['actual_course_counts']} 块，前三皮顺砖，第四皮丁砖。校验程序从已保存的有效末态继续积分 0.6 秒，逐个检查重力方向的实际承重接触、落点、砖体转角和松爪后的位移。最大落点误差 {summ['peak_wall_error_mm']:.3f} mm，0.6 秒内最大漂移 {summ['peak_wall_drift_mm']:.6f} mm。逐砖支撑实体和力以 N 记录在 CSV 中。

![实际墙体，俯视和斜视取自同一末态](media/wall_as_built.jpg)

[各皮规划图，统一比例](media/wall_plan.png) · [30 个世界系墙位、转角和承托面积](targets_world.json)

## 场景与控制

场景保留 A 初始快照的 30 个独立自由刚体、台面、砖尺寸、质量、位姿和速度。原始快照 ID 为 `{summ['source_snapshot']}`。初始位姿、源文件和状态哈希见 [source.json](evidence/full/source.json)。车型为四轮独立转向和驱动的 Ranger Mini V3，单一自由底盘携带两套 Gen3、双腕相机、前 Robotiq 双指和后 Schmalz Cobot Pump 顶吸机构。原始网格从服务器持久资产目录加载。当前输入使用 MuJoCo 真值位姿与接触，未运行视觉网络。

1. A 的增量三维支撑图以“下砖 → 上砖”为有向边。只在准许端点中按宽面朝上程度、高度和共同双侧条带筛选。候选砖 ID 对两个底盘位姿的一致性在同一初始砖堆实测。锁定后才检查当前车位与备选站位。正式记录中 {len(reposition)} 个目标先驱车换位再抓。
2. 前爪靠接触闭合并短试抬。HeldBrick 直接读实抓刚体 ID、两侧接触、前臂关节、工具到砖的 SE(3) 和砖尺寸。后臂根据实测指爪投影为完整密封圈选择吸点，求双臂交接姿态。吸附使用有效直径 25 mm、假设压差 70 kPa 的有限吸附模型。法向额定保持力 34.36 N，剪切、偏心倾覆及扭转力矩分别设上限。预载确认后卸载前爪。每块都记录后臂独持运动，实测最大相对滑移 {summ['peak_solo_slip_mm']:.3f} mm。
3. 底盘持砖行驶，实测运输路径各为 {min(travel):.3f}–{max(travel):.3f} m。墙体坐标固定于世界系。达到承重门槛后泄压、竖直撤离，后臂退出后按实际位置、姿态和支撑计数。末态核验再次检查第四皮每块丁砖至少由两块第三皮砖承重。
4. 每个选中、试抬、实抓输入、交接、行车、承重、泄压、落墙事件都带仿真时刻和砖号。[变换记录](evidence/full/audit/transforms.jsonl) 可交叉检查世界、底盘、前后工具和砖体的位姿。[连续性核验](evidence/full/audit/chain.json) 检查每块砖的自由关节、绑定对象、回放时刻、双指与吸盘阶段以及车辆真实位移。轨迹最大采样间隔 {chain['max_sample_gap_s']*1000:.3f} ms。每次续跑恢复 MuJoCo 全积分状态、执行器、求解器热启动、实际密封锚与控制目标；原始失败日志和检查点另存。

{nudge_text} 两砖原始表面最小间隙为 {passage['initial_surface_gap_m']*1000:.2f} mm。[两砖净空复核与实推动作](evidence/full/audit/passage.json)。

## 时间、镜头与干预

首块含返程的现实用时 {good[0]:.2f} s，仿真 {rr[0]['cycle_sim_s']:.3f} s。首三次完整循环中位现实用时 {median(good[:3]):.2f} s，最近三次完整循环中位 {median(good[-3:]):.2f} s。记录到的最长单项运动为 {slow[2]} 的 `{slow[1]}`，现实 {slow[0]:.2f} s。逐砖表区分完整循环、恢复后的实测部分和包含排错停顿的服务端经过时间。规划日志给出寻找站位的墙钟时间，各个运动阶段单列仿真时间与现实时间。实抓至最后一趟返程累计 {sum(elapsed)/60:.2f} 分钟，含诊断停顿。含恢复最慢的砖是 {slowest_elapsed[1]}，耗时 {slowest_elapsed[0]:.1f} s；单砖累计站位规划最长为 {slowest_station[1]} 的 {slowest_station[0]:.1f} s。未单独计时的计算、I/O 和进程停顿合并列示。首块以后移除了吸附状态下的重复约束求解，180 步一致性与耗时见 [测量](evidence/full/step_benchmark.json)。

物理与 IK/拓扑在 CPU 上运行。离线渲染实际设备为 **{movie['renderer']}**，主视频渲染和 H.264 编码流水线合计 {movie['render_and_encode_s']:.1f} s。主片 B{movie.get('feature_brick',26):03} 的取砖、交接、运输、铺砖和返程按 1× 仿真时间播放，遇到受阻砖时的实体疏理动作同样按 1× 播放，其余施工过程为 {movie.get('other_speed',20)}×。机位切换与静帧发生在同一条已记录时间线上。右侧三维图沿用 A 原始三维渲染样式，选砖和移除后显示相应仿真时刻的支撑图。末帧斜俯视为实际施工结果。

保留路径中有 {summ['resume_events']} 次恢复日志，保留轨迹中吸附超限脱开 {summ['overload_trips']} 次，计入已归档的失败试验后合计 {len(all_trips)} 个不同的超限事件。共有 {len(recover)} 份失败尝试历史归档。通过服务端检查点续跑，试验过程中有人修改控制程序、选择恢复点。{corner_text}[恢复记录及失败原因](evidence/full/audit/interventions.json) · [完整事件](evidence/full/events.jsonl) · [全部验收](evidence/full/audit/verification.json) · [模型和依赖版本](provenance.json)。

另以最终候选代码从原始砖堆冷启动，独立核验了首块 B028 实抓、交接、落墙及返程，期间恢复 0 次。[独立首块验收](evidence/fresh_start/verification.json)。该复跑单独计数。

## 在实验服务器复现

```bash
cd /home/cjx/disassembly/abc_course_final
source env.sh
bash run.sh status runs/main01
python audit.py runs/main01 runs/main01/audit_recheck
python chain_audit.py runs/main01 runs/main01/audit_recheck/chain.json
python transform_audit.py runs/main01 runs/main01/audit_recheck/transforms.jsonl
python film.py runs/main01 replay_main --until {n} --feature-slot 5 --speed {movie.get('other_speed',20)} --hd
bash run.sh start "runs/repro_$(date +%Y%m%d_%H%M%S)"
```

从已有现场继续时先复制整个运行目录，然后执行 `bash run.sh resume <运行副本> <运行副本>/snapshots/{latest}`。运行目录包含 MJCF、连续轨迹段、控制器与积分快照、原始事件和接触变化记录。正式运行使用多个调试修订，最新代码可运行完整流程；精确复核已交付实验可直接恢复快照或回放保存轨迹。后期工作进程的逐文件 SHA-256 及控制器源码在 evidence/full/controller_versions，早期版本可查阶段提交 01476b5（双指参考）、d174e5e（首块集成）及 75f6179（第一皮证据）。服务器环境 Python 3.10.19，MuJoCo 3.1.6。原始堆的 MuJoCo 积分状态、XML 与哈希收据在 evidence/full/original_pile。答辩包包括视频、末态、报告、代码和逐砖证据，完整历史轨迹可从同目录 evidence/full/segments 获取。
"""
(out/'README.md').write_text(readme)
print(summ)
