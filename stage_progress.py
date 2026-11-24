from pathlib import Path
import json,shutil,sys
root=Path(__file__).resolve().parent;run=root/'runs/main01'
out=Path('/mnt/data/cjx_data/brick-disassembly/code_v52/course_demo/abc_integrated')
out.mkdir(parents=True,exist_ok=True)
def cp(p,dest):
 dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest)
for n in ['integrated.py','scene.py','rig.py','demo.py','baseline_input.py','axes.py','vacuum.py','build_pump.py','scan.py','joint_path.py','clearance_check.py','restore.py','audit.py','film.py','graph_view.py','egl.py','env.sh','ik_seeds.json','a_reference.json','layout30.json','prototype.xml','provenance.json','benchmark_step.py']:
 cp(root/n,out/n)
if not (out/'upstream').exists():shutil.copytree(root/'upstream',out/'upstream')
for n in ['loop_verified.mp4','loop_verified_selection.jpg','loop_verified_held.jpg','loop_verified_dual.jpg','loop_verified_drive_wall.jpg','loop_verified_release.jpg','loop_verified.json']:
 cp(run/'media'/n,out/'media'/n)
for n in ['source.json','scene.xml','step_benchmark.json','cpu_tuning.json']:cp(run/n,out/'evidence'/n)
for f in sorted((run/'snapshots').glob('01_028_returned.npz')):cp(f,out/'evidence'/f.name)
for f in sorted((run/'segments').glob('0*')):
 if f.name[:3] not in ('000','002') or 'force' in f.name:continue
 cp(f,out/'evidence'/'first_cycle'/f.name)
e=list(map(json.loads,(run/'events.jsonl').read_text().splitlines()))
first=next(x for x in e if x['phase']=='cycle');f=out/'evidence/first_cycle/events.jsonl';f.write_text(''.join(json.dumps(x)+'\n' for x in e[:e.index(first)+1]))
(out/'README.md').write_text('''# Integrated pile-to-wall experiment

[Continuous first cycle, H.264](media/loop_verified.mp4) · [Timestamped first-cycle events](evidence/first_cycle/events.jsonl)

真实 30 砖堆与地面墙位在同一场景。此阶段归档展示第一块 B028 的完整取砖、试抬、双臂交接、持砖行车、落地和返程。其余砖的运行结果将单独核验。首块地面落点误差 0.44 mm，运输车程 0.610 m。成片在实时动作前后保留同步拓扑及末态短暂停留。

![Synchronized support graph](media/loop_verified_selection.jpg)

服务器工程位于 /home/cjx/disassembly/abc_course_final，正式运行目录为 runs/main01。恢复时使用快照内完整 MuJoCo 积分状态、吸附参数与控制器状态。源场景、A 版本及前后工具来源见 provenance.json 和 evidence/source.json。
''')
