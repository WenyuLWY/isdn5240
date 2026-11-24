"""Put the verified topology cut and its reproducible render in the delivery folder."""
import hashlib
import json
import shutil
import sys
from pathlib import Path

repo, source = map(Path, sys.argv[1:])
directory = repo / 'course_demo/abc_integrated'
rendered = source / 'release'
name = 'main_topology'
video = json.loads((rendered / f'{name}.json').read_text())
verified = json.loads((rendered / f'{name}_verification.json').read_text())
assert len(video['chapters']) == 30 and verified['all_frames_decode']
for file in [f'{name}.mp4', f'{name}.json', f'{name}_verification.json',
             'preview_05_selection.jpg', 'preview_05_lift.jpg', 'preview_05_after.jpg',
             'preview_18_lift.jpg', 'preview_30_after.jpg']:
    dest = directory / 'media' / (file if not file.startswith('preview_') else f'topology_{file}')
    shutil.copy2(rendered / file, dest)
for file in ['topology_panel.py', 'topology_cut.py', 'audit_bc_assets.py', 'verify_cut.py']:
    shutil.copy2(source / file, directory / file)
shutil.copy2(source / 'bc_model_audit.json', directory/'evidence/full/audit/bc_model_assets.json')
readme = directory/'README.md'
text = readme.read_text()
duration = video['duration_s']
mins, secs = divmod(duration, 60)
section = f"""## 逐砖三维拓扑讲解版

[拓扑讲解主视频 · {int(mins)} 分 {secs:.2f} 秒](media/main_topology.mp4) · [30 块的片内时间索引](media/main_topology.json) · [播放核验](media/main_topology_verification.json)

每块砖从选定到实抓抬起都显示右侧三维支撑图。砖体按仿真记录的尺寸、中心位置和姿态绘制，两侧使用同一高亮编号。箭头从承重的下方砖指向受支撑的上方砖。画面在锁定目标和移出支撑图后各留一次短停顿，方便讲解。合爪和试抬以 1× 播放，进爪 1.6×、抬离 1.35×、换位 3–6×、普通持砖行驶 16×、空手返程 20×。首块和换位示例 B026 的交接、运输、释放采用较慢的速度。

![B026 试抬与同期三维支撑图](media/topology_preview_05_lift.jpg)

渲染器直接加载最终运行的 MJCF 和原始轨迹。车型和末端来自最新 B+C 的 Schmalz 30 砖交付 `510e864`。逐项对照确认 49 个部件坐标系和 108 个机器人几何元素一致，20 个网格文件逐字节一致。前臂是 Robotiq 2F-85，后臂是 Schmalz Cobot Pump ECBPi 12。集成车增加了真实转向、驱动和圆柱轮胎碰撞体。[模型核对记录](evidence/full/audit/bc_model_assets.json) · [B+C 最新交付](../bc_wall_schmalz30/)

```bash
source env.sh
python topology_cut.py /home/cjx/disassembly/abc_course_final/runs/main01 /tmp/main_topology.mp4
```

"""
text = text.replace('[主视频 · 138.64 秒]', '[原版主视频 · 138.64 秒]', 1)
text = text.replace('## 场景与控制\n', section + '## 场景与控制\n', 1)
readme.write_text(text)
root = repo/'README.md'
text = root.read_text()
text = text.replace('[集成主视频](course_demo/abc_integrated/media/main.mp4)',
                    '[集成主视频 · 逐砖三维拓扑](course_demo/abc_integrated/media/main_topology.mp4)')
root.write_text(text)
manifest = {}
for file in sorted(directory.rglob('*')):
    if file.is_file() and file.name != 'SHA256.json':
        manifest[file.relative_to(directory).as_posix()] = hashlib.sha256(file.read_bytes()).hexdigest()
(directory/'SHA256.json').write_text(json.dumps(manifest, indent=1))
print('Scope: topology cut, screenshots, source, README and B+C model audit')
