"""Stage the compact cut alongside the full lecture version."""
import hashlib
import json
from pathlib import Path
import shutil
import sys

repo, source = map(Path, sys.argv[1:])
directory = repo / 'course_demo' / 'abc_integrated'
name = 'main_topology_2min'
rendered = source/'short'
verification = json.loads((rendered/f'{name}_verification.json').read_text())
assert verification['duration_s'] == 120 and verification['all_30_grasps_and_graph_updates']
for filename in (f'{name}.mp4', f'{name}.json', f'{name}_verification.json'):
    shutil.copy2(rendered/filename, directory/'media'/filename)
shutil.copy2(source/'topology_short.py', directory/'topology_short.py')
readme = directory/'README.md'
original = readme.read_text()
anchor = '## 逐砖三维拓扑讲解版\n'
assert anchor in original
first = '[2 分钟逐砖拓扑版](media/main_topology_2min.mp4)'
summary = (
    first + ' · [30 块的片内时间索引](media/main_topology_2min.json)'
    ' · [播放核验](media/main_topology_2min_verification.json)\n\n'
    '精简版从同一条正式轨迹渲染，保留全部 30 次选砖、合爪、试抬和拓扑更新。'
    '普通砖的拓扑侧栏保留约 2.4 秒，B026 的换位和交接给出更多时间，'
    '末段保留 B010 推移后重新选砖。运输、返程和重复交接使用加速蒙太奇。\n\n'
)
assert first not in original
readme.write_text(original.replace(anchor, anchor + '\n' + summary, 1))
landing = repo/'README.md'
original = landing.read_text()
previous = '[集成主视频 · 逐砖三维拓扑](course_demo/abc_integrated/media/main_topology.mp4)'
assert previous in original
landing.write_text(original.replace(previous,
    '[集成主视频 · 2 分钟版](course_demo/abc_integrated/media/main_topology_2min.mp4)'
    ' · [完整版 · 逐砖三维拓扑](course_demo/abc_integrated/media/main_topology.mp4)', 1))
hashfile = directory/'SHA256.json'
manifest = json.loads(hashfile.read_text())
for relpath in ('README.md', 'topology_short.py', f'media/{name}.mp4',
                f'media/{name}.json', f'media/{name}_verification.json'):
    manifest[relpath] = hashlib.sha256((directory/relpath).read_bytes()).hexdigest()
hashfile.write_text(json.dumps(manifest, indent=1))
