"""Build a classroom download with verified films, report, per-brick records and restart state."""
import json,sys,zipfile
from pathlib import Path
p=Path(sys.argv[1]);q=p/'evidence/full';s=json.loads((q/'audit/summary.json').read_text());out=p/'defense.zip'
files=sorted(p.glob('*.py'))+sorted(p.glob('*.sh'))+sorted(p.glob('*.json'))+[p/'prototype.xml',p/'README.md']
files+=sorted((p/'upstream').glob('*'))
files+=[p/'media'/n for n in ('main.mp4','mobile_cycle.mp4','fixed_cycle.mp4','loop_verified.mp4','edge_cycle.mp4','main_wall.jpg','wall_as_built.jpg','wall_plan.png','main_selection.jpg','main_held.jpg','main_dual.jpg','main_drive_wall.jpg','main_release.jpg')]
files+=[q/n for n in ('scene.xml','source.json','events.jsonl','contact_changes.jsonl','status.json','oracle73_config.yaml','checkpoint_clock.json')]
files+=list((q/'original_pile').glob('*'))+list((p/'evidence/fresh_start').glob('*'))+list((q/'audit').glob('*'))+[q/'snapshots'/s['latest']]
with zipfile.ZipFile(out,'w',zipfile.ZIP_DEFLATED,compresslevel=3) as w:
 for f in files:
  if f.is_file():w.write(f,f.relative_to(p).as_posix())
with zipfile.ZipFile(out) as w:assert w.testzip() is None
print('classroom zip',out.stat().st_size,'bytes',len(files),'entries')
