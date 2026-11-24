"""Audit an independent clean start through its first return, separate from the 30 counted bricks."""
from pathlib import Path
import sys,shutil,json,tempfile,subprocess
import numpy as np
from audit import audit
source,out=map(Path,sys.argv[1:]);out.mkdir(parents=True,exist_ok=True)
es=[json.loads(l) for l in (source/'events.jsonl').read_text().splitlines()]
cycle=next(e for e in es if e['phase']=='cycle');events=es[:es.index(cycle)+1];assert not any(e['phase'] in ('restore','blocked') for e in events)
i=cycle['brick_id'];snap=next((source/'snapshots').glob(f'01_{i:03d}_returned.npz'))
with tempfile.TemporaryDirectory() as tmp:
 t=Path(tmp);(t/'snapshots').mkdir();(t/'events.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in events))
 for k in ('source.json','scene.xml'): (t/k).symlink_to((source/k).resolve())
 for f in (source/'snapshots').glob(f'*_{i:03d}_*.npz'):(t/'snapshots'/f.name).symlink_to(f.resolve())
 (out/'events_first.jsonl').write_text((t/'events.jsonl').read_text())
 audit(t,out);q=json.loads((out/'verification.json').read_text());q['snapshot']=snap.name;(out/'verification.json').write_text(json.dumps(q,indent=1))
shutil.copy2(snap,out/snap.name)
for p in (source.parent/(source.name+'_versions')).glob('*/sha256.json'):shutil.copy2(p,out/'controller_sha256.json');break
