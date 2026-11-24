#!/usr/bin/env bash
set -euo pipefail
cd /home/cjx/disassembly/abc_course_final
source env.sh
run=runs/main01
dest=/mnt/data/cjx_data/brick-disassembly/code_v52/course_demo/abc_integrated
python inspect_movie.py "$run/media/main.mp4"
python delivery.py "$run" "$dest" --movie main
python pack.py "$dest"
python - "$dest" <<'PY'
from pathlib import Path
import re,json,sys,hashlib
root=Path(sys.argv[1]);links=re.findall(r'\]\(([^)]+)\)',(root/'README.md').read_text());assert all((root/x).exists() for x in links if not x.startswith(('https:','#')))
summary=json.loads((root/'evidence/full/audit/summary.json').read_text());assert (summary['picked'],summary['handover'],summary['placed'])==(30,30,30)
assert (root/'defense.zip').stat().st_size<96*1024**2
files={p.relative_to(root).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in root.rglob('*') if p.is_file() and p.name not in ('SHA256.json',)}
(root/'SHA256.json').write_text(json.dumps(files,indent=1))
print('Published directory ready for visual review:',len(files),'files',summary['movie_s'],'seconds')
PY
