#!/usr/bin/env bash
set -euo pipefail
cd /home/cjx/disassembly/abc_course_final
source env.sh
run=runs/main01
while ! python - "$run/status.json" <<'PY'
import sys,json
try:s=json.load(open(sys.argv[1]));sys.exit(int(s['stage']!='complete' or len(s['placed'])!=30))
except (OSError,ValueError):sys.exit(1)
PY
do sleep 4; done
python timing_capture.py "$run"
python audit.py "$run" "$run/audit_live"
python chain_audit.py "$run" "$run/audit_live/chain.json"
python transform_audit.py "$run" "$run/audit_live/transforms.jsonl"
nvidia-smi --query-gpu=index,uuid,memory.free,utilization.gpu --format=csv >"$run/audit_live/render_resource.csv"
python -u film.py "$run" main --until 30 --feature-slot 5 --speed 22 --hd
ffmpeg -nostdin -v error -xerror -i "$run/media/main.mp4" -f null -
echo 'Validated final trajectory and decoded main film.'
