#!/usr/bin/env bash
set -euo pipefail
base=/home/cjx/disassembly/abc_course_final
cd "$base"
source env.sh
dir="$base/media_revision"
movie="$dir/release/main_topology.mp4"
python "$dir/verify_cut.py" "$movie" runs/main01 "$dir/bc_model_audit.json"
