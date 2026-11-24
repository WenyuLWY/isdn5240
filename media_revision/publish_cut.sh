#!/usr/bin/env bash
set -euo pipefail
base=/home/cjx/disassembly/abc_course_final
repo=/mnt/data/cjx_data/brick-disassembly/code_v52
branch=codex/ranger-feedback-v52
dir="$base/media_revision"
folder=course_demo/abc_integrated
cd "$base"
source env.sh
test -z "$(git -C "$repo" diff --cached --name-only)"
test "$(git -C "$repo" branch --show-current)" = "$branch"
test -z "$(git -C "$repo" status --porcelain -- README.md "$folder")"
git -C "$repo" fetch github "$branch"
git -C "$repo" merge --no-edit FETCH_HEAD
python "$dir/stage_cut.py" "$repo" "$dir"
git -C "$repo" add --sparse -- README.md "$folder/README.md" "$folder/SHA256.json" "$folder/topology_panel.py" "$folder/topology_cut.py" "$folder/audit_bc_assets.py" "$folder/verify_cut.py" "$folder/evidence/full/audit/bc_model_assets.json"
git -C "$repo" add --sparse -- "$folder"/media/main_topology* "$folder"/media/topology_preview*
git -C "$repo" diff --cached --check
git -C "$repo" diff --cached --name-only | awk '$0!="README.md" && $0 !~ /^course_demo\/abc_integrated\//{exit 1}'
git -C "$repo" commit -m 'Show synchronized 3D support graphs for all 30 integrated grasps'
git -C "$repo" push github HEAD:"refs/heads/$branch" ||
 {
   git -C "$repo" fetch github "$branch"
   git -C "$repo" merge --no-edit FETCH_HEAD
   git -C "$repo" push github HEAD:"refs/heads/$branch"
 }
rev=$(git -C "$repo" rev-parse HEAD)
git -C "$base/remote_readback" fetch --depth=1 --filter=blob:none github "$branch"
git -C "$base/remote_readback" cat-file -e "$rev^{commit}"
download="$dir/readback"
mkdir -p "$download"
for artifact in media/main_topology.mp4 media/main_topology.json media/main_topology_verification.json media/topology_preview_05_lift.jpg evidence/full/audit/bc_model_assets.json; do
 file="${artifact##*/}"
 git -C "$base/remote_readback" show "$rev:$folder/$artifact" >"$download/$file"
 cmp "$download/$file" "$repo/$folder/$artifact"
done
python "$dir/verify_cut.py" "$download/main_topology.mp4" runs/main01 "$download/bc_model_assets.json"
git -C "$base/remote_readback" show "$rev:README.md" | grep -F 'media/main_topology.mp4'
echo "$rev" >"$dir/published_rev.txt"
