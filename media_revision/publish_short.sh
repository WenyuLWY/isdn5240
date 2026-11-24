#!/usr/bin/env bash
set -euo pipefail
base=/home/cjx/disassembly/abc_course_final
repo=/mnt/data/cjx_data/brick-disassembly/code_v52
branch=codex/ranger-feedback-v52
dir="$base/media_revision"
folder=course_demo/abc_integrated
name=main_topology_2min
cd "$base"
source env.sh
test -z "$(git -C "$repo" diff --cached --name-only)"
test "$(git -C "$repo" branch --show-current)" = "$branch"
test -z "$(git -C "$repo" status --porcelain -- README.md "$folder")"
git -C "$repo" fetch github "$branch"
git -C "$repo" merge --no-edit FETCH_HEAD
python "$dir/publish_short.py" "$repo" "$dir"
git -C "$repo" add --sparse -- README.md "$folder/README.md" "$folder/SHA256.json" "$folder/topology_short.py" "$folder/media/$name.mp4" "$folder/media/$name.json" "$folder/media/${name}_verification.json"
git -C "$repo" diff --cached --check
git -C "$repo" commit -m 'Add a two-minute integrated 30-brick topology cut'
git -C "$repo" push github HEAD:"refs/heads/$branch" ||
 {
   git -C "$repo" fetch github "$branch"
   git -C "$repo" merge --no-edit FETCH_HEAD
   git -C "$repo" push github HEAD:"refs/heads/$branch"
 }
rev=$(git -C "$repo" rev-parse HEAD)
bare="$base/remote_readback"
git -C "$bare" fetch --depth=1 --filter=blob:none github "$branch"
git -C "$bare" cat-file -e "$rev^{commit}"
download="$dir/readback_short"
mkdir -p "$download"
for file in "$name.mp4" "$name.json" "${name}_verification.json"; do
  git -C "$bare" show "$rev:$folder/media/$file" >"$download/$file"
  cmp "$download/$file" "$repo/$folder/media/$file"
done
python "$dir/verify_cut.py" "$download/$name.mp4" runs/main01 "$dir/bc_model_audit.json"
git -C "$bare" show "$rev:README.md" | grep -F "/$name.mp4"
echo "$rev" >"$dir/short_published_rev.txt"
