#!/usr/bin/env bash
set -euo pipefail
base=/home/cjx/disassembly/abc_course_final
repo=/mnt/data/cjx_data/brick-disassembly/code_v52
bare="$base/remote_readback"
rev=${1:?published commit}
dir=course_demo/abc_integrated
branch=codex/ranger-feedback-v52
git -C "$bare" fetch --depth=1 --filter=blob:none github "$branch"
ref=$(git -C "$bare" rev-parse FETCH_HEAD)
if [[ "$ref" != "$rev" ]]; then git -C "$bare" fetch --depth=1 --filter=blob:none github "$rev"; fi
git -C "$bare" cat-file -e "$rev^{commit}"
dst="$base/verified_download/$rev"
mkdir -p "$dst"
git -C "$bare" ls-tree -r --name-only "$rev" -- "$dir" >"$dst/tree.txt"
for n in README.md media/main.mp4 media/mobile_cycle.mp4 media/loop_verified.mp4 media/fixed_cycle.mp4 defense.zip evidence/full/audit/summary.json; do
 name=${n##*/}
 git -C "$bare" show "$rev:$dir/$n" >"$dst/$name"
 cmp "$dst/$name" "$repo/$dir/$n"
 if [[ "$n" == *.mp4 ]]; then
   ffmpeg -nostdin -v error -xerror -i "$dst/$name" -f null -
   ffprobe -v error -select_streams v:0 -show_entries stream=codec_name,width,height,pix_fmt:format=duration -of json "$dst/$name" >"$dst/$name.verify.json"
 fi
done
unzip -tq "$dst/defense.zip"
git -C "$bare" show "$rev:README.md" | grep -q 'abc_integrated/media/main.mp4'
echo "Remote bytes, four decodes, archive and entry verified at $rev"
