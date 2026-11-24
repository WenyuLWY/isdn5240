#!/usr/bin/env bash
set -euo pipefail
repo=/mnt/data/cjx_data/brick-disassembly/code_v52
branch=codex/ranger-feedback-v52
cd "$repo"
test "$(git branch --show-current)" = "$branch"
test -z "$(git diff --cached --name-only)"
git fetch github "$branch"
git merge --no-edit FETCH_HEAD
test -z "$(git diff --cached --name-only)"
if [[ "${INDEX_ENTRY:-0}" == 1 ]]; then
 python /home/cjx/disassembly/abc_course_final/index_entry.py
 git add --sparse -- README.md
fi
git add --sparse -- course_demo/abc_integrated
git diff --cached --name-only | awk '$0 != "README.md" && $0 !~ /^course_demo\/abc_integrated\//{exit 1}'
git commit -m "$1"
for trial in 1 2 3; do
 if git push github HEAD:"refs/heads/$branch"; then break; fi
 git fetch github "$branch" && git merge --no-edit FETCH_HEAD
done
git fetch github "$branch"
git merge-base --is-ancestor HEAD FETCH_HEAD
git rev-parse HEAD
