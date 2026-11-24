#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
source env.sh
mode=${1:?start, resume, or status}
run=${2:?run directory}
case "$mode" in
 status)
  python status.py "$run"
  ;;
 start)
  test ! -e "$run"; mkdir -p "$(dirname "$run")"
  python source_version.py "$run" start "${3:-30}"
  exec flock -n "$run.lock" python -u integrated.py "$run" --limit "${3:-30}"
  ;;
 resume)
  ck=${3:?existing .npz checkpoint}
  test -s "$ck" && test -s "$run/scene.xml"
  # Legacy workers also share an exact run path; never start a second integration.
  python - "$run" <<'PY'
import os,sys,pathlib
target=pathlib.Path(sys.argv[1]).resolve()
for p in pathlib.Path('/proc').iterdir():
 if not p.name.isdigit() or p.name==str(os.getpid()):continue
 try:
  args=(p/'cmdline').read_bytes().decode().rstrip('\0').split('\0')
  j=next((j for j,a in enumerate(args) if pathlib.Path(a).name in ('integrated.py','restore.py')),None)
  if j is not None and j+1<len(args) and ((p/'cwd').resolve()/args[j+1]).resolve()==target:raise SystemExit(f'active worker PID {p.name}')
 except (FileNotFoundError,PermissionError,UnicodeError):continue
PY
  python source_version.py "$run" resume "$ck"
  exec flock -n "$run.lock" python -u restore.py "$run" "$ck"
  ;;
 *)exit 2;;
esac
