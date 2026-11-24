"""Capture server filesystem timestamps once, preserving elapsed time across debugging continuations."""
from pathlib import Path
import sys,json
p=Path(sys.argv[1]);out=p/'checkpoint_clock.json';e=[json.loads(x) for x in (p/'events.jsonl').read_text().splitlines()]
metadata={f.name: f.stat().st_mtime_ns for f in (p/'snapshots').glob('*.npz')}
out.write_text(json.dumps(dict(unit='UTC Unix ns on the experiment host',snapshots=metadata),indent=1))
