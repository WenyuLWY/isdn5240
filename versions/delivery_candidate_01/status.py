import json,sys,statistics,time
from pathlib import Path
p=Path(sys.argv[1]);events=[json.loads(l) for l in (p/'events.jsonl').read_text().splitlines(True) if l.endswith('\n')]
s=json.loads((p/'status.json').read_text());cycles=[e for e in events if e['phase']=='cycle']
counts={k:len({e['brick_id'] for e in events if e['phase']==k}) for k in ['picked','handover','placed']}
full=[e['wall_s'] for e in cycles if e.get('time_scope','full')=='full']
last=events[-1];obs=[e for e in events if e['phase']=='restore']
print(json.dumps(dict(last=last['phase'],active=last['brick_id'],simulation_s=last['time'],counts=counts,snapshot=s['snapshot'],recent_three_complete_wall_median_s=statistics.median(full[-3:]) if len(full)>=3 else None,restores=len(obs),error=last.get('error','')[:250]),ensure_ascii=False))
