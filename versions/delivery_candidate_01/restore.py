"""Restore an exact checkpoint of this run, retaining the discarded attempt for audit."""
from integrated import *
import shutil
def restore(run,checkpoint):
    s=object.__new__(Integrated);s.reopening=True;Integrated.__init__(s,run)
    ck=np.load(checkpoint);s.d.time=float(ck['time']);stamp=s.d.time
    archive=run/'attempts'/time.strftime('%Y%m%d-%H%M%S');archive.mkdir(parents=True)
    for f in ['events.jsonl','contact_changes.jsonl','status.json']:
        if (run/f).exists():shutil.copy2(run/f,archive/f)
    s.events=list(map(json.loads,(run/'events.jsonl').read_text().splitlines()))[:int(ck['event_n'])]
    (run/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in s.events))
    log=run/'contact_changes.jsonl';s.contacts_file.close()
    log.write_text(''.join(l+'\n' for l in log.read_text().splitlines() if json.loads(l)[0]<=stamp))
    s.contacts_file=log.open('a')
    for f in sorted((run/'segments').glob('*.npz')):
        z=np.load(f);force='samples' in z;ts=z['samples'][:,0] if force else z['time']
        if not len(ts) or ts[-1]<=stamp+1e-8:continue
        kept=ts<=stamp+1e-8
        vals={k:z[k][kept] for k in z.files} if any(kept) else None;z.close();shutil.move(str(f),archive/f.name)
        if vals:np.savez_compressed(f,**vals)
    for f in sorted((run/'snapshots').glob('*.npz')):
        if f==checkpoint:continue
        with np.load(f) as z:future=float(z['time'])>stamp+1e-8
        if future:shutil.move(str(f),archive/f.name)
    mj.mj_setState(s.m,s.d,ck['integration'],mj.mjtState.mjSTATE_INTEGRATION);s.m.eq_data[:]=ck['eq_data']
    s.goals=ck['goals'].copy();s.home_joints=ck['home'].copy();s.i=int(ck['active']);s.picked=list(map(int,ck['picked']));s.accepted=list(map(int,ck['accepted']));s.transfers=int(ck['transfers']);s.tick=int(ck['tick']);s.fwd()
    if bool(ck['vac_on']):
        v=s.vac;v.on=True;v.local_anchor=ck['anchor'].copy();v.initial_axes=ck['axes'].copy();v.face_sign=int(ck['sign']);v.eq=s.m.equality(f'seal_{s.i:03}').id;v.v=s.m.jnt_dofadr[s.m.body_jntadr[s.bodyids[s.i]]];assert s.d.eq_active[v.eq]
    s.event('restore',snapshot=str(checkpoint),archived_attempt=str(archive),stage=str(ck['stage']))
    return s,str(ck['stage'])
def resume(s,stage,limit):
    start=s.d.time;real=time.monotonic()
    if stage=='blocked':
        last=next(e for e in reversed(s.events) if e['phase']=='blocked')
        if last.get('phase_failed')=='clear' and s.vac.on and not any(s.group(x)=='front' for x in s.hits()):
            s.finish_handover((-s.vac.initial_axes.T@s.vac.local_anchor,s.vac.initial_axes.T))
            s.segment('receiver');s.snapshot('receiver');stage='receiver'
        elif last.get('phase_failed') in ('test_lift','extract_plan') and s.i in s.picked and s.bilateral(0) and not s.vac.on:
            s.deliver_held();s.segment('held');s.snapshot('held');stage='held'
        elif last.get('phase_failed')=='lift_free' and s.vac.on and not any(s.group(x)=='front' for x in s.hits()):
            s.staged_release=bool(s.d.ctrl[s.fingers[0]]>3)
            s.retreat_after_lift((-s.vac.initial_axes.T@s.vac.local_anchor,s.vac.initial_axes.T))
            s.segment('receiver');s.snapshot('receiver');stage='receiver'
        elif last.get('phase_failed')=='unload' and s.vac.on and not any(s.group(x)=='front' for x in s.hits()):
            s.staged_release=bool(next((e.get('staged') for e in reversed(s.events) if e['phase']=='donor_release_command' and e['brick_id']==s.i),False))
            s.after_donor_release((-s.vac.initial_axes.T@s.vac.local_anchor,s.vac.initial_axes.T))
            s.segment('receiver');s.snapshot('receiver');stage='receiver'
        elif last.get('phase_failed')=='present' and not s.vac.on and s.bilateral(0):
            s.event('present_recovery',brick_in_tool=rel(s.site(0),s.brick()),contacts=s.hits())
            stage='held'
        elif last.get('phase_failed','').startswith('drive_wall') and s.vac.on and any(e['phase']=='handover' and e['brick_id']==s.i for e in s.events):
            stage='receiver'
        elif last.get('phase_failed','').startswith('return') and s.i in s.accepted and not s.vac.on:
            if s.bad(s.d):raise ContactError('current empty-arm collision requires a physical retreat')
            s.event('empty_parking_alternative',reason=last['error'],measured=[s.site(0),s.site(1)])
            s.segment('placed');s.snapshot('placed');stage='placed'
        elif last.get('phase_failed','').startswith('reposition') and not s.vac.on:
            s.clear_table_corner()
            s.move('parking_settle',.35)
            s.event('drive_recovery',measured_station=s.base_pose(),speed=np.linalg.norm(s.d.qvel[s.carv:s.carv+2]))
            stage='selection'
        else:raise RuntimeError('choose the last recoverable stage, or implement a safe continuation for this blocker')
    if stage=='held':
        h=HeldBrick(s.i,2*s.m.geom(f'brick_{s.i:03}_geom').size.copy(),s.brick(),s.d.qpos[s.q[0]].copy(),rel(s.site(0),s.brick()),s.hits())
        if not s.bilateral(0):raise ContactError('held checkpoint has no opposed pad contact')
        s.transfer(h);s.segment('receiver');s.snapshot('receiver');stage='receiver'
    if stage=='released':
        s.staged_release=bool(s.d.ctrl[s.fingers[0]]>3)
        s.after_donor_release((-s.vac.initial_axes.T@s.vac.local_anchor,s.vac.initial_axes.T))
        s.segment('receiver');s.snapshot('receiver');stage='receiver'
    if stage=='receiver':
        s.place();s.segment('placed');s.snapshot('placed');stage='placed'
    if stage=='at_wall':
        s.place_at_wall(*s.targets[len(s.accepted)]);s.segment('placed');s.snapshot('placed');stage='placed'
    if stage=='placed':
        s.drive(np.array([-.18,.32]),np.pi/2,'return_pile')
        s.event('cycle',slot=len(s.accepted)-1,wall_s=time.monotonic()-real,sim_s=s.d.time-start,stages=dict(s.clock),time_scope='restored portion')
        s.segment('return');s.snapshot('returned')
    while len(s.accepted)<limit:s.cycle()
    s.event('complete',counts=[len(s.picked),s.transfers,len(s.accepted)]);s.segment('end');s.snapshot('complete')
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('run',type=Path);ap.add_argument('checkpoint',type=Path);ap.add_argument('--limit',type=int,default=30);ar=ap.parse_args()
    s,stage=restore(ar.run,ar.checkpoint)
    try:resume(s,stage,ar.limit)
    except Exception as e:
        s.event('blocked',error=str(e),phase_failed=s.phase,brick=s.brick(),tools=[s.site(a) for a in [0,1]],contacts=s.hits(),traceback=traceback.format_exc());s.segment('blocked');s.snapshot('blocked');raise
