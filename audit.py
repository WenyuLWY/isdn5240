"""Independently restore the latest accepted snapshot; check identity, wall, grip and transport."""
from rig import *
from baseline_input import wall
import csv,sys
from disassembly.oracle_support import SupportGraph
def audit(run,output):
    snapfiles=sorted((run/'snapshots').glob('*_placed.npz'))
    if not snapfiles:raise ValueError('no placed checkpoint yet')
    candidate=snapfiles[-1]
    for tag in ('returned','complete'):
        alternate=list((run/'snapshots').glob(candidate.name[:6]+'_'+tag+'.npz'))
        if alternate:candidate=alternate[0]
    z=np.load(candidate);accepted=list(map(int,z['accepted']));N=len(accepted);assert len(set(accepted))==N
    m=mj.MjModel.from_xml_path(str(run/'scene.xml'));d=mj.MjData(m);m.eq_data[:]=z['eq_data']
    mj.mj_setState(m,d,z['integration'],mj.mjtState.mjSTATE_INTEGRATION);mj.mj_forward(m,d)
    initial=[d.body(f'brick_{i:03}').xpos.copy() for i in accepted];rest=d.time
    seeds=json.loads((run/'source.json').read_text());assert len(set(b['id'] for b in seeds['bricks']))==30
    assert all(m.jnt_type[m.body_jntadr[m.body(f'brick_{i:03}').id]]==mj.mjtJoint.mjJNT_FREE for i in range(30))
    for _ in range(round(.6/m.opt.timestep)):mj.mj_step(m,d)
    mj.mj_forward(m,d)
    events=list(map(json.loads,(run/'events.jsonl').read_text().splitlines()))
    before=[r for r in events if r['time']<=rest+1e-8]
    rows=[];failures=[]
    clock_file=run/'checkpoint_clock.json'
    captured=json.loads(clock_file.read_text())['snapshots'] if clock_file.exists() else {}
    previous_return=None;previous_sim=None
    def one(k,i):return next(e for e in before if e['phase']==k and e['brick_id']==i)
    def load_below(i):
        b=f'brick_{i:03}_geom';ans={}
        for j,c in enumerate(d.contact):
            pair=[m.geom(g).name for g in [c.geom1,c.geom2]]
            if b not in pair:continue
            other=pair[1-pair.index(b)]
            if other!='ground' and (not other.startswith('brick_') or d.geom(other).xpos[2]>=d.geom(b).xpos[2]-.009):continue
            w=np.zeros(6);mj.mj_contactForce(m,d,j,w);fz=(c.frame.reshape(3,3).T@w[:3])[2]*(1 if pair[1]==b else -1)
            ans[other]=ans.get(other,0)+fz
        return {k:v for k,v in ans.items() if v>.01}
    for slot,i in enumerate(accepted):
        key=f'brick_{i:03}';b=d.body(key);xyz=b.xpos.copy();R=b.xmat.reshape(3,3);p,goal,course=wall()[slot];p=p+[0,.65,0]
        support=load_below(i)
        available={f'brick_{j:03}_geom' for j in accepted[:slot] if wall()[accepted.index(j)][2]==course-1} if course>0 else {'ground'}
        ok=bool(support) and set(support)<=available and (course!=3 or len(support)>=2)
        err=np.linalg.norm(xyz-p);drift=np.linalg.norm(xyz-initial[slot]);rot=min(angle(R@(goal@np.diag(t)).T) for t in [(1,1,1),(-1,-1,1),(1,-1,-1),(-1,1,-1)])
        pick=one('picked',i);h=one('handover',i);drive=one('drive_wall',i).copy();drive['distance']=sum(e['distance'] for e in before if e['brick_id']==i and e['phase'].startswith('drive_wall') and 'distance' in e);plan=one('select',i)
        assert key in plan['graph']['allowed']
        valid=ok and err<.009 and drift<.001 and rot<6 and pick['dz']>.032 and h['solo_m']>.025 and h['slip_m']<.005 and drive['distance']>.1
        if not valid:failures.append(i)
        def timestamp(stage):
            f=next((run/'snapshots').glob(f'*_{i:03}_{stage}.npz'),None)
            if f is None:return None
            if f.name in captured:return captured[f.name]/1e9
            return f.stat().st_mtime

        cycle=next((x for x in events if x['phase']=='cycle' and x['brick_id']==i),{})
        stamp0=timestamp('selection');stamp1=timestamp('returned')
        # Consecutive returned checkpoints include session pauses, retries, selection and physics.
        start_wall=previous_return if previous_return is not None else stamp0
        previous_return=stamp1
        sim_start=plan['time'] if previous_sim is None else previous_sim
        previous_sim=cycle.get('time',rest)
        planning=sum(e['wall_s'] for e in before if e['brick_id']==i and e['phase']=='station_plan')
        timing=cycle.get('stages',{})
        motion_wall=sum(val for k,val in timing.items() if not k.endswith('_sim'))
        remainder=cycle.get('wall_s',0.)-motion_wall

        stage_max=max(((v,k) for k,v in timing.items() if not k.endswith('_sim')),default=(None,None))
        rows.append(dict(slot=slot+1,brick=key,body_id=b.id,course=course+1,picked_dz_mm=pick['dz']*1000,rear_solo_mm=h['solo_m']*1000,rear_slip_mm=h['slip_m']*1000,drive_m=drive['distance'],parking_error_mm=drive['error_xy']*1000,wall_error_mm=err*1000,wall_angle_deg=rot,drift_mm=drift*1000,support_N=support,accepted=valid,cycle_sim_s=(previous_sim-sim_start),station_planning_s=planning,measured_control_loop_wall_s=motion_wall,other_compute_and_io_s=max(0.,remainder),elapsed_including_recovery_s=stamp1-start_wall if start_wall and stamp1 else None,measured_cycle_wall_s=cycle.get('wall_s'),time_scope=cycle.get('time_scope','full'),max_measured_motion_stage=stage_max[1],max_stage_wall_s=stage_max[0],stages=timing))
    for tag in ('picked','handover','placed'):
        assert [r['brick_id'] for r in before if r['phase']==tag]==accepted,tag
    result=dict(snapshot=str(candidate),sim_time=rest,relaxed_time=d.time,counts=[len(accepted)]*3,actual_course_counts=[sum(x['course']==k for x in rows) for k in range(1,5)],failed_final_checks=failures,rows=rows)
    output.mkdir(exist_ok=True,parents=True)
    (output/'verification.json').write_text(json.dumps(result,default=clean,indent=1))
    with (output/'per_brick.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader()
        for row in rows:w.writerow({k:json.dumps(v,default=clean) if isinstance(v,dict) else v for k,v in row.items()})
    assert not failures,failures
    print('verified',N,rest)
if __name__=='__main__':audit(Path(sys.argv[1]),Path(sys.argv[2]))
