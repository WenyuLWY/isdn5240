import os,subprocess
rows=[r.split(', ') for r in subprocess.check_output(['nvidia-smi','--query-gpu=index,uuid,memory.free,utilization.gpu','--format=csv,noheader,nounits'],text=True).splitlines()]
allow=os.getenv('CUDA_VISIBLE_DEVICES')
pool=[r for r in rows if int(r[2])>1800 and int(r[3])<65 and (not allow or r[0] in allow.split(',') or r[1] in allow.split(','))]
if not pool:raise RuntimeError('no currently available rendering GPU')
gpu=min(pool,key=lambda r:(int(r[3]),-int(r[2])))
os.environ.update(MUJOCO_GL='egl',PYOPENGL_PLATFORM='egl',MUJOCO_EGL_DEVICE_ID=gpu[0])
