#!/usr/bin/env python3
"""Isolated integration: never connects to the deployed masters or HTTP ports."""
import concurrent.futures, fcntl, json, os, pathlib, socket, subprocess, time, urllib.request
ROOT = pathlib.Path(__file__).resolve().parents[1]
APP = ROOT.parent/'rocos-app'
OUT = ROOT/'build/hold-regression'
OUT.mkdir(parents=True, exist_ok=True)
ids = [50000000+os.getpid()*10+i for i in range(2)]
def port():
 with socket.socket() as s:
  s.bind(('127.0.0.1',0)); return s.getsockname()[1]
http, diag = port(), port()
while diag == http: diag = port()
def write(name, value):
 p=OUT/name; p.write_text(json.dumps(value,indent=2)); return str(p)
arms={}
for i,side in enumerate(('left','right')):
 master=write(side+'-master.json', {'master':{'id':ids[i],'cycle':1000,'state':'op'}})
 robot=write(side+'-robot.json', {'schema_version':1,'name':f'dualarm_{side}_shoulder_v1','package_version':'1.0.0','urdf':str(ROOT/'config/dualarm/urdf/dualarm_shoulder.urdf'),'base_link':side+'_base','tip_link':side.title()+'_Link7','ecat_id':ids[i],'hardware_config':str(ROOT/f'config/dualarm/hardware_{side}.yaml'),'master_type':'mujoco','master_config':master})
 arm=write(side+'-app.json', {'schema_version':1,'robot_config':robot,'device_config':str(APP/f'dualarm/config/devices/shoulder/{side}/device.yaml'),'tool_library':str(OUT/'tools'/side),'scripts':str(APP/'scripts'),'web_root':str(APP/'web')})
 arms[side]={'config':arm}
appcfg=write('app.json',{'schema_version':1,'http_host':'127.0.0.1','http_port':http,'http_control_ttl_seconds':300,'workcell':{'urdf':str(ROOT/'config/dualarm/urdf/dualarm_shoulder.urdf'),'asset_root':str(ROOT/'config/dualarm')},'diagnostics':{'host':'127.0.0.1','port':diag},'logging':{'directory':str(OUT/'logs'),'file_level':'info','console_level':'info'},'arms':arms})
simcfg=write('sim.json',{'model':str(ROOT/'config/dualarm/mjcf/scene_shoulder.xml'),'masters':[{'ecat_id':id,'hardware_config':str(ROOT/f'config/dualarm/hardware_{side}.yaml')} for id,side in zip(ids,arms)],'cycle_time_us':1000,'duration_sec':120,'visualize':False})
credentials={}; processes=[]; logfiles=[]; checks=[]
def request(side, path, body=None):
 data=None if body is None else json.dumps(body).encode()
 req=urllib.request.Request(f'http://127.0.0.1:{http}/api/arms/{side}/{path}',data=data,headers={'Content-Type':'application/json',**credentials.get(side,{})})
 with urllib.request.urlopen(req,timeout=3) as r: a=json.load(r)
 assert a['success'] and a['code'] in (0,100), (path,a)
 return a['data']
def state(side):
 a=request(side,'robot/state'); assert a['robot_state']!='ERROR_STATE',a; return a
def q(side): return [j['position'] for j in state(side)['joint_states']]
def err(a,b): return max(abs(x-y) for x,y in zip(a,b))
def wait(fn,timeout=15):
 end=time.monotonic()+timeout
 while time.monotonic()<end:
  if fn():return
  time.sleep(.02)
 raise AssertionError('timeout')
def check_hold(label, before, seconds=.5):
 peak={s:0 for s in before};end=time.monotonic()+seconds
 while time.monotonic()<end:
  for s in before: peak[s]=max(peak[s],err(q(s),before[s]))
  time.sleep(.02)
 assert max(peak.values())<3e-6,(label,peak)
 checks.append({'check':label,'peak_error_rad':peak})
def move(side, delta):
 target=q(side);target[1]+=delta
 request(side,'robot/movej',{'joints':target,'velocity':.1,'acceleration':.3,'jerk':5})
 def settled():
  a=state(side)
  return not a['motion_busy'] and err([j['position'] for j in a['joint_states']],target)<1e-6 and max(abs(j['velocity']) for j in a['joint_states'])<1e-5
 wait(settled)
 return target
try:
 for name,cmd in [('sim',[str(ROOT/'build/headless/bin/rocos_mujoco_sim'),'--config',simcfg]),('app',[str(APP/'build/dualarm/bin/rocosDualArmMain'),'--config',appcfg])]:
  if name=='app':
   def masters_ready():
    for id in ids:
     try:
      with open(f'/dev/shm/ecm_v2_{id}','rb') as f: header=f.read(80)
      if header[:8]!=b'ROCOSHM2' or int.from_bytes(header[68:72],'little')!=1:return False
     except FileNotFoundError:return False
    return True
   wait(masters_ready)
  log=open(OUT/(name+'.log'),'w');logfiles.append(log)
  processes.append(subprocess.Popen(cmd,cwd=APP if name=='app' else ROOT,stdout=log,stderr=subprocess.STDOUT))
 def ready():
  try:return all(state(s)['robot_state']=='IDLE' for s in arms)
  except (OSError,ValueError):return False
 wait(ready)
 check_hold('disabled startup',{s:q(s) for s in arms})
 for s in arms:
  lease=request(s,'control/acquire',{'client_id':'hold-regression','client_name':'isolated regression'})
  credentials[s]={'X-Rocos-Control-Token':lease['token'],'X-Rocos-Client-Id':lease['client_id']}
 for cycle in range(3):
  before={s:q(s) for s in arms}
  with concurrent.futures.ThreadPoolExecutor(2) as pool:
   list(pool.map(lambda s:request(s,'robot/enable',{}),arms))
  check_hold(f'enable cycle {cycle}',before)
  with concurrent.futures.ThreadPoolExecutor(2) as pool:
   goals=list(pool.map(lambda item:move(*item),[('left',.025),('right',-.025)]))
  checks.append({'check':f'independent concurrent MoveJ {cycle}','error_rad':{s:err(q(s),g) for s,g in zip(arms,goals)}})
  before={s:q(s) for s in arms}
  for s in arms:request(s,'robot/disable',{})
  check_hold(f'disable cycle {cycle}',before)
 left=q('left'); request('right','robot/enable',{});move('right',.025)
 check_hold('left stays braked while right moves',{'left':left})
 request('right','robot/disable',{})
 for s in arms: request(s,'control/release',{})
 result={'passed':True,'ecat_ids':ids,'http_port':http,'checks':checks}
 (OUT/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
finally:
 for p in reversed(processes):
  p.terminate()
  try:p.wait(timeout=5)
  except subprocess.TimeoutExpired:p.kill();p.wait()
 for f in logfiles:f.close()

 # SIGTERM can bypass the simulator destructor; remove only this run's IPC,
 # after its processes exited and the bus ownership lock is available.
 for id in ids:
  bus = pathlib.Path(f'/dev/shm/ecm_v2_{id}')
  if not bus.exists(): continue
  with bus.open('rb') as owner:
   fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
   for pattern in (f'ecm_v2_{id}', f'pd_input_v2_{id}', f'pd_output_v2_{id}', f'sem.sync_v2_{id}_*'):
    for file in pathlib.Path('/dev/shm').glob(pattern): file.unlink(missing_ok=True)
