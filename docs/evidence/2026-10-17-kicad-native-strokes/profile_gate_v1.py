"""Sample only the verified failing candidate; this run cannot certify latency."""
import hashlib,json,subprocess,time
from pathlib import Path
W=Path(__file__).resolve().parent;O=W/'gate-profile-v1';O.mkdir(exist_ok=False)
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
record=json.loads((W/'gate-observation-v2/result.json').read_text());assert len(record['processes'])==1
p=record['processes'][0];pid=p['pid']
assert Path('/proc',str(pid),'cmdline').read_bytes().decode().split('\0')==p['args']
assert sha(p['args'][0])==p['exe_sha256']
assert any(t['cpu_seconds']>120 for t in p['tasks'])
commands=[]
for name,command in [('sample',['perf','record','-F','99','-g','-o',str(O/'perf.data'),'-p',str(pid),'--','sleep','3'])]:
    start=time.monotonic()
    with (O/(name+'.stdout')).open('wb') as out,(O/(name+'.stderr')).open('wb') as err:
        r=subprocess.run(command,stdout=out,stderr=err,timeout=20)
    commands.append({'name':name,'command':command,'exit_code':r.returncode,'elapsed_seconds':time.monotonic()-start})
if r.returncode==0:
    name='report';command=['perf','report','--stdio','--no-children','--sort','symbol','--percent-limit','1','-i',str(O/'perf.data')]
else:
    name='backtrace';command=['gdb','-q','-n','-batch','-p',str(pid),'-ex','set pagination off','-ex','thread apply all bt 24','-ex','detach']
start=time.monotonic()
with (O/(name+'.stdout')).open('wb') as out,(O/(name+'.stderr')).open('wb') as err:
    r=subprocess.run(command,stdout=out,stderr=err,timeout=30)
commands.append({'name':name,'command':command,'exit_code':r.returncode,'elapsed_seconds':time.monotonic()-start})
(O/'result.json').write_text(json.dumps({'complete':True,'runner_sha256':sha(__file__),
    'observation_sha256':sha(W/'gate-observation-v2/result.json'),'pid':pid,'exe_sha256':p['exe_sha256'],
    'commands':commands,'profiled_run_is_not_performance_admission_evidence':True,
    'file_sha256':{x.name:sha(x) for x in O.iterdir() if x.is_file()}},indent=2)+'\n')
print(json.dumps({'complete':True,'commands':commands}))
