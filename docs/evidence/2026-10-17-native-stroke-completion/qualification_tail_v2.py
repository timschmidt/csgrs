"""Run dependent demo and comparison steps only after native validation succeeds."""
import hashlib,json,subprocess,time
from pathlib import Path
W=Path(__file__).resolve().parent;O=W/'qualification-tail-v2'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
O.mkdir(exist_ok=False)
pins={str(p):sha(p) for p in [Path(__file__),W/'requalify_curve_v1.py',W/'requalification_common_v1.py',W/'ui_v2.py',W/'ui_common_v1.py',W/'compare_v2.py',W/'cost_probe_v2.rs',W/'cost-contract-v1.md']}
start=time.time();dump(O/'protocol.json',{'input_sha256':pins,'started_epoch':start,'native_wait_limit_seconds':10800,
    'order':['candidate-validation-v4 (one all-features timeout)','curve-requalification-v1','ui-validation-v1','comparison-v1'],'overlapping_workload_measurements':False})
result_path=W/'candidate-validation-v4/result.json'
while not result_path.exists():
    assert time.time()-start<10800,'native qualification did not finish within the waiting limit'
    time.sleep(5)
native=read(result_path);assert native['complete'] and native['unexpected_exits']==['curve-all'],native
rows=[]
for runner in ['requalify_curve_v1.py','ui_v2.py','compare_v2.py']:
    assert all(sha(p)==h for p,h in pins.items())
    begun=time.time();print('START dependent '+runner,flush=True)
    result=subprocess.run(['python3',str(W/runner)],cwd=W)
    row={'runner':runner,'exit_code':result.returncode,'started_epoch':begun,'finished_epoch':time.time()}
    rows.append(row);dump(O/'commands.json',rows)
    assert result.returncode==0,row
dump(O/'result.json',{'complete':True,'commands':rows,'native_result_sha256':sha(result_path),
    'curve_requalification_result_sha256':sha(W/'curve-requalification-v1/result.json'),
    'ui_result_sha256':sha(W/'ui-validation-v1/result.json'),'comparison_result_sha256':sha(W/'comparison-v1/result.json'),
    'finished_epoch':time.time(),'main_sources_modified':False,'runtime_admitted':False})
print('Dependent qualification and comparison completed',flush=True)
