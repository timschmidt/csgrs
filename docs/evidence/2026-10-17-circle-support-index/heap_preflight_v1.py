"""Observe complete-process allocations without modifying the measured executable."""
import hashlib
import json
import os
import resource
import subprocess
import time
from pathlib import Path
W = Path(__file__).resolve().parent
O = W / 'heap-preflight-v1'
def sha(p):
    with Path(p).open('rb') as f: return hashlib.file_digest(f,'sha256').hexdigest()
def read(p): return json.loads(Path(p).read_text())
def dump(p,v): Path(p).write_text(json.dumps(v,indent=2)+'\n')
assert read(W / 'preflight-comparison-v1/result.json')['all_ordered_exact_rows_equal']
binaries = {}
for label in ['baseline','candidate']:
    r=read(W / (label+'-validation-v1/binary.json')); p=Path(r['retained']);assert sha(p)==r['sha256'];binaries[label]=p
O.mkdir(exist_ok=False)
pins={str(p):sha(p) for p in [Path(__file__),*binaries.values(),W/'preflight-comparison-v1/result.json',Path('/usr/bin/heaptrack'),Path('/usr/bin/heaptrack_print')]}
dump(O/'protocol.json',{'input_sha256':pins,'count':512,
    'heaptrack_version':subprocess.check_output(['heaptrack','--version'],text=True),
    'print_version':subprocess.check_output(['heaptrack_print','--version'],text=True),
    'external_timeout_seconds':1800,'address_space_bytes':8*1024**3,
    'scope':'Allocation preflight of the entire identical test process, including input construction, cold and warm lookups, exact witness assertions, ordered output materialization/printing and cleanup. Instrumented latency is not a performance sample. This does not isolate live cache bytes or establish an internal memory bound.'})
env=os.environ|{'CIRCLE_CACHE_COST_N':'512','TMPDIR':str(O),'DEBUGINFOD_URLS':''}
rows=[]
def limits():
    resource.setrlimit(resource.RLIMIT_AS,(8*1024**3,8*1024**3));resource.setrlimit(resource.RLIMIT_CORE,(0,0))
def run(name,command):
    print('START '+name,flush=True);start=time.monotonic()
    with (O/(name+'.stdout')).open('wb') as out,(O/(name+'.stderr')).open('wb') as err:
        r=subprocess.run(['/usr/bin/time','-v','-o',str(O/(name+'.resources')),'timeout','--signal=TERM','--kill-after=20s','1800s',*command],
            cwd=O,env=env,stdout=out,stderr=err,preexec_fn=limits,start_new_session=True)
    row={'name':name,'command':command,'exit_code':r.returncode,'elapsed_seconds':time.monotonic()-start,
         **{s+'_sha256':sha(O/(name+'.'+s)) for s in ['stdout','stderr','resources']}}
    rows.append(row);dump(O/'commands.json',rows);print(json.dumps({k:row[k] for k in ['name','exit_code','elapsed_seconds']}),flush=True)
    return r.returncode
traces={}
for label,binary in binaries.items():
    prefix=O/(label+'-allocations')
    rc=run(label+'-record',['heaptrack','--record-only','-o',str(prefix),str(binary),'curve_intersection::circle_support_cost_probe::support_lookup_cost','--exact','--ignored','--nocapture','--test-threads=1'])
    if rc:continue
    files=[p for p in O.glob(label+'-allocations*') if p.suffix in ['.gz','.zst']]
    assert len(files)==1,files
    trace=files[0];traces[label]={'path':str(trace),'sha256':sha(trace),'bytes':trace.stat().st_size}
    run(label+'-print',['heaptrack_print','-f',str(trace),'-m','0','-n','5','-s','2','--disable-builtin-suppressions','--disable-embedded-suppressions'])
assert all(sha(p)==h for p,h in pins.items())
dump(O/'result.json',{'complete':True,'command_failures':[r['name'] for r in rows if r['exit_code']],
    'traces':traces,'read_only_inputs_unchanged':True,'native_runtime_admitted':False,'allocation_preflight_only':True})
