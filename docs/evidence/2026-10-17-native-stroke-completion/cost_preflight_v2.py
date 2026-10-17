"""Compile and smoke the oracle/cost harness; observations are not benchmarks."""
import hashlib,json,resource,subprocess,time
from pathlib import Path
W=Path(__file__).resolve().parent;ROOT=W.parents[2];O=W/'cost-preflight-v2'
OLD=ROOT/'target/properties/kicad-track-failures'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
assert read(OLD/'candidate-validation-v3/result.json')['unexpected_exits']==[]
rows=[json.loads(s) for s in (OLD/'candidate-validation-v3/native-cli.stdout').read_text().splitlines() if s.startswith('{')]
libs={n:next(Path(p) for row in rows if row.get('reason')=='compiler-artifact' and row['target']['name']==n for p in row['filenames'] if p.endswith('.rlib')) for n in ['hypercurve','hyperlimit','serde_json']}
deps=libs['hyperlimit'].parent;readonly={str(p):sha(p) for p in deps.glob('*.rlib')}
pins={str(p):sha(p) for p in [Path(__file__),W/'cost_probe_v2.rs',W/'cost_body_v2.rs',W/'cost-contract-v1.md']}
O.mkdir(exist_ok=False);dump(O/'protocol.json',{'input_sha256':pins,'read_only_library_sha256':readonly,
    'preserved_failed_attempt':'cost-preflight-v1: borrow the owned radius value for serialization; no smoke run executed','purpose':'Harness preflight against qualified b1373812 implementation; excluded from cost comparisons.',
    'candidate_validation_concurrent':True,'no_original_timing_gate_running_at_start':True,'started_epoch':time.time(),
    'timeout_seconds':180,'address_space_bytes':8*1024**3})
commands=[]
def limit():
    resource.setrlimit(resource.RLIMIT_AS,(8*1024**3,8*1024**3));resource.setrlimit(resource.RLIMIT_CORE,(0,0))
def run(name,command):
    start=time.monotonic()
    with (O/(name+'.stdout')).open('wb') as out,(O/(name+'.stderr')).open('wb') as err:
        r=subprocess.run(['/usr/bin/time','-v','-o',str(O/(name+'.resources')),'timeout','--kill-after=10s','180s',*command],cwd=O,stdout=out,stderr=err,preexec_fn=limit,start_new_session=True)
    row={'name':name,'command':command,'exit_code':r.returncode,'elapsed_seconds':time.monotonic()-start,
        **{s+'_sha256':sha(O/(name+'.'+s)) for s in ['stdout','stderr','resources']}}
    commands.append(row);dump(O/'commands.json',commands);print(json.dumps({k:row[k] for k in ['name','exit_code','elapsed_seconds']}),flush=True);return r.returncode
flags=['rustc','--edition=2024','-D','warnings','-C','opt-level=3','-L','dependency='+str(deps)]
for n,p in libs.items():flags+=['--extern',n+'='+str(p)]
assert run('compile',[*flags,str(W/'cost_probe_v2.rs'),'-o',str(O/'probe')])==0
for case in [0,2,4]:
    assert run('case-'+str(case),[str(O/'probe'),str(case),'round',str(O/('case-'+str(case)+'.snapshot.json'))])==0
assert all(sha(p)==h for p,h in pins.items()) and all(sha(p)==h for p,h in readonly.items())
dump(O/'result.json',{'complete':True,'finished_epoch':time.time(),'not_benchmark_evidence':True,'read_only_inputs_unchanged':True})
