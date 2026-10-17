"""Compile/check the new measurement oracle; these are not performance samples."""
import hashlib,json,os,resource,subprocess,time
from pathlib import Path
W=Path(__file__).resolve().parent;O=W/'cost-preflight-v1';O.mkdir(exist_ok=False)
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
assert read(W/'cost-probe-preparation-v1.json')['probe_sha256']==sha(W/'source_cost_probe_v1.rs')
assert read(W/'candidate-v3-cancellation-v1/result.json')['cargo_target_closed']
baseline=read(W/'reproduction-v1/protocol.json')
candidate=read(W/'aggregation-reproduction-v3/protocol.json')
libraries={'baseline':{n:Path(p) for n,p in baseline['libraries'].items()},
    'candidate-v3':{n:Path(p) for n,p in candidate['libraries'].items()}}
readonly=baseline['read_only_library_and_cli_sha256']|candidate['read_only_library_sha256']
for variant,libs in libraries.items():
    deps=libs['hyperdrc'].parent
    matches=list(deps.glob('libserde_json-*.rlib'));assert len(matches)==1
    libs['serde_json']=matches[0];assert str(matches[0]) in readonly
assert all(sha(p)==h for p,h in readonly.items())
inputs=O/'inputs';inputs.mkdir()
fields=[('empty','(start 0 0) (end 2 0) (width 0)'),('coincident_disk','(start 0 0) (end 0 0) (width 2)'),
    ('axis','(start 0 0) (end 2 0) (width 2)'),('rational_diagonal','(start 0 0) (end 3 4) (width 2)'),
    ('surd_diagonal','(start 0 0) (end 1 1) (width 2)'),
    ('decimal_translation','(start 9007199254740992.1 0) (end 9007199254740994.1 0) (width 2)'),
    ('subrange','(start 0 0) (end 2e-400 0) (width 2e-400)'),
    ('superrange','(start 1e400 0) (end 1e400 2) (width 2)'),
    ('disjoint_disks','(start 0 0) (end 0 0) (width 2)')]
fixtures=[]
for n,(label,f) in enumerate(fields):
    segments='(segment (layer F.Cu) (net 1) '+f+')'
    if n==8:segments+=' (segment (layer F.Cu) (net 1) (start 4 0) (end 4 0) (width 2))'
    p=inputs/(f'{n:02d}-'+label+'.kicad_pcb');p.write_text('(kicad_pcb (net 1 signal) '+segments+')\n')
    fixtures.append({'case':n,'label':label,'path':str(p),'sha256':sha(p)})
pins={str(p):sha(p) for p in [Path(__file__),W/'source_cost_probe_v1.rs',W/'source_cost_body_v1.rs',
    W/'cost-contract-v1.md',W/'cost-probe-preparation-v1.json',W/'aggregation-reproduction-v3/protocol.json',
    W/'reproduction-v1/protocol.json']}
dump(O/'protocol.json',{'input_sha256':pins,'read_only_library_and_cli_sha256':readonly,
    'libraries':{v:{n:str(p) for n,p in libs.items()} for v,libs in libraries.items()},'fixtures':fixtures,
    'performance_samples':False,'concurrent_candidate_build':True,'completed_targets_reopened':False,
    'timeout_seconds':1800,'address_space_bytes':8*1024**3})
commands=[]
def limits():
    resource.setrlimit(resource.RLIMIT_AS,(8*1024**3,8*1024**3));resource.setrlimit(resource.RLIMIT_CORE,(0,0))
def run(name,cmd):
    begin=time.monotonic()
    with (O/(name+'.stdout')).open('wb') as out,(O/(name+'.stderr')).open('wb') as err:
        r=subprocess.run(['/usr/bin/time','-v','-o',str(O/(name+'.resources')),'timeout','--kill-after=20s','1800s',*cmd],
            cwd=O,env=os.environ|{'TMPDIR':str(O)},stdout=out,stderr=err,preexec_fn=limits,start_new_session=True)
    row={'name':name,'command':cmd,'exit_code':r.returncode,'elapsed_seconds':time.monotonic()-begin,
        **{s+'_sha256':sha(O/(name+'.'+s)) for s in ['stdout','stderr','resources']}}
    commands.append(row);dump(O/'commands.json',commands)
    print(json.dumps({k:row[k] for k in ['name','exit_code','elapsed_seconds']}),flush=True)
    return r.returncode
results={};failed=False
for variant,libs in libraries.items():
    flags=['rustc','--edition=2024','-D','warnings','-C','opt-level=3','-L','dependency='+str(libs['hyperdrc'].parent)]
    for n,p in libs.items():flags+=['--extern',n+'='+str(p)]
    binary=O/(variant+'-probe')
    rc=run(variant+'-compile',[*flags,str(W/'source_cost_probe_v1.rs'),'-o',str(binary)])
    if rc:failed=True;continue
    for fixture in fixtures:
        name=variant+'-'+fixture['label'];snapshot=O/(name+'.snapshot.json')
        rc=run(name,[str(binary),str(fixture['case']),fixture['path'],str(snapshot)])
        if rc:failed=True;continue
        rows=read(O/(name+'.stdout'));states=read(snapshot)
        assert len(rows)==len(states)==2
        results[name]={'statuses':[r['status'] for r in rows],
            'certified_completion':[r['certified_completion'] for r in rows],
            'snapshot_sha256':sha(snapshot),'snapshot_bytes':snapshot.stat().st_size}
assert all(sha(p)==h for p,h in pins.items()) and all(sha(p)==h for p,h in readonly.items())
assert all(sha(r['path'])==r['sha256'] for r in fixtures)
dump(O/'result.json',{'complete':True,'command_failures':failed,'observations':results,
    'candidate_is_rejected_v3':True,'timing_and_allocation_values_excluded_from_final_comparison':True,
    'inputs_and_read_only_libraries_unchanged':True})
print(json.dumps({'complete':True,'command_failures':failed,'observations':results}),flush=True)
