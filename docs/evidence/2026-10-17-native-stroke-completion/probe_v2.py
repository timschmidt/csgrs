"""Correct diagnostic-only Segment2 construction; retain prior full regressions."""
import hashlib,json,re,resource,subprocess,time
from pathlib import Path
W=Path(__file__).resolve().parent;ROOT=W.parents[2];O=W/'direct-observation-v2';OLD=W/'reproduction-v2'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
p=read(OLD/'protocol.json');readonly=p['read_only_library_sha256'];inputs=p['source_inputs_sha256']
assert all(sha(k)==v for k,v in readonly.items()) and all(sha(ROOT/k)==v for k,v in inputs.items())
assert all(sha(k)==v for k,v in p['inputs_sha256'].items())
rows=read(OLD/'commands.json');assert [(r['name'],r['exit_code']) for r in rows]==[('curve_regressions-compile',0),('curve_regressions',101),('drc_regressions-compile',0),('drc_regressions',101),('direct-probe-compile',1)]
counts={}
for name in ['curve_regressions','drc_regressions']:
    row=next(r for r in rows if r['name']==name)
    for suffix in ['stdout','stderr','resources']:assert sha(OLD/(name+'.'+suffix))==row[suffix+'_sha256']
    m=re.findall(r'test result: (?:ok|FAILED)\. (\d+) passed; (\d+) failed;', (OLD/(name+'.stdout')).read_text());assert len(m)==1
    counts[name]={'passed':int(m[0][0]),'failed':int(m[0][1])}
O.mkdir(exist_ok=False)
protocol={'runner_sha256':sha(__file__),'probe_sha256':sha(W/'direct_probe_v2.rs'),'base_commit':p['base_commit'],
    'prior_protocol_sha256':sha(OLD/'protocol.json'),'prior_counts':counts,'prior_commands_sha256':sha(OLD/'commands.json'),
    'preserved_failed_attempts':['reproduction-v1: missing hyperreal extern; no tests ran','reproduction-v2: full regressions ran; diagnostic probe used unavailable Segment2 From conversion'],
    'completed_targets_reopened':False,'timeout_seconds':900,'address_space_bytes':8*1024**3}
dump(O/'protocol.json',protocol);commands=[]
def limit():
    resource.setrlimit(resource.RLIMIT_AS,(8*1024**3,8*1024**3));resource.setrlimit(resource.RLIMIT_CORE,(0,0))
def run(name,command):
    start=time.monotonic()
    with (O/(name+'.stdout')).open('wb') as out,(O/(name+'.stderr')).open('wb') as err:
        r=subprocess.run(['/usr/bin/time','-v','-o',str(O/(name+'.resources')),'timeout','--kill-after=10s','900s',*command],cwd=O,stdout=out,stderr=err,preexec_fn=limit,start_new_session=True)
    row={'name':name,'command':command,'exit_code':r.returncode,'elapsed_seconds':time.monotonic()-start,**{s+'_sha256':sha(O/(name+'.'+s)) for s in ['stdout','stderr','resources']}}
    commands.append(row);dump(O/'commands.json',commands);print(json.dumps({k:row[k] for k in ['name','exit_code','elapsed_seconds']}),flush=True);return r.returncode
libs=p['libraries'];deps=Path(libs['hypercurve']).parent
flags=['rustc','--edition=2024','-D','warnings','-C','opt-level=3','-L','dependency='+str(deps)]
for n,path in libs.items():flags+=['--extern',n+'='+path]
assert run('compile',[*flags,str(W/'direct_probe_v2.rs'),'-o',str(O/'probe')])==0
rc=run('probe',[str(O/'probe')])
assert all(sha(k)==v for k,v in readonly.items()) and all(sha(ROOT/k)==v for k,v in inputs.items())
assert sha(__file__)==protocol['runner_sha256'] and sha(W/'direct_probe_v2.rs')==protocol['probe_sha256']
dump(O/'result.json',{'complete':True,'probe_exit_code':rc,'counts':counts,'read_only_sources_and_libraries_unchanged':True,'main_sources_modified':False,'full_plan_complete':False})
