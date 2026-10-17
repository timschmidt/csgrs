"""Exercise independent symbolic/extreme contracts against a completed candidate."""
import gzip,hashlib,json,resource,subprocess,time
from pathlib import Path
W=Path(__file__).resolve().parent;ROOT=W.parents[2];O=W/'contracts-observation-v2'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
qualification=read(W/'candidate-validation-v2/result.json');assert qualification['complete']
with gzip.open(W/'candidate-preparation-v2/source-hashes.json.gz','rt') as f:inputs=json.load(f)
assert all(sha(W/'candidate-source-v2'/p)==h for p,h in inputs.items())
rows=[json.loads(s) for s in (W/'candidate-validation-v2/native-library.stdout').read_text().splitlines() if s.startswith('{')]
libraries={n:next(Path(p) for row in rows if row.get('reason')=='compiler-artifact' and row['target']['name']==n for p in row['filenames'] if p.endswith('.rlib')) for n in ['hypercurve','hyperlimit','hyperreal']}
deps=libraries['hyperlimit'].parent;readonly={str(p):sha(p) for p in deps.glob('*.rlib')}
pins={str(p):sha(p) for p in [Path(__file__),W/'contract_tests_v1.rs',W/'candidate-validation-v2/result.json',W/'candidate-preparation-v2/source-hashes.json.gz']}
O.mkdir(exist_ok=False);dump(O/'protocol.json',{'libraries':{n:str(p) for n,p in libraries.items()},'read_only_library_sha256':readonly,
    'input_sha256':pins,'timeout_seconds':900,'address_space_bytes':8*1024**3,'completed_targets_reopened':False,
    'preserved_failed_attempt':'contracts-observation-v1: dependency search used the primary release directory instead of deps; no test executed','scope':'Additional independent area, point-membership, bounds, reversal, reconstruction, policy and exact extreme-coordinate zero-budget contracts. Each test stops at its first failure.'})
commands=[]
def limit():
    resource.setrlimit(resource.RLIMIT_AS,(8*1024**3,8*1024**3));resource.setrlimit(resource.RLIMIT_CORE,(0,0))
def run(name,command):
    start=time.monotonic()
    with (O/(name+'.stdout')).open('wb') as out,(O/(name+'.stderr')).open('wb') as err:
        r=subprocess.run(['/usr/bin/time','-v','-o',str(O/(name+'.resources')),'timeout','--kill-after=10s','900s',*command],cwd=O,stdout=out,stderr=err,preexec_fn=limit,start_new_session=True)
    row={'name':name,'command':command,'exit_code':r.returncode,'elapsed_seconds':time.monotonic()-start,
        **{s+'_sha256':sha(O/(name+'.'+s)) for s in ['stdout','stderr','resources']}}
    commands.append(row);dump(O/'commands.json',commands);print(json.dumps({k:row[k] for k in ['name','exit_code','elapsed_seconds']}),flush=True);return r.returncode
flags=['rustc','--edition=2024','-D','warnings','-C','opt-level=3','-L','dependency='+str(deps)]
for n,p in libraries.items():flags+=['--extern',n+'='+str(p)]
assert run('compile',[*flags,'--test',str(W/'contract_tests_v1.rs'),'-o',str(O/'contracts')])==0
rc=run('contracts',[str(O/'contracts'),'--nocapture','--test-threads=1'])
assert all(sha(p)==h for p,h in pins.items()) and all(sha(p)==h for p,h in readonly.items())
assert all(sha(W/'candidate-source-v2'/p)==h for p,h in inputs.items())
dump(O/'result.json',{'complete':True,'contract_exit_code':rc,'source_and_libraries_unchanged':True,'main_sources_modified':False,'full_plan_complete':False})
