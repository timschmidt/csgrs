"""Reproduce native failures against qualified libraries matching current inputs."""
import gzip,hashlib,json,re,resource,shutil,subprocess,time
from pathlib import Path
W=Path(__file__).resolve().parent;ROOT=W.parents[2];O=W/'reproduction-v1';P=W/'probe-source-v1'
OLD=ROOT/'target/properties/kicad-track-failures';E=ROOT/'docs/evidence/2026-10-17-kicad-track-failures'
BASE='b137381247a03342ce5b6184710475fcc22c9d63'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()==BASE
subprocess.run(['git','diff','--quiet'],cwd=ROOT,check=True);subprocess.run(['git','diff','--cached','--quiet'],cwd=ROOT,check=True)
qualification=read(OLD/'candidate-validation-v3/result.json');assert qualification['complete'] and not qualification['unexpected_exits']
with gzip.open(OLD/'candidate-preparation-v3/source-hashes.json.gz','rt') as f:qualified=json.load(f)
assert all(sha(OLD/'candidate-source-v3'/p)==h for p,h in qualified.items())
inputs=qualified|{'docs/properties-status.json':read(E/'checkpoint-v1.json')['ledger_after_sha256']}
assert len(inputs)==2832 and all(sha(ROOT/p)==h for p,h in inputs.items())
archive=ROOT/read(E/'checkpoint-v1.json')['candidate_snapshot']
O.mkdir(exist_ok=False);P.mkdir(exist_ok=False)
for crate,name in [('hypercurve','curve_regressions.rs'),('hyperdrc','drc_regressions.rs')]:
    original=archive/'crates'/crate/'tests'/('native_linear_stroke.rs' if crate=='hypercurve' else 'native_linear_strokes.rs')
    shutil.copy2(original,P/name);assert sha(P/name)==sha(original)
shutil.copytree(ROOT/'crates/hypercurve/tests/support',P/'support')
shutil.copy2(W/'direct_probe_v1.rs',P/'direct_probe_v1.rs')
pins={str(p):sha(p) for p in [Path(__file__),W/'contract-v1.md',W/'direct_probe_v1.rs',*P.rglob('*')] if p.is_file()}
rows=[json.loads(s) for s in (OLD/'candidate-validation-v3/native-cli.stdout').read_text().splitlines() if s.startswith('{')]
libraries={n:next(Path(p) for row in rows if row.get('reason')=='compiler-artifact' and row['target']['name']==n for p in row['filenames'] if p.endswith('.rlib')) for n in ['hypercurve','hyperlimit','hyperdrc','anyhow']}
deps=libraries['hypercurve'].parent;readonly={str(p):sha(p) for p in deps.glob('*.rlib')}
dump(O/'protocol.json',{'base_commit':BASE,'libraries':{n:str(p) for n,p in libraries.items()},'read_only_library_sha256':readonly,
    'source_inputs_sha256':inputs,'qualified_source_inputs_sha256':qualified,'inputs_sha256':pins,
    'qualification_result_sha256':sha(OLD/'candidate-validation-v3/result.json'),
    'qualification_source':'Previous completed target read-only; every current input equals its qualified source except the separately pinned status ledger.',
    'timeout_seconds_per_command':900,'address_space_bytes':8*1024**3,'completed_targets_reopened':False,
    'rustc':subprocess.check_output(['rustc','-vV'],text=True),'hardware':subprocess.check_output(['lscpu'],text=True)})
commands=[]
def limit():
    resource.setrlimit(resource.RLIMIT_AS,(8*1024**3,8*1024**3));resource.setrlimit(resource.RLIMIT_CORE,(0,0))
def run(name,command):
    start=time.monotonic()
    with (O/(name+'.stdout')).open('wb') as out,(O/(name+'.stderr')).open('wb') as err:
        r=subprocess.run(['/usr/bin/time','-v','-o',str(O/(name+'.resources')),'timeout','--kill-after=10s','900s',*command],cwd=O,stdout=out,stderr=err,preexec_fn=limit,start_new_session=True)
    row={'name':name,'command':command,'exit_code':r.returncode,'elapsed_seconds':time.monotonic()-start,
        **{s+'_sha256':sha(O/(name+'.'+s)) for s in ['stdout','stderr','resources']}}
    commands.append(row);dump(O/'commands.json',commands);print(json.dumps({k:row[k] for k in ['name','exit_code','elapsed_seconds']}),flush=True)
    return r.returncode
flags=['rustc','--edition=2024','-D','warnings','-C','opt-level=3','-L','dependency='+str(deps)]
for n,p in libraries.items():flags+=['--extern',n+'='+str(p)]
counts={}
for name in ['curve_regressions','drc_regressions']:
    assert run(name+'-compile',[*flags,'--test',str(P/(name+'.rs')),'-o',str(O/name)])==0
    rc=run(name,[str(O/name),'--nocapture','--test-threads=1']);assert rc in [0,101,124]
    matches=re.findall(r'test result: (?:ok|FAILED)\. (\d+) passed; (\d+) failed;', (O/(name+'.stdout')).read_text())
    counts[name]={'exit_code':rc,'passed':int(matches[0][0]) if matches else None,'failed':int(matches[0][1]) if matches else None}
assert run('direct-probe-compile',[*flags,str(P/'direct_probe_v1.rs'),'-o',str(O/'direct-probe')])==0
run('direct-probe',[str(O/'direct-probe')])
assert all(sha(p)==h for p,h in readonly.items()) and all(sha(p)==h for p,h in pins.items())
assert all(sha(ROOT/p)==h for p,h in inputs.items())
dump(O/'result.json',{'complete':True,'counts':counts,'read_only_libraries_and_sources_unchanged':True,
    'tests_unmodified':True,'scope':'Each regression stops at its first failure; no credit for later unexecuted checks.',
    'main_sources_modified':False,'full_plan_complete':False})
print(json.dumps({'complete':True,'counts':counts}),flush=True)
