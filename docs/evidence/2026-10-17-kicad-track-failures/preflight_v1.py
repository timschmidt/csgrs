"""Check evidence consumers against frozen v2 production; not final admission."""
import hashlib,json,resource,subprocess,time
from pathlib import Path
W=Path(__file__).resolve().parent;O=W/'consumer-preflight-v1'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
q=W/'candidate-validation-v2';assert read(q/'result.json')['complete']
rows=[json.loads(s) for s in (q/'native-cli.stdout').read_text().splitlines() if s.startswith('{')]
libs={name:next(Path(p) for row in rows if row.get('reason')=='compiler-artifact' and row['target']['name']==name for p in row['filenames'] if p.endswith('.rlib')) for name in ['hyperdrc','hypercurve','hyperlimit','serde_json']}
deps=libs['hyperlimit'].parent
pins={str(p):sha(p) for p in [Path(__file__),W/'load_cost_v1.rs',W/'disk_corpus_v1.rs',*(W/'source-disks-v1').iterdir(),*libs.values(),*deps.glob('*.rlib')] if p.is_file()}
O.mkdir(exist_ok=False)
dump(O/'protocol.json',{'inputs_sha256':pins,'purpose':'Evidence-consumer preflight against frozen v2 production. V2 unit and workload targets did not compile; final v3 qualification and comparison remain required.',
    'elapsed_limit_seconds_per_command':180,'address_space_limit_bytes':8*1024**3,'completed_cargo_targets_reopened':False})
def limits():
    resource.setrlimit(resource.RLIMIT_AS,(8*1024**3,8*1024**3));resource.setrlimit(resource.RLIMIT_CORE,(0,0))
commands=[]
def run(name,cmd):
    start=time.monotonic()
    with (O/(name+'.stdout')).open('wb') as out,(O/(name+'.stderr')).open('wb') as err:
        r=subprocess.run(['/usr/bin/time','-v','-o',str(O/(name+'.resources')),'timeout','--kill-after=10s','180s',*map(str,cmd)],cwd=O,stdout=out,stderr=err,preexec_fn=limits,start_new_session=True)
    row={'name':name,'command':list(map(str,cmd)),'exit_code':r.returncode,'elapsed_seconds':time.monotonic()-start,
        **{s+'_sha256':sha(O/(name+'.'+s)) for s in ['stdout','stderr','resources']}}
    commands.append(row);dump(O/'commands.json',commands);print(json.dumps({k:row[k] for k in ['name','exit_code','elapsed_seconds']}),flush=True)
    return r.returncode
flags=['rustc','--edition=2024','-D','warnings','-C','opt-level=3','-L','dependency='+str(deps)]
for name,p in libs.items():flags+=['--extern',name+'='+str(p)]
assert run('load-compile',[*flags,W/'load_cost_v1.rs','-o',O/'load'])==0
assert run('disk-compile',[*flags,W/'disk_corpus_v1.rs','-o',O/'disk'])==0
assert run('ordinary-load',[O/'load',W/'source-disks-v1/ordinary64.kicad_pcb',O/'ordinary.geometry.json'])==0
assert run('full-board-disk-oracle',[O/'disk',W/'source-disks-v1/hvp109a.json'])==0
assert len(read(O/'full-board-disk-oracle.stdout'))==15
assert all(sha(p)==h for p,h in pins.items())
dump(O/'result.json',{'complete':True,'all_commands_passed':True,'independent_disk_observations':15,'read_only_inputs_unchanged':True,'final_admission_established':False})
