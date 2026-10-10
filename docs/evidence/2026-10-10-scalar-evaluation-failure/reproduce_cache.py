"""Reproduce release-mode cache poisoning without any cancellation signal."""
import gzip,hashlib,json,resource,shutil,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
EVIDENCE=Path(__file__).resolve().parent
OUT=EVIDENCE/'cache-poison';OUT.mkdir()
WORK=ROOT/'target/scalar-evaluation-cache-poison';WORK.mkdir()
read=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
info=lambda p:{'bytes':p.stat().st_size,'sha256':sha(p)}
parent=read(EVIDENCE/'baseline-v3/summary.json');record=read(Path(parent['source_record']))
target=Path(record['target_directory'])
libraries={Path(p).stem.split('-')[0].removeprefix('lib'):Path(p) for p in parent['libraries']}
for p,expected in parent['libraries'].items():assert info(Path(p))==expected
shutil.copyfile(EVIDENCE/'cache_probe.rs',OUT/'probe.rs');shutil.copyfile(__file__,OUT/'reproduce.py')
guards=[]
for kind,base in [('source',Path(record['source_directory'])),('main',ROOT)]:
 p=Path(record[kind+'_hashes_file']);assert sha(p)==record[kind+'_hashes_file_sha256'];guards.append((base,json.loads(gzip.decompress(p.read_bytes()))))
def verify():
 assert all(sha(base/name)==expected for base,values in guards for name,expected in values.items())
 assert all(info(Path(p))==expected for p,expected in parent['libraries'].items())
def cap():
 resource.setrlimit(resource.RLIMIT_AS,(192*1024*1024,192*1024*1024));resource.setrlimit(resource.RLIMIT_CORE,(0,0))
commands=[]
def run(name,command,capped=False):
 start=time.monotonic()
 with (OUT/(name+'.stdout')).open('w') as stdout,(OUT/(name+'.stderr')).open('w') as stderr:
  result=subprocess.run(['timeout','--kill-after=2s','3' if capped else '120',*map(str,command)],cwd=ROOT,stdout=stdout,stderr=stderr,preexec_fn=cap if capped else None,check=False)
 commands.append({'name':name,'command':list(map(str,command)),'exit_code':result.returncode,'wall_seconds':time.monotonic()-start,'address_space_cap_mib':192 if capped else None,'timeout_seconds':3 if capped else 120})
 (OUT/'commands.json').write_text(json.dumps(commands,indent=2)+'\n');assert result.returncode==0,name
 return (OUT/(name+'.stdout')).read_text()
verify()
run('format',['rustfmt','--edition','2024','--check',OUT/'probe.rs'])
command=['rustc','--edition=2024','-O','-D','warnings',OUT/'probe.rs','-L','dependency='+str(target/'release/deps')]
for name in ['hyperreal','serde_json']:command+=['--extern',name+'='+str(libraries[name])]
run('build',command+['-o',WORK/'probe'])
rows=[]
for trial in range(3):
 for mode in ['control','cold-poison','warm-poison']:
  row=json.loads(run(str(trial)+'-'+mode,[WORK/'probe',mode],True));row['trial']=trial;rows.append(row)
  assert row['interval_excludes_pi']==(mode!='control')
verify()
summary={'source_record':parent['source_record'],'source_record_sha256':parent['source_record_sha256'],'libraries':parent['libraries'],'binary':info(WORK/'probe'),'rows':rows,'commands':commands,'guards_unchanged':True,'conclusion':'In release mode, a no-signal pi approximation at i32::MIN returns zero and poisons the shared constant cache. Subsequent ordinary-precision queries return zero and certified_rational_interval(-64) returns an interval below 3, which excludes pi. All three clean controls and six poisoned processes repeat; prewarming does not prevent poisoning.'}
summary['artifacts']={str(p.relative_to(OUT)):info(p) for p in OUT.iterdir() if p.is_file()}
(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print('Three controls and six false certified intervals reproduced.',flush=True)
