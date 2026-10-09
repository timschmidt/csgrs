import gzip,hashlib,json,os,shutil,subprocess,sys,time
from pathlib import Path
out=Path(sys.argv[1]).resolve()
build=json.loads((out/'build.json').read_text())
record=json.loads(Path(build['source_reference']).read_text())
def verify():
 assert all(hashlib.sha256(Path(name).read_bytes()).hexdigest()==digest for name,digest in record['source_hashes'].items())
verify();binary=Path(build['binary']);binary_hash=build['binary_sha256']
assert hashlib.sha256(binary.read_bytes()).hexdigest()==binary_hash
assert not (out/'perf.data').exists()
(out/'reproduce.py').write_text(Path(__file__).read_text())
environment={**os.environ,'DEBUGINFOD_URLS':''}
commands=[]
def run(label,command):
 verify();start=time.monotonic()
 with (out/(label+'.log')).open('w') as log:
  code=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,env=environment).returncode
 verify();assert hashlib.sha256(binary.read_bytes()).hexdigest()==binary_hash
 commands.append(dict(name=label,command=command,exit_code=code,wall_seconds=time.monotonic()-start))
 (out/'commands.json').write_text(json.dumps(commands,indent=2)+'\n')
 print(label,code,flush=True)
 return code
profile=out/'perf.data'
command=['perf','record','-F','49','-e','cycles:u','--call-graph','fp','--compression-level=5','--no-buildid-cache','-o',str(profile),'--',str(binary),'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite','--exact','--test-threads=1','--nocapture']
exit_code=run('record',command)
assert profile.exists() and profile.stat().st_size>0
assert run('self-cost',['perf','report','--stdio','--stdio-color','never','--no-children','--call-graph','none','--sort','symbol,dso','--percent-limit','0.05','-i',str(profile)])==0
assert run('inclusive-cost',['perf','report','--stdio','--stdio-color','never','--no-inline','--children','--call-graph','flat,1,caller','--sort','symbol,dso','--percent-limit','0.5','-i',str(profile)])==0
assert run('header',['perf','report','--header-only','--stdio','-i',str(profile)])==0
script_command=['perf','script','--show-lost-events','--no-inline','-F','comm,pid,tid,time,period,event,ip,sym,dso','-i',str(profile)]
with (out/'script-stderr.log').open('w') as error:
 process=subprocess.Popen(script_command,stdout=subprocess.PIPE,stderr=error,env=environment)
 with gzip.open(out/'stacks.txt.gz','wb',compresslevel=6) as zipped:shutil.copyfileobj(process.stdout,zipped)
 script_code=process.wait()
assert script_code==0
verify();assert hashlib.sha256(binary.read_bytes()).hexdigest()==binary_hash
(out/'manifest.json').write_text(json.dumps(dict(scope='Separate userspace sampled diagnostic profile of the unchanged full-board fixture. 49 Hz cycles:u sampling with frame-pointer call chains. Separate profiling-only binary enables Rust frame pointers and line tables. Timing is perturbed, and is not an admission result or matched speedup claim.',source_reference=build['source_reference'],source_unchanged=True,binary=str(binary),binary_sha256=binary_hash,binary_unchanged=True,commands=commands,script_command=script_command,script_exit_code=script_code,record_exit_code=exit_code,perf_version=subprocess.check_output(['perf','version'],text=True),artifacts={f.name:dict(bytes=f.stat().st_size,sha256=hashlib.sha256(f.read_bytes()).hexdigest()) for f in out.iterdir() if f.is_file() and f.name!='manifest.json'}),indent=2)+'\n')
print('profile complete',flush=True)
