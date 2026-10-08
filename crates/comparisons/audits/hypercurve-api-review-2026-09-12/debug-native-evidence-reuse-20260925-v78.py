from pathlib import Path
import hashlib,json,os,signal,subprocess,time
A=Path(__file__).resolve().parent
W=A.parent
prefix='native-evidence-reuse-20260925-v78-candidate'
r=json.loads((A/(prefix+'-terminal.json')).read_text())
assert r['all_processes_reaped'] and r['all_sources_unchanged']
manifest=json.loads((A/r['source_manifest']).read_text())
guard=json.loads((A/r['workspace_guard']).read_text())
root=A/'source-archives'/prefix
exe=Path(r['binaries']['hypercurve']['path'])
hashfile=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
self_hash=hashfile(Path(__file__))
def verify():
 assert hashfile(Path(__file__))==self_hash
 assert hashfile(exe)==r['binaries']['hypercurve']['sha256']
 for name,sha in guard.items():assert hashfile(W/name)==sha,name
 for name,sha in manifest.items():
  assert hashfile(root/name)==sha,name
  assert hashfile(Path(r['build_source_directory'])/name)==sha,name
verify()
command=['gdb','--nx','--nh','-batch','-iex','set debuginfod enabled off','-ex','set confirm off','-ex','set pagination off','-ex','set disable-randomization off','-ex','set print frame-arguments none','-ex','set print raw-values on','-ex','run','-ex','thread apply all backtrace 48','-ex','quit','--args',str(exe),'--exact','curve_region_boolean::certified_successor_tests::algebraic_chord_analytic_parallel_pair_replays_contacts_and_overlap','--nocapture','--test-threads=1','--color','never']
log=A/'native-evidence-reuse-20260925-v78-gdb-child.log'
assert not log.exists()
start=time.monotonic()
with log.open('w') as out:
 p=subprocess.Popen(command,stdout=out,stderr=subprocess.STDOUT,start_new_session=True,env=dict(os.environ,DEBUGINFOD_URLS=''))
 time.sleep(8)
 if p.poll() is None:p.send_signal(signal.SIGINT)
 try:code=p.wait(timeout=20)
 except subprocess.TimeoutExpired:
  os.killpg(p.pid,signal.SIGKILL)
  code=p.wait()
verify()
record=dict(command=command,pid=p.pid,returncode=code,elapsed_seconds=time.monotonic()-start,driver_sha256=self_hash,source_manifest=r['source_manifest'],workspace_guard=r['workspace_guard'],executable=str(exe),executable_sha256=hashfile(exe),gdb_process_reaped=True,all_inputs_unchanged=True)
(A/'native-evidence-reuse-20260925-v78-gdb-child-terminal.json').write_text(json.dumps(record,indent=2)+'\n')
print('gdb',code,'seconds',record['elapsed_seconds'])
print(log.read_text()[-18000:])
