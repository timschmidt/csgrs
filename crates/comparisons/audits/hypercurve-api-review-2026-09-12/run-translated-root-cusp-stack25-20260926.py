from pathlib import Path
import hashlib, json, os, signal, subprocess, time
A=Path(__file__).resolve().parent
W=A.parent
prefix='translated-root-compaction-20260926-v196-cusp-stack25'
r=json.loads((A/'translated-root-compaction-20260926-v196-terminal.json').read_text())
q=r
assert r['all_processes_reaped'] and q['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text())
compiled=json.loads((A/q['source_manifest']).read_text())
binary=r['binaries']['hypercurve']
parents={name:subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/name,text=True).strip() for name in ['hypercurve','hypersolve','hyperreal']}
def verify():
 for name,sha in manifest.items():
  for root in [W,Path(r['source_directory'])]:
   assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
  assert hashlib.sha256((Path(q['build_source_directory'])/name).read_bytes()).hexdigest()==compiled[name],name
 assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest()==binary['sha256']
 for name,head in parents.items():
  assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/name,text=True).strip()==head,name
verify()
command=['/usr/bin/gdb','-nx','-nh','--batch']
options=['set pagination off','set confirm off','set debuginfod enabled off','set print frame-arguments none','set print entry-values no', 'set substitute-path '+r['build_source_directory']+' '+r['source_directory'], 'run','thread apply all bt 12','thread apply all bt -55','kill','quit']
for option in options: command.extend(['-ex',option])
command.extend(['--args',binary['path'],'--exact','curve_region_boolean::certified_successor_tests::selected_parallel_arrangement_splits_interior_cusps','--test-threads=1','--nocapture'])
assert not (A/(prefix+'-terminal.json')).exists()
start=time.monotonic();interrupted=False
with (A/(prefix+'.log')).open('w') as out:
 p=subprocess.Popen(command,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
 try: code=p.wait(timeout=25)
 except subprocess.TimeoutExpired:
  interrupted=True
  os.killpg(p.pid,signal.SIGINT)
  try: code=p.wait(timeout=20)
  except subprocess.TimeoutExpired:
   os.killpg(p.pid,signal.SIGKILL);p.wait();raise
verify()
report=dict(parents=parents,source_manifest=r['source_manifest'],build_source_manifest=q['source_manifest'],binary=binary,command=command,returncode=code,elapsed_seconds=time.monotonic()-start,interrupted=interrupted,all_sources_unchanged=True,all_processes_reaped=True,debugger_reaped=True,diagnostic_only=True)
(A/(prefix+'-terminal.json')).write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report),flush=True)
print((A/(prefix+'.log')).read_text()[-17000:],flush=True)
