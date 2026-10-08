from pathlib import Path
import hashlib,json,os,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='stationary-boundary-20260927-v477'
r=json.loads((A/'stationary-corner-contract-20260927-v476-terminal.json').read_text());assert r['all_processes_reaped'] and r['probe_complete']
manifest=json.loads((A/r['source_manifest']).read_text());binary=A/'stationary-corner-contract-20260927-v476-tests'
def verify():
 assert hashlib.sha256(binary.read_bytes()).hexdigest()==r['binary_sha256']
 for name,sha in manifest.items():assert hashlib.sha256((W/name).read_bytes()).hexdigest()==sha,name
verify()
script=A/f'{prefix}.gdb'
assert not script.exists()
script.write_text("""set pagination off
set confirm off
set debuginfod enabled off
set print frame-arguments none
set print entry-values no
python
import gdb
class BoundaryBreakpoint(gdb.Breakpoint):
    hits=0
    def stop(self):
        self.hits+=1
        if self.hits<=16:
            print('BLOCKER_CALL '+str(self.hits))
            frame=gdb.newest_frame()
            for index in range(26):
                if frame is None:break
                print(str(index)+' '+str(frame.name()))
                frame=frame.older()
        return False
BoundaryBreakpoint('hypercurve::error::ExactCurveError::blocked')
end
run
quit
""")
report=dict(diagnostic_only=True,cases=[],all_processes_reaped=False,source_manifest=r['source_manifest'],binary_sha256=r['binary_sha256'])
for name in ['stationary_reparameterization_strict','one_sided_cusp_strict','interior_stationary_contact_strict']:
 log=A/f'{prefix}-{name}.log';started=time.monotonic()
 command=['/usr/bin/gdb','--nx','--quiet','--batch','-x',str(script),'--args',str(binary),'--exact',name,'--nocapture','--test-threads=1']
 with log.open('w')as out:
  process=subprocess.Popen(command,cwd=W,stdout=out,stderr=subprocess.STDOUT,env=dict(os.environ,DEBUGINFOD_URLS='',XDG_CACHE_HOME='/tmp/hypercurve-gdb-cache'),start_new_session=True)
  try:code=process.wait(timeout=45)
  except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait();code=124
  except BaseException:os.killpg(process.pid,signal.SIGKILL);process.wait();raise
 report['cases'].append(dict(name=name,returncode=code,stack_captured='BLOCKER_CALL' in log.read_text(),log=log.name,elapsed_seconds=time.monotonic()-started))
 print(report['cases'][-1],flush=True)
verify();report['all_processes_reaped']=True
(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
raise SystemExit(0 if all(c['stack_captured']for c in report['cases'])else 1)
