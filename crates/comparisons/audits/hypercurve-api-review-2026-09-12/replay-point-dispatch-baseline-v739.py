from pathlib import Path
import hashlib,json,os,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='point-dispatch-baseline-v739';prior=json.loads((A/'unit-domain-core-20260928-v649-terminal.json').read_text());current=json.loads((A/'common-point-classification-v738-sources.json').read_text());old=json.loads((A/prior['source_manifest']).read_text());assert prior['all_processes_reaped'];assert json.loads((A/'common-point-classification-v738-reaped.json').read_text())['outer_exit_code']==1
binary=next(b['binaries']['hypercurve_curve_region_promotion']for b in prior['builds']if 'hypercurve_curve_region_promotion'in b.get('binaries',{}));name='nonconvex_algebraic_chord_expansion_is_exact_and_local_collapse_is_explicit'
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def verify():
 for n,sha in current.items():
  for root in [W,A/'build-workspace-20260925',A/'source-archives/common-point-classification-v738']:assert digest(root/n)==sha,(root,n)
 for n,sha in old.items():assert digest(Path(prior['source_directory'])/n)==sha,n
 assert digest(Path(binary['path']))==binary['sha256']
verify();log=A/f'{prefix}.log';start=time.monotonic();cmd=[binary['path'],'--exact',name,'--include-ignored','--nocapture','--test-threads=1']
with log.open('w')as out:
 p=subprocess.Popen(cmd,cwd=prior['source_directory']+'/hypercurve',stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
 try:rc=p.wait(timeout=240)
 except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait();rc=124
 except BaseException:os.killpg(p.pid,signal.SIGKILL);p.wait();raise
verify();s=log.read_text();report=dict(command=cmd,returncode=rc,elapsed_seconds=time.monotonic()-start,binary=binary,source_manifest=prior['source_manifest'],current_source_manifest='common-point-classification-v738-sources.json',log=log.name,all_processes_reaped=True,same_dispatch_assertion_failed=rc==101 and 'strictly separated recursive projective coordinates must avoid exact cross-product expansion'in s)
(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True);print(s[:950],flush=True)
