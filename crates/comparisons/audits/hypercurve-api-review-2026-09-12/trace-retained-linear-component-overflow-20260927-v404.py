from pathlib import Path
import hashlib,json,os,subprocess,time,signal
A=Path(__file__).resolve().parent;W=A.parent;prefix='trace-retained-linear-component-overflow-20260927-v404';prior=json.loads((A/'retained-linear-component-replay-20260927-v403-terminal.json').read_text());assert prior['all_processes_reaped'];manifest=json.loads((A/prior['source_manifest']).read_text());binary=Path(prior['builds'][0]['binary']['path'])
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def verify():
 for n,h in manifest.items():
  for root in [W,Path(prior['source_directory']),A/'build-workspace-20260925']:assert sha(root/n)==h,(root,n)
 assert sha(binary)==prior['builds'][0]['binary']['sha256']
verify();env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()),RUST_BACKTRACE='full');cmd=[str(binary),'--exact','curve::curve_fillet::tests::nonlinear_linear_fillet_components_retain_unique_contacts_and_tangents','--nocapture','--test-threads=1'];log=A/f'{prefix}.log';start=time.monotonic();report=dict(source_manifest=prior['source_manifest'],binary=prior['builds'][0]['binary'],all_processes_reaped=False,command=cmd)
with log.open('w')as out:
 p=subprocess.Popen(cmd,cwd=A,env=env,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
 try:report['returncode']=p.wait(timeout=120)
 except BaseException:
  os.killpg(p.pid,signal.SIGKILL);p.wait();raise
report['elapsed_seconds']=time.monotonic()-start;verify();report['all_processes_reaped']=True;(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n');print(log.read_text()[-13000:],flush=True)
