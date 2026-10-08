from pathlib import Path
import hashlib, json, os, re, subprocess, time
A=Path(__file__).resolve().parent; W=A.parent
prefix='corner-source-chart-full-20260926-v191'
r=json.loads((A/(prefix+'-terminal.json')).read_text())
assert r['all_processes_reaped'] and len(r['cases'])==498
assert all(c['returncode']==0 for c in r['cases'][:-1]+r['checks'])
assert r['cases'][-1]['returncode']=='timeout'
m=json.loads((A/r['source_manifest']).read_text())
def verify():
    for n,h in m.items():
        for root in [W,Path(r['source_directory']),Path(r['build_source_directory'])]:
            assert hashlib.sha256((root/n).read_bytes()).hexdigest()==h,n
    for b in r['binaries'].values():
        assert hashlib.sha256(Path(b['path']).read_bytes()).hexdigest()==b['sha256']
    for n,h in r['parents'].items():
        assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/n,text=True).strip()==h
verify()
target,name=r['selection'][498]; b=r['binaries'][target]
report=dict(qualification=prefix+'-terminal.json',source_manifest=r['source_manifest'],binary=b,name=name,all_processes_reaped=False)
receipt=A/(prefix+'-last-case-terminal.json'); assert not receipt.exists()
receipt.write_text(json.dumps(report,indent=2)+'\n')
command=[b['path'],'--exact',name,'--nocapture','--test-threads=1','--color','never']
log=A/(prefix+'-case-498.log')
start=time.monotonic()
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
with log.open('w') as out:
    try: code=subprocess.run(command,cwd=Path(r['build_source_directory'])/'hypercurve',env=env,stdout=out,stderr=subprocess.STDOUT,timeout=180).returncode
    except subprocess.TimeoutExpired:code='timeout'
verify()
if code==0: assert re.search(r'test result: ok\. 1 passed; 0 failed; 0 ignored;',log.read_text())
report.update(returncode=code,elapsed_seconds=time.monotonic()-start,command=command,log=log.name,all_processes_reaped=True,all_sources_and_binaries_unchanged=True)
receipt.write_text(json.dumps(report,indent=2)+'\n')
print('remaining case:',code,round(report['elapsed_seconds'],3),flush=True)
print(log.read_text()[-2000:],flush=True)
