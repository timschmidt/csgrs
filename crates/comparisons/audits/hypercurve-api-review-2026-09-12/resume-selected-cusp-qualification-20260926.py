from pathlib import Path
import hashlib,json,os,re,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent
base='selected-cusp-ranges-isolated-20260926-v185';prefix=base+'-resumed'
report=json.loads((A/(base+'-terminal.json')).read_text())
assert report['all_processes_reaped']
baseline=json.loads((A/'selected-cusp-ranges-20260926-v185-chamfer-baseline-terminal.json').read_text())
assert baseline['all_processes_reaped'] and baseline['returncode']=='timeout' and baseline['test_file_unchanged']
assert len(report['cases'])==223 and report['cases'][-1]['returncode']=='timeout'
deferred=report['cases'].pop();deferred.update(target=report['selection'][222][0],name=report['selection'][222][1],baseline_comparison='selected-cusp-ranges-20260926-v185-chamfer-baseline-terminal.json')
report['deferred_cases']=[deferred];report['initial_qualification']=base+'-terminal.json'
report['all_processes_reaped']=False
manifest=json.loads((A/report['source_manifest']).read_text());guard=json.loads((A/report['workspace_guard']).read_text())
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
def save(): (A/(prefix+'-terminal.json')).write_text(json.dumps(report,indent=2)+'\n')
def verify():
    for repo,head in report['parents'].items():assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/repo,text=True).strip()==head
    for n,h in guard.items():
        assert hashlib.sha256((W/n).read_bytes()).hexdigest()==h,n
        for root in [report['source_directory'],report['build_source_directory']]:assert hashlib.sha256((Path(root)/n).read_bytes()).hexdigest()==manifest[n],n
    for b in report['binaries'].values():assert hashlib.sha256(Path(b['path']).read_bytes()).hexdigest()==b['sha256']
verify();save()
for index,(target,name) in enumerate(report['selection'][223:],223):
    binary=report['binaries'][target]['path'];label=f'case-{index:03}'
    report['active']=label;save();start=time.monotonic();log=A/(prefix+'-'+label+'.log')
    with log.open('w') as out:
        try:code=subprocess.run([binary,'--exact',name,'--nocapture','--test-threads=1','--color','never'],cwd=Path(report['build_source_directory'])/'hypercurve',env=env,stdout=out,stderr=subprocess.STDOUT,timeout=180).returncode
        except subprocess.TimeoutExpired:code='timeout'
    if code==0:assert re.search(r'test result: ok\. 1 passed; 0 failed; 0 ignored;',log.read_text()),name
    row=dict(target=target,name=name,label=label,returncode=code,elapsed_seconds=time.monotonic()-start,log=log.name)
    report['cases'].append(row);report.pop('active');save()
    print(label,code,round(row['elapsed_seconds'],3),log.read_text()[-2000:] if code else '',flush=True)
verify();report.update(all_processes_reaped=True,all_sources_unchanged=True,hypercurve_passed=sum(c['returncode']==0 for c in report['cases']),scoped_qualification_complete=all(c['returncode']==0 for c in report['cases']+report['checks']),qualification_complete=False)
save();print('All owned processes reaped; passed',report['hypercurve_passed'],'pre-existing timeouts',len(report['deferred_cases']),flush=True)
