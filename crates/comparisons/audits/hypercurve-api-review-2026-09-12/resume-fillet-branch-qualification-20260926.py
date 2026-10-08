from pathlib import Path
import hashlib,json,os,re,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent
base='fillet-branches-isolated-20260926-v190';prefix=base+'-resumed'
report=json.loads((A/(base+'-terminal.json')).read_text());assert report['all_processes_reaped']
assert len(report['cases'])==129 and report['cases'][-1]['returncode']==101
comparison_name='fillet-branches-20260926-v190-arc-boundary-comparison-terminal.json'
comparison=json.loads((A/comparison_name).read_text());assert comparison['all_processes_reaped']
assert [(c['role'],c['returncode']) for c in comparison['cases']]==[('parent',101),('combined',0)]
parent=json.loads((A/'selected-cusp-ranges-isolated-20260926-v185-resumed-terminal.json').read_text())
combined=json.loads((A/'selected-cusp-fillet-fixed-20260926-v189-terminal.json').read_text())
guard=json.loads((A/report['workspace_guard']).read_text());manifest=json.loads((A/report['source_manifest']).read_text())
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
deferred=report['cases'].pop();deferred.update(target=report['selection'][128][0],name=report['selection'][128][1],baseline_comparison=comparison_name)
report.update(deferred_cases=[deferred],initial_qualification=base+'-terminal.json',all_processes_reaped=False,baseline_comparisons=[])
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save():(A/(prefix+'-terminal.json')).write_text(json.dumps(report,indent=2)+'\n')
def verify():
    for repo,head in report['parents'].items():assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/repo,text=True).strip()==head
    for n,h in guard.items():assert digest(W/n)==h,n
    for r in [report,parent,combined]:
        m=json.loads((A/r['source_manifest']).read_text())
        for n,h in m.items():assert digest(Path(r['source_directory'])/n)==h,n
        for b in r['binaries'].values():assert digest(Path(b['path']))==b['sha256']
    for n,h in manifest.items():assert digest(Path(report['build_source_directory'])/n)==h,n

def body(r,target,name):
    local='src/'+name.split('::')[0]+'.rs' if target=='hypercurve' else 'tests/'+target+'.rs'
    s=(Path(r['source_directory'])/'hypercurve'/local).read_text();marker='fn '+name.rsplit('::',1)[-1]+'('
    a=s.index(marker);pattern=r'\n(?:    )?#\[test\]';match=re.search(pattern,s[a:]);b=a+match.start() if match else len(s)
    return hashlib.sha256(s[a:b].encode()).hexdigest()
def signature(log):
    s=log.read_text();match=re.search(r'panicked at [^\n]+\n(.*?)(?:\nnote:|\nFAILED)',s,re.S)
    return match.group(1).strip() if match else None

def run(r,target,name,label,limit):
    report['active']=label;save();log=A/(prefix+'-'+label+'.log');start=time.monotonic();b=r['binaries'][target]
    with log.open('w') as out:
        try:code=subprocess.run([b['path'],'--exact',name,'--nocapture','--test-threads=1','--color','never'],cwd=Path(r['source_directory'])/'hypercurve',env=env,stdout=out,stderr=subprocess.STDOUT,timeout=limit).returncode
        except subprocess.TimeoutExpired:code='timeout'
    if code==0:assert re.search(r'test result: ok\. 1 passed; 0 failed; 0 ignored;',log.read_text()),name
    row=dict(target=target,name=name,label=label,returncode=code,elapsed_seconds=time.monotonic()-start,log=log.name,binary=b)
    report.pop('active');save();print(label,code,round(row['elapsed_seconds'],3),log.read_text()[-1600:] if code else '',flush=True)
    return row
verify();save()
for index,(target,name) in enumerate(report['selection'][129:],129):
    limit=300 if 'quartic' in name else 180
    row=run(report,target,name,f'case-{index:03}',limit)
    if row['returncode']:
        same_body=body(report,target,name)==body(parent,target,name)
        old=run(parent,target,name,f'parent-{index:03}',limit) if same_body else None
        equal_failure=old and old['returncode']==row['returncode'] and (
            row['returncode']=='timeout' or signature(A/row['log'])==signature(A/old['log']))
        entry=dict(candidate=row,parent=old,identical_test_body=same_body,equivalent_failure=bool(equal_failure))
        if body(report,target,name)==body(combined,target,name):entry['combined']=run(combined,target,name,f'combined-{index:03}',limit)
        report['baseline_comparisons'].append(entry)
        if equal_failure:
            row['baseline_comparison_index']=len(report['baseline_comparisons'])-1
            report['deferred_cases'].append(row)
        else:report['cases'].append(row)
    else:report['cases'].append(row)
    save()
verify();report.update(all_processes_reaped=True,all_sources_unchanged=True,hypercurve_passed=sum(c['returncode']==0 for c in report['cases']),scoped_qualification_complete=all(c['returncode']==0 for c in report['cases']+report['checks']),qualification_complete=False)
save();print('All owned processes reaped; passed',report['hypercurve_passed'],'pre-existing failures',len(report['deferred_cases']),flush=True)
