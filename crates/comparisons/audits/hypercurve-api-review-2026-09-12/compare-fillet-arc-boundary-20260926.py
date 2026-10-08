from pathlib import Path
import hashlib,json,os,re,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent
prefix='fillet-branches-20260926-v190-arc-boundary-comparison'
name='bezier_region::tests::general_nonrepresented_chord_and_retained_rational_arc_complete_the_fillet_kernel'
reports={
'parent':json.loads((A/'selected-cusp-ranges-isolated-20260926-v185-resumed-terminal.json').read_text()),
'candidate':json.loads((A/'fillet-branches-isolated-20260926-v190-terminal.json').read_text()),
'combined':json.loads((A/'selected-cusp-fillet-fixed-20260926-v189-terminal.json').read_text()),
}
guard=json.loads((A/reports['candidate']['workspace_guard']).read_text())
assert reports['candidate']['all_processes_reaped']
assert reports['candidate']['cases'][-1]['returncode']==101
result=dict(workspace_guard=reports['candidate']['workspace_guard'],cases=[],all_processes_reaped=False)
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def verify():
    for n,h in guard.items():assert digest(W/n)==h,n
    bodies=[]
    for role,r in reports.items():
        manifest=json.loads((A/r['source_manifest']).read_text())
        for n,h in manifest.items():assert digest(Path(r['source_directory'])/n)==h,n
        b=r['binaries']['hypercurve'];assert digest(Path(b['path']))==b['sha256']
        text=(Path(r['source_directory'])/'hypercurve/src/bezier_region.rs').read_text()
        start=text.index('    fn '+name.rsplit('::',1)[-1]+'(')
        end=text.index('\n    #[test]',start)
        bodies.append(hashlib.sha256(text[start:end].encode()).hexdigest())
    assert len(set(bodies))==1
    result['identical_test_body_sha256']=bodies[0]
def save():(A/(prefix+'-terminal.json')).write_text(json.dumps(result,indent=2)+'\n')
verify();save()
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
for role in ['parent','combined']:
    r=reports[role];b=r['binaries']['hypercurve'];log=A/(prefix+'-'+role+'.log');start=time.monotonic()
    result['active']=role;save()
    with log.open('w') as out:
        try:code=subprocess.run([b['path'],'--exact',name,'--nocapture','--test-threads=1','--color','never'],cwd=Path(r['source_directory'])/'hypercurve',env=env,stdout=out,stderr=subprocess.STDOUT,timeout=180).returncode
        except subprocess.TimeoutExpired:code='timeout'
    result['cases'].append(dict(role=role,binary=b,name=name,returncode=code,elapsed_seconds=time.monotonic()-start,log=log.name));result.pop('active');save()
    print(role,code,round(result['cases'][-1]['elapsed_seconds'],3),log.read_text()[-2000:],flush=True)
verify();result.update(all_processes_reaped=True,all_sources_and_binaries_unchanged=True);save()
