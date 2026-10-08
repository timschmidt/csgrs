from pathlib import Path
import hashlib,json,os,shutil,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent
prefix='selected-cusp-ranges-20260926-v185-chamfer-baseline'
current=json.loads((A/'selected-cusp-ranges-isolated-20260926-v185-terminal.json').read_text())
assert current['all_processes_reaped'] and current['cases'][-1]['returncode']=='timeout'
parent=json.loads((A/'retained-circle-parallel-isolated-20260926-v180-terminal.json').read_text())
guard=json.loads((A/current['workspace_guard']).read_text())
current_manifest=json.loads((A/current['source_manifest']).read_text())
parent_manifest=json.loads((A/parent['source_manifest']).read_text())
target='hypercurve_analytic_parallel_region'
old=parent['binaries'][target]
assert hashlib.sha256(Path(old['path']).read_bytes()).hexdigest()==old['sha256']
binary=A/(prefix+'-test');assert not binary.exists();shutil.copy2(old['path'],binary)
assert (binary.stat().st_dev,binary.stat().st_ino)!=(Path(old['path']).stat().st_dev,Path(old['path']).stat().st_ino)
assert current_manifest['hypercurve/tests/'+target+'.rs']==parent_manifest['hypercurve/tests/'+target+'.rs']
def verify():
    for n,h in guard.items():assert hashlib.sha256((W/n).read_bytes()).hexdigest()==h,n
    for r,m in [(current,current_manifest),(parent,parent_manifest)]:
        for n,h in m.items():assert hashlib.sha256((Path(r['source_directory'])/n).read_bytes()).hexdigest()==h,n
    for n,h in current_manifest.items():assert hashlib.sha256((Path(current['build_source_directory'])/n).read_bytes()).hexdigest()==h,n
    assert hashlib.sha256(binary.read_bytes()).hexdigest()==old['sha256']
    for b in current['binaries'].values():assert hashlib.sha256(Path(b['path']).read_bytes()).hexdigest()==b['sha256']
verify()
name='algebraic_endpoint_analytic_parallel_chamfers_replay_selected_distance'
report=dict(workspace_guard=current['workspace_guard'],parent_qualification='retained-circle-parallel-isolated-20260926-v180-terminal.json',current_qualification='selected-cusp-ranges-isolated-20260926-v185-terminal.json',test_file_unchanged=True,binary=dict(path=str(binary),sha256=old['sha256']),name=name,all_processes_reaped=False)
(A/(prefix+'-terminal.json')).write_text(json.dumps(report,indent=2)+'\n')
start=time.monotonic()
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
with (A/(prefix+'.log')).open('w') as out:
    try: code=subprocess.run([str(binary),'--exact',name,'--nocapture','--test-threads=1','--color','never'],cwd=Path(parent['source_directory'])/'hypercurve',env=env,stdout=out,stderr=subprocess.STDOUT,timeout=180).returncode
    except subprocess.TimeoutExpired:code='timeout'
verify();report.update(returncode=code,elapsed_seconds=time.monotonic()-start,all_processes_reaped=True,all_sources_and_binaries_unchanged=True)
(A/(prefix+'-terminal.json')).write_text(json.dumps(report,indent=2)+'\n')
print('committed baseline:',code,round(report['elapsed_seconds'],3),flush=True)
print((A/(prefix+'.log')).read_text()[-2000:],flush=True)
