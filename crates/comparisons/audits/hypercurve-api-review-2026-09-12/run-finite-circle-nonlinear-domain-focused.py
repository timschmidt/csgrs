from pathlib import Path
import hashlib,json,os,subprocess,sys,time
root=Path('/home/tim/Documents/GitHub/workspace'); audit=root/'hypercurve-api-review-2026-09-12'; repo=root/'hypercurve'; stem='finite-circle-nonlinear-domain-'+sys.argv[1]
files=subprocess.check_output(['git','diff','--name-only'],cwd=repo,text=True).splitlines(); sources={f:hashlib.sha256((repo/f).read_bytes()).hexdigest() for f in files}
(audit/(stem+'.patch')).write_bytes(subprocess.check_output(['git','diff'],cwd=repo))
(audit/(stem+'-sources.json')).write_text(json.dumps(sources,indent=2)+'\n')
env=dict(os.environ,CARGO_BUILD_JOBS='2')
for name,args in [('check',['check','--tests','--all-features','--offline']),('build',['test','--release','--all-features','--offline','--lib','--tests','--no-run','--message-format=json'])]:
    start=time.monotonic()
    with (audit/(stem+'-'+name+'.log')).open('w') as out, (audit/(stem+'-'+name+'.jsonl')).open('w') as data:
        p=subprocess.run(['cargo',*args],cwd=repo,stdout=data if name=='build' else out,stderr=out,env=env,timeout=600)
    (audit/(stem+'-'+name+'.exit')).write_text(str(p.returncode)+'\n')
    print(name,p.returncode,round(time.monotonic()-start,2),flush=True)
    if p.returncode:
        print((audit/(stem+'-'+name+'.log')).read_text()[-6000:],flush=True); raise SystemExit(p.returncode)
rows=[json.loads(line) for line in (audit/(stem+'-build.jsonl')).read_text().splitlines() if line.startswith('{')]
binary=next(r['executable'] for r in rows if r.get('reason')=='compiler-artifact' and r.get('executable') and r['target']['name']=='hypercurve')
results=[]
for name in ['denominator_sign_tracks_the_requested_range_and_keeps_unit_cache_scope', 'denominator_sign_excludes_a_pole_outside_selected_bounds', 'rational_circle_component_adapter_preserves_mixed_evidence_when_clipped', 'selected_fiber_circle_overlap_preserves_an_isolated_parameter_visit']:
    command=[binary,name,'--nocapture','--test-threads=1']; start=time.monotonic()
    with (audit/(stem+'-'+name+'.log')).open('w') as out:
        try: p=subprocess.run(command,cwd=repo,stdout=out,stderr=subprocess.STDOUT,timeout=120); code=p.returncode
        except subprocess.TimeoutExpired: code=124
    log=(audit/(stem+'-'+name+'.log')).read_text(); result={'name':name,'command':command,'binary':binary,'sha256':hashlib.sha256(Path(binary).read_bytes()).hexdigest(),'returncode':code,'elapsed_seconds':time.monotonic()-start,'log':stem+'-'+name+'.log'}
    assert 'running 1 test' in log,log
    results.append(result); (audit/(stem+'-focused.json')).write_text(json.dumps(results,indent=2)+'\n');print(name,code,round(result['elapsed_seconds'],2),flush=True)
    if code: print(log[-6000:],flush=True);break
assert all(hashlib.sha256((repo/f).read_bytes()).hexdigest()==sha for f,sha in sources.items())
raise SystemExit(int(any(row['returncode'] for row in results)))
