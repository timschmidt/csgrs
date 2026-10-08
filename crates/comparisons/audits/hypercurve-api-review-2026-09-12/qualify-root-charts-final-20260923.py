from pathlib import Path
import concurrent.futures, hashlib, json, os, shutil, subprocess, time

audit=Path(__file__).resolve().parent; workspace=audit.parent
root=Path('/tmp/hypercurve-parameter-order-2026-09-23')
prefix='root-charts-20260923-final2'
previous=json.loads((audit/'root-charts-20260923-final1-terminal.json').read_text())
assert previous['new_failures']==[] and previous['all_processes_reaped']
for name in ['src/bezier_offset.rs','src/bezier_parameter.rs']:
    shutil.copy2(workspace/'hypercurve'/name,root/'hypercurve'/name)
binding={str(path.relative_to(root/'hypercurve')):hashlib.sha256(path.read_bytes()).hexdigest()
         for path in (root/'hypercurve').rglob('*') if path.is_file() and 'target' not in path.parts}
working={name:hashlib.sha256((workspace/'hypercurve'/name).read_bytes()).hexdigest()
         for name in ['src/bezier_offset.rs','src/bezier_parameter.rs','src/bezier_region.rs']}
dependencies=json.loads((audit/'boundary-api-20260923-check5-sources.json').read_text())['isolated']
(audit/(prefix+'-sources.json')).write_text(json.dumps(dict(isolated=binding,working=working),indent=2)+'\n')
def verify():
    for name,sha in binding.items(): assert hashlib.sha256((root/'hypercurve'/name).read_bytes()).hexdigest()==sha,name
    for name,sha in working.items(): assert hashlib.sha256((workspace/'hypercurve'/name).read_bytes()).hexdigest()==sha,name
    for name,sha in dependencies.items():
        if not name.startswith('hypercurve/'): assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
verify()
env=dict(os.environ,**json.loads((audit/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
builds=[]; binary=None
for label,args in [('check',['check','--all-targets','--all-features']),
                   ('libtest',['test','--lib','--release','--all-features','--no-run','--message-format=json'])]:
    cmd=[cargo,*args,'--locked','--offline']; start=time.monotonic()
    with (audit/(prefix+'-'+label+'.log')).open('w') as err,(audit/(prefix+'-'+label+'.stdout')).open('w') as out:
        code=subprocess.run(cmd,cwd=root/'hypercurve',env=env,stdout=out,stderr=err,timeout=1200).returncode
    verify(); row=dict(label=label,command=cmd,returncode=code,elapsed_seconds=time.monotonic()-start); builds.append(row)
    (audit/(prefix+'-builds.json')).write_text(json.dumps(builds,indent=2)+'\n'); print(row,flush=True)
    assert code==0,(audit/(prefix+'-'+label+'.log')).read_text()[-5000:]
    if label=='libtest':
        for line in (audit/(prefix+'-'+label+'.stdout')).read_text().splitlines():
            row=json.loads(line)
            if row.get('reason')=='compiler-artifact' and row.get('executable') and row['profile']['test'] and row['target']['name']=='hypercurve':
                assert not row['fresh']; binary=audit/(prefix+'-libtest'); shutil.copy2(row['executable'],binary)
assert binary
listing=subprocess.check_output([str(binary),'--list'],text=True)
listed=[line.removesuffix(': test') for line in listing.splitlines() if line.endswith(': test')]
names=[name for name in listed if name.startswith(('bezier_parameter::','bezier_split::'))
       or (name.startswith('bezier_offset::') and any(token in name for token in ['selected_fiber','selected_scalar','projective','affine','incident_ray']))
       or name=='bezier_region::tests::selected_corner_candidates_reenter_normalization_with_retained_contacts']
names=[name for name in names if name not in previous['excluded']]
new='bezier_offset::conversion_tests::selected_fiber_chart_uncertainty_preserves_exact_coefficient_replay'
assert new in names
def run(job):
    label,exe,name,index=job; start=time.monotonic(); log=audit/(prefix+f'-{label}-{index:04d}.log')
    with log.open('w') as out:
        try: code=subprocess.run([str(exe),'--exact',name,'--nocapture','--test-threads=1','--color','never'],cwd=root/'hypercurve',env=env,stdout=out,stderr=subprocess.STDOUT,timeout=75).returncode
        except subprocess.TimeoutExpired: code='timeout'
    output=log.read_text(); row=dict(label=label,name=name,returncode=code,elapsed_seconds=time.monotonic()-start,
                                   passed=code==0 and '1 passed;' in output,ignored=code==0 and '1 ignored;' in output,log=log.name)
    if not row['passed'] and not row['ignored']: print('Not passed',name,code,output[-2000:],flush=True)
    return row
rows=[]
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
    for row in pool.map(run,[('candidate',binary,name,index) for index,name in enumerate(names)]):
        rows.append(row); (audit/(prefix+'-cases.json')).write_text(json.dumps(rows,indent=2)+'\n')
        if len(rows)%25==0: print('completed',len(rows),'/',len(names),flush=True)
verify()
failed=[row for row in rows if not row['passed'] and not row['ignored']]
assert next(row for row in rows if row['name']==new)['passed']
control=audit/'root-charts-20260923-final1-candidate-libtest'
control_listing=subprocess.check_output([str(control),'--list'],text=True)
assert all(row['name']+': test' in control_listing for row in failed)
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
    replays=list(pool.map(run,[('control',control,row['name'],index) for index,row in enumerate(failed)]))
verify()
report=dict(builds=builds,passed=sum(row['passed'] for row in rows),ignored=sum(row['ignored'] for row in rows),
            failed=failed,control_replays=replays,new_failures=[row['name'] for row in replays if row['passed']],
            binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),all_bound_sources_unchanged=True,all_processes_reaped=True)
(audit/(prefix+'-terminal.json')).write_text(json.dumps(report,indent=2)+'\n')
print('Terminal',report,flush=True)
