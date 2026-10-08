from pathlib import Path
import concurrent.futures, hashlib, json, os, re, shutil, subprocess, time

audit = Path(__file__).resolve().parent
workspace = audit.parent
root = Path('/tmp/hypercurve-parameter-order-2026-09-23')
baseline = Path('/tmp/hypercurve-boundary-api-2026-09-23')
parent = Path('/tmp/hypercurve-parameter-order-parent-2026-09-23')
prefix = 'root-charts-20260923-final1'
new_tests = [
    'bezier_parameter::conversion_tests::composed_projective_charts_share_one_root_without_growing_history_or_scale',
    'bezier_offset::conversion_tests::selected_fiber_comparison_reuses_high_degree_affine_root_identity',
    'bezier_offset::conversion_tests::selected_fiber_comparison_replays_composed_projective_charts_with_signed_denominators',
]
held_out = [
    'selected_parallel_normal_circle_intersects_genuinely_analytic_parallel_in_one_fiber',
    'independent_oblique_chord_pair_fillets_extend_on_infinite_supports',
    'selected_circle_and_analytic_parallel_extend_on_full_supports',
    'pair_native_boolean_algebraic_chord_corner_publishes_a_third_generation_fillet',
]
for name in ['src/bezier_parameter.rs', 'src/bezier_offset.rs']:
    shutil.copy2(workspace/'hypercurve'/name, root/'hypercurve'/name)
assert not parent.exists()
parent.mkdir()
for path in baseline.iterdir():
    if path.name != 'hypercurve' and path.is_dir():
        (parent/path.name).symlink_to(path, target_is_directory=True)
shutil.copytree(baseline/'hypercurve', parent/'hypercurve')
source = (root/'hypercurve/src/bezier_offset.rs').read_text()
test_name = new_tests[1].split('::')[-1]
start = source.index('    #[test]\n    fn '+test_name)
end = source.index('\n    #[test]', start+1)
test = source[start:end]
p = parent/'hypercurve/src/bezier_offset.rs'
control = p.read_text()
needle = '    #[test]\n    fn selected_fiber_comparison_accepts_a_newly_represented_root'
assert control.count(needle)==1
p.write_text(control.replace(needle, test+'\n'+needle))
dependencies = json.loads((audit/'boundary-api-20260923-check5-sources.json').read_text())['isolated']
bindings = {}
for label, directory in [('candidate',root/'hypercurve'),('parent',parent/'hypercurve')]:
    bindings[label] = {str(p.relative_to(directory)):hashlib.sha256(p.read_bytes()).hexdigest()
                       for p in directory.rglob('*') if p.is_file() and 'target' not in p.parts}
bindings['working'] = {name:hashlib.sha256((workspace/'hypercurve'/name).read_bytes()).hexdigest()
                       for name in ['src/bezier_parameter.rs','src/bezier_offset.rs','src/bezier_region.rs']}
(audit/(prefix+'-sources.json')).write_text(json.dumps(bindings,indent=2)+'\n')
def verify():
    for label, directory in [('candidate',root/'hypercurve'),('parent',parent/'hypercurve'),('working',workspace/'hypercurve')]:
        for name, sha in bindings[label].items():
            assert hashlib.sha256((directory/name).read_bytes()).hexdigest()==sha,(label,name)
    for name, sha in dependencies.items():
        if not name.startswith('hypercurve/'):
            assert hashlib.sha256((baseline/name).read_bytes()).hexdigest()==sha,name
verify()
env = dict(os.environ,**json.loads((audit/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo = '/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
builds=[]
def build(label,repo):
    cmd=[cargo,'test','--lib','--release','--all-features','--no-run','--message-format=json','--locked','--offline']
    log=audit/(prefix+'-'+label+'-build.log'); listing=audit/(prefix+'-'+label+'-build.jsonl'); start=time.monotonic()
    with log.open('w') as err,listing.open('w') as out:
        code=subprocess.run(cmd,cwd=repo,env=env,stdout=out,stderr=err,timeout=1200).returncode
    row=dict(label=label,command=cmd,returncode=code,elapsed_seconds=time.monotonic()-start)
    builds.append(row); (audit/(prefix+'-builds.json')).write_text(json.dumps(builds,indent=2)+'\n'); verify(); print('build',row,flush=True)
    assert code==0,log.read_text()[-5000:]
    binary=audit/(prefix+'-'+label+'-libtest')
    found=False
    for line in listing.read_text().splitlines():
        entry=json.loads(line)
        if entry.get('reason')=='compiler-artifact' and entry.get('executable') and entry['profile']['test'] and entry['target']['name']=='hypercurve':
            assert not entry['fresh']; shutil.copy2(entry['executable'],binary); found=True
    assert found
    return binary

def run(job):
    label,binary,name,index=job
    log=audit/(prefix+'-'+label+f'-case-{index:04d}.log')
    cmd=[str(binary),'--exact',name,'--test-threads=1','--nocapture','--color','never']; start=time.monotonic()
    with log.open('w') as out:
        try: code=subprocess.run(cmd,cwd=root/'hypercurve',env=env,stdout=out,stderr=subprocess.STDOUT,timeout=75).returncode
        except subprocess.TimeoutExpired: code='timeout'
    output=log.read_text()
    row=dict(label=label,name=name,returncode=code,elapsed_seconds=time.monotonic()-start,
             passed=code==0 and '1 passed;' in output,ignored=code==0 and '1 ignored;' in output,log=log.name)
    if not row['passed'] and not row['ignored']: print('Not passed',label,name,code,output[-1800:],flush=True)
    return row

candidate=build('candidate',root/'hypercurve')
focused=[run(('focused',candidate,name,index)) for index,name in enumerate(new_tests)]
(audit/(prefix+'-focused.json')).write_text(json.dumps(focused,indent=2)+'\n')
assert all(row['passed'] for row in focused)
cmd=[cargo,'check','--all-targets','--all-features','--locked','--offline']; start=time.monotonic()
with (audit/(prefix+'-check.log')).open('w') as out:
    code=subprocess.run(cmd,cwd=root/'hypercurve',env=env,stdout=out,stderr=subprocess.STDOUT,timeout=1200).returncode
verify(); check=dict(command=cmd,returncode=code,elapsed_seconds=time.monotonic()-start)
(audit/(prefix+'-check.json')).write_text(json.dumps(check,indent=2)+'\n'); print('check',check,flush=True); assert code==0
control=build('parent',parent/'hypercurve')
regression=run(('parent-regression',control,new_tests[1],0))
assert regression['returncode']==101 and not regression['passed'],regression
listing=subprocess.check_output([str(candidate),'--list'],text=True)
names=[line.removesuffix(': test') for line in listing.splitlines() if line.endswith(': test')]
excluded=[name for name in names if name.split('::')[-1] in held_out]
assert len(excluded)==4,excluded
names=[name for name in names if name not in excluded and name not in new_tests]
rows=[]
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
    for row in pool.map(run,[('candidate',candidate,name,index) for index,name in enumerate(names)]):
        rows.append(row)
        (audit/(prefix+'-cases.json')).write_text(json.dumps(rows,indent=2)+'\n')
        if len(rows)%100==0: print('cases completed',len(rows),'/',len(names),flush=True)
verify()
failed=[row for row in rows if not row['passed'] and not row['ignored']]
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
    replays=list(pool.map(run,[('parent',control,row['name'],i) for i,row in enumerate(failed)]))
verify()
report=dict(builds=builds,check=check,focused=focused,parent_regression=regression,
            passed=sum(row['passed'] for row in rows)+len(focused),ignored=sum(row['ignored'] for row in rows),
            failed=failed,parent_replays=replays,excluded=excluded,
            new_failures=[row['name'] for row in replays if row['passed']],
            binary_sha256={label:hashlib.sha256(binary.read_bytes()).hexdigest() for label,binary in [('candidate',candidate),('parent',control)]},
            all_bound_sources_unchanged=True,all_processes_reaped=True)
(audit/(prefix+'-terminal.json')).write_text(json.dumps(report,indent=2)+'\n')
print('Terminal:',json.dumps({k:v for k,v in report.items() if k not in ['failed','parent_replays','focused','builds']}),flush=True)
