from pathlib import Path
import concurrent.futures, hashlib, json, os, re, subprocess, time

audit=Path(__file__).resolve().parent
root=Path('/tmp/hypercurve-corner-publication-postchart-2026-09-23')
repo=root/'hypercurve'
prefix='corner-publication-postchart-20260923-broad1'
focused='corner-publication-postchart-20260923-focused1'
binary=audit/f'{focused}-libtest'
binary_hash=hashlib.sha256(binary.read_bytes()).hexdigest()
bindings=json.loads((audit/f'{focused}-sources.json').read_text())
def verify():
    for key,sha in bindings.items():
        assert hashlib.sha256((root/key).read_bytes()).hexdigest()==sha,key
    assert hashlib.sha256(binary.read_bytes()).hexdigest()==binary_hash
verify()
env=dict(os.environ,**json.loads((audit/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
checks=[]
for feature in ['--all-features','--no-default-features']:
    cmd=[cargo,'check','--all-targets',feature,'--locked','--offline']
    with (audit/f'{prefix}-check-{len(checks)}.log').open('w') as out:
        code=subprocess.run(cmd,cwd=repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=1200).returncode
    verify()
    checks.append(dict(command=cmd,returncode=code))
    assert code==0
    print('Passed all-target',feature,flush=True)
names=[line[:-6] for line in subprocess.check_output([str(binary),'--list'],text=True).splitlines() if line.endswith(': test')]
focused_suffixes={
 'selected_circle_corner_candidates_publish_normalized_single_loops',
 'selected_corner_candidates_reenter_normalization_with_retained_contacts',
 'independent_oblique_chord_pair_fillet_crosses_a_rational_line_exactly',
 'general_nonrepresented_chord_and_retained_rational_arc_complete_the_fillet_kernel',
 'extended_fillet_region_classifies_both_sides_of_its_companion',
}
focused_names={name for name in names if name.rsplit('::',1)[-1] in focused_suffixes}
assert len(focused_names)==len(focused_suffixes)
parent_rows={r['name']:r for r in json.loads((audit/'fillet-companion-chart-20260923-broad2-cases.json').read_text())}
parent_summary=json.loads((audit/'fillet-companion-chart-20260923-broad2-terminal.json').read_text())
assert parent_summary['all_processes_reaped'] and not parent_summary['new_failures']
parent_binary=audit/'fillet-companion-chart-20260923-focused3-candidate-libtest'
assert hashlib.sha256(parent_binary.read_bytes()).hexdigest()==parent_summary['binary_sha256']
new_names=set(names)-set(parent_rows)
assert len(new_names)==1 and new_names <= focused_names

def run(job):
    index,name=job
    log=audit/f'{prefix}-case-{index:04d}.log'
    start=time.monotonic()
    with log.open('w') as out:
        try:
            code=subprocess.run([str(binary),'--exact',name,'--test-threads=1','--nocapture','--color','never'],cwd=repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=75).returncode
        except subprocess.TimeoutExpired:
            code='timeout'
    output=log.read_text()
    assert 'running 1 test' in output
    row=dict(name=name,returncode=code,passed=code==0 and '1 passed;' in output,ignored=code==0 and '1 ignored;' in output,elapsed_seconds=time.monotonic()-start,limit_seconds=75,log=log.name)
    if not row['passed'] and not row['ignored']:
        parent=parent_rows.get(name)
        print('Not passed',name,code,'parent status',None if parent is None else parent['returncode'],output[-1500:],flush=True)
    return row
rows=[]
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
    jobs=[pool.submit(run,(i,name)) for i,name in enumerate(names) if name not in focused_names]
    for future in concurrent.futures.as_completed(jobs):
        rows.append(future.result())
        (audit/f'{prefix}-cases.json').write_text(json.dumps(rows,indent=2)+'\n')
        if len(rows)%100==0:print('Completed',len(rows),'of',len(names)-len(focused_names),'additional cases',flush=True)
verify()
# The focused runner executes these same binary/source-bound cases once. Its
# larger membership-case bound covers all four newly normalized publications.
focused_report=json.loads((audit/f'{focused}-terminal.json').read_text())
assert focused_report['all_processes_reaped'] and focused_report['all_sources_unchanged']
assert focused_report['binary_sha256']==binary_hash
assert {r['name'] for r in focused_report['cases']}==focused_names
rows.extend(dict(r,ignored=False,evidence='focused1 same executable and source bindings') for r in focused_report['cases'])
assert {r['name'] for r in rows}==set(names)
(audit/f'{prefix}-cases.json').write_text(json.dumps(rows,indent=2)+'\n')
def detail(row):
    output=(audit/row['log']).read_text()
    pos=output.find(' panicked at ')
    if pos<0:return None
    output=output[pos:].split('note: run with')[0].split('failures:')[0].strip()
    return re.sub(r'(?:/tmp/[^/]+/hypercurve/)?(src/[^:\n]+):\d+:\d+',r'\1:<location>',output)
failed=[r for r in rows if not r['passed'] and not r['ignored']]
new_failures=[r for r in failed if r['name'] not in parent_rows or parent_rows[r['name']]['passed']]
changed=[r for r in failed if r['name'] in parent_rows and not parent_rows[r['name']]['passed'] and (r['returncode']!=parent_rows[r['name']]['returncode'] or detail(r)!=detail(parent_rows[r['name']]))]
report=dict(passed=sum(r['passed'] for r in rows),ignored=sum(r['ignored'] for r in rows),failed=failed,new_failures=new_failures,changed_existing_nonpasses=changed,improved=[r['name'] for r in rows if r['passed'] and r['name'] in parent_rows and not parent_rows[r['name']]['passed']],checks=checks,binary_sha256=binary_hash,parent_binary_sha256=parent_summary['binary_sha256'],all_sources_unchanged=True,all_processes_reaped=True)
verify()
(audit/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
print('Terminal',{'passed':report['passed'],'ignored':report['ignored'],'nonpasses':len(failed),'new_failures':[r['name'] for r in new_failures],'changed_existing_nonpasses':[r['name'] for r in changed],'improved':report['improved']},flush=True)
