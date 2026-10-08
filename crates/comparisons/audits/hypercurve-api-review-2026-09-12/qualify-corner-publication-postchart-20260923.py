from pathlib import Path
import hashlib, json, os, shutil, subprocess, time

audit = Path(__file__).resolve().parent
root = Path('/tmp/hypercurve-corner-publication-postchart-2026-09-23')
repo = root/'hypercurve'
prefix = 'corner-publication-postchart-20260923-focused1'
env = dict(os.environ, **json.loads((audit/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo = '/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
bindings = json.loads((audit/'fillet-companion-chart-20260923-focused3-candidate-sources.json').read_text())
bindings.update({str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in repo.rglob('*') if p.is_file()})
def verify():
    for name, sha in bindings.items():
        assert hashlib.sha256((root/name).read_bytes()).hexdigest() == sha, name
verify()
(audit/f'{prefix}-sources.json').write_text(json.dumps(bindings,indent=2)+'\n')
command=[cargo,'test','--lib','--release','--all-features','--no-run','--message-format=json','--locked','--offline']
with (audit/f'{prefix}-build.jsonl').open('w') as out, (audit/f'{prefix}-build.log').open('w') as err:
    code=subprocess.run(command,cwd=repo,env=env,stdout=out,stderr=err,timeout=1200).returncode
verify()
assert code == 0
binary=audit/f'{prefix}-libtest'
artifacts=[]
for line in (audit/f'{prefix}-build.jsonl').read_text().splitlines():
    row=json.loads(line)
    if row.get('reason')=='compiler-artifact' and row.get('executable') and row['target']['name']=='hypercurve':
        assert not row['fresh']
        artifacts.append(row)
assert len(artifacts)==1
shutil.copy2(artifacts[0]['executable'],binary)
names=[line[:-6] for line in subprocess.check_output([str(binary),'--list'],text=True).splitlines() if line.endswith(': test')]
selected=[
 ('selected_circle_corner_candidates_publish_normalized_single_loops',75),
 ('selected_corner_candidates_reenter_normalization_with_retained_contacts',75),
 ('independent_oblique_chord_pair_fillet_crosses_a_rational_line_exactly',75),
 ('general_nonrepresented_chord_and_retained_rational_arc_complete_the_fillet_kernel',75),
 ('extended_fillet_region_classifies_both_sides_of_its_companion',300),
]
rows=[]
print('Built normalized corner candidate',hashlib.sha256(binary.read_bytes()).hexdigest(),flush=True)
for index,(suffix,bound) in enumerate(selected):
    matches=[name for name in names if name.rsplit('::',1)[-1]==suffix]
    assert len(matches)==1
    name=matches[0]
    log=audit/f'{prefix}-case-{index}.log'
    start=time.monotonic()
    with log.open('w') as out:
        try:
            code=subprocess.run([str(binary),'--exact',name,'--nocapture','--test-threads=1','--color','never'],cwd=repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=bound).returncode
        except subprocess.TimeoutExpired:
            code='timeout'
    verify()
    output=log.read_text()
    assert 'running 1 test' in output
    row=dict(name=name,returncode=code,passed=code==0 and '1 passed;' in output,limit_seconds=bound,elapsed_seconds=time.monotonic()-start,log=log.name)
    rows.append(row)
    (audit/f'{prefix}-cases.json').write_text(json.dumps(rows,indent=2)+'\n')
    print(row,output[-1500:],flush=True)
report=dict(cases=rows,binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),all_sources_unchanged=True,all_processes_reaped=True)
(audit/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
print('Terminal; all focused candidate cases reaped.',flush=True)
