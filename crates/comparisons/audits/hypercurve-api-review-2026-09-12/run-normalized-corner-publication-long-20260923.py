from pathlib import Path
import concurrent.futures, hashlib, json, os, shutil, subprocess, time

audit = Path(__file__).resolve().parent
root = Path('/tmp/hypercurve-normalized-corner-callers-2026-09-23')
repo = root/'hypercurve'
prefix = 'normalized-corner-publication-20260923-long1'
env = dict(os.environ, **json.loads((audit/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo = '/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
focused='consumed-corner-ranges-20260923-focused2'
bindings=json.loads((audit/f'{focused}-sources.json').read_text())
def verify():
    for name,sha in bindings.items():
        assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
verify()
summary=json.loads((audit/f'{focused}-terminal.json').read_text())
assert summary['all_processes_reaped'] and all(r['passed'] for r in summary['cases'])
broad=json.loads((audit/'normalized-corner-publication-20260923-broad1-terminal.json').read_text())
assert broad['all_processes_reaped']
binary=audit/f'{focused}-libtest'
assert hashlib.sha256(binary.read_bytes()).hexdigest()==summary['binary_sha256']
(audit/f'{prefix}-sources.json').write_text(json.dumps(bindings,indent=2)+'\n')
names=[line[:-6] for line in subprocess.check_output([str(binary),'--list'],text=True).splitlines() if line.endswith(': test')]
selected=[
 ('extended_fillet_region_classifies_both_sides_of_its_companion',300),
 ('nonlinear_algebraic_endpoint_fillet_uses_complete_incident_domain',300),
 ('nonrepresented_chord_and_selected_circle_complete_the_fillet_kernel',300),
 ('one_fragment_ph_loop_fillets_through_rational_self_contact',300),
 ('selected_circle_and_promoted_line_extend_through_the_chord_support_cell',300),
 ('selected_parallel_companion_fillets_without_range_promotion',300),
]


print('Reusing normalized candidate',hashlib.sha256(binary.read_bytes()).hexdigest(),flush=True)
def run(job):
    index,(suffix,bound)=job
    matches=[name for name in names if name.rsplit('::',1)[-1]==suffix]
    assert len(matches)==1
    name=matches[0]
    log=audit/f'{prefix}-case-{index}.log'
    print('Starting extended qualification',name,'limit',bound,flush=True)
    start=time.monotonic()
    with log.open('w') as out:
        try:
            code=subprocess.run([str(binary),'--exact',name,'--nocapture','--test-threads=1','--color','never'],cwd=repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=bound).returncode
        except subprocess.TimeoutExpired:
            code='timeout'
    output=log.read_text()
    assert 'running 1 test' in output
    row=dict(name=name,returncode=code,passed=code==0 and '1 passed;' in output,limit_seconds=bound,elapsed_seconds=time.monotonic()-start,log=log.name)
    print(row,output[-1500:],flush=True)
    return row
rows=[]
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
    for future in concurrent.futures.as_completed([pool.submit(run,job) for job in enumerate(selected)]):
        rows.append(future.result())
        (audit/f'{prefix}-cases.json').write_text(json.dumps(rows,indent=2)+'\n')
verify()
report=dict(cases=rows,binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),all_sources_unchanged=True,all_processes_reaped=True)
(audit/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
print('Terminal; all focused candidate cases reaped.',flush=True)
