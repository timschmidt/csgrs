from pathlib import Path
import hashlib, json, re, statistics, subprocess, time

A = Path(__file__).resolve().parent
W = A.parent
prefix = 'retained-contact-extension-timing-20260925-v95-v99'
assert not (A/f'{prefix}-terminal.json').exists()
bindings = {}
manifests = {}
for version,label in [('v95','selected-frame-integrated'),('v99','retained-contact-integrated')]:
    report = json.loads((A/f'{label}-20260925-{version}-terminal.json').read_text())
    assert report['all_processes_reaped'] and report['all_sources_unchanged']
    bindings[version] = report['binaries']['hypercurve_curve']
    manifests[version] = json.loads((A/report['source_manifest']).read_text())
changed = [name for name in manifests['v95'] if manifests['v95'][name] != manifests['v99'][name]]
assert set(changed) == {'hypersolve/src/algebraic_fiber.rs','hypersolve/src/ordered_field_roots.rs',
                        'hypersolve/src/root_sign.rs','hypercurve/src/bezier_offset.rs'}
def verify():
    for version,manifest in manifests.items():
        root = A/'source-archives'/f'hypercurve-local-chord-complete-replay-{version}-20260924'
        for name,sha in manifest.items():
            assert hashlib.sha256((root/name).read_bytes()).hexdigest() == sha,name
            if version == 'v99':
                assert hashlib.sha256((W/name).read_bytes()).hexdigest() == sha,name
        binary = bindings[version]
        assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest() == binary['sha256']
verify()
cases = ['polynomial_chamfer_materializes_represented_incident_extension',
         'rational_chamfer_materializes_the_incident_projective_cell']
report = dict(binary_bindings=bindings,changed_inputs=changed,repetitions=3,
              matched_public_test_sources=True,cases=[],all_processes_reaped=False)
for case in cases:
    for repetition in range(3):
        for version in ['v95','v99']:
            log = A/f'{prefix}-{case}-{version}-{repetition}.log'
            command = [bindings[version]['path'],'--exact',case,'--test-threads=1','--color','never']
            start = time.perf_counter()
            with log.open('w') as out:
                try:
                    code = subprocess.run(command,cwd=W/'hypercurve',stdout=out,stderr=subprocess.STDOUT,timeout=75).returncode
                except subprocess.TimeoutExpired:
                    code = 'timeout'
            elapsed = time.perf_counter()-start
            output = log.read_text()
            match = re.search(r'test result: ok\. 1 passed; 0 failed; 0 ignored;.*?finished in ([\d.]+)s',output)
            row = dict(case=case,version=version,repetition=repetition,command=command,
                       returncode=code,elapsed_seconds=elapsed,libtest_seconds=float(match[1]) if match else None,log=log.name)
            report['cases'].append(row)
            print(case,version,repetition,code,round(elapsed,6),flush=True)
            if code or not match:
                verify()
                report['all_processes_reaped'] = True
                (A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
                raise SystemExit(1)
verify()
report['summary'] = {case:{version:dict(
    median_seconds=statistics.median(values),minimum_seconds=min(values),maximum_seconds=max(values))
    for version in bindings
    for values in [[row['elapsed_seconds'] for row in report['cases'] if row['case']==case and row['version']==version]]}
    for case in cases}
report.update(all_processes_reaped=True,all_sources_and_executables_unchanged=True)
(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report['summary']),flush=True)
