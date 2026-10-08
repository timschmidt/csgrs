from pathlib import Path
import hashlib,json,os,subprocess,time
A=Path(__file__).resolve().parent
W=A.parent
versions=['v39','v44']
inputs={}
for version in versions:
    prefix=f'local-chord-complete-replay-20260924-{version}'
    root=Path(f'/tmp/hypercurve-local-chord-complete-replay-{version}-20260924')
    manifest=json.loads((A/f'{prefix}-sources.json').read_text())
    terminal=json.loads((A/f'{prefix}-terminal.json').read_text())
    assert terminal['all_processes_reaped']
    binary=A/f'{prefix}-libtest'
    inputs[version]=(root,manifest,binary,terminal['binary_sha256'])
def verify():
    for version,(root,manifest,binary,sha) in inputs.items():
        assert hashlib.sha256(binary.read_bytes()).hexdigest()==sha
        for name,expected in manifest.items():
            assert hashlib.sha256((root/name).read_bytes()).hexdigest()==expected,name
            if version=='v44':
                assert hashlib.sha256((W/name).read_bytes()).hexdigest()==expected,name
verify()
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
rows=[]
for suffix in ['retained_polynomial_chamfer_extends_exact_and_algebraic_incident_roots','retained_rational_chamfer_extends_exact_and_algebraic_pre_pole_roots']:
    for version in versions:
        root,manifest,binary,sha=inputs[version]
        log=A/f'{version}-extension-comparison-{suffix}.log'
        start=time.monotonic()
        with log.open('w') as out:
            try:
                code=subprocess.run([str(binary),'--exact','bezier_region::tests::'+suffix,'--test-threads=1','--color','never'],cwd=root/'hypercurve',env=env,stdout=out,stderr=subprocess.STDOUT,timeout=75).returncode
            except subprocess.TimeoutExpired:
                code='timeout'
        rows.append(dict(version=version,name=suffix,returncode=code,elapsed_seconds=time.monotonic()-start,binary_sha256=sha,log=log.name))
        print(version,suffix,code,rows[-1]['elapsed_seconds'],flush=True)
verify()
(A/'retained-extension-runtime-comparison-20260925.json').write_text(json.dumps(dict(cases=rows,all_owned_processes_reaped=True,all_archived_inputs_match=True,current_v44_inputs_match=True,diagnostic_only=True),indent=2)+'\n')
raise SystemExit(int(any(row['returncode'] for row in rows)))
