from pathlib import Path
import hashlib,json,os,shutil,subprocess,time

A=Path(__file__).resolve().parent;root=Path('/tmp/hypercurve-similarity-point-probe4-2026-09-23');repo=root/'hypercurve'
prefix='similarity-point-20260923-probe4'
bindings=json.loads((A/f'{prefix}-sources.json').read_text())
def verify():
    for name,sha in bindings.items():assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
verify()
artifacts=[json.loads(line) for line in (A/f'{prefix}-build.jsonl').read_text().splitlines()]
assert any(row.get('reason')=='build-finished' and row['success'] for row in artifacts)
dependencies=[]
for crate in ['hyperreal','hyperlattice','hyperlimit','hypersolve']:
    matches=[row for row in artifacts if row.get('reason')=='compiler-artifact' and row['target']['name']==crate]
    assert matches
    for row in matches:
        manifest=Path(row['manifest_path']);expected=Path('/tmp/hypercurve-biquadratic-basis-v5-2026-09-23')/crate/'Cargo.toml'
        assert str(root/crate) in row['package_id'] and manifest.resolve()==expected
        dependencies.append(dict(crate=crate,package_id=row['package_id'],physical_manifest=str(manifest.resolve())))
tests=[row for row in artifacts if row.get('reason')=='compiler-artifact' and row.get('executable') and row['target']['name']=='hypercurve']
assert len(tests)==1 and not tests[0]['fresh'] and str(repo) in tests[0]['package_id']
binary=A/f'{prefix}-libtest';assert not binary.exists();shutil.copy2(tests[0]['executable'],binary)
name='bezier_offset::conversion_tests::selected_fiber_transverse_mapped_cut_inverts_by_point'
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
log=A/f'{prefix}-case-0.log';start=time.monotonic()
with log.open('w') as out:
    try:code=subprocess.run([str(binary),'--exact',name,'--test-threads=1','--nocapture','--color','never'],cwd=repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=120).returncode
    except subprocess.TimeoutExpired:code='timeout'
verify();output=log.read_text();assert 'running 1 test' in output
report=dict(cases=[dict(name=name,returncode=code,passed=code==0,log=log.name,elapsed_seconds=time.monotonic()-start)],dependencies=dependencies,binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),all_processes_reaped=True,all_sources_unchanged=True,initial_runner_failure='Cargo retains logical symlink paths in package IDs; replay verifies resolved manifests and every source byte instead.')
(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
print(output,flush=True)
