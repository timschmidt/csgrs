from pathlib import Path
import hashlib,json,os,subprocess,sys,time
A=Path(__file__).resolve().parent
W=A.parent
version=sys.argv[1]
assert version.startswith('v') and version[1:].isdigit()
prefix=f'local-chord-complete-replay-20260924-{version}'
root=Path(f'/tmp/hypercurve-local-chord-complete-replay-{version}-20260924')
manifest=json.loads((A/f'{prefix}-sources.json').read_text())
terminal=json.loads((A/f'{prefix}-terminal.json').read_text())
assert terminal['all_sources_unchanged'] and terminal['all_processes_reaped']
binary=A/f'{prefix}-libtest'
sha=terminal['binary_sha256']
report_path=A/f'{prefix}-long-diagnostic-terminal.json'
assert not report_path.exists()
def verify():
    assert hashlib.sha256(binary.read_bytes()).hexdigest()==sha
    for name,expected in manifest.items():
        assert hashlib.sha256((W/name).read_bytes()).hexdigest()==expected,name
        assert hashlib.sha256((root/name).read_bytes()).hexdigest()==expected,name
verify()
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
command=[str(binary),'--exact','bezier_region::tests::one_fragment_nonzero_parallel_loop_extends_chamfer_cuts_on_one_finite_envelope','--nocapture','--test-threads=1','--color','never']
log=A/f'{prefix}-long-diagnostic.log'
start=time.monotonic()
with log.open('w') as out:
    try:
        code=subprocess.run(command,cwd=root/'hypercurve',env=env,stdout=out,stderr=subprocess.STDOUT,timeout=180).returncode
    except subprocess.TimeoutExpired:
        code='timeout'
verify()
output=log.read_text()
report=dict(command=command,returncode=code,elapsed_seconds=time.monotonic()-start,limit_seconds=180,diagnostic_only=True,qualification_limit_unchanged=75,binary_sha256=sha,source_manifest=f'{prefix}-sources.json',all_sources_unchanged=True,all_processes_reaped=True,log=log.name,passed=code==0 and '1 passed;' in output)
report_path.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2),flush=True)
print(output[-3000:],flush=True)
