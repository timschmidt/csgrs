from pathlib import Path
import hashlib,json,subprocess,time
A=Path(__file__).resolve().parent
root=Path('/tmp/hypercurve-biquadratic-basis-v5-2026-09-23')
binary=A/'biquadratic-basis-20260923-v5-hypercurve-libtest'
expected='0e2c08aa9a4ea7214b11fad11d9e0ee4f2c2f347ecfdcea323e90215a0057559'
bindings=json.loads((A/'biquadratic-basis-20260923-v5-sources.json').read_text())
def verify():
 assert hashlib.sha256(binary.read_bytes()).hexdigest()==expected
 for name,sha in bindings.items(): assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
verify()
name='bezier_region::tests::one_fragment_ph_loop_fillets_through_rational_self_contact'
log=A/'tower-reuse-20260924-parent-ph.log'
start=time.monotonic()
with log.open('w') as out:
 try:code=subprocess.run([str(binary),'--exact',name,'--test-threads=1','--color','never'],cwd=root/'hypercurve',stdout=out,stderr=subprocess.STDOUT,timeout=75).returncode
 except subprocess.TimeoutExpired:code='timeout'
verify()
text=log.read_text();assert 'running 1 test' in text
report=dict(name=name,returncode=code,passed=code==0 and '1 passed;' in text,elapsed_seconds=time.monotonic()-start,limit_seconds=75,binary_sha256=expected,all_sources_unchanged=True,all_processes_reaped=True,log=log.name)
(A/'tower-reuse-20260924-parent-ph-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
print(report,flush=True)
print(text[-1200:],flush=True)
