from pathlib import Path
import hashlib, json, subprocess

audit=Path(__file__).resolve().parent
repo=audit.parent/'hypercurve'
root=Path('/tmp/hypercurve-fillet-companion-final-2026-09-23')
name='src/curve_corner_chain.rs'
meta=json.loads((audit/'fillet-companion-chart-20260923-final-candidate.json').read_text())
broad=json.loads((audit/'fillet-companion-chart-20260923-broad2-terminal.json').read_text())
public=json.loads((audit/'fillet-companion-chart-20260923-public2-terminal.json').read_text())
focused=json.loads((audit/'fillet-companion-chart-20260923-focused3-candidate-result.json').read_text())
assert broad['all_processes_reaped'] and public['all_processes_reaped'] and focused['all_processes_reaped']
assert broad['all_sources_unchanged'] and public['all_sources_unchanged'] and focused['all_sources_unchanged']
assert not broad['new_failures']
assert all(x['returncode']==0 for x in broad['checks'])
assert len(public['cases'])==4 and all(x['returncode']==0 for x in public['cases'])
assert len(focused['cases'])==3 and all(x['passed'] for x in focused['cases'])
head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip()
assert head==meta['parent']
assert subprocess.check_output(['git','diff','--name-only'],cwd=repo,text=True).splitlines()==['src/bezier_region.rs']
assert not subprocess.check_output(['git','diff','--cached','--name-only'],cwd=repo,text=True).strip()
assert hashlib.sha256((repo/'src/bezier_region.rs').read_bytes()).hexdigest()==meta['excluded_region_sha256']
bindings=json.loads((audit/'fillet-companion-chart-20260923-focused3-candidate-sources.json').read_text())
for key,sha in bindings.items():
    assert hashlib.sha256((root/key).read_bytes()).hexdigest()==sha,key
content=(root/'hypercurve'/name).read_bytes()
sha=hashlib.sha256(content).hexdigest()
assert sha==meta['sha256']==bindings['hypercurve/'+name]
(repo/name).write_bytes(content)
subprocess.run(['git','diff','--check'],cwd=repo,check=True)
record=dict(parent=head,files={name:sha},library_passed=broad['passed'],library_ignored=broad['ignored'],known_nonpasses=len(broad['failed']),new_failures=broad['new_failures'],changed_failures=broad['changed_failures'],focused=focused['cases'],public=public['cases'],checks=broad['checks'],library_binary_sha256=broad['binary_sha256'],public_binary_sha256=public['binary_sha256'],excluded_region_sha256=meta['excluded_region_sha256'],all_sources_unchanged=True,all_processes_reaped=True)
(audit/'fillet-companion-chart-20260923-qualification.json').write_text(json.dumps(record,indent=2)+'\n')
print('Installed only the qualified companion-chart file:',sha)
print('Library:',broad['passed'],'passed;',broad['ignored'],'ignored;',len(broad['failed']),'known nonpasses;',len(broad['new_failures']),'new failures.')
