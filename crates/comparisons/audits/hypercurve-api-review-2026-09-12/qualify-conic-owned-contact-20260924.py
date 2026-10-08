from pathlib import Path
import hashlib,json,subprocess
A=Path(__file__).resolve().parent;W=A.parent
prefix='conic-owned-contact-20260924'
bindings=json.loads((A/f'{prefix}-v6-sources.json').read_text())
root=Path('/tmp/hypercurve-conic-owned-contact-v6-20260924')
for name,sha in bindings.items():
    assert hashlib.sha256((W/name).read_bytes()).hexdigest()==sha,name
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
focused=json.loads((A/f'{prefix}-v6-terminal.json').read_text())
broad=json.loads((A/f'{prefix}-broad-terminal.json').read_text())
assert focused['all_processes_reaped'] and broad['all_processes_reaped']
assert focused['all_sources_unchanged'] and broad['all_sources_unchanged']
assert all(c['passed'] for c in focused['cases'])
assert all(c['returncode']==0 for c in focused['checks'])
previous=json.loads((A/'selected-projective-import-20260924-broad-cases.json').read_text())
old={(r['target'],r['name']):r for r in previous}
for row in broad['nonpasses']:
    baseline=old[(row['target'],row['name'])]
    assert not baseline['passed'] and not baseline['ignored'],row
    assert baseline['returncode']==row['returncode'],row
rows=json.loads((A/f'{prefix}-broad-cases.json').read_text())
assert all(r['passed'] for r in rows if r['target']!='hypercurve')
fixed=[r['name'] for r in rows if r['passed'] and (r['target'],r['name']) in old and not old[(r['target'],r['name'])]['passed']]
repos=[]
for repo in sorted(W.iterdir()):
    if (repo/'.git').exists():
        status=subprocess.check_output(['git','status','--short'],cwd=repo,text=True)
        if repo.name!='hypercurve': assert not status,(repo.name,status)
        repos.append(dict(name=repo.name,head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip(),status=status))
report=dict(candidate=f'{prefix}-broad-terminal.json',focused=f'{prefix}-v6-terminal.json',source_manifest=f'{prefix}-v6-sources.json',source_count=len(bindings),no_new_nonpasses=True,newly_passing=fixed,rustfmt_check=0,all_owned_processes_reaped=True,repositories=repos)
(A/f'{prefix}-qualification.json').write_text(json.dumps(report,indent=2)+'\n')
print(dict(attempted=broad['attempted'],passed=broad['passed'],ignored=broad['ignored'],nonpasses=[r['name'] for r in broad['nonpasses']],newly_passing=fixed,repositories=len(repos)))
