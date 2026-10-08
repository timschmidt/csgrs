from pathlib import Path
import hashlib, json, subprocess
A=Path(__file__).resolve().parent
W=A.parent
prefix='normalized-ph-corner-20260924-v1'
root=Path('/tmp/hypercurve-normalized-ph-corner-v1-20260924')
manifest=json.loads((A/f'{prefix}-sources.json').read_text())
terminal=json.loads((A/f'{prefix}-terminal.json').read_text())
scope=json.loads((A/f'{prefix}-input-scope.json').read_text())
assert terminal['all_sources_unchanged'] and terminal['all_processes_reaped']
assert len(terminal['checks'])==3 and len(terminal['cases'])==5
assert all(r['returncode']==0 for r in terminal['checks']+terminal['cases'])
assert hashlib.sha256((A/f'{prefix}-libtest').read_bytes()).hexdigest()==terminal['binary_sha256']
for name,sha in manifest.items():
    assert hashlib.sha256((W/name).read_bytes()).hexdigest()==sha,name
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
repo=W/'hypercurve'
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip()==scope['hypercurve_parent']
parent=subprocess.check_output(['git','show','HEAD:src/bezier_region.rs'],cwd=repo,text=True)
current=(repo/'src/bezier_region.rs').read_text()
def outside(text,name):
    start=text.index('    fn '+name+'() {'); end=text.index('\n    #[test]',start)
    return text[:start],text[end:]
assert outside(parent,'one_fragment_ph_loop_projective_corners_retain_one_extended_interval')==outside(current,'closed_ph_corner_edits_preserve_both_normalized_source_lobes')
repositories=[]
for repo in sorted(W.iterdir()):
    if not (repo/'.git').exists(): continue
    paths=subprocess.check_output(['git','diff','--name-only'],cwd=repo,text=True).splitlines()
    assert paths==(['src/bezier_region.rs'] if repo.name=='hypercurve' else []),(repo.name,paths)
    assert not subprocess.check_output(['git','diff','--cached','--name-only'],cwd=repo,text=True)
    assert not subprocess.check_output(['git','ls-files','--others','--exclude-standard'],cwd=repo,text=True)
    subprocess.run(['git','diff','--check'],cwd=repo,check=True)
    repositories.append(dict(name=repo.name,head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip()))
report=dict(scope=scope,checks=terminal['checks'],cases=terminal['cases'],files={'src/bezier_region.rs':manifest['hypercurve/src/bezier_region.rs']},repositories=repositories,all_2044_inputs_match=True,all_owned_processes_reaped=True,full_production_qualification='rational-polynomial-20260924-v8-qualification.json')
(A/f'{prefix}-qualification.json').write_text(json.dumps(report,indent=2)+'\n')
print('Qualified: five PH cases, both Clippy configurations, formatting; only the one test function changes.')
