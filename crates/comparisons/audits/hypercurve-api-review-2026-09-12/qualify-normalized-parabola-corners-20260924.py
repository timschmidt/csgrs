from pathlib import Path
import hashlib, json, subprocess, sys
A=Path(__file__).resolve().parent
W=A.parent
version=sys.argv[1]
assert version.startswith('v') and version[1:].isdigit()
prefix=f'normalized-parabola-corners-20260924-{version}'
root=Path(f'/tmp/hypercurve-normalized-parabola-corners-{version}-20260924')
manifest=json.loads((A/f'{prefix}-sources.json').read_text())
terminal=json.loads((A/f'{prefix}-terminal.json').read_text())
scope=json.loads((A/f'{prefix}-input-scope.json').read_text())
assert terminal['all_sources_unchanged'] and terminal['all_processes_reaped']
assert len(terminal['checks'])==3 and len(terminal['cases'])==5
assert all(r['returncode']==0 for r in terminal['checks']+terminal['cases'])
assert hashlib.sha256((A/f'{prefix}-libtest').read_bytes()).hexdigest()==terminal['binary_sha256']
baseline=json.loads((A/scope['baseline_manifest']).read_text())
assert len(manifest)==2044
assert [name for name,sha in manifest.items() if sha!=baseline[name]]==['hypercurve/src/bezier_region.rs']
for name,sha in manifest.items():
    assert hashlib.sha256((W/name).read_bytes()).hexdigest()==sha,name
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
repo=W/'hypercurve'
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip()==scope['hypercurve_parent']
parent=subprocess.check_output(['git','show','HEAD:src/bezier_region.rs'],cwd=repo,text=True)
current=(repo/'src/bezier_region.rs').read_text()
start=parent.index('    fn analytic_fragment_has_exact_endpoint(')
end=parent.index('    fn retained_fragment_has_exact_endpoint(',start)
parent=parent[:start]+parent[end:]
def without_test(s,name):
    start=s.index('    #[test]\n    fn '+name+'()')
    end=s.index('    #[test]\n    fn one_fragment_materialized_loop_chamfers_to_one_middle_interval()',start)
    return s[:start]+s[end:]
assert without_test(parent,'retained_analytic_multifragment_corner_extends_chamfer_and_fillet')==without_test(current,'retained_analytic_corners_preserve_normalized_sets')
repositories=[]
for repo in sorted(W.iterdir()):
    if not (repo/'.git').exists():continue
    paths=subprocess.check_output(['git','diff','--name-only'],cwd=repo,text=True).splitlines()
    assert paths==(['src/bezier_region.rs'] if repo.name=='hypercurve' else []),(repo.name,paths)
    assert not subprocess.check_output(['git','diff','--cached','--name-only'],cwd=repo,text=True)
    assert not subprocess.check_output(['git','ls-files','--others','--exclude-standard'],cwd=repo,text=True)
    subprocess.run(['git','diff','--check'],cwd=repo,check=True)
    repositories.append(dict(name=repo.name,head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip()))
report=dict(scope=scope,checks=terminal['checks'],cases=terminal['cases'],files={'src/bezier_region.rs':manifest['hypercurve/src/bezier_region.rs']},repositories=repositories,all_2044_inputs_match=True,all_owned_processes_reaped=True,full_production_qualification=scope['baseline_full_qualification'],full_goal_complete=False)
(A/f'{prefix}-qualification.json').write_text(json.dumps(report,indent=2)+'\n')
print('Qualified: five corner cases, both Clippy configurations, formatting; only one test and its obsolete helpers change.')
