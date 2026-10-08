from pathlib import Path
import hashlib, json, subprocess, sys

A=Path(__file__).resolve().parent
W=A.parent
phase=sys.argv[1]
assert phase in {'qualified','staged','committed'}
prefix='native-evidence-reuse-20260925-v83-candidate'
r=json.loads((A/f'{prefix}-terminal.json').read_text())
manifest=json.loads((A/r['source_manifest']).read_text())
guard=json.loads((A/r['workspace_guard']).read_text())
root=A/'source-archives'/prefix
sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
assert r['all_processes_reaped'] and r['all_sources_unchanged'] and r['build_returncode']==0
assert len(r['cases'])==len(r['selection'])==244
assert all(row['returncode']==0 and not row['ignored'] for row in r['cases'])
assert len(r['checks'])==3 and all(row['returncode']==0 for row in r['checks'])
assert sum(row['target']=='hypercurve' for row in r['cases'])==135
assert sum(row['target']!='hypercurve' for row in r['cases'])==109
assert len(manifest)==len(guard)==2044
for name, expected in manifest.items():
    assert sha(root/name)==expected,name
    assert sha(Path(r['build_source_directory'])/name)==expected,name
for name, expected in guard.items():
    assert sha(W/name)==expected,name
for binary in r['binaries'].values():
    assert sha(Path(binary['path']))==binary['sha256']

baseline=json.loads((A/'native-root-witness-20260925-v71-terminal.json').read_text())
baseline_manifest=json.loads((A/'local-chord-complete-replay-20260924-v71-sources.json').read_text())
baseline_root=A/'source-archives/hypercurve-local-chord-complete-replay-v71-20260924'
assert baseline['all_processes_reaped'] and baseline['all_sources_unchanged']
assert baseline['build_returncode']==0
assert len(baseline['cases'])==8
regressions={'strict_linear_queries_retain_witnesses_after_rational_reconstruction_declines',
             'strict_scalar_equalities_retain_arbitrary_real_parameter_witnesses'}
assert {row['label'] for row in baseline['cases'] if row['returncode']!=0}==regressions
for row in baseline['cases']:
    assert row['returncode']==(101 if row['label'] in regressions else 0)
for name,expected in baseline_manifest.items():
    assert sha(baseline_root/name)==expected,name
    if not name.startswith('hypercurve/'):
        assert manifest[name]==expected,name
for binary in baseline['binaries'].values():
    assert sha(Path(binary['path']))==binary['sha256']
old=(baseline_root/'hypercurve/src/bezier_parameter.rs').read_text()
new=(root/'hypercurve/src/bezier_parameter.rs').read_text()
def body(text,name):
    start=text.index('    #[test]\n    fn '+name+'()')
    end=text.index('    #[test]',start+12)
    return text[start:end]
for regression in regressions:
    assert body(old,regression)==body(new,regression),regression
    assert any(row['name'].endswith('::'+regression) for row in r['cases'])

selected=r['selected_files']
assert selected==['src/bezier_algebraic_image.rs','src/bezier_offset.rs','src/bezier_parameter.rs','src/bezier_split.rs','src/curve_region_boolean.rs']
remaining=sorted(name.removeprefix('hypercurve/') for name in guard
                 if name.startswith('hypercurve/') and guard[name]!=manifest[name])
assert remaining==['src/bezier_offset.rs','src/bezier_region.rs','src/curve.rs','src/curve_corner_chain.rs','tests/hypercurve_curve.rs']
heads=[]
for repo in sorted(W.iterdir()):
    if not (repo/'.git').exists():continue
    run=lambda *args:subprocess.check_output(['git',*args],cwd=repo,text=True).strip()
    head=run('rev-parse','HEAD')
    paths=run('diff','HEAD','--name-only').splitlines()
    staged=run('diff','--cached','--name-only').splitlines()
    untracked=run('ls-files','--others','--exclude-standard').splitlines()
    assert not untracked,(repo.name,untracked)
    if repo.name=='hypercurve':
        assert paths==(remaining if phase=='committed' else r['workspace_changed']),paths
        assert staged==(selected if phase=='staged' else []),staged
        if phase=='committed':
            assert run('rev-parse','HEAD^')==r['parent']
            assert run('diff-tree','--no-commit-id','--name-only','-r','HEAD').splitlines()==selected
        else:assert head==r['parent']
        if phase!='qualified':
            for path in selected:
                data=subprocess.check_output(['git','show',(':' if phase=='staged' else 'HEAD:')+path],cwd=repo)
                assert hashlib.sha256(data).hexdigest()==manifest['hypercurve/'+path],path
    else:assert not paths and not staged,(repo.name,paths,staged)
    subprocess.run(['git','diff','--check'],cwd=repo,check=True)
    subprocess.run(['git','diff','--cached','--check'],cwd=repo,check=True)
    heads.append(dict(name=repo.name,head=head))
assert len(heads)==30
receipt=dict(phase=phase,parent=r['parent'],candidate=f'{prefix}-terminal.json',
             files={path:manifest['hypercurve/'+path] for path in selected},
             source_manifest=r['source_manifest'],workspace_guard=r['workspace_guard'],
             pre_change='native-root-witness-20260925-v71-terminal.json',
             strict_witness_regressions=sorted(regressions),checks=3,passed=244,
             library_passes=135,public_passes=109,ignored=0,remaining_uncommitted=remaining,
             all_owned_processes_reaped=True,all_inputs_and_executables_match=True,
             repositories=heads,full_goal_complete=False)
(A/f'native-evidence-reuse-20260925-v83-{phase}.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(phase+': 244 passes, 109 public cases, original witness regressions and all sources/binaries bound; 30 repositories audited')
