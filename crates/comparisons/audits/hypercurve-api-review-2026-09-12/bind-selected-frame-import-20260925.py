from pathlib import Path
import hashlib,json,subprocess,sys
A=Path(__file__).resolve().parent
W=A.parent
phase=sys.argv[1]
assert phase in {'qualified','staged','committed'}
prefix='selected-frame-import-20260925-v94-candidate'
r=json.loads((A/f'{prefix}-terminal.json').read_text())
m=json.loads((A/r['source_manifest']).read_text())
g=json.loads((A/r['workspace_guard']).read_text())
root=Path(r['source_directory'])
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert r['all_processes_reaped'] and r['all_sources_unchanged'] and r['build_returncode']==0
assert len(r['cases'])==len(r['selection'])==252
assert all(c['returncode']==0 and not c['ignored'] for c in r['cases'])
assert sum(c['target']=='hypercurve' for c in r['cases'])==142
assert sum('reused_from' in c for c in r['cases'])==251
assert len(r['checks'])==3 and all(c['returncode']==0 for c in r['checks'])
assert len(m)==len(g)==2044
for name,expected in m.items():
    assert sha(root/name)==expected,name
    assert sha(Path(r['build_source_directory'])/name)==expected,name
    assert sha(W/name)==g[name],name
for binary in r['binaries'].values():assert sha(Path(binary['path']))==binary['sha256']
prior=json.loads((A/r['reused_from']).read_text())
pm=json.loads((A/prior['source_manifest']).read_text())
pr=A/'source-archives/selected-frame-import-20260925-v92-candidate'
assert prior['all_processes_reaped'] and prior['all_sources_unchanged']
assert [n for n in m if m[n]!=pm[n]]==['hypercurve/tests/hypercurve_curve.rs']
assert len(prior['cases'])==252 and sum(c['returncode']==0 for c in prior['cases'])==251
for name,expected in pm.items():assert sha(pr/name)==expected,name
for binary in prior['binaries'].values():assert sha(Path(binary['path']))==binary['sha256']
for case in r['cases']:
    if 'reused_from' in case:
        original=next(c for c in prior['cases'] if c['target']==case['target'] and c['name']==case['name'])
        assert {k:v for k,v in case.items() if k!='reused_from'}==original
baseline=json.loads((A/'translated-selected-contact-20260925-v88-terminal.json').read_text())
bm=json.loads((A/'local-chord-complete-replay-20260924-v88-sources.json').read_text())
br=A/'source-archives/hypercurve-local-chord-complete-replay-v88-20260924'
assert baseline['all_processes_reaped'] and baseline['all_sources_unchanged']
assert baseline['build_returncode']==0
regression='translated_selected_contact_import_omits_the_unused_center'
assert next(c for c in baseline['cases'] if c['name'].endswith('::'+regression))['returncode']==101
for name,expected in bm.items():
    assert sha(br/name)==expected,name
    if not name.startswith('hypercurve/'):assert expected==m[name],name
for binary in baseline['binaries'].values():assert sha(Path(binary['path']))==binary['sha256']
old=(br/'hypercurve/src/bezier_offset.rs').read_text()
new=(root/'hypercurve/src/bezier_offset.rs').read_text()
def regression_body(s):
    a=s.index('    #[test]\n    fn '+regression+'()')
    return s[a:s.index('    fn check_selected_fiber_rational_overlap(',a)]
assert regression_body(old)==regression_body(new)
head_offset=subprocess.check_output(['git','show',r['parent']+':src/bezier_offset.rs'],cwd=W/'hypercurve',text=True)
def old_point_import(s):
    a=s.index('    fn recursive_projective_point(',s.index('impl BezierAlgebraicCuspChordDerivedPoint2 {'))
    return s[a:s.index('    /// Compares affine images',a)]
assert old_point_import(old)==old_point_import(head_offset)
probe=json.loads((A/'native-root-witness-overlap-isolated-20260925-v92-terminal.json').read_text())
assert probe['returncode']==0 and probe['all_processes_reaped'] and probe['all_inputs_unchanged']
assert sha(Path(probe['executable']))==probe['executable_sha256']
selected=r['selected_files']
assert selected==['src/bezier_offset.rs','tests/hypercurve_curve.rs']
remaining=sorted(n.removeprefix('hypercurve/') for n in g if n.startswith('hypercurve/') and g[n]!=m[n])
assert remaining==['src/bezier_offset.rs','src/bezier_region.rs','src/curve.rs','src/curve_corner_chain.rs']
heads=[]
for repo in sorted(W.iterdir()):
    if not (repo/'.git').exists():continue
    run=lambda *args:subprocess.check_output(['git',*args],cwd=repo,text=True).strip()
    head=run('rev-parse','HEAD')
    paths=run('diff','HEAD','--name-only').splitlines()
    staged=run('diff','--cached','--name-only').splitlines()
    assert not run('ls-files','--others','--exclude-standard'),repo.name
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
                assert hashlib.sha256(data).hexdigest()==m['hypercurve/'+path],path
    else:assert not paths and not staged,(repo.name,paths,staged)
    subprocess.run(['git','diff','--check'],cwd=repo,check=True)
    subprocess.run(['git','diff','--cached','--check'],cwd=repo,check=True)
    heads.append(dict(name=repo.name,head=head))
assert len(heads)==30
receipt=dict(phase=phase,parent=r['parent'],candidate=f'{prefix}-terminal.json',files={p:m['hypercurve/'+p] for p in selected},checks=3,passed=252,library_passes=142,public_passes=110,unchanged_cases_reused=251,fresh_geometric_fillet_case=True,ignored=0,pre_change='translated-selected-contact-20260925-v88-terminal.json',workspace_guard=r['workspace_guard'],source_manifest=r['source_manifest'],remaining_uncommitted=remaining,all_owned_processes_reaped=True,all_inputs_and_executables_match=True,repositories=heads,full_goal_complete=False)
(A/f'selected-frame-import-20260925-v94-{phase}.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(phase+': 252 qualified passes (251 source-identical reused cases + fresh geometric fillet); original failing regression, all source/binary hashes and 30 repositories audited')
