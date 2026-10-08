from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='polynomial-zero-jet-v874'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
reaped=json.loads((A/f'{prefix}-reaped.json').read_text());assert reaped['outer_exit_code']==0
r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['normal_production_build']and r['qualification_complete']and r['probe_complete']and r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text());assert len(manifest)==2048
for name,sha in manifest.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
assert len(r['builds'])==1 and len(r['test_listings'])==12 and len(r['checks'])==7 and all(c['returncode']==0 for c in r['checks'])
binaries={}
for build in r['builds']:
 assert build['returncode']==0
 artifacts={}
 for line in (A/build['log']).read_text().splitlines():
  try:m=json.loads(line)
  except ValueError:continue
  if m.get('reason')=='compiler-artifact'and m.get('target',{}).get('name')in build['binaries']and m.get('executable'):artifacts[m['target']['name']]=m
 assert set(artifacts)==set(build['binaries'])
 for target,binary in build['binaries'].items():
  assert digest(Path(artifacts[target]['executable']))==binary['sha256']==digest(Path(binary['path']))
  assert target not in binaries;binaries[target]=binary
expected=json.loads((A/'polynomial-zero-jet-cases-v874.json').read_text());expected_set=set()
for target,listing in r['test_listings'].items():
 assert listing['returncode']==0
 available={s.removesuffix(': test')for s in(A/listing['log']).read_text().splitlines()if s.endswith(': test')}
 selected=set(expected[target])if expected[target]is not None else available
 assert selected<=available and selected==set(r['expected_cases'][target]);expected_set.update((target,name)for name in selected)
assert len(r['cases'])==len(expected_set)
assert expected_set=={(c['target'],c['name'])for c in r['cases']}and len({c['log']for c in r['cases']})==len(expected_set)
for c in r['cases']:
 assert c['passed']and c['returncode']==0 and 'test result: ok. 1 passed;'in(A/c['log']).read_text()
 assert c['command'][0]==binaries[c['target']]['path']and '--include-ignored'in c['command']
promotion=json.loads((A/'polynomial-zero-jet-promotion-v874.json').read_text());changed=set(promotion['promoted']);assert len(changed)==2
old_manifest=json.loads((A/'equal-curvature-v871-sources.json').read_text());assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:
 assert digest(Path(promotion['candidates'][name]))==manifest[name]==promotion['promoted'][name]
 assert promotion['base'][name]==old_manifest[name]
old_root=A/'source-archives/equal-curvature-v871'
name='hypercurve/src/bezier_arrangement.rs';before=(old_root/name).read_text();after=(W/name).read_text()
a=after.index('    #[test]\n    fn polynomial_derivative_sources_retain_exact_zero()');b=after.index('    #[test]\n    fn equal_curvature_graph_jets_ignore_source_acceleration()',a);normalized=after[:a]+after[b:]
a=normalized.index('    if matches!(',normalized.index('fn retained_algebraic_derivative('));b=normalized.index('    let derivative = match source.curve.as_ref()',a)
guard=normalized[a:b]
assert 'BezierSubcurve2::Quadratic(_), 3..'in guard and 'BezierSubcurve2::Cubic(_), 4..'in guard
assert 'Rational'not in guard and 'AlgebraicRootRepresentation::from_exact_value(&Real::zero())'in guard
assert 'BezierAlgebraicTangentVector2::new(zero.clone(), zero)'in guard
normalized=normalized[:a]+normalized[b:]
new='                    include_higher_derivatives.then(|| TangentVector {\n                        dx: Real::zero(),\n                        dy: Real::zero(),\n                    }),'
assert normalized.count(new)==1;normalized=normalized.replace(new,'                    None,')
assert normalized==before
name='hypercurve/tests/hypercurve_bezier_arrangement.rs';before=(old_root/name).read_text();after=(W/name).read_text();assert after.startswith(before.rstrip())
assert len(expected_set)==339
for target,name in [('hypercurve','bezier_arrangement::endpoint_adjacency_tests::polynomial_derivative_sources_retain_exact_zero'),('hypercurve_bezier_arrangement','native_quadratic_ordering_agrees_with_rational_carriers')]:assert(target,name)in expected_set
for probe,blocked in [('quadratic-zero-jet-probe-v873',6),('quadratic-zero-jet-replay-v875',0)]:
 assert json.loads((A/f'{probe}-reaped.json').read_text())['outer_exit_code']==0
 data=json.loads((A/f'{probe}-terminal.json').read_text());assert data['probe_complete']and data['all_processes_reaped']and all(p['returncode']==0 for p in data['processes'])
 assert f'complete requests=18 blocked={blocked}'in data['output']
 assert data['source_sha256']==digest(A/'quadratic-zero-jet-probe-v873.rs')
 if blocked==0:assert data['baseline']==r['source_manifest']
previous=json.loads((A/'equal-curvature-v871-repositories-after.json').read_text());repositories={};repos={}
for name,old in previous.items():
 head=git(name,'rev-parse','HEAD').decode().strip();status=git(name,'status','--porcelain=v1').decode();assert head==old['head'],name
 paths=sorted(p.removeprefix(name+'/')for p in changed if p.startswith(name+'/'))
 if paths:
  assert status==''.join(' M '+p+'\n'for p in paths),status
  assert not git(name,'diff','--cached','--name-only').strip();subprocess.run(['git','diff','--check'],cwd=W/name,check=True)
  repos[name]=dict(parent=head,paths={p:manifest[name+'/'+p]for p in paths})
 else:assert not status,name
 repositories[name]=dict(head=head,status=status)
(A/f'{prefix}-repositories-before.json').write_text(json.dumps(repositories,indent=2)+'\n')
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=len(expected_set),checks=7,repos=repos,scope='Retain exact zero derivative evidence above the degree of native polynomial sources. Supply the native quadratic third derivative only when higher evidence is requested; selected quadratic/cubic sources return represented zero without parameter arithmetic. Preserve all rational, domain, root, ordering and cache kernels. An unchanged eighteen-case public probe repairs six native-quadratic blockers and preserves conic/general carrier equivalence.',unresolved='The full exact-geometry implementation goal remains active; this migration does not establish universal composition closure or a measured speedup.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
print('Reviewed',len(expected_set),'release cases,twelve pinned binaries,seven checks,2048sources and30repositories')
