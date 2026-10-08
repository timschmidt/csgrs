from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='mixed-endpoint-order-v877'
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
expected=json.loads((A/'mixed-endpoint-order-cases-v877.json').read_text());expected_set=set()
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
promotion=json.loads((A/'mixed-endpoint-order-promotion-v877.json').read_text());changed=set(promotion['promoted']);assert len(changed)==2
old_manifest=json.loads((A/'polynomial-zero-jet-v874-sources.json').read_text());assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:
 assert digest(Path(promotion['candidates'][name]))==manifest[name]==promotion['promoted'][name]
 assert promotion['base'][name]==old_manifest[name]
old_root=A/'source-archives/polynomial-zero-jet-v874'
name='hypercurve/src/bezier_arrangement.rs';before=(old_root/name).read_text();after=(W/name).read_text()
def function(source,name):
 match=re.search(r'(?m)^([ \t]*)fn '+re.escape(name)+r'[(<]',source);assert match,name
 end=re.search(r'(?m)^'+match[1]+r'}',source[match.end():]);assert end,name
 return source[match.start():match.end()+end.end()]
def compact(value):return re.sub(r'\s+','',value)
adapter=function(after,'retained_tangent_as_algebraic')
assert 'Cow::Borrowed(tangent.as_ref())'in adapter and '.clone()'not in adapter
assert adapter.count('AlgebraicRootRepresentation::from_exact_value(')==2
old_compare=function(before,'compare_retained_same_tangent_second_order');new_compare=function(after,'compare_retained_same_tangent_second_order')
def native_arm(value):return value[value.index('    match (&first_tangent, &second_tangent)'):value.index('            policy,\n        ),')+len('            policy,\n        ),')]
assert native_arm(old_compare)==native_arm(new_compare)
assert new_compare.count('let first_tangent = retained_tangent_as_algebraic(')==1
assert new_compare.count('let second_tangent = retained_tangent_as_algebraic(')==1
assert new_compare.count('.map(retained_tangent_as_algebraic)')==2
assert compact(new_compare).count('Classification::Decided(TurnOrdering::SameDirection)')==3
old_third=function(before,'compare_retained_algebraic_same_tangent_third_order');new_third=function(after,'compare_retained_algebraic_same_tangent_third_order')
assert new_third.count('.map(retained_tangent_as_algebraic)')==2
assert 'if zero_curvature {'in new_third and 'compare_algebraic_equal_curvature_third_order('in new_third
# Strip only the reviewed adapter/comparison functions and fixture changes;
# every other byte, including the source loader and exact ordering kernels,
# must match the qualified parent.
normalized=after.replace('borrow::Cow, ','')
for fn in ['retained_tangent_as_algebraic','compare_retained_same_tangent_second_order','compare_retained_algebraic_same_tangent_third_order','equal_curvature_graph_jets_ignore_source_acceleration','lazy_polynomial_endpoint_derivatives_match_eager_images']:
 normalized=normalized.replace(function(normalized,fn),function(before,fn))
removed=function(before,'retained_algebraic_vector')
assert 'retained_algebraic_vector'not in after
normalized=normalized.replace('fn compare_same_tangent_second_order(',removed+'\n\nfn compare_same_tangent_second_order(',1)
assert normalized==before
unit=function(after,'equal_curvature_graph_jets_ignore_source_acceleration')
assert '[(false, false), (false, true), (true, false), (true, true)]'in unit
assert 'Real::pi()'in unit and 'second_algebraic' in unit
name='hypercurve/tests/hypercurve_bezier_arrangement.rs';before=(old_root/name).read_text();after=(W/name).read_text();assert after.startswith(before.rstrip())
assert len(expected_set)==340
for target,name in [('hypercurve','bezier_arrangement::endpoint_adjacency_tests::polynomial_derivative_sources_retain_exact_zero'),('hypercurve_bezier_arrangement','mixed_native_and_selected_endpoints_share_tangent_ordering')]:assert(target,name)in expected_set
for probe,blocked in [('mixed-endpoint-order-probe-v876',48),('mixed-endpoint-order-replay-v878',0)]:
 assert json.loads((A/f'{probe}-reaped.json').read_text())['outer_exit_code']==0
 data=json.loads((A/f'{probe}-terminal.json').read_text());assert data['probe_complete']and data['all_processes_reaped']and all(p['returncode']==0 for p in data['processes'])
 assert f'complete requests=48 blocked={blocked}'in data['output']
 assert data['source_sha256']==digest(A/'mixed-endpoint-order-probe-v876.rs')
 if blocked==0:assert data['baseline']==r['source_manifest']
previous=json.loads((A/'polynomial-zero-jet-v874-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=len(expected_set),checks=7,repos=repos,scope='Compare native and selected algebraic endpoint derivatives through one exact retained ordering adapter. Borrow represented roots, promote exact native coordinates, remove the representation filter and retain the unchanged all-native fast path. Preserve derivative demand, source identity, zero-curvature guards, exact arithmetic and domain behavior. An unchanged 48-case public probe repairs all 48 mixed-carrier blockers; the graph-jet oracle also covers mixed arbitrary exact pi coefficients.',unresolved='The full exact-geometry implementation goal remains active; this migration does not establish universal composition closure or a measured speedup.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
print('Reviewed',len(expected_set),'release cases,twelve pinned binaries,seven checks,2048sources and30repositories')
