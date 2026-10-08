from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='homogeneous-projection-v800'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
reaped=json.loads((A/f'{prefix}-reaped.json').read_text());assert reaped['outer_exit_code']==0
r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['normal_production_build']and r['qualification_complete']and r['probe_complete']and r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text());assert len(manifest)==2048
for name,sha in manifest.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
assert len(r['builds'])==1 and len(r['test_listings'])==11 and len(r['checks'])==7 and all(c['returncode']==0 for c in r['checks'])
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
expected=json.loads((A/'homogeneous-projection-cases-v800.json').read_text());expected_set=set()
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
promotion=json.loads((A/'homogeneous-projection-promotion-v800.json').read_text());changed=set(promotion['promoted']);assert len(changed)==5
old_manifest=json.loads((A/'conic-contact-domain-v796-sources.json').read_text());assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:
 assert digest(Path(promotion['candidates'][name]))==manifest[name]==promotion['promoted'][name]
 assert promotion['base'][name]==old_manifest[name]
def interval(source,start,end):
 a=source.index(start);b=source.index(end,a);return source[a:b]
old_root=A/'source-archives/conic-contact-domain-v796'
quad_name='hypercurve/src/rational_bezier.rs';general_name='hypercurve/src/rational_bezier_general.rs'
before=(old_root/quad_name).read_text();after=(W/quad_name).read_text()
assert before[:before.index('        let denominator = self.denominator_at(&t);')]==after[:after.index('        let denominator = self.denominator_at(&t);')]
assert before[before.index('    fn point_at_quadratic_weight('):]==after[after.index('    fn point_at_quadratic_weight('):]
quad=interval(after,'    pub fn point_at(', '    fn point_at_quadratic_weight(')
assert 'crate::rational_bezier_general::project_homogeneous(' in quad
assert 'numerator_x /' not in quad and 'numerator_y /' not in quad
rational=(W/general_name).read_text()
projection=interval(rational,'pub(crate) fn project_homogeneous<', 'fn from_homogeneous(')
assert projection.index('match is_zero(weight, policy)')<projection.index('let [x, y] = numerators();')
assert 'numerators: impl FnOnce() -> [T; 2]' in projection
assert projection.count('UncertaintyReason::from_real_division(error)')==2
assert 'clone()' not in projection and 'inverse_ref' not in projection
classifier=(W/'hypercurve/src/classify.rs').read_text()
assert 'hyperreal::Problem::DivideByZero => Self::Boundary' in classifier
assert 'hyperreal::Problem::UnknownZero => Self::RealSign' in classifier
assert rational.count('UncertaintyReason::from_real_division(error)')==7
assert set(expected)=={'hypercurve','hypercurve_bezier_algebraic_image','hypercurve_curve_point','hypercurve_bezier_algebraic_parameter','hypercurve_bezier_split_materialization','hypercurve_bezier_tangent_order','hypercurve_bezier_arrangement','hypercurve_curve_region_promotion','hypercurve_rational_bezier','hypercurve_bspline','hypercurve_nurbs'}
assert len(expected_set)>169
probe=json.loads((A/'nonzero-projection-probe-v801-terminal.json').read_text())
probe_reap=json.loads((A/'nonzero-projection-probe-v801-reaped.json').read_text())
assert probe_reap['outer_exit_code']==0 and probe['probe_complete']and probe['all_processes_reaped']
assert probe['baseline']==r['source_manifest']and all(p['returncode']==0 for p in probe['processes'])
assert digest(Path(probe['rlib']))==probe['rlib_sha256']
assert probe['source_sha256']==digest(A/'nonzero-projection-probe-v799.rs')==json.loads((A/'nonzero-projection-probe-v799-terminal.json').read_text())['source_sha256']
assert (A/'nonzero-projection-probe-v801-run.log').read_bytes()==(A/'nonzero-projection-probe-v799-run.log').read_bytes()
previous=json.loads((A/'conic-contact-domain-v796-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=len(expected_set),checks=7,repos=repos,scope='Share deferred homogeneous projection across rational curve families and retain precise scalar division blockers in projection, derivatives and projective endpoint correspondence; preserve specialized quadratic-weight arithmetic and borrowed controls without new public or scalar APIs',unresolved='The full exact-geometry implementation goal remains active; this cleanup does not claim full spatial BREP closure or a measured speedup.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
print('Reviewed',len(expected_set),'release cases,eleven pinned binaries,seven checks,2048sources and30repositories')
