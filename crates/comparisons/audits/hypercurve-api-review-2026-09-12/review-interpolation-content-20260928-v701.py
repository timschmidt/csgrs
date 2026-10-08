from pathlib import Path
import hashlib,json,re,subprocess

A=Path(__file__).resolve().parent;W=A.parent;prefix='interpolation-content-20260928-v701'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
reaped=json.loads((A/f'{prefix}-reaped.json').read_text());assert reaped['outer_exit_code']==0
r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['normal_production_build'] and r['qualification_complete'] and r['probe_complete'] and r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text());assert len(manifest)==2047
for name,sha in manifest.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
assert len(r['builds'])==2 and len(r['checks'])==9 and len(r['test_listings'])==6
# Verify original Cargo artifact ownership and both executable copies.
actual_build_artifacts={};binaries={}
for crate,build in zip(['hypersolve','hypercurve'],r['builds']):
 assert build['returncode']==0 and build['log']==f'{prefix}-{crate}-build.log'
 requested={'hypersolve'}if crate=='hypersolve'else set(r['expected_cases'])-{'hypersolve'}
 artifacts={}
 for line in (A/build['log']).read_text().splitlines():
  try:message=json.loads(line)
  except ValueError:continue
  target=message.get('target',{}).get('name')
  if message.get('reason')=='compiler-artifact'and target in requested and message.get('executable'):
   artifacts[target]=message
 assert set(artifacts)==requested,(crate,set(artifacts),requested)
 for target,message in artifacts.items():
  artifact=build['binaries'][target]
  assert digest(Path(message['executable']))==artifact['sha256'],target
  assert digest(Path(artifact['path']))==artifact['sha256'],target
  assert target not in binaries
  binaries[target]=artifact
 actual_build_artifacts[crate]=sorted(artifacts)
assert len(binaries)==6
for build in r['builds']:
 for target,artifact in build['binaries'].items():assert artifact==binaries[target],target
expected={(target,name)for target,names in r['expected_cases'].items()for name in names}
assert len(r['cases'])==len(expected)
assert {(c['target'],c['name'])for c in r['cases']}==expected
assert len({c['log']for c in r['cases']})==len(r['cases'])
for c in r['cases']:assert c['passed'] and c['returncode']==0 and re.search(r'test result: ok\. 1 passed;',(A/c['log']).read_text())
assert all(c['returncode']==0 for c in r['checks'])
hc=sum(c['target']!='hypersolve'for c in r['cases']);hs=len(r['cases'])-hc;assert hc>=123 and hs==219
assert set(r['expected_cases'])=={'hypersolve','hypercurve','hypercurve_curve','hypercurve_curve_intersection','hypercurve_analytic_parallel_region','hypercurve_stationary_fillets'}
for target,listing in r['test_listings'].items():
 assert listing['returncode']==0
 available={line.removesuffix(': test')for line in(A/listing['log']).read_text().splitlines()if line.endswith(': test')}
 assert set(r['expected_cases'][target])<=available
prior=json.loads((A/'projection-overlap-20260928-v695-terminal.json').read_text());assert prior['qualification_complete'] and prior['all_processes_reaped']
old_manifest=json.loads((A/prior['source_manifest']).read_text())
changed={'hypersolve/src/curve_resultant.rs'}
assert {name for name,sha in manifest.items()if sha!=old_manifest[name]}==changed
old_root=Path(prior['source_directory'])
paths={}
for name in sorted(changed):
 repo,path=name.split('/',1);paths.setdefault(repo,[]).append(path)
old_times={(c['target'],c['name']):c['elapsed_seconds']for p in ['resultant-interpolation-20260928-v687','projection-overlap-20260928-v695']for c in json.loads((A/f'{p}-terminal.json').read_text())['cases']}
regressions=[]
for c in r['cases']:
 old=old_times.get((c['target'],c['name']))
 if old is not None and old>.1 and c['elapsed_seconds']>old*1.25+.2:regressions.append(dict(name=c['name'],before=old,after=c['elapsed_seconds']))
previous=json.loads((A/'projection-overlap-20260928-v695-repositories-after.json').read_text());repositories={};repos={}
for name,old in previous.items():
 head=git(name,'rev-parse','HEAD').decode().strip();status=git(name,'status','--porcelain=v1').decode();assert head==old['head'],name
 if name in paths:
  assert status==''.join(' M '+p+'\n'for p in paths[name]),(name,status)
  assert not git(name,'diff','--cached','--name-only').strip()
  subprocess.run(['git','diff','--check'],cwd=W/name,check=True)
  repos[name]=dict(parent=head,paths={p:manifest[name+'/'+p]for p in paths[name]})
 else:assert not status,name
 repositories[name]=dict(head=head,status=status)
for name in manifest:
 data=(W/name).read_bytes()
 assert b'NurbsBezierDecomposition2'not in data and b'PolynomialSplineBezierDecomposition2'not in data,name
 assert b'BezierParameter2::exact('not in data and b'BezierParameter2::algebraic('not in data,name
 if b'PolynomialBSplineCurve2'in data or b'RationalBSplineCurve2'in data:assert name in {'hypercurve/src/bspline.rs','hypercurve/src/nurbs.rs','hypercurve/src/polynomial_spline.rs'},name
# Candidate receipts may be reused only for exactly matching promoted sources
# and normal Cargo executables, with the original cases and checks unchanged.
probe=json.loads((A/'interpolation-content-20260928-v697-terminal.json').read_text())
assert probe['probe_complete'] and probe['all_processes_reaped']
assert manifest==json.loads((A/probe['source_manifest']).read_text())
for build in probe['builds']:
 for target,old in build['binaries'].items():
  assert digest(Path(old['path']))==old['sha256']==binaries[target]['sha256']
reused=[c for c in r['cases'] if 'reused_from' in c]
assert len(reused)==220
assert [{k:v for k,v in c.items()if k!='reused_from'}for c in reused]==probe['cases']
assert [{k:v for k,v in c.items()if k!='reused_from'}for c in r['checks']if 'reused_from'in c]==probe['checks']
assert len(probe['cases'])==220 and len(probe['checks'])==3
stress=[c for c in reused if c['target']=='hypercurve']
assert len(stress)==1 and all(c['elapsed_seconds']<60 for c in stress)
# Geometry implementation is unchanged; its sole diff retains the independent
# exact monotone/reversed polynomial reparameterization regressions.
old_offset=(old_root/'hypercurve/src/bezier_offset.rs').read_text()
new_offset=(W/'hypercurve/src/bezier_offset.rs').read_text()
assert new_offset==old_offset
(A/f'{prefix}-repositories-before.json').write_text(json.dumps(repositories,indent=2)+'\n')
comparison=json.loads((A/'interpolation-content-comparison-20260928-v699-terminal.json').read_text())
assert comparison['comparison_complete'] and comparison['all_processes_reaped'] and len(comparison['cases'])==6
assert json.loads((A/'interpolation-content-comparison-20260928-v699-reaped.json').read_text())['outer_exit_code']==0
assert all(c['passed'] and c['returncode']==0 for c in comparison['cases'])
baseline=comparison['summary']['baseline'];candidate=comparison['summary']['candidate']
assert max(candidate['samples'])<min(baseline['samples'])
percent=(candidate['median_seconds']/baseline['median_seconds']-1)*100
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),selected_geometry_tests=hc,selected_algebraic_tests=hs,checks=9,repos=repos,scope='Remove common rational ordinate content once before exact Newton interpolation and restore complete coefficient values afterward; arbitrary exact fallback unchanged',performance_changes=regressions,matched_comparison='interpolation-content-comparison-20260928-v699-terminal.json',matched_median_change_percent=percent,actual_build_artifacts=actual_build_artifacts,unresolved='This small common-scale normalization does not establish all arbitrary operation compositions; full goal remains active.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
(A/f'{prefix}-commit.txt').write_text(f"""Remove common rational content before exact interpolation

Newton interpolation is linear in sampled ordinates. Normalize exact rational samples to primitive integers once, retain their exact common scale, and restore every signed coefficient after reconstruction. This avoids carrying a redundant common factor through every divided difference and basis conversion. Arbitrary exact sample values retain the existing general path, with no new public interface or rational coefficient restriction.

An independent scaled binomial(t,6) identity verifies fractional coefficients despite integer sample values, nonuniform/reversed nodes, zero data, and a large negative rational scale. All prior arbitrary-exact and full signed resultant identities remain covered.

Validation: {hs} algebraic and {hc} geometry release tests across six pinned binaries, nine lint/format/fuzz/docs/downstream checks. Three serial matched runs of the unchanged degree-six offset/re-entry workload moved the median from {baseline['median_seconds']:.3f}s to {candidate['median_seconds']:.3f}s ({percent:.2f}%). All candidate samples were faster than all baseline samples; interval ownership, two crossing contacts and public overlap clipping remained exact.
""")
print('Reviewed',hc,'geometry and',hs,'algebraic tests; nine checks; median change',round(percent,2),'percent; timing flags',regressions)
