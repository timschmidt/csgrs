from pathlib import Path
import hashlib,json,re,subprocess

A=Path(__file__).resolve().parent;W=A.parent;prefix='resultant-interpolation-20260928-v687'
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
hc=sum(c['target']!='hypersolve'for c in r['cases']);hs=len(r['cases'])-hc;assert hc>=123 and hs==218
assert set(r['expected_cases'])=={'hypersolve','hypercurve','hypercurve_curve','hypercurve_curve_intersection','hypercurve_analytic_parallel_region','hypercurve_stationary_fillets'}
for target,listing in r['test_listings'].items():
 assert listing['returncode']==0
 available={line.removesuffix(': test')for line in(A/listing['log']).read_text().splitlines()if line.endswith(': test')}
 assert set(r['expected_cases'][target])<=available
prior=json.loads((A/'spline-authoring-20260928-v678-terminal.json').read_text());assert prior['qualification_complete'] and prior['all_processes_reaped']
old_manifest=json.loads((A/prior['source_manifest']).read_text())
changed={'hypersolve/src/resultant.rs','hypersolve/src/curve_resultant.rs','hypercurve/src/bezier_offset.rs'}
assert {name for name,sha in manifest.items()if sha!=old_manifest[name]}==changed
old_root=Path(prior['source_directory'])
paths={}
for name in sorted(changed):
 repo,path=name.split('/',1);paths.setdefault(repo,[]).append(path)
old_times={(c['target'],c['name']):c['elapsed_seconds']for p in ['unit-domain-core-20260928-v649','known-fillet-centers-20260928-v671','spline-authoring-20260928-v678']for c in json.loads((A/f'{p}-terminal.json').read_text())['cases']}
regressions=[]
for c in r['cases']:
 old=old_times.get((c['target'],c['name']))
 if old is not None and old>.1 and c['elapsed_seconds']>old*1.25+.2:regressions.append(dict(name=c['name'],before=old,after=c['elapsed_seconds']))
previous=json.loads((A/'spline-authoring-20260928-v678-repositories-after.json').read_text());repositories={};repos={}
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
probe=json.loads((A/'resultant-newton-20260928-v685-terminal.json').read_text())
assert probe['probe_complete'] and probe['all_processes_reaped']
assert manifest==json.loads((A/probe['source_manifest']).read_text())
for build in probe['builds']:
 for target,old in build['binaries'].items():
  assert digest(Path(old['path']))==old['sha256']==binaries[target]['sha256']
reused=[c for c in r['cases'] if 'reused_from' in c]
assert len(reused)==222
assert [{k:v for k,v in c.items()if k!='reused_from'}for c in reused]==probe['cases']
assert [{k:v for k,v in c.items()if k!='reused_from'}for c in r['checks']if 'reused_from'in c]==probe['checks']
assert len(probe['cases'])==222 and len(probe['checks'])==3
stress=[c for c in reused if c['target']=='hypercurve']
assert len(stress)==4 and all(c['elapsed_seconds']<60 for c in stress)
# Geometry implementation is unchanged; its sole diff retains the independent
# exact monotone/reversed polynomial reparameterization regressions.
old_offset=(old_root/'hypercurve/src/bezier_offset.rs').read_text()
new_offset=(W/'hypercurve/src/bezier_offset.rs').read_text()
assert new_offset.startswith(old_offset.rstrip())
assert 'mod structural_overlap_trace_regression' in new_offset[len(old_offset.rstrip()):]
(A/f'{prefix}-repositories-before.json').write_text(json.dumps(repositories,indent=2)+'\n')
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),selected_geometry_tests=hc,selected_algebraic_tests=hs,checks=9,repos=repos,scope='Reuse rational scalar resultant samples with exact signed content and construct interpolation in quadratic work; retain independent polynomial reparameterization geometry regressions; full implementation goal remains active',performance_changes=regressions,actual_build_artifacts=actual_build_artifacts,unresolved='The four degree-six probes now complete in 47-48 seconds; further curve/API simplification and computational closure work remains. This qualification does not establish every arbitrary composition.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
(A/f'{prefix}-hypersolve-commit.txt').write_text(f"""Reuse exact rational samples and quadratic resultant interpolation

Bivariate resultant interpolation now reuses the exact scalar-value kernel after certifying rational coefficients. Clear each polynomial's rational content once, use the existing flat integer determinant, and restore both signed contents to their resultant homogeneity powers. General exact coefficients retain the existing predicate path.

Replace repeated Lagrange basis construction with exact divided differences and nested Newton conversion, reducing reconstruction from cubic to quadratic arithmetic work with linear live storage. Preserve nonuniform sample nodes, skipped degree-drop fibers, full coefficient scale and final strict trimming. No curve-specific algorithm or public interface is added.

Validation: {hs} algebraic and {hc} geometry release tests across six pinned binaries, plus nine lint/format/fuzz/docs/downstream checks. Independent tests verify fractional coefficients against the full determinant report, a signed degree-13 identity on both axes including exceptional fibers, and pi/sqrt(2) interpolation. Four degree-six offset cases that exceeded 180 seconds now complete in 47-48 seconds with both expected crossing contacts.
""")
(A/f'{prefix}-hypercurve-commit.txt').write_text(f"""Cover offset crossings through exact polynomial reparameterization

Retain independent same/reversed degree-six representations of the S-cubic composed with (t+t^2)/2. Its chart derivative is positive on the unit span, so both source-normal selection and the two offset crossing contacts must survive. Check complete intersections under STRICT and APPROXIMATE_512.

The shared Hypersolve resultant simplification makes all four cases complete in 47-48 seconds after the previous 180-second timeouts. Validation: {hc} geometry and {hs} algebraic release tests, six pinned binaries, and nine checks. Existing source-domain, component, pole, fillet/chamfer and Boolean caller regressions remain passing.
""")
print('Reviewed',hc,'geometry and',hs,'algebraic tests; 9 checks;',len(manifest),'sources;',len(repositories),'repositories; performance changes',regressions)
