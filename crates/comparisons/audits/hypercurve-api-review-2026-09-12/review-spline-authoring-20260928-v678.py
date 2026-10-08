from pathlib import Path
import hashlib,json,re,subprocess

A=Path(__file__).resolve().parent;W=A.parent;prefix='spline-authoring-20260928-v678'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
reaped=json.loads((A/f'{prefix}-reaped.json').read_text());assert reaped['outer_exit_code']==0
r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['normal_production_build'] and r['qualification_complete'] and r['probe_complete'] and r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text());assert len(manifest)==2047
for name,sha in manifest.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
assert len(r['builds'])==1 and len(r['checks'])==6 and len(r['test_listings'])==5
# Verify original Cargo artifact ownership and both executable copies.
actual_build_artifacts={};binaries={}
for crate,build in zip(['hypercurve'],r['builds']):
 assert build['returncode']==0 and build['log']==f'{prefix}-build.log'
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
assert len(binaries)==5
for build in r['builds']:
 for target,artifact in build['binaries'].items():assert artifact==binaries[target],target
expected={(target,name)for target,names in r['expected_cases'].items()for name in names}
assert len(r['cases'])==len(expected)
assert {(c['target'],c['name'])for c in r['cases']}==expected
assert len({c['log']for c in r['cases']})==len(r['cases'])
for c in r['cases']:assert c['passed'] and c['returncode']==0 and re.search(r'test result: ok\. 1 passed;',(A/c['log']).read_text())
assert all(c['returncode']==0 for c in r['checks'])
hc=sum(c['target']!='hypersolve'for c in r['cases']);hs=len(r['cases'])-hc;assert hc==156 and hs==0
assert set(r['expected_cases'])=={'hypercurve','hypercurve_polynomial_spline','hypercurve_bspline','hypercurve_curve','hypercurve_nurbs'}
for target,listing in r['test_listings'].items():
 assert listing['returncode']==0
 available={line.removesuffix(': test')for line in(A/listing['log']).read_text().splitlines()if line.endswith(': test')}
 assert set(r['expected_cases'][target])<=available
prior=json.loads((A/'polynomial-decomposition-20260928-v675-terminal.json').read_text());assert prior['qualification_complete'] and prior['all_processes_reaped']
old_manifest=json.loads((A/prior['source_manifest']).read_text())
changed={'hypercurve/src/polynomial_spline.rs', 'hypercurve/tests/hypercurve_bspline.rs', 'hypercurve/src/bspline.rs', 'hypercurve/src/lib.rs', 'hypercurve/benches/bspline.rs', 'hypercurve/src/nurbs.rs', 'hypercurve/fuzz/fuzz_targets/bspline.rs'}
assert {name for name,sha in manifest.items()if sha!=old_manifest[name]}==changed
old_root=Path(prior['source_directory'])
def facts(s):
 a=s.index('fn native_span_fact_evidence(');b=s.index('\nfn subcurve_certified_bounds',a);return s[a:b]
assert facts((W/'hypercurve/src/bspline.rs').read_text())==facts((old_root/'hypercurve/src/bspline.rs').read_text())
paths={}
for name in sorted(changed):
 repo,path=name.split('/',1);paths.setdefault(repo,[]).append(path)
old_times={(c['target'],c['name']):c['elapsed_seconds']for p in ['nurbs-decomposition-20260928-v667','polynomial-decomposition-20260928-v675']for c in json.loads((A/f'{p}-terminal.json').read_text())['cases']}
regressions=[]
for c in r['cases']:
 old=old_times.get((c['target'],c['name']))
 if old is not None and old>.1 and c['elapsed_seconds']>old*1.25+.2:regressions.append(dict(name=c['name'],before=old,after=c['elapsed_seconds']))
previous=json.loads((A/'polynomial-decomposition-20260928-v675-repositories-after.json').read_text());repositories={};repos={}
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
# The migrated public extraction tests retain every original case.
old_tests=(old_root/'hypercurve/tests/hypercurve_bspline.rs').read_text()
new_tests=(W/'hypercurve/tests/hypercurve_bspline.rs').read_text()
assert re.findall(r'#\[test\]\s*fn (\w+)',old_tests)==re.findall(r'#\[test\]\s*fn (\w+)',new_tests)
assert len(r['expected_cases']['hypercurve_bspline'])==24
# All changes to extraction and knot editing are only visibility or constructor names.
old_bspline=(old_root/'hypercurve/src/bspline.rs').read_text()
new_bspline=(W/'hypercurve/src/bspline.rs').read_text()
for struct in ['PolynomialBSplineCurve2','RationalBSplineCurve2']:
 assert 'pub(crate) struct '+struct+' {' in new_bspline
assert 'try_new_with_periodicity' not in new_bspline
assert 'from_homogeneous_with_periodicity' not in new_bspline
for name in ['extract_refined_bezier_spans','extract_refined_rational_spans','native_span_fact_evidence','validate_bspline_layout']:
 def body(text):
  start=text.index('fn '+name+'('); opening=text.index('{',start);depth=1;end=opening+1
  while depth:
   if text[end]=='{':depth+=1
   elif text[end]=='}':depth-=1
   end+=1
  return text[start:end]
 assert body(old_bspline)==body(new_bspline),name
(A/f'{prefix}-repositories-before.json').write_text(json.dumps(repositories,indent=2)+'\n')
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),selected_geometry_tests=hc,checks=6,repos=repos,scope='Use retained polynomial and NURBS curves as the single public authoring interfaces; keep the control-net definitions private, remove forwarding constructors and duplicate layout validation, and migrate all controlled callers directly; full implementation goal remains active',performance_changes=regressions,actual_build_artifacts=actual_build_artifacts,unresolved='Independent monotone-reparameterization overlap probes still time out in resultant construction; the full architecture/API implementation remains active.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
(A/f'{prefix}-hypercurve-commit.txt').write_text(f"""Consolidate public spline authoring on the retained curves

Make the polynomial and rational B-spline control-net definitions crate-private and remove their three default-periodicity forwarding constructors. PolynomialSplineCurve2 and NurbsCurve2 remain the public authoring and cache owners. Remove duplicate polynomial layout validation; the private definition already performs the same checked validation before all other work.

Migrate all 24 extraction integration cases, the benchmark and the fuzz target directly. Preserve pole, signed/zero homogeneous weight, knot, arbitrary-degree, native-promotion and span-fact coverage. Cold benchmarks now construct fresh public curves and explicitly name that workload; cached queries remain separate. The fuzzer reuses its retained curve for knot edits instead of reconstructing it. No compatibility API remains.

Validation: {hc} release tests across five pinned binaries, including complete B-spline, polynomial spline, NURBS and public Curve integration inventories, plus six lint/format/fuzz/docs/downstream checks. Knot extraction, layout validation and standalone span-fact helper bodies are byte-identical.
""")
print('Reviewed',hc,'geometry tests; 6 checks;',len(manifest),'sources;',len(repositories),'repositories; performance changes',regressions)
