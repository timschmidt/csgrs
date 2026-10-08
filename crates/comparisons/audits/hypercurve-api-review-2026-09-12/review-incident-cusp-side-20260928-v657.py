from pathlib import Path
import hashlib,json,re,subprocess

A=Path(__file__).resolve().parent;W=A.parent;prefix='incident-cusp-side-20260928-v657'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
reaped=json.loads((A/f'{prefix}-reaped.json').read_text());assert reaped['outer_exit_code']==0
r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['normal_production_build'] and r['qualification_complete'] and r['probe_complete'] and r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text());assert len(manifest)==2047
for name,sha in manifest.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
assert len(r['builds'])==1 and len(r['checks'])==6 and len(r['test_listings'])==2
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
assert len(binaries)==2
for build in r['builds']:
 for target,artifact in build['binaries'].items():assert artifact==binaries[target],target
expected={(target,name)for target,names in r['expected_cases'].items()for name in names}
assert len(r['cases'])==len(expected)
assert {(c['target'],c['name'])for c in r['cases']}==expected
assert len({c['log']for c in r['cases']})==len(r['cases'])
for c in r['cases']:assert c['passed'] and c['returncode']==0 and re.search(r'test result: ok\. 1 passed;',(A/c['log']).read_text())
assert all(c['returncode']==0 for c in r['checks'])
hc=sum(c['target']!='hypersolve'for c in r['cases']);hs=len(r['cases'])-hc;assert hc==58 and hs==0
for target,listing in r['test_listings'].items():
 assert listing['returncode']==0
 available={line.removesuffix(': test')for line in(A/listing['log']).read_text().splitlines()if line.endswith(': test')}
 assert set(r['expected_cases'][target])<=available
prior=json.loads((A/'conic-subdivision-20260928-v653-terminal.json').read_text());assert prior['qualification_complete'] and prior['all_processes_reaped']
old_manifest=json.loads((A/prior['source_manifest']).read_text())
changed={'hypercurve/src/bezier_offset.rs','hypercurve/src/curve.rs','hypercurve/src/curve_fillet.rs'}
assert {name for name,sha in manifest.items()if sha!=old_manifest[name]}==changed
paths={}
for name in sorted(changed):
 repo,path=name.split('/',1);paths.setdefault(repo,[]).append(path)
old_times={c['name']:c['elapsed_seconds']for c in json.loads((A/'unit-domain-core-20260928-v649-terminal.json').read_text())['cases']}
regressions=[]
for c in r['cases']:
 old=old_times.get(c['name'])
 if old is not None and old>.1 and c['elapsed_seconds']>old*1.25+.2:regressions.append(dict(name=c['name'],before=old,after=c['elapsed_seconds']))
previous=json.loads((A/'conic-subdivision-20260928-v653-repositories-after.json').read_text());repositories={};repos={}
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
 assert b'BezierParameter2::exact('not in data and b'BezierParameter2::algebraic('not in data,name
(A/f'{prefix}-repositories-before.json').write_text(json.dumps(repositories,indent=2)+'\n')
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),selected_geometry_tests=hc,checks=6,repos=repos,scope='Exact surviving-side tangents for incident original-offset cusp fillets; full implementation goal remains active',performance_changes=regressions,actual_build_artifacts=actual_build_artifacts,unresolved='Independent monotone-reparameterization overlap probes still time out in resultant construction; the full architecture/API implementation remains active.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
(A/f'{prefix}-hypercurve-commit.txt').write_text(f"""Retain surviving-side tangents for incident offset-cusp fillets

A regular source can have an offset cusp beyond the authored corner. Its surviving side supplies an exact nonzero contact tangent even though the pointwise offset derivative vanishes. Reuse the existing Taylor-sign theorem for that original contact side in fillet selection, orientation and regular source frames. Keep the center-locus derivative pointwise and preserve undefined source-normal, pole and collapsed-curve exclusions.

Normal component evidence carries its side and source axis through operand swaps, negative affine maps and reversed incident charts. The ordinary nonzero derivative path is unchanged. All callers migrate directly; no compatibility interfaces are added.

Validation: {hc} release cases across two pinned binaries plus six lint/caller/format/fuzz/docs/downstream checks. Independent rational cusp fixtures exercise center, exact parameter and exact point constraints, both policies, reversed traversal and native/polynomial line partners. Additional guards cover chart reversal, source poles, undefined normals, collapsed derivatives, and existing stationary fillet compositions.
""")
print('Reviewed',hc,'geometry tests; 6 checks;',len(manifest),'sources;',len(repositories),'repositories; performance changes',regressions)
