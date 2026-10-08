from pathlib import Path
import hashlib,json,re,subprocess

A=Path(__file__).resolve().parent;W=A.parent;prefix='known-fillet-centers-20260928-v670'
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
hc=sum(c['target']!='hypersolve'for c in r['cases']);hs=len(r['cases'])-hc;assert hc==93 and hs==0
for target,listing in r['test_listings'].items():
 assert listing['returncode']==0
 available={line.removesuffix(': test')for line in(A/listing['log']).read_text().splitlines()if line.endswith(': test')}
 assert set(r['expected_cases'][target])<=available
prior=json.loads((A/'incident-cusp-side-20260928-v657-terminal.json').read_text());assert prior['qualification_complete'] and prior['all_processes_reaped']
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
previous=json.loads((A/'incident-cusp-side-20260928-v657-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),selected_geometry_tests=hc,checks=6,repos=repos,scope='Apply exact fillet center/contact constraints before generic pair elimination; preserve source identity and full contact multiplicity; full implementation goal remains active',performance_changes=regressions,actual_build_artifacts=actual_build_artifacts,unresolved='Independent monotone-reparameterization overlap probes still time out in resultant construction; the full architecture/API implementation remains active.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
(A/f'{prefix}-hypercurve-commit.txt').write_text(f"""Apply exact fillet constraints before center elimination

An explicit center or contact previously entered full parallel-pair elimination before filtering. Generalize collapsed-center contact recovery to known centers and narrow regular-source pairs first. Keep every contact preimage and selected parameter authority, reject extensions beyond source barriers, reuse the regular-pair tangent proof, and preserve native coincidence certificates and stationary-source regular-cell ownership. Retained analytic points without optional Cartesian views now use the existing coefficient-field point-incidence authority. No compatibility wrapper remains.

Validation: {hc} release cases across two pinned binaries plus six lint/caller/format/fuzz/docs/downstream checks. The independent nonlinear cusp fixture previously timed out at 180 seconds; all twelve constraint/policy/reversal combinations now complete together in the recorded test time. A regular nodal cubic verifies two distinct tangent centers for one point constraint, domain admission, conjunctive center selection and exact junctions. A cold nonzero-offset algebraic point guard verifies native retained-field inversion and opposite-normal rejection. Existing continuous families, collapsed/free contacts, stationary sources, incidence replay and compositions remain covered.
""")
print('Reviewed',hc,'geometry tests; 6 checks;',len(manifest),'sources;',len(repositories),'repositories; performance changes',regressions)
