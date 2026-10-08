from pathlib import Path
import hashlib,json,re,subprocess

A=Path(__file__).resolve().parent;W=A.parent;prefix='unit-domain-core-20260928-v649'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
reaped=json.loads((A/f'{prefix}-reaped.json').read_text());assert reaped['outer_exit_code']==0
r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['normal_production_build'] and r['qualification_complete'] and r['probe_complete'] and r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text());assert len(manifest)==2047
for name,sha in manifest.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
assert len(r['builds'])==1 and len(r['checks'])==6 and len(r['test_listings'])==12
# Verify original Cargo artifact ownership and both executable copies.
actual_build_artifacts={};binaries={}
for crate,build in zip(['hypercurve'],r['builds']):
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
assert len(binaries)==12
for build in r['builds']:
 for target,artifact in build['binaries'].items():assert artifact==binaries[target],target
expected={(target,name)for target,names in r['expected_cases'].items()for name in names}
assert len(r['cases'])==len(expected)
assert {(c['target'],c['name'])for c in r['cases']}==expected
assert len({c['log']for c in r['cases']})==len(r['cases'])
for c in r['cases']:assert c['passed'] and c['returncode']==0 and re.search(r'test result: ok\. 1 passed;',(A/c['log']).read_text())
assert all(c['returncode']==0 for c in r['checks'])
hc=sum(c['target']!='hypersolve'for c in r['cases']);hs=len(r['cases'])-hc;assert hc==637 and hs==0
for target,listing in r['test_listings'].items():
 assert listing['returncode']==0
 available={line.removesuffix(': test')for line in(A/listing['log']).read_text().splitlines()if line.endswith(': test')}
 assert set(r['expected_cases'][target])<=available
prior=json.loads((A/'parameter-root-closure-20260928-v638-terminal.json').read_text());assert prior['qualification_complete'] and prior['all_processes_reaped']
old_manifest=json.loads((A/prior['source_manifest']).read_text())
changed={'hypercurve/src/bezier_offset.rs'}
assert {name for name,sha in manifest.items()if sha!=old_manifest[name]}==changed
paths={}
for name in sorted(changed):
 repo,path=name.split('/',1);paths.setdefault(repo,[]).append(path)
old_times={c['name']:c['elapsed_seconds']for c in prior['cases']}
regressions=[]
for c in r['cases']:
 old=old_times.get(c['name'])
 if old is not None and old>.1 and c['elapsed_seconds']>old*1.25+.2:regressions.append(dict(name=c['name'],before=old,after=c['elapsed_seconds']))
previous=json.loads((A/'parameter-root-closure-20260928-v638-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),selected_geometry_tests=hc,checks=6,repos=repos,scope='One domain-aware parallel component projection; full implementation goal remains active',performance_changes=regressions,actual_build_artifacts=actual_build_artifacts,unresolved='The independent monotone reparameterization probes still time out in resultant construction, with and without a regular source frame. Finite conic subdivision and constrained incident-cusp fillets also remain confirmed follow-up defects.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
(A/f'{prefix}-hypercurve-commit.txt').write_text(f"""Share domain projection for parallel overlap and self-contact queries

A certified source correspondence does not prove that off-correspondence contacts are absent. Remove both unit-domain fallbacks that claimed a complete overlap after residual projection failed. Unit, finite-domain and self-contact queries now share axis extraction, norm constraints, component selection and residual projection. Retained contacts survive replay, and self-contact queries preserve the increasing-parameter filter.

Remove the duplicate unit saturation helper and its forwarding result wrapper. Update the independent support/norm regression to exercise the common extractor directly.

Validation: {hc} release geometry/API cases across 12 pinned binaries, plus 6 lint, formatting, fuzz, documentation and downstream checks. Includes radical components, ordered self contacts, finite/exterior ownership, source cusps and both full reoffset policies.

The separate degree-six monotone-reparameterization probe still exceeds its diagnostic time limit; this change does not claim to resolve that resultant cost.
""")
print('Reviewed',hc,'geometry tests; 6 checks;',len(manifest),'sources;',len(repositories),'repositories; performance changes',regressions)
