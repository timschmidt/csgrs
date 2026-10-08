from pathlib import Path
import hashlib,json,re,subprocess

A=Path(__file__).resolve().parent;W=A.parent;prefix='parameter-root-closure-20260928-v633'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
reaped=json.loads((A/f'{prefix}-reaped.json').read_text());assert reaped['outer_exit_code']==0
r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['normal_production_build'] and r['qualification_complete'] and r['probe_complete'] and r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text());assert len(manifest)==2047
for name,sha in manifest.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
assert len(r['builds'])==2 and len(r['checks'])==9 and len(r['test_listings'])==13
assert sum(len(b['binaries'])for b in r['builds'])==13
for build in r['builds']:
 assert build['returncode']==0
 for artifact in build['binaries'].values():assert digest(Path(artifact['path']))==artifact['sha256']
expected={(target,name)for target,names in r['expected_cases'].items()for name in names}
assert len(r['cases'])==len(expected)
assert {(c['target'],c['name'])for c in r['cases']}==expected
assert len({c['log']for c in r['cases']})==len(r['cases'])
for c in r['cases']:assert c['passed'] and c['returncode']==0 and re.search(r'test result: ok\. 1 passed;',(A/c['log']).read_text())
assert all(c['returncode']==0 for c in r['checks'])
hc=sum(c['target']!='hypersolve'for c in r['cases']);hs=len(r['cases'])-hc;assert hc==623 and hs>2
for target,listing in r['test_listings'].items():
 assert listing['returncode']==0
 available={line.removesuffix(': test')for line in(A/listing['log']).read_text().splitlines()if line.endswith(': test')}
 assert set(r['expected_cases'][target])<=available
prior=json.loads((A/'common-point-incidence-broad-20260928-v593-terminal.json').read_text());assert prior['qualification_complete'] and prior['all_processes_reaped']
old_manifest=json.loads((A/prior['source_manifest']).read_text())
changed=set(json.loads((A/'parameter-construction-candidate-v595.json').read_text()))|{'hypersolve/src/root_isolation.rs'}
assert {name for name,sha in manifest.items()if sha!=old_manifest[name]}==changed
paths={}
for name in sorted(changed):
 repo,path=name.split('/',1);paths.setdefault(repo,[]).append(path)
old_times={c['name']:c['elapsed_seconds']for c in prior['cases']}
regressions=[]
for c in r['cases']:
 old=old_times.get(c['name'])
 if old is not None and old>.1 and c['elapsed_seconds']>old*1.25+.2:regressions.append(dict(name=c['name'],before=old,after=c['elapsed_seconds']))
previous=json.loads((A/'common-point-incidence-broad-20260928-v593-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),selected_geometry_tests=hc,selected_algebraic_tests=hs,checks=9,repos=repos,scope='Explicit parameter domains, conservative rational-image selection, retained structural correspondences, and primitive rational root evidence; full implementation goal remains active',performance_changes=regressions)
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
(A/f'{prefix}-hypercurve-commit.txt').write_text(f'''Make parameter domains explicit and preserve retained curve closure

Construct BezierParameter2 through its Exact and Algebraic variants. Remove the wrappers that hid a unit-domain restriction or merely forwarded to the enum, migrate every production/test/benchmark/fuzz caller directly, and retain explicit admission at operation boundaries.

Select rational images only after excluding competing roots by certified disjointness within a complete domain. Reuse learned scalar views, refine competing deflated isolators, and replay exact unit-endpoint images. This repairs both endless refinement after learning a scalar and premature selection of a warmed conjugate.

Reuse certified structural source correspondences on retained closed unit subranges while projecting every off-diagonal residual onto the actual domains. Preserve exterior and opposite-normal guards, and share retained-contact merging. Together with primitive Hypersolve root evidence, this restores construction and offset composition at radical cusps.

Validation: {hc} release geometry/API regressions and {hs} algebraic regressions across 13 pinned binaries, plus 9 static/downstream checks. Includes both policies, full reoffsets, finite/exterior ownership, importer forms, exact image endpoints, close conjugates, and retained off-diagonal contacts. All callers and four affected fuzz targets compile; no compatibility shims added.
''')
(A/f'{prefix}-hypersolve-commit.txt').write_text(f'''Remove redundant rational scale from square-free root evidence

Clear rational denominators and content before square-free reduction, and retain the primitive integer GCD through exact quotient construction. This keeps unnecessary rational factors out of retained root polynomials and subsequent scalar fields. An expanded cusp polynomial could certify zero in primitive integer form but lost that proof after a common rational scaling; normalization restores geometric root admission and replay without a cusp-specific rule.

Clarify that AlgebraicRootRepresentation stores a finite isolating interval, with domain restrictions owned by geometric operations. Arbitrary exact coefficient fields keep their existing guarded reduction.

Validation: {hs} algebraic and {hc} geometry release regressions, plus 9 static/downstream checks. New tests cover signed 1024-bit content, exact radical boundary identity, and primitive quotients of repeated nonmonic factors. The public radical-cusp region constructs and offsets under both policies.
''')
print('Reviewed',hc,'geometry tests;',hs,'algebraic tests; 9 checks;',len(manifest),'sources;',len(repositories),'repositories; performance changes',regressions)
