from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='projection-overlap-20260928-v695'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
reaped=json.loads((A/f'{prefix}-reaped.json').read_text());assert reaped['outer_exit_code']==0
r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['normal_production_build'] and r['qualification_complete'] and r['probe_complete'] and r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text());assert len(manifest)==2047
for name,sha in manifest.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
assert len(r['builds'])==1 and len(r['checks'])==6 and len(r['test_listings'])==5
build=r['builds'][0];assert build['returncode']==0 and build['log']==f'{prefix}-hypercurve-build.log'
artifacts={}
for line in (A/build['log']).read_text().splitlines():
 try:message=json.loads(line)
 except ValueError:continue
 target=message.get('target',{}).get('name')
 if message.get('reason')=='compiler-artifact' and target in r['expected_cases'] and message.get('executable'):artifacts[target]=message
assert set(artifacts)==set(r['expected_cases'])
for target,message in artifacts.items():
 artifact=build['binaries'][target]
 assert digest(Path(message['executable']))==artifact['sha256']==digest(Path(artifact['path'])),target
expected={(target,name) for target,names in r['expected_cases'].items() for name in names}
assert len(r['cases'])==len(expected)==174
assert {(c['target'],c['name'])for c in r['cases']}==expected
assert len({c['log']for c in r['cases']})==len(r['cases'])
for c in r['cases']:assert c['passed'] and c['returncode']==0 and re.search(r'test result: ok\. 1 passed;',(A/c['log']).read_text())
assert all(c['returncode']==0 for c in r['checks'])
for target,listing in r['test_listings'].items():
 assert listing['returncode']==0
 available={line.removesuffix(': test')for line in(A/listing['log']).read_text().splitlines()if line.endswith(': test')}
 assert set(r['expected_cases'][target])<=available
prior=json.loads((A/'resultant-interpolation-20260928-v687-terminal.json').read_text());old_manifest=json.loads((A/prior['source_manifest']).read_text())
assert {name for name,sha in manifest.items()if sha!=old_manifest[name]}=={'hypercurve/src/bezier_offset.rs'}
assert r['expected_cases']=={target:cases for target,cases in prior['expected_cases'].items()if target!='hypersolve'}
old_times={(c['target'],c['name']):c['elapsed_seconds']for c in prior['cases']};regressions=[]
for c in r['cases']:
 old=old_times.get((c['target'],c['name']))
 if old is not None and old>.1 and c['elapsed_seconds']>old*1.25+.2:regressions.append(dict(name=c['name'],before=old,after=c['elapsed_seconds']))
old_source=(Path(prior['source_directory'])/'hypercurve/src/bezier_offset.rs').read_text();source=(W/'hypercurve/src/bezier_offset.rs').read_text()
# Source owners and all component construction/transport/replay remain unchanged.
def between(text,start,end):return text[text.index(start):text.index(end,text.index(start))]
for start,end in [('pub(crate) struct BezierParameterComponentOverlap2 {','#[cfg(test)]\npub(crate) fn nonlinear_parameter_component_overlap_for_test'),('fn parameter_component_evidence_from_drafts(','fn parameter_component_fiber_root_rank(')]:assert between(old_source,start,end)==between(source,start,end),start
projection=between(source,'struct BezierParallelPairProjection2 {','struct ExtractedBivariateSystemComponents2 {')
assert 'component_overlap_evidence: Arc<[BezierParameterComponentOverlap2]>'in projection and 'component_overlaps:'not in projection
assert 'let mut overlaps = component_overlaps;'in source and 'projection.component_overlaps ='not in source
for c in r['cases']:
 if 'structural_overlap_trace_regression::'in c['name']:
  log=(A/c['log']).read_text();assert 'complete=true contacts=2 overlaps=1'in log
previous=json.loads((A/'resultant-interpolation-20260928-v687-repositories-after.json').read_text());repositories={}
for name,old in previous.items():
 head=git(name,'rev-parse','HEAD').decode().strip();status=git(name,'status','--porcelain=v1').decode();assert head==old['head'],name
 if name=='hypercurve':
  assert status==' M src/bezier_offset.rs\n',status
  assert not git(name,'diff','--cached','--name-only').strip()
  subprocess.run(['git','diff','--check'],cwd=W/name,check=True)
 else:assert not status,name
 repositories[name]=dict(head=head,status=status)
(A/f'{prefix}-repositories-before.json').write_text(json.dumps(repositories,indent=2)+'\n')
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),selected_geometry_tests=174,checks=6,repos={'hypercurve':dict(parent=repositories['hypercurve']['head'],paths={'src/bezier_offset.rs':manifest['hypercurve/src/bezier_offset.rs']})},scope='Remove the derived interval mirror from parallel-pair projections; publish identical interval records once without removing selected correspondence evidence',performance_changes=regressions,unresolved='Full implementation goal and broader computational closure remain active; overlap inventory demonstrated redundancy, not a prior geometric re-entry failure.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
(A/f'{prefix}-commit.txt').write_text("""Derive parallel overlap intervals from retained certificates

Remove the mirrored interval array from pair projections and its eager construction. Replay derives one list of structurally distinct interval records from the retained component evidence and moves it directly into the published result. Every component support, selected fiber, chart, witness and ownership flag remains retained; no uncertain comparison or geometric equivalence is used to merge records.

The degree-six reparameterized S-cubic exposed duplicate raw interval entries that caused consumers to visit all matching correspondences repeatedly. Strengthen all four strict/approximate, same/reversed regressions to require one closed trace overlap and to reuse each retained map through public overlap clipping. Both isolated crossing contacts remain present.

Validation: 174 release tests across five pinned binaries; six format, lint, fuzz, documentation and downstream checks. Source/transport certificate construction verified unchanged. Full implementation goal remains active.
""")
print('Reviewed 174 geometry tests, six checks, 2047 sources and 30 repositories; timing flags',regressions)
