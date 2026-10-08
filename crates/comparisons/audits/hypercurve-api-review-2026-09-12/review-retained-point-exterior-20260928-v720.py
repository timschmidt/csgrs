from pathlib import Path
import hashlib,json,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='retained-point-exterior-20260928-v720'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
reaped=json.loads((A/f'{prefix}-reaped.json').read_text());assert reaped['outer_exit_code']==0
r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['normal_production_build']and r['qualification_complete']and r['probe_complete']and r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text());assert len(manifest)==2048
for name,sha in manifest.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
assert len(r['builds'])==1 and len(r['test_listings'])==1 and len(r['checks'])==6 and all(c['returncode']==0 for c in r['checks'])
build=r['builds'][0];assert build['returncode']==0
artifacts=[]
for line in (A/build['log']).read_text().splitlines():
 try:m=json.loads(line)
 except ValueError:continue
 if m.get('reason')=='compiler-artifact'and m.get('target',{}).get('name')=='hypercurve'and m.get('executable'):artifacts.append(m)
assert len(artifacts)==1;binary=build['binaries']['hypercurve'];assert digest(Path(artifacts[0]['executable']))==binary['sha256']==digest(Path(binary['path']))
expected=json.loads((A/'retained-point-exterior-cases-v720.json').read_text());assert len(r['cases'])==len(expected)==15
assert set(expected)=={c['name']for c in r['cases']}and len({c['log']for c in r['cases']})==15
for c in r['cases']:
 assert c['passed']and c['returncode']==0 and 'test result: ok. 1 passed;'in(A/c['log']).read_text()
 if 'retained_point_classification_tests::'in c['name']:assert 'checks=90 failures=0'in(A/c['log']).read_text()
listing=r['test_listings']['hypercurve'];assert listing['returncode']==0
available={s.removesuffix(': test')for s in(A/listing['log']).read_text().splitlines()if s.endswith(': test')};assert set(expected)<=available
promotion=json.loads((A/'retained-point-exterior-promotion-v720.json').read_text());changed=set(promotion['promoted']);assert len(changed)==2
old_manifest=json.loads((A/'composition-evidence-20260928-v711-sources.json').read_text());assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:
 assert digest(A/promotion['candidate']/name)==manifest[name]==promotion['promoted'][name]
 probe=(A/'retained-point-exterior-candidate-v716'/name).read_text().replace('mod retained_point_classification_probe_v713 {','mod retained_point_classification_tests {')
 assert probe==(W/name).read_text(),name
source=(W/'hypercurve/src/bezier_region.rs').read_text();prior=json.loads((A/'composition-evidence-20260928-v711-terminal.json').read_text());old_source=(Path(prior['source_directory'])/'hypercurve/src/bezier_region.rs').read_text()
assert source.split('\n#[cfg(test)]\nmod retained_point_classification_tests {')[0].rstrip()==old_source.rstrip()
old_boolean=(Path(prior['source_directory'])/'hypercurve/src/curve_region_boolean.rs').read_text();new_boolean=(W/'hypercurve/src/curve_region_boolean.rs').read_text()
def section(text,start,end):return text[text.index(start):text.index(end,text.index(start))]
assert section(old_boolean,'fn retained_probe_outer_bounds(','impl CurveRegionCarrier2 {')==section(new_boolean,'fn retained_probe_outer_bounds(','impl CurveRegionCarrier2 {')
assert 'Err(CurveError::ZeroLengthLine) => {\n                    return Ok(Classification::Decided(RegionPointLocation::Outside));'in new_boolean
previous=json.loads((A/'composition-evidence-20260928-v711-repositories-after.json').read_text());repositories={};repos={}
for name,old in previous.items():
 head=git(name,'rev-parse','HEAD').decode().strip();status=git(name,'status','--porcelain=v1').decode();assert head==old['head'],name
 if name=='hypercurve':
  paths=sorted(p.removeprefix(name+'/')for p in changed);assert status==''.join(' M '+p+'\n'for p in paths),status
  assert not git(name,'diff','--cached','--name-only').strip();subprocess.run(['git','diff','--check'],cwd=W/name,check=True)
  repos[name]=dict(parent=head,paths={p:manifest[name+'/'+p]for p in paths})
 else:assert not status,name
 repositories[name]=dict(head=head,status=status)
(A/f'{prefix}-repositories-before.json').write_text(json.dumps(repositories,indent=2)+'\n')
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=15,new_oracle_checks=360,checks=6,repos=repos,scope='Use exact equality with a certified exterior candidate as an Outside witness; preserve all other probe behavior',unresolved='Public point-classification API consolidation remains unimplemented; full implementation goal remains active.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
(A/f'{prefix}-commit.txt').write_text('''Classify retained points equal to an exterior probe origin

A retained query point can equal the first exterior candidate of a certified boundary enclosure. The probe chord then reports ZeroLengthLine, which previously escaped as a classification error. Reuse that exact equality as an Outside witness. Other errors, uncertain-direction retries, endpoint ownership and winding decisions are unchanged; no additional equality evaluation or coordinate materialization is needed.

Independent rectangle oracles cover nonmaterialized chord-normal, analytic-parallel, transformed recursive and lazy endpoint points at interior, exterior, boundary and vertex positions. The original probe reproduced four failures; all 360 oracle checks now pass under both policies.

Validation: 15 release regressions, including existing winding, nesting, endpoint ownership and predicate guards; six all-target feature/lint, formatting, region-Boolean fuzz, documentation and downstream checks pass. Public classification API migration remains a separate follow-up.
''')
print('Reviewed 15 release tests,360 independent point checks,six checks,2048 sources and30 repositories')
