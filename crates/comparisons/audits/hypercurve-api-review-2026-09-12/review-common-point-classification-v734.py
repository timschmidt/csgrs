from pathlib import Path
import hashlib,json,subprocess,re
A=Path(__file__).resolve().parent;W=A.parent;prefix='common-point-classification-v734'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
reaped=json.loads((A/f'{prefix}-reaped.json').read_text());assert reaped['outer_exit_code']==0
r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['normal_production_build']and r['qualification_complete']and r['probe_complete']and r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text());assert len(manifest)==2048
for name,sha in manifest.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
assert len(r['builds'])==3 and len(r['checks'])==8 and all(c['returncode']==0 for c in r['checks'])
binaries={}
for build in r['builds']:
 assert build['returncode']==0
 for target,binary in build['binaries'].items():
  assert digest(Path(binary['path']))==binary['sha256'];binaries[target]=binary
  listing=r['test_listings'][target];assert listing['returncode']==0
  available={s.removesuffix(': test')for s in(A/listing['log']).read_text().splitlines()if s.endswith(': test')}
  assert set(r['expected_cases'][target])<=available
assert len(binaries)==len(r['expected_cases'])==20
expected={(target,name)for target,names in r['expected_cases'].items()for name in names}
assert len(r['cases'])==len(expected) and {(c['target'],c['name'])for c in r['cases']}==expected
assert len({c['log']for c in r['cases']})==len(expected)
for c in r['cases']:assert c['passed']and c['returncode']==0 and 'test result: ok. 1 passed;'in(A/c['log']).read_text()
promotion=json.loads((A/'common-point-classification-promotion-v733.json').read_text());changed=set(promotion['promoted']);assert len(changed)==40
old_manifest=json.loads((A/'retained-point-exterior-20260928-v720-sources.json').read_text());assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:assert digest(A/promotion['candidate']/name)==manifest[name]==promotion['promoted'][name]
# Every existing test remains; the change adds three public classification cases and one downstream nesting case.
old_root=A/'source-archives/retained-point-exterior-20260928-v720';added=[]
pattern=re.compile(r'#\[test\]\s*(?:#\[[^\n]*\]\s*)*fn\s+(\w+)')
for name in changed:
 if not name.endswith('.rs'):continue
 old=set(pattern.findall((old_root/name).read_text()));new=set(pattern.findall((W/name).read_text()));assert old<=new,(name,old-new);added.extend((name,t)for t in new-old)
assert len(added)==4,added
region=(W/'hypercurve/src/bezier_region.rs').read_text();native=(W/'hypercurve/src/region.rs').read_text()
assert 'pub fn classify_algebraic_point('not in region and 'pub fn classify_points('not in native
assert region.count('point: &CurvePoint2,')>0 and 'points: &[CurvePoint2],'in region
# The scalar and one-field kernels remain byte-identical to their old definitions.
def section(text,start,end):return text[text.index(start):text.index(end,text.index(start))]
old_region=(old_root/'hypercurve/src/bezier_region.rs').read_text()
for start,end in [('    pub(crate) fn classify_algebraic_point_raw(','    pub(crate) fn classify_algebraic_point_off_boundary_raw('),('    pub(crate) fn classify_algebraic_point_off_boundary_raw(','    fn classify_algebraic_point_with_boundary_contract(')]:
 assert section(old_region,start,end)==section(region,start,end),start
# Review every repository parent and exactly the intended unstaged paths.
previous=json.loads((A/'retained-point-exterior-20260928-v720-repositories-after.json').read_text());repositories={};repos={}
for name,old in previous.items():
 head=git(name,'rev-parse','HEAD').decode().strip();status=git(name,'status','--porcelain=v1').decode();assert head==old['head'],name
 paths=sorted(p.removeprefix(name+'/')for p in changed if p.startswith(name+'/'))
 if paths:
  assert status==''.join(' M '+p+'\n'for p in paths),(name,status)
  assert not git(name,'diff','--cached','--name-only').strip();subprocess.run(['git','diff','--check'],cwd=W/name,check=True)
  repos[name]=dict(parent=head,paths={p:manifest[name+'/'+p]for p in paths})
 else:assert not status,name
 repositories[name]=dict(head=head,status=status)
(A/f'{prefix}-repositories-before.json').write_text(json.dumps(repositories,indent=2)+'\n')
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=len(expected),binaries=len(binaries),checks=8,added_tests=added,repos=repos,scope='One public retained-point classification API; preserve scalar and selected-field predicates, native batch indexes, generated boundary preparation, closed-path trace semantics, and downstream exact endpoint nesting.',unresolved='The full Hypercurve implementation goal remains active. V735 separately removes one remaining Cartesian projection guard in Hyperbrep retained clipping.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
for repo,title,body in [
 ('hypercurve','Unify region and path classification on retained exact points','CurveRegion2 and closed CurvePath2 queries accept CurvePoint2, including generated endpoints and contacts. Remove the separate public algebraic-point query and two obsolete native batch forwarders. Keep private scalar/selected-field kernels and prepare native indexes and generated boundary carriers once per batch. Borrow the existing operand carrier partition for probes. Preserve retraced path boundaries and uncertainty when strict boundary validation cannot certify a join. Update every controlled test, example, benchmark, fuzz caller and the README directly.'),
 ('hyperbrep','Classify retained planar endpoints without coordinate projection','Migrate region and path queries to CurvePoint2. Wire nesting, profile nesting and closed face splitting pass retained endpoints directly, removing nine stored-coordinate requirements. Author the prism query point once before testing its outer boundary and holes. Add an exact parabolic-offset nesting regression with a nonmaterialized endpoint.'),
 ('csgrs','Pass common exact points to planar region classification','Update the planar XY containment adapter to the unified CurvePoint2 query. Boundary and uncertainty handling remain unchanged.'),
]:
 (A/f'{prefix}-{repo}-commit.txt').write_text(title+'\n\n'+body+'\n\nValidation: '+str(len(expected))+' release regressions across '+str(len(binaries))+' executables, including all affected Hypercurve integration suites, stationary fillets, downstream nesting and native planar adapters. Eight feature/lint, formatting, documentation, doctest, fuzz and downstream checks pass.\n')
print('Reviewed',len(expected),'release cases,',len(binaries),'binaries,8 checks,40 changed paths,2048 sources and30 repository parents')
