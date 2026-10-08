from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='signed-depth-removal-v748'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
reaped=json.loads((A/f'{prefix}-reaped.json').read_text());assert reaped['outer_exit_code']==0
r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['normal_production_build']and r['qualification_complete']and r['probe_complete']and r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text());assert len(manifest)==2048
for name,sha in manifest.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
assert len(r['builds'])==1 and len(r['test_listings'])==2 and len(r['checks'])==7 and all(c['returncode']==0 for c in r['checks'])
binaries={}
for build in r['builds']:
 assert build['returncode']==0
 artifacts={}
 for line in (A/build['log']).read_text().splitlines():
  try:m=json.loads(line)
  except ValueError:continue
  if m.get('reason')=='compiler-artifact'and m.get('target',{}).get('name')in build['binaries']and m.get('executable'):artifacts[m['target']['name']]=m
 assert set(artifacts)==set(build['binaries'])
 for target,binary in build['binaries'].items():
  assert digest(Path(artifacts[target]['executable']))==binary['sha256']==digest(Path(binary['path']))
  assert target not in binaries;binaries[target]=binary
expected=json.loads((A/'signed-depth-removal-cases-v748.json').read_text());expected_set={(t,n)for t,ns in expected.items()for n in ns}
assert len(r['cases'])==len(expected_set)==45
assert expected_set=={(c['target'],c['name'])for c in r['cases']}and len({c['log']for c in r['cases']})==45
for c in r['cases']:
 assert c['passed']and c['returncode']==0 and 'test result: ok. 1 passed;'in(A/c['log']).read_text()
 assert c['command'][0]==binaries[c['target']]['path']and '--include-ignored'in c['command']
for target,listing in r['test_listings'].items():
 assert listing['returncode']==0
 available={s.removesuffix(': test')for s in(A/listing['log']).read_text().splitlines()if s.endswith(': test')};assert set(expected[target])<=available
promotion=json.loads((A/'signed-depth-removal-promotion-v748.json').read_text());changed=set(promotion['promoted']);assert len(changed)==7
old_manifest=json.loads((A/'point-query-cleanup-v746-sources.json').read_text());assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:
 assert digest(Path(promotion['candidates'][name]))==manifest[name]==promotion['promoted'][name]
 assert promotion['base'][name]==old_manifest[name]
source=(W/'hypercurve/src/bezier_region.rs').read_text()
assert 'fn signed_depth'not in source and 'fn curve_region_role_depth'not in source
prior=json.loads((A/'point-query-cleanup-v746-terminal.json').read_text());prior_archive=Path(prior['source_directory'])
old_region=(prior_archive/'hypercurve/src/region.rs').read_text();new_region=(W/'hypercurve/src/region.rs').read_text()
marker='/// Borrowed view over material and hole contours.'
assert old_region[old_region.index(marker):]==new_region[new_region.index(marker):]
assert manifest['hypercurve/src/prepared.rs']==old_manifest['hypercurve/src/prepared.rs']
renames={'explicit_signed_loops_classify_without_regularized_native_fast_path':'explicit_signed_loops_classify_after_regularization','sparse_region_classification_and_hole_depth_are_exact':'sparse_region_and_hole_classification_are_exact','material_island_inside_hole_adds_depth_back':'material_island_inside_hole_restores_membership','native_contour_constructors_and_signed_depth_need_no_region_wrapper':'native_contour_constructors_publish_regularized_membership'}
for name in changed:
 before=set(re.findall(r'#\[test\]\s*fn (\w+)',(prior_archive/name).read_text()));after=set(re.findall(r'#\[test\]\s*fn (\w+)',(W/name).read_text()))
 assert {renames.get(n,n)for n in before}==after,name
previous=json.loads((A/'point-query-cleanup-v746-repositories-after.json').read_text());repositories={};repos={}
for name,old in previous.items():
 head=git(name,'rev-parse','HEAD').decode().strip();status=git(name,'status','--porcelain=v1').decode();assert head==old['head'],name
 paths=sorted(p.removeprefix(name+'/')for p in changed if p.startswith(name+'/'))
 if paths:
  assert status==''.join(' M '+p+'\n'for p in paths),status
  assert not git(name,'diff','--cached','--name-only').strip();subprocess.run(['git','diff','--check'],cwd=W/name,check=True)
  repos[name]=dict(parent=head,paths={p:manifest[name+'/'+p]for p in paths})
 else:assert not status,name
 repositories[name]=dict(head=head,status=status)
(A/f'{prefix}-repositories-before.json').write_text(json.dumps(repositories,indent=2)+'\n')
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=45,checks=7,repos=repos,scope='Remove the public signed-depth query and duplicate kernel; use common exact point membership for normalized region callers',unresolved='The full exact-geometry implementation goal remains active; this cleanup does not claim full spatial BREP closure or a measured speedup.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
print('Reviewed45 release cases,two pinned binaries,seven checks,2048sources and30repositories')
