from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='unordered-arrangement-api-v752'
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
expected=json.loads((A/'unordered-arrangement-api-cases-v752.json').read_text());expected_set={(t,n)for t,ns in expected.items()for n in ns}
assert len(r['cases'])==len(expected_set)==32
assert expected_set=={(c['target'],c['name'])for c in r['cases']}and len({c['log']for c in r['cases']})==32
for c in r['cases']:
 assert c['passed']and c['returncode']==0 and 'test result: ok. 1 passed;'in(A/c['log']).read_text()
 assert c['command'][0]==binaries[c['target']]['path']and '--include-ignored'in c['command']
for target,listing in r['test_listings'].items():
 assert listing['returncode']==0
 available={s.removesuffix(': test')for s in(A/listing['log']).read_text().splitlines()if s.endswith(': test')};assert set(expected[target])<=available
promotion=json.loads((A/'unordered-arrangement-api-promotion-v752.json').read_text());changed=set(promotion['promoted']);assert len(changed)==5
old_manifest=json.loads((A/'signed-depth-removal-v748-sources.json').read_text());assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:
 assert digest(Path(promotion['candidates'][name]))==manifest[name]==promotion['promoted'][name]
 assert promotion['base'][name]==old_manifest[name]
source=(W/'hypercurve/src/bezier_region.rs').read_text()
assert 'arrange_unordered_segments_borrowed'not in source
assert 'pub fn arrange_unordered_segments(\n        source_segments: &[Segment2],'in source
prior=json.loads((A/'signed-depth-removal-v748-terminal.json').read_text());prior_archive=Path(prior['source_directory'])
before=(prior_archive/'hypercurve/src/bezier_region.rs').read_text();start='    pub fn arrange_unordered_segments(';end='    /// Constructs a unified region directly from explicit native contour roles.'
assert before[:before.index(start)]==source[:source.index(start)]and before[before.index(end):]==source[source.index(end):]
expected=before[before.index('    pub fn arrange_unordered_segments_borrowed('):before.index(end)].replace('arrange_unordered_segments_borrowed','arrange_unordered_segments')
assert source[source.index(start):source.index(end)]==expected
for name in changed:
 before=set(re.findall(r'#\[test\]\s*fn (\w+)',(prior_archive/name).read_text()));after=set(re.findall(r'#\[test\]\s*fn (\w+)',(W/name).read_text()))
 assert not after-before
 assert before-after==({'borrowed_unordered_lines_and_segments_have_identical_semantics'}if name=='hypercurve/src/native_region_tests.rs'else set()),name
previous=json.loads((A/'signed-depth-removal-v748-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=32,checks=7,repos=repos,scope='Consolidate unordered native arrangement on one borrowed slice API without changing its mathematical kernel',unresolved='The full exact-geometry implementation goal remains active; this cleanup does not claim full spatial BREP closure or a measured speedup.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
print('Reviewed32 release cases,two pinned binaries,seven checks,2048sources and30repositories')
