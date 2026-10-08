from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='contact-fallback-consolidation-v793'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
reaped=json.loads((A/f'{prefix}-reaped.json').read_text());assert reaped['outer_exit_code']==0
r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['normal_production_build']and r['qualification_complete']and r['probe_complete']and r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text());assert len(manifest)==2048
for name,sha in manifest.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
assert len(r['builds'])==1 and len(r['test_listings'])==8 and len(r['checks'])==7 and all(c['returncode']==0 for c in r['checks'])
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
expected=json.loads((A/'contact-fallback-consolidation-cases-v793.json').read_text());expected_set=set()
for target,listing in r['test_listings'].items():
 assert listing['returncode']==0
 available={s.removesuffix(': test')for s in(A/listing['log']).read_text().splitlines()if s.endswith(': test')}
 selected=set(expected[target])if expected[target]is not None else available
 assert selected<=available and selected==set(r['expected_cases'][target]);expected_set.update((target,name)for name in selected)
assert len(r['cases'])==len(expected_set)
assert expected_set=={(c['target'],c['name'])for c in r['cases']}and len({c['log']for c in r['cases']})==len(expected_set)
for c in r['cases']:
 assert c['passed']and c['returncode']==0 and 'test result: ok. 1 passed;'in(A/c['log']).read_text()
 assert c['command'][0]==binaries[c['target']]['path']and '--include-ignored'in c['command']
promotion=json.loads((A/'contact-fallback-consolidation-promotion-v793.json').read_text());changed=set(promotion['promoted']);assert len(changed)==1
old_manifest=json.loads((A/'contact-blockers-v792-sources.json').read_text());assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:
 assert digest(Path(promotion['candidates'][name]))==manifest[name]==promotion['promoted'][name]
 assert promotion['base'][name]==old_manifest[name]
def interval(source,start,end):
 a=source.index(start);b=source.index(end,a);return source[a:b]
name='hypercurve/src/bezier_offset.rs'
assert changed=={name}
before=(A/'source-archives/contact-blockers-v792'/name).read_text();after=(W/name).read_text()
assert before.count('from_parametric_source(')-after.count('from_parametric_source(')==5
assert before.count('exact_contact_point_evidence(')-after.count('exact_contact_point_evidence(')==5
assert after.count('rational_point_evidence_at_parameter(')-before.count('rational_point_evidence_at_parameter(')==5
assert interval(before,'fn rational_point_evidence_at_parameter(', '/// Evaluates a rational source at its native retained parameter carrier.')==interval(after,'fn rational_point_evidence_at_parameter(', '/// Evaluates a rational source at its native retained parameter carrier.')
# No new adapter, tests, candidate equations, root solving or point carrier.
assert before.count('fn ')==after.count('fn ')
assert before[before.index('#[cfg(test)]\nmod conversion_tests {'):]==after[after.index('#[cfg(test)]\nmod conversion_tests {'):]
assert len(expected_set)==166
assert all(('hypercurve','bezier_offset::conversion_tests::'+n)in expected_set for n in [
 'contact_fallback_never_replaces_a_certified_pole',
 'algebraic_cusp_semicircle_rational_contacts_distinguish_both_endpoints',
 'rational_circle_overlap_preserves_an_isolated_parameter_visit',
 'selected_fiber_rational_circle_overlaps_complete_region_booleans',
 'algebraic_cusp_semicircle_reoffsets_pair_mapped_lens',
])
previous=json.loads((A/'contact-blockers-v792-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=len(expected_set),checks=7,repos=repos,scope='Consolidate five identical rational contact point fallbacks through their existing authority, preserve exact scalar blockers instead of inventing Boundary, and remove repeated private parametric construction without changing candidate or branch equations',unresolved='The full exact-geometry implementation goal remains active; this cleanup does not claim full spatial BREP closure or a measured speedup.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
print('Reviewed',len(expected_set),'release cases,eight pinned binaries,seven checks,2048sources and30repositories')
