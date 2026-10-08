from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='conic-contact-domain-v796'
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
expected=json.loads((A/'conic-contact-domain-cases-v796.json').read_text());expected_set=set()
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
promotion=json.loads((A/'conic-contact-domain-promotion-v796.json').read_text());changed=set(promotion['promoted']);assert len(changed)==1
old_manifest=json.loads((A/'contact-fallback-consolidation-v793-sources.json').read_text());assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:
 assert digest(Path(promotion['candidates'][name]))==manifest[name]==promotion['promoted'][name]
 assert promotion['base'][name]==old_manifest[name]
def interval(source,start,end):
 a=source.index(start);b=source.index(end,a);return source[a:b]
name='hypercurve/src/rational_bezier_general.rs';assert changed=={name}
before=(A/'source-archives/contact-fallback-consolidation-v793'/name).read_text();after=(W/name).read_text()
start=after.index('        let common_weight_sign =',after.index('    fn implicit_conic_intersection_contacts('))
end=after.index('            // The quadratic frame is nonsingular',start)
gate=after[start:end]
assert 'other.control_weight_sign()' in gate
assert 'Classification::Decided(RealSign::Positive | RealSign::Negative)' in gate
assert '&other.homogeneous_power_basis()?.weight' in gate
assert 'Classification::Decided(RealSign::Zero) => continue' in gate
assert '&strict' in gate
old_tail=before[before.index('            let certified_transverse =',before.index('    fn implicit_conic_intersection_contacts(')):before.index('#[cfg(test)]\nmod tests {')]
new_tail=after[after.index('            let certified_transverse =',after.index('    fn implicit_conic_intersection_contacts(')):after.index('#[cfg(test)]\nmod tests {')]
assert old_tail==new_tail
assert len(expected_set)==169
assert ('hypercurve','rational_bezier_general::tests::implicit_conic_contacts_exclude_projective_poles')in expected_set
probe=json.loads((A/'conic-pole-contact-probe-v797-terminal.json').read_text())
probe_reap=json.loads((A/'conic-pole-contact-probe-v797-reaped.json').read_text())
assert probe_reap['outer_exit_code']==0 and probe['probe_complete']and probe['all_processes_reaped']
assert probe['baseline']==r['source_manifest']and all(p['returncode']==0 for p in probe['processes'])
assert digest(Path(probe['rlib']))==probe['rlib_sha256']
assert probe['source_sha256']==digest(A/'conic-pole-contact-probe-v795.rs')
probe_output=(A/'conic-pole-contact-probe-v797-run.log').read_text()
assert 'completed_cases=12 finite_controls=24 pole_contacts=0' in probe_output
assert 'result=incomplete' not in probe_output and 'blocked=' not in probe_output
previous=json.loads((A/'contact-fallback-consolidation-v793-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=len(expected_set),checks=7,repos=repos,scope='Exclude projective poles from the implicit conic affine contact shortcut using strict original-source denominator evidence, reuse common-sign unit-weight certificates, and retain finite contact images and transversality semantics',unresolved='The full exact-geometry implementation goal remains active; this cleanup does not claim full spatial BREP closure or a measured speedup.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
print('Reviewed',len(expected_set),'release cases,eight pinned binaries,seven checks,2048sources and30repositories')
