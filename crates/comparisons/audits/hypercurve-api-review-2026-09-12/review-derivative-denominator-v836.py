from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='derivative-denominator-v836'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
reaped=json.loads((A/f'{prefix}-reaped.json').read_text());assert reaped['outer_exit_code']==0
r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['normal_production_build']and r['qualification_complete']and r['probe_complete']and r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text());assert len(manifest)==2048
for name,sha in manifest.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
assert len(r['builds'])==1 and len(r['test_listings'])==12 and len(r['checks'])==7 and all(c['returncode']==0 for c in r['checks'])
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
expected=json.loads((A/'derivative-denominator-cases-v836.json').read_text());expected_set=set()
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
promotion=json.loads((A/'derivative-denominator-promotion-v836.json').read_text());changed=set(promotion['promoted']);assert len(changed)==2
old_manifest=json.loads((A/'tangent-extraction-v825-sources.json').read_text());assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:
 assert digest(Path(promotion['candidates'][name]))==manifest[name]==promotion['promoted'][name]
 assert promotion['base'][name]==old_manifest[name]
old_root=A/'source-archives/tangent-extraction-v825'
name='hypercurve/src/bezier_algebraic_image.rs';before=(old_root/name).read_text();after=(W/name).read_text()
start='pub(crate) fn rational_derivative_images_from_power_basis(';end='\nfn reduce_algebraic_image_polynomial('
a0=before.index(start);b0=before.index(end,a0);a1=after.index(start);b1=after.index(end,a1)
assert before[:a0]==after[:a1] and before[b0:]==after[b1:]
old=before[a0:b0];new=after[a1:b1]
start='    for order in 1..=max_order {'
old_num=old[old.index(start):old.index('        denominator_power = multiply_polynomials')]
new_num=new[new.index(start):new.index('        // Denominator powers are values')]
assert old_num==new_num
assert 'multiply_polynomials(&denominator_power, &denominator_image)' in new
assert 'denominator: denominator_power.clone()' in new
assert new.count('reduce_algebraic_image_polynomial(')==4
assert len(expected)==12 and len(expected_set)==324
assert ('hypercurve_bezier_algebraic_image','high_order_derivative_images_preserve_selected_source_domains') in expected_set
for replay,source,previous in [('derivative-growth-replay-v837','derivative-growth-probe-v829.rs','derivative-growth-probe-v829'),('derivative-domain-replay-v838','derivative-domain-probe-v830.rs','derivative-domain-probe-v830')]:
 p=json.loads((A/f'{replay}-terminal.json').read_text());reap=json.loads((A/f'{replay}-reaped.json').read_text());old_probe=json.loads((A/f'{previous}-terminal.json').read_text())
 assert reap['outer_exit_code']==0 and p['probe_complete']and p['all_processes_reaped']
 assert p['baseline']==r['source_manifest']and all(v['returncode']==0 for v in p['processes'])
 assert digest(A/source)==p['source_sha256']==old_probe['source_sha256']
 assert digest(Path(p['rlib']))==p['rlib_sha256']
 assert digest(A/f'{replay}-binary')==p['binary_sha256']
 assert digest(A/f'{previous}-binary')==old_probe['binary_sha256']
assert (A/'derivative-domain-replay-v838-run.log').read_text()==(A/'derivative-domain-probe-v830-run.log').read_text()
comparison=json.loads((A/'derivative-growth-comparison-v834.json').read_text());assert comparison['independent_exact_samples']==36 and comparison['domain_cases']==4
previous=json.loads((A/'tangent-extraction-v825-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=len(expected_set),checks=7,repos=repos,scope='Keep denominator powers reduced at the selected source root between derivative iterations. Preserve full numerator differentiation, source pole semantics, exact coefficient/root identities and existing image construction; validate independent high-order values and matched allocation-request measurements.',unresolved='The full exact-geometry implementation goal remains active; this migration does not establish universal composition closure or a measured speedup.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
print('Reviewed',len(expected_set),'release cases,twelve pinned binaries,seven checks,2048sources and30repositories')
