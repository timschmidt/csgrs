from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='image-ownership-v917'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
reaped=json.loads((A/f'{prefix}-reaped.json').read_text());assert reaped['outer_exit_code']==0
r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['normal_production_build']and r['qualification_complete']and r['probe_complete']and r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text());assert len(manifest)==2048
for name,sha in manifest.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
assert len(r['builds'])==2 and len(r['test_listings'])==13 and len(r['checks'])==9 and all(c['returncode']==0 for c in r['checks'])
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
expected=json.loads((A/'image-ownership-cases-v917.json').read_text());expected_set=set()
for target,listing in r['test_listings'].items():
 assert listing['returncode']==0
 available={line.removesuffix(': test')for line in(A/listing['log']).read_text().splitlines()if line.endswith(': test')}
 selected=set(expected[target])if expected[target]is not None else {n for n in available if target!='hypersolve' or n.startswith(('algebraic::','algebraic_fiber::','algebraic_tensor_image::','algebraic_polynomial_image::','algebraic_rational_image::','root_isolation::'))}
 assert selected<=available and selected==set(r['expected_cases'][target]);expected_set.update((target,name)for name in selected)
assert len(r['cases'])==len(expected_set)==599
assert expected_set=={(c['target'],c['name'])for c in r['cases']}and len({c['log']for c in r['cases']})==len(expected_set)
for c in r['cases']:
 assert c['passed']and c['returncode']==0 and 'test result: ok. 1 passed;'in(A/c['log']).read_text()
 assert c['command'][0]==binaries[c['target']]['path']and '--include-ignored'in c['command']
promotion=json.loads((A/'image-ownership-promotion-v917.json').read_text());changed=set(promotion['promoted']);assert len(changed)==3
old_manifest=json.loads((A/'affine-field-v910-sources.json').read_text());assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:
 assert digest(Path(promotion['candidates'][name]))==manifest[name]==promotion['promoted'][name]
 assert promotion['base'][name]==old_manifest[name]
old_root=A/'source-archives/affine-field-v910'
def function(source,name):
 match=re.search(r'(?m)^([ \t]*)(?:pub(?:\(crate\))? )?fn '+re.escape(name)+r'[(<]',source);assert match,name
 end=re.search(r'(?m)^'+match[1]+r'}',source[match.end():]);assert end,name
 return source[match.start():match.end()+end.end()]
root=(W/'hypersolve/src/root_isolation.rs').read_text();old_root_source=(old_root/'hypersolve/src/root_isolation.rs').read_text()
for fn in ['refine_isolated_univariate_polynomial_interval','polynomial_div_rem','square_free_part','polynomial_has_no_distinct_root_in_closed_interval','polynomial_interval_bernstein_variations','quadratic_multiple_variation_has_one_distinct_root']:
 assert function(root,fn)==function(old_root_source,fn),fn
assert 'enum UpperEndpointOwnership'not in root
admission=function(root,'certify_algebraic_image_interval')
assert 'let policy = PredicatePolicy::STRICT;'in admission and 'policy: PredicatePolicy'not in admission
assert 'Option<AlgebraicImageEnclosure>'in admission and 'image.lower_included'in admission and 'image.upper_included'in admission
assert 'exact_root: Some(root)'in admission
count=function(root,'polynomial_has_one_distinct_root_with_ownership')
assert 'lower_included && root_at_lower'in count and 'upper_included && root_at_upper'in count
for file,functions in [
 ('algebraic_polynomial_image.rs',['resultant_polynomial_for_image','exact_constant_source_relation','exact_constant_image','evaluate_rational_interval_polynomial']),
 ('algebraic_rational_image.rs',['resultant_polynomial_for_rational_image','strengthen_rational_image_domain_evidence','strengthen_rational_image_denominator','direct_rational_map','reduce_rational_map_modulo_source']),
]:
 current=(W/'hypersolve/src'/file).read_text();old=(old_root/'hypersolve/src'/file).read_text()
 for fn in functions:assert function(current,fn)==function(old,fn),(file,fn)
 for fn in (['polynomial_image_interval','polynomial_image_enclosure']if file=='algebraic_polynomial_image.rs'else['rational_image_interval','rational_image_enclosure']):
  body=function(current,fn);assert 'Option<AlgebraicImageEnclosure>'in body and 'distinct_root_count:'not in body and 'exact_root:'not in body,fn
for probe,wrong in [('image-ownership-probe-v916',4),('image-ownership-replay-v918',0)]:
 assert json.loads((A/f'{probe}-reaped.json').read_text())['outer_exit_code']==0
 data=json.loads((A/f'{probe}-terminal.json').read_text());assert data['probe_complete']and data['all_processes_reaped']and all(p['returncode']==0 for p in data['processes'])
 assert f'complete cases=4 wrong={wrong} blocked=0'in data['output']
 assert data['output'].count('premise selected_source_is_one=true cached_witness=false exact_image_is_zero=true')==2
 assert data['source_sha256']==digest(A/'image-ownership-probe-v916.rs')
 if wrong==0:assert data['baseline']==r['source_manifest']
previous=json.loads((A/'affine-field-v910-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=len(expected_set),checks=9,repos=repos,scope='Separate conservative image enclosures from certified root isolators. Preserve included/excluded endpoints through monotone and general polynomial/rational maps; certify owned root counts strictly before emitting canonical points or half-open isolators. Migrate internal callers directly and remove the obsolete sorting and ownership enum machinery. The unchanged V916 probe repairs four wrong exact images. Resultant, division, domain, square-free and root-refinement kernels are unchanged.',unresolved='The full exact-geometry implementation goal remains active. V913 nonlinear parameter transport, stationary/arbitrary higher-jet ordering and universal operation composition closure remain open.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
print('Reviewed',len(expected_set),'release cases,thirteen pinned binaries,nine checks,2048sources and30repositories')
