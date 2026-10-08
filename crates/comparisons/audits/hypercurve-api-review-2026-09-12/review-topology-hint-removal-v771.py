from pathlib import Path
import ast,hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='topology-hint-removal-v771'
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
 for line in(A/build['log']).read_text().splitlines():
  try:m=json.loads(line)
  except ValueError:continue
  if m.get('reason')=='compiler-artifact'and m.get('target',{}).get('name')in build['binaries']and m.get('executable'):artifacts[m['target']['name']]=m
 assert set(artifacts)==set(build['binaries'])
 for target,binary in build['binaries'].items():
  assert digest(Path(artifacts[target]['executable']))==binary['sha256']==digest(Path(binary['path']))
  assert target not in binaries;binaries[target]=binary
expected=json.loads((A/'topology-hint-removal-cases-v771.json').read_text());expected_set=set()
for target,listing in r['test_listings'].items():
 assert listing['returncode']==0
 available={s.removesuffix(': test')for s in(A/listing['log']).read_text().splitlines()if s.endswith(': test')}
 selected=set(expected[target])if expected[target]is not None else available
 assert selected<=available and selected==set(r['expected_cases'][target]);expected_set.update((target,name)for name in selected)
assert expected_set=={(c['target'],c['name'])for c in r['cases']}and len({c['log']for c in r['cases']})==len(expected_set)==len(r['cases'])
for c in r['cases']:
 assert c['passed']and c['returncode']==0 and 'test result: ok. 1 passed;'in(A/c['log']).read_text()
 assert c['command'][0]==binaries[c['target']]['path']and '--include-ignored'in c['command']
promotion=json.loads((A/'topology-hint-removal-promotion-v771.json').read_text());changed=set(promotion['promoted']);assert len(changed)==17
old_manifest=json.loads((A/'compound-fill-v766-sources.json').read_text());assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:
 assert digest(Path(promotion['candidates'][name]))==manifest[name]==promotion['promoted'][name]
 assert promotion['base'][name]==old_manifest[name]
prior=json.loads((A/'compound-fill-v766-terminal.json').read_text());prior_archive=Path(prior['source_directory'])
script=(A/'prepare-topology-hint-removal-v770.py').read_text();tree=ast.parse(script);namespace={'re':re}
nodes=[n for n in tree.body if isinstance(n,ast.FunctionDef)and n.name in {'arguments','change_calls'}];assert len(nodes)==2;exec(compile(ast.Module(body=nodes,type_ignores=[]),'migration-parser','exec'),namespace)
arguments=namespace['arguments'];change_calls=namespace['change_calls'];migrations=json.loads((A/'topology-hint-removal-migrations-v770.json').read_text());assert sum(migrations.values())==55
for name,count in migrations.items():
 before=(prior_archive/name).read_text();after=(W/name).read_text()
 before,found=change_calls(before,r'::try_from_boundary_paths_with_loop_topology\(',{3},expected_count=count,rename='::try_from_boundary_paths_with_loop_semantics')
 if name.endswith('/hypercurve_curve_region_promotion.rs'):after=after[:after.index('#[test]\nfn authored_region_sides_are_certified_before_offset_and_boolean_reentry()')]
 calls=[]
 for text in [before,after]:
  calls.append([[re.sub(r'\s+','',arg)for arg in arguments(text,m.end())[0]]for m in re.finditer(r'::try_from_boundary_paths_with_loop_semantics\(',text)])
 assert calls[0]==calls[1],name
before=(prior_archive/'hypercurve/src/bezier_region.rs').read_text();after=(W/'hypercurve/src/bezier_region.rs').read_text()
for start,end in [('    pub(crate) fn try_new_with_loop_topology(','    fn from_certified_boundary_loops('),('    pub(crate) fn with_certified_filled_side_is_left(','    pub(crate) fn with_regularized_filled_left_topology(')]:
 assert before[before.index(start):before.index(end,before.index(start))]==after[after.index(start):after.index(end,after.index(start))],start
for name in ['hypercurve/src/curve_region_boolean.rs','hypercurve/src/curve_region_trim.rs']:
 before_code=(prior_archive/name).read_text().split('#[cfg(test)]',1)[0];after_code=(W/name).read_text().split('#[cfg(test)]',1)[0];assert before_code==after_code,name
assert 'certified_filled_sides: Option<Vec<bool>>'not in after and 'pub(crate) enum CurveBoundaryInteriorSide2'in after
for name in manifest:
 if not name.endswith(('.rs','.md')):continue
 text=(W/name).read_text();assert 'try_from_boundary_paths_with_loop_topology'not in text,name
 if not name.startswith('hypercurve/src/'):assert 'CurveBoundaryInteriorSide2'not in text,name
added='authored_region_sides_are_certified_before_offset_and_boolean_reentry'
for name in changed:
 before_tests=set(re.findall(r'#\[test\]\s*fn (\w+)',(prior_archive/name).read_text()));after_tests=set(re.findall(r'#\[test\]\s*fn (\w+)',(W/name).read_text()))
 assert not before_tests-after_tests,name
 assert after_tests-before_tests==({added}if name.endswith('/hypercurve_curve_region_promotion.rs')else set()),name
probe=json.loads((A/'topology-hint-probe-v768-terminal.json').read_text());probe_reaped=json.loads((A/'topology-hint-probe-v768-reaped.json').read_text())
assert probe_reaped['outer_exit_code']==0 and probe['all_processes_reaped']and probe['probe_complete']
assert probe['baseline']=='compound-fill-v766-sources.json'
assert digest(A/'topology-hint-probe-v768-binary')==probe['binary_sha256']and digest(A/'topology-hint-probe-v768.rs')==probe['source_sha256']
assert all(p['returncode']==0 for p in probe['processes'])and 'correct_controls=4 inconsistent_accepted=4 rejected_hints=0'in(A/probe['processes'][-1]['log']).read_text()
previous=json.loads((A/'compound-fill-v766-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=len(r['cases']),checks=7,migrated_call_sites=55,repos=repos,scope='Remove unchecked authored-side constructor and redundant hint plumbing;infer orientation through exact winding while retaining private operation certificates',unresolved='Full implementation goal remains active;no measured speedup or full spatial BREP closure claim.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n');print('Reviewed',len(r['cases']),'release cases,8pinned binaries,7checks,55call migrations,2048sources and30repositories')
