from pathlib import Path
import ast,hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='compound-fill-v766'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
reaped=json.loads((A/f'{prefix}-reaped.json').read_text());assert reaped['outer_exit_code']==0
r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['normal_production_build']and r['qualification_complete']and r['probe_complete']and r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text());assert len(manifest)==2048
for name,sha in manifest.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
assert len(r['builds'])==3 and len(r['test_listings'])==12 and len(r['checks'])==8 and all(c['returncode']==0 for c in r['checks'])
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
expected=json.loads((A/'compound-fill-cases-v766.json').read_text());expected_set=set()
for target,listing in r['test_listings'].items():
 assert listing['returncode']==0
 available={s.removesuffix(': test')for s in(A/listing['log']).read_text().splitlines()if s.endswith(': test')}
 selected=set(expected[target])if expected[target]is not None else available
 assert selected<=available and selected==set(r['expected_cases'][target])
 expected_set.update((target,name)for name in selected)
assert expected_set=={(c['target'],c['name'])for c in r['cases']}and len({c['log']for c in r['cases']})==len(expected_set)==len(r['cases'])
for c in r['cases']:
 assert c['passed']and c['returncode']==0 and 'test result: ok. 1 passed;'in(A/c['log']).read_text()
 assert c['command'][0]==binaries[c['target']]['path']and '--include-ignored'in c['command']
promotion=json.loads((A/'compound-fill-promotion-v766.json').read_text());changed=set(promotion['promoted']);assert len(changed)==30
old_manifest=json.loads((A/'region-admission-v763-sources.json').read_text());assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:
 assert digest(Path(promotion['candidates'][name]))==manifest[name]==promotion['promoted'][name]
 assert promotion['base'][name]==old_manifest[name]
prior=json.loads((A/'region-admission-v763-terminal.json').read_text());prior_archive=Path(prior['source_directory'])
# Replay each purely mechanical migration and format it independently. The
# baseline path expressions and policy arguments must survive byte-for-byte.
script=(A/'prepare-compound-fill-v765.py').read_text();tree=ast.parse(script)
node=next(n for n in tree.body if isinstance(n,ast.FunctionDef)and n.name=='comma_after_argument');namespace={};exec(compile(ast.Module(body=[node],type_ignores=[]),'argument-parser','exec'),namespace)
comma_after_argument=namespace['comma_after_argument'];migrations=json.loads((A/'compound-fill-migrations-v765.json').read_text());assert sum(migrations.values())==104
rustfmt='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt'
mechanical=[]
for name,count in migrations.items():
 if name in {'hypercurve/src/bezier_region.rs','hypercurve/src/curve_region_boolean.rs'}:continue
 s=(prior_archive/name).read_text();matches=list(re.finditer(r'::try_from_boundary_paths\(',s));assert len(matches)==count
 positions=[comma_after_argument(s,m.end())+1 for m in matches]
 rule='crate::FillRule::EvenOdd'if name.startswith('hypercurve/src/')else'hypercurve::FillRule::EvenOdd'
 for p in reversed(positions):s=s[:p]+' '+rule+','+s[p:]
 args=[rustfmt,'--edition','2024','--config','skip_children=true','--emit','stdout']
 if name.startswith('csgrs/'):args+=['--config-path',str(W/'csgrs/.rustfmt.toml')]
 expected_text=subprocess.check_output(args,input=s.encode(),cwd=W/name.split('/')[0]).decode().rstrip()
 current=(W/name).read_text()
 if name=='hypercurve/tests/hypercurve_curve_region_promotion.rs':current=current[:current.index('#[test]\nfn compound_fill_retains_signed_multiplicity_and_reversal_identity()')]
 assert expected_text==current.rstrip(),name
 mechanical.append(name)
# Independently compare the nine calls in files with core algorithm edits.
def call_arguments(text,start):
 args=[];stack=[];begin=start;i=start
 while i<len(text):
  if text.startswith('//',i):i=text.index('\n',i);continue
  if text.startswith('/*',i):i=text.index('*/',i+2)+2;continue
  c=text[i]
  if c=='"':
   i+=1
   while text[i]!='"':i+=2 if text[i]=='\\' else 1
  elif c in '([{':stack.append(c)
  elif c in ')]}':
   if not stack:
    assert c==')';last=text[begin:i].strip()
    if last:args.append(last)
    return args
   assert '([{'.index(stack.pop())==')]}'.index(c)
  elif c==','and not stack:args.append(text[begin:i].strip());begin=i+1
  i+=1
 raise ValueError('unterminated call')
for name in ['hypercurve/src/bezier_region.rs','hypercurve/src/curve_region_boolean.rs']:
 before_calls=[];after_calls=[]
 for root,out in [(prior_archive,before_calls),(W,after_calls)]:
  content=(root/name).read_text()
  out.extend(call_arguments(content,m.end())for m in re.finditer(r'::try_from_boundary_paths\(',content))
 assert len(before_calls)==len(after_calls)==migrations[name]
 for before_call,after_call in zip(before_calls,after_calls):
  assert len(before_call)==2 and len(after_call)==3 and after_call[1]=='crate::FillRule::EvenOdd'
  assert [re.sub(r'\s+','',a)for a in before_call]==[re.sub(r'\s+','',a)for a in [after_call[0],after_call[2]]]
# Published representation and explicit material/hole authoring are unchanged.
before=(prior_archive/'hypercurve/src/bezier_region.rs').read_text();after=(W/'hypercurve/src/bezier_region.rs').read_text()
for start,end in [('struct CurveRegionData2 {','impl CurveRegionData2 {'),('    pub fn try_from_native_contours(','    /// Constructs a top-level exact curved region from closed boundary paths.')]:
 left=before[before.index(start):before.index(end,before.index(start))]
 end_after='    /// Constructs the exact regularized fill of closed boundary paths.'if 'top-level exact'in end else end
 right=after[after.index(start):after.index(end_after,after.index(start))]
 assert left==right,start
svg=(W/'hypercurve/src/svg.rs').read_text();block=svg[svg.index('fn region_from_paths('):svg.index('\nfn geometry_from_paths(')]
assert 'CurveRegion2::try_from_boundary_paths(paths, fill_rule, &CurveContext::STRICT)'in block and 'loop_roles_raw'not in block and 'vec!'not in block
boolean=(W/'hypercurve/src/curve_region_boolean.rs').read_text()
assert boolean.count('regularization_fill_rule: None,')==17
assert 'context.data.regularization_fill_rule = Some(fill_rule);'in boolean
assert 'region.region_location_from_loop_windings(windings)'in boolean
for old in ['classify_point_from_boundary_side_ray_with_windings','classify_algebraic_point_from_boundary_side_ray_with_windings']:
 assert old not in after and old not in boolean
added={'hypercurve/tests/hypercurve_svg.rs':{'compound_fill_uses_global_winding_before_nesting_and_overlap_selection','compound_fill_preserves_recursive_islands_and_cancels_opposed_traversals'},'hypercurve/tests/hypercurve_curve_region_promotion.rs':{'compound_fill_retains_signed_multiplicity_and_reversal_identity','compound_circle_fill_selects_exact_algebraic_overlap_and_canceled_seams','compound_fill_reuses_retained_rational_and_generated_boundaries'}}
for name in changed:
 before_tests=set(re.findall(r'#\[test\]\s*fn (\w+)',(prior_archive/name).read_text()));after_tests=set(re.findall(r'#\[test\]\s*fn (\w+)',(W/name).read_text()))
 assert not before_tests-after_tests,name
 assert after_tests-before_tests==added.get(name,set()),name
probe=json.loads((A/'compound-fill-probe-v764-terminal.json').read_text());probe_reaped=json.loads((A/'compound-fill-probe-v764-reaped.json').read_text())
assert probe_reaped['outer_exit_code']==0 and probe['all_processes_reaped']and probe['reproduced_compound_winding_defects']
assert probe['baseline']=='region-admission-v763-sources.json'
assert digest(A/'compound-fill-probe-v764-binary')==probe['binary_sha256']and digest(A/'compound-fill-probe-v764.rs')==probe['source_sha256']
assert all(p['returncode']==0 for p in probe['processes'])and 'wrong=1 rejected=4 controls=3'in(A/probe['processes'][-1]['log']).read_text()
previous=json.loads((A/'region-admission-v763-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=len(r['cases']),checks=8,mechanically_verified_files=mechanical,migrated_call_sites=sum(migrations.values()),repos=repos,scope='Apply global compound winding at canonical path admission, retain raw winding evidence for shared face selection and migrate all callers directly',unresolved='Full exact-geometry goal remains active. No measured speedup or full spatial BREP closure claim.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
print('Reviewed',len(r['cases']),'release cases,12pinned binaries,8checks,104call migrations,2048sources and30repositories')
