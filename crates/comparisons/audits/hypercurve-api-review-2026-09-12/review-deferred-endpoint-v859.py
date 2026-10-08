from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='deferred-endpoint-v859'
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
expected=json.loads((A/'deferred-endpoint-cases-v859.json').read_text());expected_set=set()
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
promotion=json.loads((A/'deferred-endpoint-promotion-v859.json').read_text());changed=set(promotion['promoted']);assert len(changed)==5
old_manifest=json.loads((A/'shared-tangent-cross-v858-sources.json').read_text());assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:
 assert digest(Path(promotion['candidates'][name]))==manifest[name]==promotion['promoted'][name]
 assert promotion['base'][name]==old_manifest[name]
old_root=A/'source-archives/shared-tangent-cross-v858'
name='hypercurve/src/bezier_split_endpoint.rs';before=(old_root/name).read_text();after=(W/name).read_text()
def block(text,start,end):
 a=text.index(start);b=text.index(end,a);return text[a:b]
# Every removed wrapper held the same source/parameter/policy and empty caches.
wrappers=[]
for method,family in [('rational_quadratic_first_order','RationalQuadratic'),('rational_first_order','Rational'),('quadratic_first_order','Quadratic'),('cubic_first_order','Cubic')]:
 marker='    pub(crate) fn '+method+'('
 a=before.index(marker);b=before.index('\n    }\n',a)+len('\n    }\n')
 body=before[a:b];body=body[body.index('        Ok(Self {'):]
 body=body.replace('BezierSubcurve2::'+family+'(curve.clone())','source_curve.clone()')
 wrappers.append(body);assert method not in after
assert len(set(wrappers))==1
constructor=block(after,'    pub(crate) fn from_source_curve_first_order(','    /// Constructs endpoint evidence for a polynomial quadratic Bezier.')
expected=wrappers[0].replace('        Ok(Self {','        Self {').replace('\n        })\n    }','\n        }\n    }')
assert ''.join(constructor[constructor.index('        Self {'):].split())==''.join(expected.split())
assert ') -> Self {' in constructor and 'CurveResult' not in constructor
# All eager admission/evaluation/cache behavior is byte-identical.
for start,end in [('    /// Constructs endpoint evidence for a polynomial quadratic Bezier.','    pub(crate) fn rational_quadratic_first_order('),('    /// Constructs endpoint evidence for an arbitrary-degree rational Bezier.','    pub(crate) fn rational_first_order(')]:
 old=block(before,start,end)
 next_after='    /// Constructs endpoint evidence for an arbitrary-degree rational Bezier.'if 'quadratic Bezier.'in start else'    /// Returns the algebraic Bezier parameter at this endpoint.'
 new=block(after,start,next_after)
 assert old==new
assert block(before,'    /// Returns the algebraic Bezier parameter at this endpoint.','#[cfg(test)]')==block(after,'    /// Returns the algebraic Bezier parameter at this endpoint.','#[cfg(test)]')
assert len(expected_set)==330
for name in ['bezier_split_endpoint::tests::lazy_endpoint_images_preserve_affine_domain_blockers','bezier_arrangement::endpoint_adjacency_tests::lazy_polynomial_endpoint_derivatives_match_eager_images']:
 assert ('hypercurve',name)in expected_set
previous=json.loads((A/'shared-tangent-cross-v858-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=len(expected_set),checks=7,repos=repos,scope='Consolidate four identical private deferred endpoint constructors into one infallible source constructor and migrate five caller files. Preserve the source enum, selected parameter, terminal policy, empty success caches and all eager/lazy admission and evaluation kernels.',unresolved='The full exact-geometry implementation goal remains active; this migration does not establish universal composition closure or a measured speedup.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
print('Reviewed',len(expected_set),'release cases,twelve pinned binaries,seven checks,2048sources and30repositories')
