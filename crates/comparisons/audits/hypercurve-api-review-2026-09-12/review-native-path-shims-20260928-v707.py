from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='native-path-shims-20260928-v707';prior_prefix='interpolation-content-20260928-v701'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
reaped=json.loads((A/f'{prefix}-reaped.json').read_text());assert reaped['outer_exit_code']==0
r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['normal_production_build'] and r['qualification_complete'] and r['probe_complete'] and r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text());assert len(manifest)==2047
for name,sha in manifest.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
assert len(r['builds'])==2 and len(r['checks'])==6 and len(r['test_listings'])==3
artifacts={}
for build in r['builds']:
 assert build['returncode']==0
 crate=Path(build['log']).name.removeprefix(prefix+'-').removesuffix('-build.log');assert crate in {'hypercurve','hyperbrep'}
 found={}
 for line in (A/build['log']).read_text().splitlines():
  try:message=json.loads(line)
  except ValueError:continue
  target=message.get('target',{}).get('name')
  if message.get('reason')=='compiler-artifact' and target in build['binaries'] and message.get('executable'):found[target]=message
 assert set(found)==set(build['binaries'])
 for target,message in found.items():
  artifact=build['binaries'][target];assert digest(Path(message['executable']))==artifact['sha256']==digest(Path(artifact['path'])),target
  assert target not in artifacts;artifacts[target]=message
assert set(artifacts)==set(r['expected_cases'])
expected={(target,name)for target,names in r['expected_cases'].items()for name in names}
assert len(r['cases'])==len(expected) and {(c['target'],c['name'])for c in r['cases']}==expected
assert len({c['log']for c in r['cases']})==len(r['cases'])
for c in r['cases']:assert c['passed'] and c['returncode']==0 and re.search(r'test result: ok\. 1 passed;',(A/c['log']).read_text())
assert all(c['returncode']==0 for c in r['checks'])
for target,listing in r['test_listings'].items():
 assert listing['returncode']==0
 available={line.removesuffix(': test')for line in(A/listing['log']).read_text().splitlines()if line.endswith(': test')}
 assert set(r['expected_cases'][target])<=available
 if target=='hypercurve_curve_string':assert set(r['expected_cases'][target])==available
assert len(r['expected_cases']['hypercurve'])==30 and all(n.startswith('curve_region_trim::tests::')for n in r['expected_cases']['hypercurve'])
assert len(r['expected_cases']['hyperbrep'])==12
assert set(r['expected_cases']['hyperbrep'])>={'boolean::tests::intersection_graph_clips_plane_extrusion_curves_in_both_parameter_spaces','boolean::tests::axial_cone_rays_partition_an_exact_half_frustum_cut'}
prior=json.loads((A/f'{prior_prefix}-terminal.json').read_text());old_manifest=json.loads((A/prior['source_manifest']).read_text())
changed=set(json.loads((A/'native-path-shims-base-v707.json').read_text()))
assert {name for name,sha in manifest.items()if sha!=old_manifest[name]}==changed
promotion=json.loads((A/'native-path-shims-promotion-v704.json').read_text());assert promotion['promoted']=={n:manifest[n]for n in promotion['promoted']}
for name in promotion['promoted']:assert digest(A/promotion['candidate']/name)==manifest[name]
for name,row in json.loads((A/'native-path-shims-lint-v707.json').read_text()).items():assert row['base']==old_manifest[name] and row['updated']==manifest[name]
old_source=(Path(prior['source_directory'])/'hypercurve/src/curve_string.rs').read_text();source=(W/'hypercurve/src/curve_string.rs').read_text()
def between(text,start,end):return text[text.index(start):text.index(end,text.index(start))]
start='        let mut iter = curve_strings.into_iter();';end='        Ok(Classification::Decided(accumulated))\n    }'
assert between(old_source,start,end)==between(source,start,end)
start='    /// Extends one endpoint segment to an exact target point.'
assert old_source[old_source.index(start):]==source[source.index(start):]
assert 'pub fn trim_inside_region('not in source and 'impl IntoIterator<Item = Self>'in source
for name in manifest:
 if name.endswith('.rs'):
  text=(W/name).read_text()
  assert 'extend_line_endpoint_to_point'not in text and 'link_ordered_connected_endpoints_borrowed'not in text,name
old_test=(Path(prior['source_directory'])/'hypercurve/tests/hypercurve_curve_string.rs').read_text()
old_names=set(re.findall(r'#\[test\]\s*fn (\w+)\(',old_test))
renames={'curve_string_trim_inside_region_'+suffix:'curve_path_trim_inside_region_'+suffix for suffix in ['splits_disconnected_inside_windows','respects_holes','retains_boundary_overlap']}
assert {renames.get(n,n)for n in old_names}==set(r['expected_cases']['hypercurve_curve_string'])
old_times={(c['target'],c['name']):c['elapsed_seconds']for c in prior['cases']};regressions=[]
for c in r['cases']:
 old=old_times.get((c['target'],c['name']))
 if old is not None and old>.1 and c['elapsed_seconds']>old*1.25+.2:regressions.append(dict(name=c['name'],before=old,after=c['elapsed_seconds']))
previous=json.loads((A/f'{prior_prefix}-repositories-after.json').read_text());repositories={};repos={}
for name,old in previous.items():
 head=git(name,'rev-parse','HEAD').decode().strip();status=git(name,'status','--porcelain=v1').decode();assert head==old['head'],name
 paths=sorted(n.removeprefix(name+'/')for n in changed if n.startswith(name+'/'))
 if paths:
  assert status==''.join(' M '+p+'\n'for p in paths),(name,status)
  assert not git(name,'diff','--cached','--name-only').strip()
  subprocess.run(['git','diff','--check'],cwd=W/name,check=True)
  repos[name]=dict(parent=head,paths={p:manifest[name+'/'+p]for p in paths})
 else:assert not status,name
 repositories[name]=dict(head=head,status=status)
(A/f'{prefix}-repositories-before.json').write_text(json.dumps(repositories,indent=2)+'\n')
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=len(expected),checks=6,repos=repos,scope='Remove three native open-path forwarding methods and author general region trim paths directly in controlled callers',performance_changes=regressions,unresolved='Full implementation goal remains active; this does not consolidate the distinct native path representation or claim a matched benchmark speedup.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
for repo,body in {
 'hypercurve':'''Remove native path forwarding methods and migrate callers

Keep one endpoint extension method and accept an owned iterator for ordered linking, removing the line-named and borrowed forwarding aliases. The linking and endpoint extension kernels remain unchanged.

Remove native-path region trimming, which reconstructed a general path on each query. Tests author CurvePath2 directly, the fuzzer uses its existing general path, and the benchmark retains that path outside its timer. Preserve all existing test cases and boundary/hole semantics without a compatibility adapter.
''',
 'hyperbrep':'''Use general exact paths for parameter-space region trimming

Construct CurvePath2 directly for clipped face parameter lines, preserving source locations and the existing general clipping kernel. Remove the sole production CurveString2 dependency and its now-removed forwarding call.

Clean the existing Clippy findings exposed by the full downstream check: express tail errors directly, copy typed triangle chunks without per-index reconstruction, and use an array for a fixed test fixture. Geometry and error semantics are unchanged.
''',
}.items():
 body+=f'\nValidation: {len(expected)} release tests across three pinned binaries, including the full native-path integration inventory, 30 general path/region trim cases and 12 downstream trim, builder, tessellation and homogeneous-control cases. Six format, lint, fuzz and documentation checks pass.\n'
 (A/f'{prefix}-{repo}-commit.txt').write_text(body)
print('Reviewed',len(expected),'release tests, six checks, 2047 sources and 30 repositories; timing flags',regressions)
