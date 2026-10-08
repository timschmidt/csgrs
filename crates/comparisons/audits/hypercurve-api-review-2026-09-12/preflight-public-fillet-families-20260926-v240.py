from pathlib import Path
import hashlib, json, re, subprocess
A=Path(__file__).resolve().parent
W=A.parent
prefix='public-fillet-families-coverage-20260926-v240'
report=json.loads((A/f'{prefix}-terminal.json').read_text())
assert report['coverage_complete'] and report['all_processes_reaped']
assert not report['qualification_complete']
assert report['hypercurve_build_returncode']==0
assert len(report['checks'])==4 and all(row['returncode']==0 for row in report['checks'])
assert len(report['cases'])==len(report['selection'])==637
assert report['hypercurve_passed']==635
expected_failures={'strict_trim_or_extend_analytic_parallel_support_corners_retain_algebraic_fillet_extensions','approximate_512_trim_or_extend_analytic_parallel_support_corners_retain_algebraic_fillet_extensions'}
assert {row['name'] for row in report['cases'] if row['returncode']}==expected_failures
for row in report['cases']:
 if row['returncode']==0:
  assert re.search(r'test result: ok\. 1 passed; 0 failed; 0 ignored;', (A/row['log']).read_text()),row['name']
 else:
  assert row['returncode']==101
  assert 'operation: Offset, family: RationalBezier, reason: Predicate' in (A/row['log']).read_text(),row['name']
manifest=json.loads((A/report['source_manifest']).read_text())
for name,sha in manifest.items():
 for root in (W,Path(report['source_directory']),Path(report['build_source_directory'])):
  assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,(str(root),name)
for binary in report['binaries'].values():
 assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest()==binary['sha256']
repos={}
for repo in sorted(W.iterdir()):
 if (repo/'.git').exists():
  repos[repo.name]=dict(head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip(),
   status=subprocess.check_output(['git','status','--short'],cwd=repo,text=True))
  assert not subprocess.check_output(['git','diff','--cached','--name-only'],cwd=repo),repo.name
  if repo.name!='hypercurve':assert not repos[repo.name]['status'],repo.name
for repo,head in report['parents'].items():assert repos[repo]['head']==head
files=['README.md', 'benches/editing.rs', 'src/bezier_offset.rs', 'src/bezier_region.rs', 'src/curve.rs', 'src/curve_corner_chain.rs', 'src/curve_corner_domain.rs', 'src/curve_parameter_component.rs', 'src/curve_subdivision.rs', 'src/curve_support_intersection.rs', 'src/lib.rs', 'tests/hypercurve_analytic_parallel_region.rs', 'tests/hypercurve_curve.rs', 'tests/hypercurve_curve_intersection.rs', 'tests/hypercurve_curve_region_boolean.rs', 'tests/hypercurve_curve_region_promotion.rs', 'tests/hypercurve_path_closure.rs', 'src/curve_fillet.rs']
changed=set(subprocess.check_output(['git','diff','--name-only','HEAD','-z'],cwd=W/'hypercurve').decode().strip('\0').split('\0'))
changed.update(subprocess.check_output(['git','ls-files','--others','--exclude-standard','-z'],cwd=W/'hypercurve').decode().strip('\0').split('\0'))
changed.discard('')
assert changed==set(files),changed
subprocess.run(['git','diff','--check'],cwd=W/'hypercurve',check=True)
qualified=dict(parent=report['parents']['hypercurve'],coverage=f'{prefix}-terminal.json',qualification_complete=False,passed=635,unresolved_failures=report['unresolved_failures'],
 all_processes_reaped=True,outer_session_id=54211,outer_exit_code=0,
 source_manifest=report['source_manifest'],files={file:manifest['hypercurve/'+file] for file in files},repositories=repos)
(A/f'{prefix}-reviewed.json').write_text(json.dumps(qualified,indent=2)+'\n')
print('Reviewed incremental milestone with two explicitly unresolved offset failures:',len(manifest),'inputs;',len(report['binaries']),'binaries;',len(report['cases']),'release cases;',len(repos),'repositories audited.')
print('Ready to stage:',', '.join(files))
