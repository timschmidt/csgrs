from pathlib import Path
import hashlib,json,re,shutil,subprocess
w=Path('/home/tim/Documents/GitHub/workspace');a=w/'hypercurve-api-review-2026-09-12';repo=w/'hypersolve';root=Path('/tmp/hypercurve-mapped-point-qualification/hypersolve');prefix='fiber-unit-pivots';destination=a/(prefix+'-qualification.json')
assert not destination.exists()
read=lambda name:json.loads((a/(name+'.json')).read_text())
hashfile=lambda path:hashlib.sha256(Path(path).read_bytes()).hexdigest()
build=read(prefix+'-candidate1');full=read(prefix+'-candidate1-full');check=read(prefix+'-check')
assert all(row['returncode']==0 for row in [build,full,check])
assert '496 passed; 0 failed;' in (a/(prefix+'-candidate1-full.log')).read_text()
assert hashfile(full['binary'])==full['sha256']==hashfile(full['archived_binary'])
assert 'warning:' not in (a/(prefix+'-check.log')).read_text()
messages=[json.loads(line) for line in (a/(prefix+'-candidate1.jsonl')).read_text().splitlines()]
assert not [m for m in messages if m.get('reason')=='compiler-message']
for name,sha in build['source_files'].items():
 assert hashfile(repo/name)==sha==hashfile(root/name)==hashfile(a/(prefix+'-candidate1-sources')/name)
changed=subprocess.check_output(['git','diff','--name-only'],cwd=repo,text=True).splitlines();assert set(changed)==set(build['source_files'])
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip()==build['parent']
subprocess.run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true','--check',*changed],cwd=repo,check=True)
subprocess.run(['git','diff','--check'],cwd=repo,check=True)
(a/(prefix+'-source.patch')).write_bytes(subprocess.check_output(['git','diff'],cwd=repo))
archive=a/(prefix+'-libraries');archive.mkdir()
for label in ['parent','candidate']:
 row=read(prefix+'-identity-'+label)
 assert row['returncode']==0
 assert hashfile(a/(prefix+'-identity-probe.rs'))==row['source_sha256']
 assert hashfile(a/(prefix+'-identity-'+label))==row['binary_sha256']
 for name,data in row['libraries'].items():
  assert hashfile(data['path'])==data['sha256']
  if name=='hypersolve':shutil.copy2(data['path'],archive/Path(data['path']).name)
 log=(a/(prefix+'-identity-'+label+'.log')).read_text()
 assert 'selected_sign=None' in log and 'witness_sign=None' in log and 'factored_sign=Some(Equal)' in log
mapped=read('mapped-point-inverse-attempt15/mapped-point-inverse-focused')
assert len(mapped)==28 and sum(row['returncode']==0 for row in mapped)==27
failed=[row for row in mapped if row['returncode']!=0]
assert len(failed)==1 and failed[0]['command'][2].endswith('mapped_compact_point_inverse_preserves_analytic_normal_sheet')
assert hashfile(a/'mapped-point-inverse-attempt15'/Path(mapped[0]['binary']).name)==mapped[0]['sha256']
for name,sha in build['source_files'].items():
 assert hashfile(a/('mapped-point-inverse-candidate15-hypersolve-'+name.removeprefix('src/').replace('/','-')))==sha
report=dict(status='passed for the independently scoped Hypersolve determinant change',goal_status='active',parent=build['parent'],commit=None,source_sha256=build['source_files'],library_tests=496,new_tests=2,failed=0,build=build,full=full,check=check,normal_libraries=str(archive),timings=read(prefix+'-identity-timings'),hypercurve_candidate='mapped-point-inverse-attempt15',hypercurve_focused_passed=27,hypercurve_focused_total=28,remaining_required_failure=failed[0]['command'][2],failure_status='Unchanged exact scalar identity uncertainty; required test retained and not excluded.',dependency_sources='finite-projective-parameter-snapshot.json',hyperreal_pin='a2da8e2b5de9a1a4653d3662f2f05cb6533fd7ef',other_session_changes='Hyperreal working changes left untouched; no build uses them.',computational_scope='Schur complements eliminate manifest nonzero rational units, reduce each product modulo the source fiber, and retain the division-free fallback. No source-coordinate element is inverted. One resultant fixture has lower construction cost; no general throughput or complete closure claim.')
destination.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:report[k] for k in ['status','library_tests','hypercurve_focused_passed','remaining_required_failure']},indent=2))
