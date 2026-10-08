from pathlib import Path
import hashlib,json,shutil,subprocess

workspace=Path('/home/tim/Documents/GitHub/workspace')
root=Path('/tmp/hypercurve-affine-qualification')
audit=workspace/'hypercurve-api-review-2026-09-12'
prefix='finite-parallel-affine'
destination=audit/(prefix+'-qualification.json')
assert not destination.exists(), 'Do not overwrite an existing qualification.'
read=lambda suffix:json.loads((audit/(prefix+'-'+suffix+'.json')).read_text())
working=read('source-before-tests'); isolated=read('isolated-sources')
for directory,rows in [(workspace,working),(root,isolated)]:
    for row in rows: assert hashlib.sha256((directory/row['file']).read_bytes()).hexdigest()==row['sha256'],row['file']
toolchain=json.loads((audit/'finite-parallel-pruning-toolchain.json').read_text())
for name,sha in toolchain['binary_sha256'].items(): assert hashlib.sha256((Path(toolchain['direct_toolchain_directory'])/name).read_bytes()).hexdigest()==sha,name
builds=read('builds'); focused=read('focused'); full=read('full-results')
assert len(builds)==4 and len(focused)==10 and len(full)==49
assert all(row['returncode']==0 for row in builds+focused)
assert all(not row['timed_out'] and not row['new_failures'] for row in full)
for row in full:
    assert row['summaries'] and row['returncode'] in [0,101]
    assert row['returncode']==0 or row['known_failures']
    assert hashlib.sha256(Path(row['binary']).read_bytes()).hexdigest()==row['sha256'],row['binary']
items=[json.loads(line) for line in (audit/(prefix+'-test-build.jsonl')).read_text().splitlines()]
library=Path(next(f for x in items if x.get('reason')=='compiler-artifact' and x['target']['name']=='hypercurve' and not x['profile']['test'] for f in x['filenames'] if f.endswith('.rlib')))
library_sha=hashlib.sha256(library.read_bytes()).hexdigest()
matrices=[]
for path in sorted(audit.glob(prefix+'-*.json')):
    row=json.loads(path.read_text())
    if isinstance(row,dict) and 'normal_library_sha256' in row:
        assert row['returncode']==0 and row['normal_library_sha256']==library_sha,path.name
        if 'source' in row: assert hashlib.sha256((audit/row['source']).read_bytes()).hexdigest()==row['source_sha256'],row['source']
        matrices.append(path.name)
assert len(matrices)==29,len(matrices)
public=read('public')
assert hashlib.sha256((audit/(prefix+'-public.rs')).read_bytes()).hexdigest()==public['source_sha256']
assert hashlib.sha256((audit/(prefix+'-public')).read_bytes()).hexdigest()==public['binary_sha256']
changed=subprocess.check_output(['git','diff','--name-only'],cwd=workspace/'hypercurve',text=True).splitlines()
command=[str(Path(toolchain['direct_toolchain_directory'])/'rustfmt'),'--edition','2024','--config','skip_children=true','--check',*changed]
with (audit/(prefix+'-format-check.log')).open('w') as out:
    result=subprocess.run(command,cwd=workspace/'hypercurve',stdout=out,stderr=subprocess.STDOUT)
assert result.returncode==0
subprocess.run(['git','diff','--check'],cwd=workspace/'hypercurve',check=True)
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=workspace/'hypercurve',text=True).strip()=='4ac78841cc571f3f802296e0af1e19fd520544b4'
archive=audit/(prefix+'-libraries');archive.mkdir()
shutil.copy2(library,archive/library.name)
libtest=next(Path(row['binary']) for row in full if row['repo']=='hypercurve' and row['target']=='lib');shutil.copy2(libtest,archive/libtest.name)
benchmark=read('derivative-benchmark');expensive=read('expensive-cusp')
report=dict(status='passed within the recorded broad qualification scope',goal_status='active',previous_commit='4ac78841cc571f3f802296e0af1e19fd520544b4',commit=None,targets=len(full),passed=sum(len(row['passed']) for row in full),passed_by_repo={name:sum(len(row['passed']) for row in full if row['repo']==name) for name in ['hypercurve','hyperbrep']},known_failed=sorted(name for row in full for name in row['known_failures']),new_failures=[],ignored=sum(len(row['ignored']) for row in full),expensive_previously_unqualified_exclusions=8,timeouts_in_broad_run=0,builds=builds,focused_checks=len(focused),public_matrices_and_probes=matrices,public_cusp_representation_checks=80,source_files_verified=dict(working=len(working),isolated=len(isolated)),dependency_commits='finite-parallel-pruning-isolated-sources.json',isolated_dependency_sources=True,workspace_clean_claim=False,other_session_owns='Hyperreal verified/ and verification/ changes; left untouched',normal_library=str(library),normal_library_sha256=library_sha,archived_library=str(archive/library.name),libtest_sha256=hashlib.sha256(libtest.read_bytes()).hexdigest(),toolchain='finite-parallel-pruning-toolchain.json',baseline=prefix+'-baseline/result.json',failed_runtime_candidate=prefix+'-attempt1/',compile_only_attempts=[prefix+'-compile-attempt2/',prefix+'-public-compile-attempt1/',prefix+'-public-compile-attempt2/'],derivative_microbenchmark=prefix+'-derivative-benchmark.json',additional_expensive_cusp_attempt=expensive,evidence_audit=prefix+'-worklog.md',remaining='Range-owned convex-turn proof, finite analytic fragment and boundary-loop admission, finite analytic pair domains and incidence, normalized public-region construction, independent inverse replay, known failures/exclusions and the broader algebraic/computational architecture goal.')
destination.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({key:report[key] for key in ['status','passed','passed_by_repo','known_failed','source_files_verified','normal_library_sha256','libtest_sha256']},indent=2))
