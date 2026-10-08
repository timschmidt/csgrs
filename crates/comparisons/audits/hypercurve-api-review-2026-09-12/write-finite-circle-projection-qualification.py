from pathlib import Path
import hashlib,json,shutil,subprocess

workspace=Path('/home/tim/Documents/GitHub/workspace')
root=Path('/tmp/hypercurve-circle-projection-qualification')
audit=workspace/'hypercurve-api-review-2026-09-12'
prefix='finite-circle-projection'
parent='8228394f174be2ae607694682defb167f030f540'
destination=audit/(prefix+'-qualification.json')
assert not destination.exists(), 'Do not overwrite an existing qualification.'
read=lambda suffix:json.loads((audit/(prefix+'-'+suffix+'.json')).read_text())
hashfile=lambda path:hashlib.sha256(Path(path).read_bytes()).hexdigest()
working=read('source-before-tests');isolated=read('isolated-sources')
for directory,rows in [(workspace,working),(root,isolated)]:
    for row in rows: assert hashfile(directory/row['file'])==row['sha256'],row['file']
toolchain=json.loads((audit/'finite-parallel-pruning-toolchain.json').read_text())
for name,sha in toolchain['binary_sha256'].items():assert hashfile(Path(toolchain['direct_toolchain_directory'])/name)==sha,name
builds=read('builds');focused=read('focused');full=read('full-results')
assert len(builds)==4 and len(focused)==8 and len(full)==49
assert all(row['returncode']==0 for row in builds+focused)
for row in focused:assert hashfile(row['binary'])==row['sha256']
for row in full:
    assert not row['timed_out'] and not row['new_failures']
    assert row['summaries'] and row['returncode'] in [0,101]
    assert row['returncode']==0 or row['known_failures']
    assert hashfile(row['binary'])==row['sha256'],row['binary']
items=[json.loads(line) for line in (audit/(prefix+'-test-build.jsonl')).read_text().splitlines()]
library=Path(next(f for x in items if x.get('reason')=='compiler-artifact' and x['target']['name']=='hypercurve' and not x['profile']['test'] for f in x['filenames'] if f.endswith('.rlib')))
library_sha=hashfile(library)
matrices=[]
for path in sorted(audit.glob(prefix+'-*.json')):
    row=json.loads(path.read_text())
    if isinstance(row,dict) and 'normal_library_sha256' in row and 'commit' not in row:
        assert row['returncode']==0 and row['normal_library_sha256']==library_sha,path.name
        assert hashfile(audit/row['source'])==row['source_sha256'],row['source']
        assert hashfile(audit/path.stem)==row['executable_sha256'],path.name
        matrices.append(path.name)
assert len(matrices)==34,len(matrices)
remaining_public_path=audit/'finite-circle-analytic-domains-projection-check.json'
remaining_public=json.loads(remaining_public_path.read_text())
assert remaining_public['returncode']==101 and remaining_public['counts']==dict(cases=32,contacts=16,point_replays=32,failures=16)
assert remaining_public['normal_library_sha256']==library_sha
assert hashfile(audit/remaining_public['source'])==remaining_public['source_sha256']
assert hashfile(audit/remaining_public['executable'])==remaining_public['executable_sha256']
changed=subprocess.check_output(['git','diff','--name-only'],cwd=workspace/'hypercurve',text=True).splitlines()
assert set(changed)=={'src/bezier_offset.rs'}
command=[str(Path(toolchain['direct_toolchain_directory'])/'rustfmt'),'--edition','2024','--config','skip_children=true','--check',*changed]
with (audit/(prefix+'-format-check.log')).open('w') as out:
    result=subprocess.run(command,cwd=workspace/'hypercurve',stdout=out,stderr=subprocess.STDOUT)
assert result.returncode==0
subprocess.run(['git','diff','--check'],cwd=workspace/'hypercurve',check=True)
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=workspace/'hypercurve',text=True).strip()==parent
archive=audit/(prefix+'-libraries');archive.mkdir()
shutil.copy2(library,archive/library.name)
libtest=next(Path(row['binary']) for row in full if row['repo']=='hypercurve' and row['target']=='lib');shutil.copy2(libtest,archive/libtest.name)
parent_results=read('parent-results')
assert len(parent_results['tests'])==2 and all(row['returncode']==101 for row in parent_results['tests'])
parent_artifact=audit/(prefix+'-parent-libtest')
assert all(hashfile(parent_artifact)==row['binary_sha256'] for row in parent_results['tests'])
assert sum(len(row['passed']) for row in full)==2296
report=dict(
    status='passed within the recorded broad qualification scope', goal_status='active',
    previous_commit=parent, commit=None, targets=len(full),
    passed=sum(len(row['passed']) for row in full),
    passed_by_repo={name:sum(len(row['passed']) for row in full if row['repo']==name) for name in ['hypercurve','hyperbrep']},
    known_failed=sorted(name for row in full for name in row['known_failures']), new_failures=[],
    ignored=sum(len(row['ignored']) for row in full), expensive_previously_unqualified_exclusions=8,
    timeouts_in_broad_run=0, builds=builds, focused_checks=len(focused), public_matrices_and_probes=matrices,
    remaining_public_circle_guard_check=remaining_public_path.name,
    private_factored_polynomial_domain_cases=72,
    private_selected_endpoint_projection_cases=12,
    parent_regressions=read('parent-results'),
    public_scope='All 34 previous public probes requalified; the outer selected-circle/analytic domain guard remains for the next migration.',
    attempts=['finite-circle-projection-parent-zero-filter-attempt1/', 'finite-circle-projection-compile-attempt1/'],
    source_files_verified=dict(working=len(working), isolated=len(isolated)),
    dependency_commits='finite-parallel-pruning-isolated-sources.json', isolated_dependency_sources=True,
    workspace_clean_claim=False, other_session_owns='Hyperreal verified/ and verification/ changes; left untouched',
    normal_library=str(library), normal_library_sha256=library_sha, archived_library=str(archive/library.name),
    libtest_sha256=hashfile(libtest), toolchain='finite-parallel-pruning-toolchain.json',
    baseline=prefix+'-parent-results.json', evidence_audit=prefix+'-worklog.md',
    remaining='Geometric circle/parallel optional-range interfaces and domain guard, remaining native-only bounds and shortcut premises, selected-point finite incidence, one-sided source-cusp frames, normalized public-region construction, independent inverse replay, known failures/exclusions and broader algebraic/computational consolidation.'
)
destination.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({key:report[key] for key in ['status','passed','passed_by_repo','known_failed','source_files_verified','normal_library_sha256','libtest_sha256']},indent=2))
