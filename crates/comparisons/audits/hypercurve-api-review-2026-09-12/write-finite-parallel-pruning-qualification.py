from pathlib import Path
import hashlib,json,shutil,subprocess
r=Path('/home/tim/Documents/GitHub/workspace');a=r/'hypercurve-api-review-2026-09-12';p='finite-parallel-pruning'
sha=lambda path:hashlib.sha256(Path(path).read_bytes()).hexdigest()
toolchain=json.loads((a/(p+'-toolchain.json')).read_text())
for name,digest in toolchain['binary_sha256'].items(): assert sha(Path(toolchain['direct_toolchain_directory'])/name)==digest,name
rows=json.loads((a/(p+'-full-results.json')).read_text());assert len(rows)==49,len(rows)
baseline=json.loads((a/(p+'-baseline-failures.json')).read_text())
known=sorted(n for row in rows for n in row['known_failures']);assert known==sorted(n for names in baseline.values() for n in names)
for row in rows:
 assert not row['new_failures'] and not row['timed_out'],row
 assert row['returncode']==(101 if row['known_failures'] else 0),row
 assert row['summaries'] and sha(row['binary'])==row['sha256'],row['target']
manifest=json.loads((a/(p+'-source-before-tests.json')).read_text());changed=[x['file'] for x in manifest if sha(r/x['file'])!=x['sha256']];assert not changed,changed
verification=dict(files_checked=len(manifest),changed=changed,source_manifest=p+'-source-before-tests.json',all_sources_unchanged_during_builds_and_tests=True)
(a/(p+'-source-verification.json')).write_text(json.dumps(verification,indent=2)+'\n')
formatted={}
for name in ['hypercurve','hyperbrep']:
 files=[n for n in subprocess.check_output(['git','diff','--name-only'],cwd=r/name,text=True).splitlines() if n.endswith('.rs')];formatted[name]=files
 if files:subprocess.run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true','--check',*files],cwd=r/name,check=True)
 subprocess.run(['git','diff','--check'],cwd=r/name,check=True)
checks={s:int((a/(p+s+'.exit')).read_text()) for s in ['-check','-test-build','-hyperbrep-check','-hyperbrep-test-build']};assert all(x==0 for x in checks.values()),checks
artifacts=[json.loads(l) for l in (a/(p+'-test-build.jsonl')).read_text().splitlines()]
lib=Path(next(f for x in artifacts if x.get('reason')=='compiler-artifact' and x['target']['name']=='hypercurve' and not x['profile']['test'] for f in x['filenames'] if f.endswith('.rlib')));lib_sha=sha(lib)
callers=json.loads((a/(p+'-caller-checks.json')).read_text());assert len(callers)==3 and all(x['returncode']==0 for x in callers)
probes=[]
for suffix in ['public-replay','affine-parallel-replay','exterior-completed','tangent-matrix','finite-circle-probe','family-matrix','completed','unit-replay','retained-chord-matrix','finite-matrix','cubic-replay','baseline-recheck','replay-matrix-run','analytic-probe','endpoint-probe','completed-probe','probe-final','circle-matrix','region-carrier-matrix','general-trim-matrix','constant-image-trim-matrix','region-public','region-explicit','bounds-public','winding-public','trim-public','self-public','retrace-public']:
 name=p+'-'+suffix;d=json.loads((a/(name+'.json')).read_text());assert d['returncode']==0 and d['normal_library_sha256']==lib_sha,d
 assert d['source_sha256']==sha(a/d['source']) and d['executable_sha256']==sha(a/name),name
 if suffix in ['region-public','region-explicit']:assert d['counts']==dict(queries=48,point_replays=96,topologies=48,splits=96,child_queries=192,region_queries=12,boolean_reentries=12)
 probes.append(name+'.json')
focused=json.loads((a/(p+'-focused.json')).read_text());assert len(focused)==5
for row in focused:
 assert row['returncode']==0 and sha(row['binary'])==row['sha256'],row
 assert '1 passed; 0 failed;' in (a/row['log']).read_text(),row
parent=json.loads((a/'finite-parallel-regularity-qualification.json').read_text());assert parent['commit']=='2c16c30bb1aba38f08456bed462b5ea058bc5c71'
baseline_dir=a/'finite-parallel-pruning-baseline'
probe=json.loads((baseline_dir/'finite-parallel-pruning-baseline-build-cusp_separated_parallel_contacts_are_not_declared_distinct.json').read_text())
assert probe['returncode']==101 and sha(baseline_dir/Path(probe['binary']).name)==probe['sha256']
assert sha(baseline_dir/'finite-parallel-pruning-baseline-build.patch')==probe['source_patch_sha256']
archive=a/(p+'-libraries');archive.mkdir(exist_ok=True);shutil.copy2(lib,archive/lib.name)
result=dict(status='qualified against the documented baseline; commit pending',goal_status='active',previous_commit=parent['commit'],commit=None,targets=len(rows),passed=sum(len(x['passed']) for x in rows),passed_by_repo={name:sum(len(x['passed']) for x in rows if x['repo']==name) for name in ['hypercurve','hyperbrep']},known_failed=known,new_failures=[],ignored=sum(len(x['ignored']) for x in rows),expensive_previously_unqualified_exclusions=8,timeouts=0,builds_and_checks=checks,formatting_and_whitespace='passed on final frozen sources',formatted_files=formatted,additional_caller_checks=callers,source_verification=p+'-source-verification.json',focused_regression_checks=p+'-focused.json',normal_library=str(lib),normal_library_sha256=lib_sha,archived_library=str(archive/lib.name),independent_probes_and_matrices=probes,discovery_probes=['finite-parallel-pruning-baseline/finite-parallel-pruning-baseline-build-cusp_separated_parallel_contacts_are_not_declared_distinct.json'],prequalification_candidate='finite-parallel-pruning-attempt1/',compile_only_attempts=[],evidence_audit=p+'-worklog.md',performance_claim=None,toolchain=p+'-toolchain.json',remaining='Analytic fragment and boundary-loop finite admission (earlier public baseline remains open), affine analytic derivative replay; analytic finite pair discovery and incidence; normalized public region construction; independent inverse replay; five existing promotion failures; eight unqualified expensive tests; original algebraic and computational architecture goal.')
(a/(p+'-qualification.json')).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:result[k] for k in ['status','targets','passed','passed_by_repo','known_failed','new_failures','ignored','timeouts','normal_library_sha256']},indent=2))
