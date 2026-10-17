"""Assess only the completed source-validation contract; retain unrelated failures."""
import hashlib,json,re
from pathlib import Path
WORK=Path(__file__).resolve().parent;ROOT=WORK.parents[2]
E=ROOT/'docs/evidence/2026-10-16-containment-input-validation'
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
application=read(E/'application-v1.json')
assert application['format_exit_code']==0 and application['all_final_inputs_match_qualified_sources']
assert all(sha(ROOT/p)==h for p,h in application['implementation_sha256'].items())
validation={}
for folder in ['baseline-validation-v1','candidate-validation-v2','wasm-validation-v2']:
 result=read(E/folder/'result.json');assert result['complete'] and not result['unexpected_exits']
 rows=[]
 for row in read(E/folder/'commands.json'):
  for suffix in ['stdout','stderr','resources']:assert sha(E/folder/(row['name']+'.'+suffix))==row[suffix+'_sha256']
  text=(E/folder/(row['name']+'.stdout')).read_text()
  summaries=re.findall(r'test result: (?:ok|FAILED)\. (\d+) passed; (\d+) failed; (\d+) ignored;',text)
  rows.append({'name':row['name'],'command':row['command'],'exit_code':row['exit_code'],'elapsed_seconds':row['elapsed_seconds'],'summaries':len(summaries),'passed':sum(int(r[0]) for r in summaries),'failed':sum(int(r[1]) for r in summaries),'ignored':sum(int(r[2]) for r in summaries)})
 validation[folder]=rows
baseline=validation['baseline-validation-v1'][0];assert (baseline['passed'],baseline['failed'])==(4,6)
focused=next(r for r in validation['candidate-validation-v2'] if r['name']=='containment-inputs');assert (focused['passed'],focused['failed'])==(10,0)
assert all(r['exit_code']==0 and r['failed']==0 for folder in ['candidate-validation-v2','wasm-validation-v2'] for r in validation[folder])
assert read(E/'wasm-reproduction-v1/result.json')['expected_failure_reproduced']
wasm=read(E/'wasm-validation-v2/result.json');assert wasm['real_js_and_compiled_typescript_consumers_passed']
cost=read(E/'cost-v1/result.json');assert cost['complete'] and cost['all_expected_values_and_certainties_match']
assert read(E/'winding-followup-v1/result.json')['nonzero_winding_mismatches']==6
assert (E/'winding-followup-v1/stdout').read_bytes()==(E/'winding-diagnostic-v1/stdout').read_bytes()
next_steps=[
 'Repair solid containment agreement with nonzero winding using the retained nested-same-orientation and coincident-opposite-orientation diagnostics; audit boundary and tangent-ray semantics at the native owning layer before implementation.',
 'Continue typed failures, ordinary strict completion, provenance, clean external distribution and the remaining properties without weakening preserved tests or the rejected native integration workload gates.'
]
decision={'status':'kept: native containment validates source topology and consumes its certainty before query shortcuts','main_parent_commit':application['main_parent_commit'],'implementation_sha256':application['implementation_sha256'],'contract':'contract-v2.md','validation':validation,'baseline_native':{'passed':4,'failed':6},'baseline_wasm':{'passed':10,'failed':3,'evidence':'wasm-reproduction-v1/result.json'},'new_native_tests':10,'mesh_wasm_tests':13,'prior_io_wasm_tests':12,'typescript_scope':'geometry_certainty.ts typechecked; io_contracts.ts and mesh_errors.ts compiled and executed','all_final_source_inputs_match_qualified_snapshot':True,'cost':'cost-v1/result.json','cost_summary':cost['paired_summary'],'cost_claim':'Focused instrumented cost/allocation observations for the added validation; no speedup or universal memory claim','intentional_migration':'Malformed source fails typed; a strict query whose source validity is undecided now remains undecided even for an out-of-bounds point; approximate validation contributes its receipt. See contract-v2.md.','known_winding_gap':{'diagnostic':'winding-followup-v1/result.json','cases':2,'policy_observations':6,'unchanged_nonzero_winding_mismatches':6,'fixed':False},'preparation_failures_preserved':'preparation-failure-v1.json','baseline_wasm_infrastructure_failure':'baseline-wasm-limit-note-v1.json','containment_input_validation_admitted':True,'full_set_membership_correctness_claimed':False,'published_package_pipeline_executed':False,'full_pre_push_matrix_executed':False,'complete_artifact_audit_passed':False,'historical_missing_paths':27093,'native_disposition_preserved':True,'full_plan_complete':False,'next':next_steps}
dump(E/'decision-v1.json',decision)
lines=['# Containment input validation acceptance','',
 'The native query now reuses the existing closed-PWN validator before bounds or boundary shortcuts. Canonical empty geometry remains a decided, Certified false. The modeling and WASM entry points share this implementation; the latter preserves typed GeometryError failures and its existing Unknown representation.','',
 'This is the input/evidence migration in [contract-v2.md](contract-v2.md), not a claim that every valid PWN containment answer has correct nonzero-winding semantics. Strict queries now remain undecided when source validity cannot be proved, even if bounds could exclude the point. Approximate validation keeps its qualification. No fixture, policy, assertion, native winding algorithm, or prior certified-success test was weakened.','',
 'The frozen baseline ran ten native tests: four passed and six failed. The actual baseline WASM consumer ran thirteen tests: ten passed and three failed. A separate baseline invocation under an 8-GiB address-space limit failed during WASM instantiation before tests; it supplies no test evidence. Its raw output and the successful 64-GiB reproduction are retained. The first preparation script also stopped on a documentation-text mismatch, and its premature validation startup failed before running Cargo. Both attempts and their unused snapshots/targets remain preserved.','',
 'Candidate checks (each command and full output is retained):','',
 '| Command group | Exit | Passed | Failed | Existing ignored |','| --- | ---: | ---: | ---: | ---: |']
for folder in ['candidate-validation-v2','wasm-validation-v2']:
 for r in validation[folder]:lines.append(f"| {r['name']} | {r['exit_code']} | {r['passed'] if r['summaries'] else '—'} | {r['failed'] if r['summaries'] else '—'} | {r['ignored'] if r['summaries'] else '—'} |")
lines+=['','The real WASM script passed all 13 mesh and 12 IO tests, both earlier JavaScript consumers, geometry-certainty TypeScript typechecking, and the compiled/executed IO and mesh TypeScript consumers. Independent tetrahedron halfspaces and invalid edge/index counts are the semantic oracles; agreement between wrappers only verifies propagation. The cache tests cover clones, concurrent readers, mutation and strict/approximate/strict source validity.','',
 'Five alternating fresh-process pairs completed 2,560 Certified queries with the same full expected bool rows. Cold measurements include source construction, all queries and result materialization; warm measurements reuse the same meshes. Allocator counters report requested Rust bytes, not total heap capacity. Timings are instrumented observations, not a production speedup claim. All adverse samples, drop measurements, whole-process RSS, binary sizes, compiler/hardware/lock/source pins and external limits are retained in [cost-v1/result.json](cost-v1/result.json).','']
for r in cost['paired_summary']:
 d=r['all_pairs'];lines.append(f"- {r['policy']} / {r['phase']}: median candidate/baseline elapsed ratio {r['median_elapsed_ratio']:.3f}; allocation-call deltas {[v['allocation_calls_delta'] for v in d]}; allocated-byte deltas {[v['allocated_bytes_delta'] for v in d]}.")
lines+=['','The unchanged winding diagnostic still disagrees with nonzero-winding expectations in both fixtures and all six policy observations: nested outward shells and coincident opposite shells. This is an open correctness gap, preserved in [winding-followup-v1/stdout](winding-followup-v1/stdout); the validation guard does not repair it.','',
 'All 2,827 final source inputs match the qualified candidate before the admission-only ledger update. Main-tree formatting passed after applying the five files. No completed Cargo target was reopened. No source/fixture/dependency exception was needed.','',
 'Remaining limits: no internal total-work, heap-byte or cancellation guarantee; no complete set-membership, end-to-end provenance, release, full CI/pre-push, coverage or whole-history preservation claim. The 27,093 inherited missing artifact paths remain unresolved. The rejected 35-file native integration and its 22 failing regression tests remain preserved. Nothing was pushed.','',
 'Next: repair the retained winding mismatch at the native layer, then continue the full properties plan.']
(E/'progress-v1.md').write_text('\n'.join(lines)+'\n')
print(json.dumps({'baseline':decision['baseline_native'],'candidate_focused':10,'mesh_wasm':13,'known_winding_mismatches':6}),flush=True)
