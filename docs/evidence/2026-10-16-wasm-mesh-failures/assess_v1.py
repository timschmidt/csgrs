"""Record the narrow native-mesh WASM exception migration and its measured scope."""
import hashlib,json,re
from pathlib import Path
ROOT=Path('/home/tim/Documents/GitHub/workspace/csgrs')
E=ROOT/'docs/evidence/2026-10-16-wasm-mesh-failures'
def sha(path):
 with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()
def read(path):return json.loads(Path(path).read_text())
def dump(path,value):Path(path).write_text(json.dumps(value,indent=2)+'\n')
assert not (E/'decision-v1.json').exists()
app=read(E/'application-v1.json')
assert app['format_exit_code']==0 and app['pending_native_paths_remaining']==0
for path,digest in app['implementation_sha256'].items():assert sha(ROOT/path)==digest
for name,row in app['retained_completed_evidence'].items():assert sha(E/name)==row['sha256']==sha(row['original'])
for folder in ['baseline-v1','native-validation-v1','wasm-validation-v1','cost-v1']:
 for row in read(E/folder/'commands.json'):
  for suffix in ['stdout','stderr','resources']:
   assert sha(E/folder/(row['name']+'.'+suffix))==row[suffix+'_sha256']
baseline=read(E/'baseline-v1/result.json')
assert baseline['complete'] and baseline['javascript_exit_code']==1 and baseline['typescript_compile_exit_code']==0 and baseline['typescript_consumer_exit_code']==1
text=(E/'baseline-v1/real-js.stdout').read_text()
assert all(line in text for line in ['# tests 9','# pass 2','# fail 7','# skipped 0'])
wasm=read(E/'wasm-validation-v1/result.json');native=read(E/'native-validation-v1/result.json')
assert all(r['complete'] and not r['unexpected_exits'] for r in [wasm,native])
assert wasm['real_js_and_compiled_typescript_consumers_passed'] and wasm['public_contract_tests']==21
cost=read(E/'cost-v1/result.json')
assert cost['complete'] and cost['processes']==10 and cost['complete_success_values_and_stl_bytes_equal'] and cost['native_diagnostic_messages_equal']
status='kept: native mesh WASM exceptions preserve existing validation categories as host Error objects'
record={'status':status,'main_parent_commit':app['main_parent_commit'],'qualified_runtime_base_commit':app['qualified_runtime_base_commit'],
 'owning_layer':'CSGRS WASM error conversion; existing ValidationError::kind remains the shared category implementation',
 'properties':['P6','P18','P28','P30','P36','P37'],
 'domain':'Existing MeshJs native mesh-operation failures and zero subdivision level, preserving current native geometry and policy domains.',
 'implementation':'A typed HypermeshError adapter calls the existing GeometryError conversion; zero subdivision level uses the same ordinary host Error constructor. The error message remains the native diagnostic.',
 'migration':'Former string exceptions from native mesh Boolean, membership, hull and subdivision paths become Error objects with name GeometryError and the existing GeometryValidationError code union. Callers inspect code or message. STL retains GeometryIoError. Host exceptions require no free and outlive geometry handles.',
 'baseline':{'new_js_tests':9,'passed':2,'failed':7,'ignored':0,'typescript_compiled':True,'typescript_execution_exit_code':1},
 'candidate':{'new_js_tests_passed':9,'prior_io_tests_passed':12,'earlier_js_consumers_passed':True,'new_and_prior_typescript_consumers_compiled_and_executed':True,'wasm_clippy_warnings_denied':True,'native_all_feature_all_target_clippy_warnings_denied':True,'all_feature_docs_warnings_denied':True,'format_after_application_passed':True},
 'completed_contract_evidence':{'processes':cost['processes'],'successful_geometry_pipelines':50,'diagnostic_observations':250,'full_success_values_and_stl_bytes_equal':True,'native_diagnostic_messages_equal':True,'error_surface_intentionally_migrated':True,'raw_samples':'cost-v1/result.json'},
 'processed_wasm_bytes':{'baseline':cost['baseline_processed_wasm_bytes'],'candidate':cost['candidate_processed_wasm_bytes']},
 'certainty_and_provenance':'No geometry, predicate policy, proof or receipt is changed. Exact empty compositions retain Certified even under an approximate policy; existing qualified Unknown query outcomes remain separate. This patch does not certify earlier approximate intent or close all P30/provenance gaps.',
 'limits':'New regressions exercise invalid-input failures, successful empty composition and decided membership. Reuse of the existing native category mapper is not a claim that all six categories or every native error variant has a new JS trigger. No internal work, heap-byte, cancellation or arbitrary-subdivision bound is added. Root native geometry behavior is unchanged; the new host checks are compilation/lint/docs, not extra native runtime coverage.',
 'cost_interpretation':'Every raw sample is kept. Identical completed geometry outputs are compared separately from the deliberately changed exception contract; diagnostic construction and forced stack materialization are timed. No speedup, performance-only optimization or broad resource bound is claimed.',
 'source_identity':'All runtime, dependency, fixture and policy inputs match the qualified candidate. The only source-manifest difference is the prior admission ledger current_work value, with every protected byte and other decoded key verified equal.',
 'admission_metadata_only_difference':app['admission_metadata_only_difference'],
 'implementation_sha256':app['implementation_sha256'],'wasm_mesh_failures_admitted':True,
 'pending_native_paths_remaining':0,'native_disposition_preserved':True,'native_candidate_admitted':False,
 'historical_artifact_audit_passed':False,'historical_preservation_complete':False,
 'published_package_pipeline_executed':False,'full_pre_push_matrix_executed':False,'push_performed':False,'full_plan_complete':False,
 'next':['Continue typed failures, certainty and provenance across remaining public boundaries; keep qualified Unknown distinct from a successful false result.',
         'Use the preserved native regressions for smaller independently admitted repairs without weakening original workload gates.',
         'Continue ordinary strict completion, clean external distribution and remaining properties work.']}
dump(E/'decision-v1.json',record)
lines=['# Native mesh failures through real WASM consumers','',
 'Native mesh Boolean, membership, hull and subdivision failures now throw ordinary `GeometryError` objects with the existing validation category in `code` and the native diagnostic in `message`. Zero subdivision level uses that same contract. The adapter reuses `ValidationError::kind`; STL continues to use `GeometryIoError`.','',
 '**Migration:** these paths previously threw strings. Callers should inspect `error.code` or `error.message`. Error objects belong to JavaScript, need no `free()`, and remain usable after native geometry handles are released. The generated class documentation explains this change.','',
 'The [declared contract](prototype-v1/contract.json) and [initial real-JS probe](preflight-v1/result.json) establish the owning layer and missing acceptance before implementation. The probe observed 28 string exceptions. The unchanged new acceptance suite [failed seven of nine tests on the baseline](baseline-v1/real-js.stdout); its TypeScript consumer compiled and failed at runtime on the same issue.','',
 'The qualified candidate [passed all nine new JS tests](wasm-validation-v1/real-consumers.stdout), both earlier JS consumers, all twelve earlier IO tests, and the new and previous compiled TypeScript consumers. Invalid operands beside empty fail visibly in either order and both policies; successful empty results retain certainty through Boolean composition, STL import and further composition. Diagnostics survive native-handle cleanup. Valid membership retains its own status and certainty.','',
 'WASM Clippy, native all-feature/all-target Clippy, all-feature docs with warnings denied, and formatting passed. Main-tree formatting passed after application. These native checks establish compilation/lint/documentation; the runtime boundary evidence comes from actual generated bindings.','',
 'The [ten retained process samples](cost-v1/result.json) completed 50 geometry pipelines and 250 diagnostic observations. Full success values and STL bytes agree; original diagnostic messages agree. Successful geometry and the intentionally changed error surface have separate timing observations, and whole-process CPU/RSS records are retained. The old and new diagnostic contracts perform different work. No speedup or performance-only optimization is claimed.','',
 f"Processed WASM sizes were {cost['baseline_processed_wasm_bytes']:,} bytes for the baseline and {cost['candidate_processed_wasm_bytes']:,} bytes for the candidate. Source, dependency/tool versions, generated artifacts, raw output and all unfavorable samples are retained.",'',
 'The source snapshot was prepared while the native disposition was running, from its qualified committed-code snapshot. Application waited for the disposition commit. All runtime, policy, fixture and dependency bytes match; the only source-manifest difference is the admission ledger’s `current_work` value. The [application record](application-v1.json) verifies that protected raw bytes and all other decoded ledger keys are identical.','',
 'This is a narrow boundary improvement. It adds no internal resource bound, native geometry capability or certification of earlier approximate intent. New tests cover the stated invalid-input cases, not new public triggers for every native error variant or all six categories. Remaining provenance, query, distribution, release, pre-push and other properties work stays open. The historical artifact audit remains failed/incomplete; inherited expectations and missing paths remain preserved.','',
 'The 35-file native integration remains rejected and preserved by the preceding disposition; no native edit is reintroduced. See the [decision](decision-v1.json) and [checkpoint](checkpoint-v1.json).']
(E/'progress-v1.md').write_text('\n'.join(lines)+'\n')
print(json.dumps({'wasm_mesh_failures_admitted':True,'new_js_tests_passed':9,'prior_io_tests_passed':12,'implementation_paths':len(app['implementation_sha256'])}),flush=True)
