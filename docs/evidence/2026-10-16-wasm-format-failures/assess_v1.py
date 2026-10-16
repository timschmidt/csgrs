"""Assess public native-import and typed-error contracts against retained evidence."""
import hashlib
import json
from pathlib import Path
import re

E=Path(__file__).resolve().parent
ROOT=E.parents[2]


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream,'sha256').hexdigest()


def read(path):return json.loads(Path(path).read_text())

assert not (E/'decision-v1.json').exists() and not (E/'progress-v1.md').exists()
application=read(E/'application-v1.json')
assert application['format_exit_code']==0 and application['source_input_count']==2830
assert all(sha(ROOT/path)==digest for path,digest in application['implementation_sha256'].items())
assert all(sha(E/name)==row['sha256']==sha(row['original']) for name,row in application['retained_completed_evidence'].items())
baseline=read(E/'baseline-v1/result.json')
assert baseline['complete'] and baseline['all_expected_exits']
assert baseline['public_tests']=={'passed':1,'failed':11,'total':12}
commands={}
for folder,count in [('native-validation-v1',6),('wasm-validation-v1',3)]:
    result=read(E/folder/'result.json')
    assert result['complete'] and not result['unexpected_exits']
    commands[folder]=read(E/folder/'commands.json')
    assert len(commands[folder])==count
    for row in commands[folder]:
        assert row['exit_code']==row['expected_exit_code']==0
        for suffix in ['stdout','stderr','resources']:
            assert sha(E/folder/(row['name']+'.'+suffix))==row[suffix+'_sha256']
counts={}
for name,expected in [('root-all',(429,0,1)),('wasm-feature',(346,0,0))]:
    rows=[tuple(map(int,row)) for row in re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;', (E/'native-validation-v1'/(name+'.stdout')).read_text())]
    total=tuple(sum(row[i] for row in rows) for i in range(3))
    assert total==expected
    counts[name]=dict(zip(['passed','failed','ignored'],total))|{'test_binaries_and_doctest_groups':len(rows)}
wasm=read(E/'wasm-validation-v1/result.json')
assert wasm['real_js_and_compiled_typescript_consumers_passed'] and wasm['public_contract_tests']==12
assert all(sha(path)==row['sha256'] for path,row in wasm['artifacts'].items())
consumer=(E/'wasm-validation-v1/real-consumers.stdout').read_text()
for text in ['# tests 12','# pass 12','# fail 0','TypeScript native STL pipeline and discriminated errors passed',
             'WASM certainty, query status, geometry, and invalid-input checks passed',
             'WASM export projection, failure, cache and composition contracts passed']:
    assert text in consumer
resources=read(E/'import-resources-v1/result.json')
assert resources['complete'] and len(resources['observations'])==8
assert resources['observations'][0]['repeats']==128
cost=read(E/'cost-v1/result.json')
assert cost['complete'] and cost['processes']==95 and cost['rows']==190
assert cost['complete_inputs_and_outputs_byte_equal'] and cost['independent_integer_oracle_passed']
assert cost['strict_parsed_geometry_pipeline_receipts_preserved'] and cost['typed_failures_never_returned_models']
for name in ['summary','commands','rows']:
    assert sha(E/'cost-v1'/(name+'.json'))==cost[name+'_sha256']
summary=read(E/'cost-v1/summary.json')
assert len(summary['groups'])==26
status='kept: native STL imports and typed format/array failures through real WASM consumers'
record={
 'status':status,'base_commit':application['main_parent_commit'],
 'owning_layers':['CSGRS WASM exception structure and array ingress','unchanged CSGRS STL parser and Hypermesh strict closed-PWN validation'],
 'properties':['P6','P9','P18','P19','P25','P26','P28','P30','P35','P36','P37'],
 'implementation_sha256':application['implementation_sha256'],'baseline':baseline,
 'candidate':{'public_tests':{'passed':12,'failed':0},'cargo_tests':counts,'native_commands':6,'wasm_commands':3,
              'typescript_compiled_and_executed':True,'existing_js_and_typescript_consumers_preserved':True},
 'native_delegation':'MeshJs.fromSTL calls unchanged crate::io::stl::from_stl. Exports retain their existing native implementations. No adapter geometry kernel or duplicated validation is introduced.',
 'numeric_claim':'Finite binary32 coordinates parsed from either STL encoding are promoted exactly. Strict import and subsequent strict operations establish claims about this parsed mesh.',
 'topology_claim':'Canonical empty or native certified closed-PWN input, including disconnected components, holes, inversion and winding multiplicity. This is not a manifoldness assertion. fromArrays retains its raw-buffer domain and defers topology to consuming operations.',
 'fidelity_limit':'ASCII import quantizes to binary32 too. A reimported mesh and its strict receipt do not certify preservation of exact source coordinates, analytic intent or inherited construction provenance. No new end-to-end provenance carrier is introduced.',
 'exception_migration':'fromArrays now throws ordinary GeometryError/invalid_input objects; STL exports and the new importer throw GeometryIoError objects instead of strings. Catch error.message for text, discriminate error.code for native variant data, and inspect construction error.cause.code for the retained validation category. Error objects are owned by JavaScript and require no Wasm free call.',
 'typed_error_scope':'The generated TypeScript union describes all twelve native IO variants under all features. Actual public STL tests exercise geometry, unrepresentable_coordinate, std_io, malformed_input and construction, plus GeometryError/invalid_input array ingress. All-feature native compilation checks other match arms; no runtime coverage of unreachable format variants or OS-specific errors is claimed.',
 'diagnostic_scope':'Nested construction causes reuse existing ValidationErrorKind categories and native display text. They do not serialize every native proof witness or underlying Rust cause field.',
 'ownership_evidence':'Input-byte mutation after import does not alter parsed geometry. Consuming GeometryOutcome wrappers transfer mesh ownership once. Host errors and causes remain readable after mesh disposal, and failures return no MeshJs.',
 'resource_observations':resources['observations'],
 'hostile_header_scope':'128 imports of an 84-byte header declaring 4294967295 triangles fail as typed UnexpectedEof. Pinned stl_io 0.8.6 source deliberately avoids reserve(size_hint) and stops at read failure. This is a focused probe, not a global parser resource proof.',
 'validation_partition':application['validation_partition'],'source_inputs':2830,
 'cost_result':'cost-v1/result.json','cost_summary':'cost-v1/summary.json','cost_groups':summary['groups'],
 'artifact_size_changes':summary['artifact_size_changes'],
 'cost_scope':'60 matched successful export processes, 30 candidate-only complete import/strict-compose/export processes and five candidate-only typed-error processes. All 190 first/warm rows and complete output bytes are retained. First-call timings include runtime lazy compilation; module loading and fixture construction are excluded. Host load is not controlled. No timer-scale speedup or supported-corpus optimization claim.',
 'memory_scope':'Observed Node process memory, Wasm linear-memory high-water and external peak RSS; not Rust allocation counts or live-heap bounds. Complete outputs and pipeline meshes/error receipts remain retained at observation. Module size and generated JS/declaration sizes are recorded.',
 'resource_limits':'No public total-work, heap-byte or cancellation budget is added. External elapsed/address-space limits contain probes. Existing caches still have entry-count limits only.',
 'wasm_format_contracts_admitted':True,'optimization_claimed':False,
 'pending_native_files_unchanged':35,'native_runtime_admitted':False,
 'historical_artifact_audit_passed':False,'historical_preservation_complete':False,
 'published_package_pipeline_executed':False,'full_pre_push_matrix_executed':False,
 'push_performed':False,'full_plan_complete':False,
 'next':['Continue typed failures and claim scope across remaining legacy WASM boundaries; full P30 and inherited provenance remain open.',
         'Keep the 35 pending native changes unadmitted until their original full-board and copper-neck runtime gates pass.',
         'Continue ordinary strict completion, clean external distribution, remaining properties and complete pre-push validation.']}
(E/'decision-v1.json').write_text(json.dumps(record,indent=2)+'\n')
lines=['# Native STL import and typed format failures in WASM','',
 'This is a narrow source admission under [the declared contract](contract-v1.md), `properties.md` and `docs/admission.md`. The full properties goal remains active. It improves the supported public pipeline without changing the native parser, geometry kernels, predicate policies, fixtures, or successful export bytes.','',
 'On parent `3d335db43eac00d003bc2fb0ed1beb2a9c19d79b`, the new real-JavaScript matrix passed one test and failed eleven; TypeScript lacked the importer and error types. Both existing consumers passed. [Baseline commands and raw outputs](baseline-v1/commands.json) preserve these findings.','',
 '`MeshJs.fromSTL(Uint8Array)` now delegates to native strict closed-PWN validation. Binary and ASCII encodings accept canonical empty, disconnected, nested, inverted and winding-multiplicity input within that native domain. Coordinates parse as binary32 and are promoted exactly. These facts concern the parsed mesh; they do not establish manifoldness, exact-source preservation, analytic fidelity, or inherited provenance.','',
 'STL import/export failures are ordinary `GeometryIoError` objects with a native `code`, format context, variant fields, and retained validation `cause` where present. Malformed `fromArrays` input uses `GeometryError`/`invalid_input`. This intentionally changes former string exceptions: use `message` for text, `code` for classification, and `cause.code` for native construction categories. Host errors require no `free()` and outlive mesh handles. Generated TypeScript declarations describe these fields.','',
 'The final shared source snapshot contains 2,830 pinned inputs. All six [native commands](native-validation-v1/commands.json) passed: all-feature tests (429 passed, one existing README-image test ignored), wasm-feature tests (346 passed), both all-target Clippy combinations with `-D warnings`, formatting, and docs with warnings denied. All three [WASM commands](wasm-validation-v1/commands.json) passed, including actual WASM Clippy and the real binding harness. All twelve new JS tests passed; both prior JS consumers and the old TypeScript check passed; the new TypeScript consumer compiled and executed. Main-tree formatting passed after byte-identical application.','',
 'The [broad-domain/resource probe](import-resources-v1/result.json) retained eight observations: 128 hostile maximum-count headers failed typed, minimum-subnormal/minimum-normal/maximum binary32 cubes imported and composed strictly, and disconnected/hole/inverted/double-winding fixtures preserved their independently encoded oriented facets and signed volumes. These externally contained probes do not establish a public work, heap or cancellation bound.','',
 'The [cost contract](cost-contract-v1.md), [95 process records](cost-v1/commands.json), [190 raw rows](cost-v1/rows.json), and [full summary](cost-v1/summary.json) retain favorable and unfavorable results. Existing exports use five alternating pairs per fixture, complete retained API outputs, independent integer geometry oracles, and full byte comparisons. Every process succeeded and every matched complete input/output agreed. First calls include lazy runtime compilation. Module loading, setup and verification are outside timing; host load was not controlled. No optimization is claimed.','',
 '| Matched export | Cubes | First baseline µs | First candidate µs | Paired ratio | Warm baseline µs | Warm candidate µs | Paired ratio |',
 '| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |']
for encoding in ['binary','ascii']:
 for cubes in [0,1,256]:
  pair=[next(r for r in summary['groups'] if (r['mode'],r['encoding'],r['cubes'],r['phase'])==('export',encoding,cubes,phase)) for phase in ['first','warm']]
  values=[]
  for r in pair:
   values.extend([f"{r['variants'][variant]['ns_per_operation_median']/1000:.3f}" for variant in ['baseline','candidate']])
   values.append(f"{r['paired_ratio_median']:.4f}")
  lines.append('| '+encoding+' | '+str(cubes)+' | '+' | '.join(values)+' |')
lines+=['','New native import pipelines have no equivalent completed baseline API. Their absolute observations retain strict receipts and every output mesh. The typed-error path likewise has a different contract from old string failures.','',
 '| New candidate path | Cubes | First µs/operation | Warm µs/operation | Warm retained Wasm linear bytes | Peak process RSS KiB |',
 '| --- | ---: | ---: | ---: | ---: | ---: |']
for mode,encoding,cubes in [('pipeline',encoding,cubes) for encoding in ['binary','ascii'] for cubes in [0,1,256]]+[('errors','none',0)]:
 pair=[next(r for r in summary['groups'] if (r['mode'],r['encoding'],r['cubes'],r['phase'])==(mode,encoding,cubes,phase))['variants']['candidate'] for phase in ['first','warm']]
 lines.append(f"| {mode} {encoding} | {cubes} | {pair[0]['ns_per_operation_median']/1000:.3f} | {pair[1]['ns_per_operation_median']/1000:.3f} | {pair[1]['retained_memory_medians']['wasm_linear_bytes']} | {pair[1]['peak_rss_kib_max']} |")
lines+=['','The error observation averages eight different failure triggers, retaining 520 host errors per process; it is not a per-variant latency claim. Wasm linear memory is reserved/high-water storage, not measured live Rust heap. The complete per-phase Node memory values and maximum observations remain in the raw rows.','',
 '| Artifact | Baseline bytes | Candidate bytes | Increase |','| --- | ---: | ---: | ---: |']
for name,row in summary['artifact_size_changes'].items():
 lines.append(f"| {name} | {row['baseline_bytes']} | {row['candidate_bytes']} | {row['delta_bytes']} |")
lines+=['','Only reachable STL failure variants receive runtime coverage here; all-feature compilation checks the remaining native match arms. The declared error union does not itself prove coverage. Existing validation categories survive, but deeper native proof witnesses and end-to-end construction provenance remain open. Other legacy WASM string errors, full release/distribution validation, and the complete pre-push workflow matrix also remain open. No push was performed.','',
 'The 35 pre-existing native files remain byte-identical and separately unadmitted: their original full-board and copper-neck runtime gates have not passed. The historical artifact audit remains failed/incomplete with 27,093 missing inherited paths. This checkpoint adds independently hashed physical evidence to 934,961 inherited expectations; it does not reverify that history or claim zero loss. Only the raw `current_work` ledger value changes; bytes and decoded keys outside it remain bound to revision `9025de62`.','',
 'See [the decision](decision-v1.json), [exact application](application-v1.json), and [checkpoint](checkpoint-v1.json) for source, command, artifact and limitation records.']
(E/'progress-v1.md').write_text('\n'.join(lines)+'\n')
print(json.dumps({'status':status,'tests':counts,'js_tests':12,'cost_processes':95,'full_plan_complete':False}),flush=True)
