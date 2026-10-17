"""Record the completed contract, allocation gates and every adverse cost row."""
import hashlib, json, re, statistics
from pathlib import Path
W=Path(__file__).resolve().parent; ROOT=W.parents[2]
E=ROOT/'docs/evidence/2026-10-16-containment-bounds'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
a=read(E/'application-v1.json')
assert a['format_exit_code']==0 and a['all_final_inputs_match_qualified_sources']
assert all(sha(ROOT/p)==h for p,h in a['implementation_sha256'].items())
validation={}
for folder in ['candidate-validation-v1','wasm-validation-v1']:
    result=read(E/folder/'result.json');assert result['complete'] and not result['unexpected_exits']
    rows=[]
    for row in read(E/folder/'commands.json'):
        assert row['exit_code']==0
        for suffix in ['stdout','stderr','resources']:
            assert sha(E/folder/(row['name']+'.'+suffix))==row[suffix+'_sha256']
        output=(E/folder/(row['name']+'.stdout')).read_text()
        counts=re.findall(r'test result: (?:ok|FAILED)\. (\d+) passed; (\d+) failed; (\d+) ignored;',output)
        warnings=[s for s in (E/folder/(row['name']+'.stderr')).read_text().splitlines() if s.startswith('warning:')]
        rows.append({'name':row['name'],'command':row['command'],'exit_code':row['exit_code'],
            'elapsed_seconds':row['elapsed_seconds'],'summaries':len(counts),
            'passed':sum(int(r[0]) for r in counts),'failed':sum(int(r[1]) for r in counts),
            'ignored':sum(int(r[2]) for r in counts),'warnings':warnings})
    validation[folder]=rows
native={r['name']:r for r in validation['candidate-validation-v1']}
assert native['containment-regressions']['passed']==29
assert (native['mesh-and-modeling-all-features']['passed'],native['mesh-and-modeling-all-features']['ignored'])==(708,8)
assert native['modeling-minimal']['passed']==161
assert all(r['failed']==0 for rows in validation.values() for r in rows)
assert '4 passed; 0 failed' in (E/'baseline-v1/native-tests.stdout').read_text()
assert '# pass 10' in (E/'baseline-v1/wasm-tests.stdout').read_text()
wasm=read(E/'wasm-validation-v1/result.json')
assert wasm['real_js_and_compiled_typescript_consumers_passed'] and wasm['public_contract_tests']==35
cost=read(E/'cost-v1/result.json')
assert cost['complete'] and cost['all_preregistered_allocation_gates_passed']
assert cost['baseline_and_candidate_all_correct_certified']
assert cost['matrix_completion_categories']=={'baseline':{'certified_completion':3360},'candidate':{'certified_completion':3360}}
adverse=[r for r in cost['paired_summary'] if r['median_elapsed_ratio']>1]
allocation_increases=[r for r in cost['paired_summary'] if any(p['allocation_calls_delta']>0 or p['allocated_bytes_delta']>0 for p in r['all_pairs'])]
memory_increases=[r for r in cost['paired_summary'] if any(p['retained_requested_bytes_delta_delta']>0 or p['peak_extra_requested_bytes_delta']>0 for p in r['all_pairs'])]
limitations=[
    'Containment and fallback arrangement still have no internal total-work, heap-byte, expression-growth or cancellation bound. The focused probe counts allocator requests and peak live requested bytes; it does not instrument scalar expression depth or coefficient growth.',
    'Five allocator-instrumented pairs establish these completed-query allocation changes; they do not establish uninstrumented production latency, a universal speedup or cross-library superiority.',
    'No new claim of end-to-end construction provenance, manifoldness or fidelity to an analytic source follows from the local containment receipt.',
    'C, C++, Python and Go do not expose this native containment query and were not rerun for this change. Real JavaScript and compiled TypeScript consumers were run.',
    'The published-package pipeline, full pre-push/CI matrix, coverage and broader properties plan remain open; nothing was pushed.',
    'The inherited artifact audit remains failed/incomplete with 27093 missing paths; fresh physical files are audited separately.',
    'The rejected 35-file native integration remains archived and excluded from runtime source; its original workload gates remain in force.',
    'The combined native package test command retains the pre-existing Cargo output-name collision warnings for the two basic examples. The same warnings occur in the qualified E32 baseline.'
]
next_steps=[
    'Continue ordinary strict completion and typed failure repairs from the preserved native regressions, using smaller changes with independent exact expectations and unchanged original workload gates.',
    'Complete end-to-end certainty/provenance and the clean external documented distribution pipeline, then the remaining properties, workflow matrix and measured coverage.'
]
decision={'status':'kept: certified supporting bounds avoid unnecessary containment arrangement',
    'main_parent_commit':a['main_parent_commit'],'implementation_sha256':a['implementation_sha256'],
    'owning_layer':'Hypermesh TriangleMesh::contains_point_decision','contract':'contract-v1.md',
    'validation':validation,'baseline_native':{'passed':4,'failed':0,'evidence':'baseline-v1/native-tests.stdout'},
    'baseline_wasm':{'passed':10,'failed':0,'evidence':'baseline-v1/wasm-tests.stdout'},
    'new_native_tests':4,'winding_wasm_tests':10,'candidate_native_focused':29,
    'typescript_scope':'geometry_certainty.ts typechecked; io_contracts.ts and mesh_errors.ts compiled and executed',
    'source_proof':'Validated finite closed PWN sources have zero winding outside their certified position bounds. A supporting plane has arbitrarily close exterior points and cannot contain strict interior. The aggregate receipt must remain Certified after the equality comparison.',
    'preserved_contract':'Strict interior of the same regularized nonzero-winding set; source validation precedes shortcuts; existing exterior comparisons, approximate receipts, exact near-coincidence, internal source faces and fallback algorithms remain.',
    'new_representation_cache_or_policy':False,'intentional_migration':None,
    'new_certified_completion_cases_separately_demonstrated':0,
    'all_final_source_inputs_match_qualified_snapshot':True,'cost':'cost-v1/result.json',
    'cost_completion_categories':cost['matrix_completion_categories'],
    'control_certified_queries':cost['control_certified_query_completions'],
    'matrix_certified_queries':6720,'allocation_gates':cost['preregistered_allocation_gates'],
    'adverse_instrumented_timing_rows':adverse,'allocation_increase_rows':allocation_increases,
    'retained_or_peak_requested_memory_increase_rows':memory_increases,
    'containment_bounds_admitted':True,'universal_set_membership_correctness_claimed':False,
    'published_package_pipeline_executed':False,'full_pre_push_matrix_executed':False,
    'complete_artifact_audit_passed':False,'historical_missing_paths':27093,
    'native_disposition_preserved':True,'full_plan_complete':False,'limitations':limitations,'next':next_steps}
dump(E/'decision-v1.json',decision)
lines=['# Certified containment bounds acceptance','',
    'Hypermesh now uses a certified global supporting plane to prove that a query cannot be strictly inside a finite closed PWN solid. Source validation still runs first. The aggregate certainty of validation, bounds and equality must be Certified; approximate equality continues through the existing fallback. The public Rust and WASM paths consume the same native result.',
    '',
    'This changes proof search without changing represented sets, source buffers, public policy, serialized geometry or the nonzero-winding contract. There is no new cache, representation or float conversion. The theorem supplies the shortcut: every neighborhood of a supporting-plane point contains points outside the source bounds and hence outside the represented solid.',
    '',
    'Before implementation, the new four-test native probe and ten-test actual JavaScript consumer both passed against the qualified E32 libraries. The tests independently check all six planes with exact 1/10^12 offsets and pi translation, an enlarged AABB that contains exterior tetrahedron points, invalid triangle mutation after warming bounds, and unresolved trigonometric coordinates that preserve qualified receipts. JavaScript checks distinguishable binary64 neighbors. These supplement the entire existing source-validation and winding corpus; no policy, fixture or assertion was weakened.',
    '',
    '| Command group | Exit | Passed | Failed | Existing ignored |',
    '| --- | ---: | ---: | ---: | ---: |']
for folder in validation.values():
    for r in folder:
        lines.append(f"| {r['name']} | {r['exit_code']} | {r['passed'] if r['summaries'] else '—'} | {r['failed'] if r['summaries'] else '—'} | {r['ignored'] if r['summaries'] else '—'} |")
lines+=['',
    'The real WASM suite passes 35 JavaScript tests, geometry-certainty TypeScript typechecking, and the compiled/executed IO and mesh TypeScript clients. Clippy denies warnings for affected all-feature/all-target, minimal/all-target and WASM configurations; affected docs deny warnings. All 2,830 final inputs match the qualified snapshot before the admission-only current_work update, and main formatting passes after applying the three paths.',
    '',
    'The unchanged control and 21-case matrix compare against the corrected E32 implementation. Five alternating fresh-process pairs complete 2,560 control and 6,720 matrix queries with independently expected values and Certified receipts on both sides. Cold construction plus query, warm queries, result materialization, source drop, allocation calls/bytes, live/peak requested bytes, process resources and binary sizes are retained. Process scalar caches are not reset between cases or policies, as in the unchanged probe.',
    '',
    'All 16 preregistered allocation gates pass. Targeted warm matrix rows require at least 50% fewer allocation calls in every pair; the tetrahedron control requires fewer calls and requested bytes in every cold/warm policy pair.',
    '',
    '| Targeted warm case | Policy | Allocation-call reduction over five pairs |',
    '| --- | --- | ---: |']
for r in cost['preregistered_allocation_gates']:
    if r['scope']=='matrix':
        reductions=r['allocation_call_reductions']
        lines.append(f"| {r['case']} | {r['policy']} | {100*min(reductions):.2f}%–{100*max(reductions):.2f}% |")
lines+=['','Control observations (candidate divided by baseline for timing):','']
for r in cost['paired_summary']:
    if r['scope']=='control':
        pairs=r['all_pairs']
        lines.append(f"- {r['policy']} / {r['phase']}: median instrumented elapsed ratio {r['median_elapsed_ratio']:.3f}; allocation-call deltas {[p['allocation_calls_delta'] for p in pairs]}; requested-byte deltas {[p['allocated_bytes_delta'] for p in pairs]}.")
lines+=['','Largest instrumented timing ratios, including every adverse row in decision-v1.json:','']
for r in sorted(cost['paired_summary'],key=lambda r:r['median_elapsed_ratio'],reverse=True)[:8]:
    lines.append(f"- {r.get('case',r['scope'])} / {r['policy']} / {r['phase']}: {r['median_elapsed_ratio']:.3f} times baseline.")
lines+=['',f"Rows with any increase in allocation calls/requested bytes: {len(allocation_increases)}. Rows with any increase in retained/peak requested bytes: {len(memory_increases)}. All pairs, source-drop observations and raw process measurements remain in [cost-v1/result.json](cost-v1/result.json).",'',
    f"Native control probe bytes: {cost['variants']['baseline']['binaries']['control']['bytes']:,} before, {cost['variants']['candidate']['binaries']['control']['bytes']:,} after. Matrix probe bytes: {cost['variants']['baseline']['binaries']['matrix']['bytes']:,} before, {cost['variants']['candidate']['binaries']['matrix']['bytes']:,} after. These are linked probe sizes, not the complete product.",'',
    'This admission is a measured allocation reduction for the same completed containment contract. Instrumented latency is reported with all unfavorable samples and is not an unqualified production speedup claim. No additional previously undecided input is credited as newly certified without separate evidence.','',
    'Remaining limits:','',*['- '+x for x in limitations],'','Next:','',*['- '+x for x in next_steps]]
(E/'progress-v1.md').write_text('\n'.join(lines)+'\n')
print(json.dumps({'focused_native':29,'all_features':708,'minimal':161,'wasm_js':35,'all_allocation_gates_passed':True,'full_plan_complete':False}),flush=True)
