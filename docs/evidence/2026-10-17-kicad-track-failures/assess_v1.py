"""Record actual completion, migration, retained failures and every measured cost."""
import hashlib,json,re
from pathlib import Path
W=Path(__file__).resolve().parent;ROOT=W.parents[2];E=ROOT/'docs/evidence/2026-10-17-kicad-track-failures'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
a=read(E/'application-v1.json');assert a['format_exit_code']==0 and a['all_final_inputs_match_qualified_sources']
assert all(sha(ROOT/p)==h for p,h in a['implementation_sha256'].items())
validation={}
for folder in ['baseline-validation-v1','candidate-validation-v3']:
    result=read(E/folder/'result.json');assert result['complete'] and not result['unexpected_exits']
    rows=[]
    for row in read(E/folder/'commands.json'):
        assert row['exit_code']==0
        for suffix in ['stdout','stderr','resources']:assert sha(E/folder/(row['name']+'.'+suffix))==row[suffix+'_sha256']
        output=(E/folder/(row['name']+'.stdout')).read_text()
        counts=re.findall(r'test result: (?:ok|FAILED)\. (\d+) passed; (\d+) failed; (\d+) ignored;',output)
        resources=(E/folder/(row['name']+'.resources')).read_text()
        measured={key:re.search(pattern,resources).group(1) for key,pattern in {
            'user_seconds':r'User time \(seconds\): ([\d.]+)',
            'system_seconds':r'System time \(seconds\): ([\d.]+)',
            'maximum_rss_kib':r'Maximum resident set size \(kbytes\): (\d+)'}.items()}
        warnings=[s for s in (E/folder/(row['name']+'.stderr')).read_text().splitlines() if s.startswith('warning:')]
        rows.append({'name':row['name'],'command':row['command'],'exit_code':0,'elapsed_seconds':row['elapsed_seconds'],
            'passed':sum(int(r[0]) for r in counts),'failed':sum(int(r[1]) for r in counts),'ignored':sum(int(r[2]) for r in counts),
            'warnings':warnings,'process_resources_including_any_build':measured})
    validation[folder]=rows
final={r['name']:r for r in validation['candidate-validation-v3']}
assert final['track-failures']['passed']==39 and final['native-tests']['passed']==1653
assert all(r['failed']==0 and not r['warnings'] for r in final.values())
assert all(final[n]['passed']==1 for n in ['full-board','copper-neck','gerber'])
before=read(E/'baseline-probe-v1/result.json');after=read(E/'candidate-probe-v1/result.json')
assert before['invalid_source_categories']=={'missing_feature':81,'successful_geometry':15,'retained_failure':0}
assert after['all_invalid_sources_rejected_typed'] and after['invalid_source_observations']==96
passed={name:set(re.findall(r'^test (\S+) \.\.\. ok$',(E/(name+'-probe-v1/original-regressions.stdout')).read_text(),re.M)) for name in ['baseline','candidate']}
assert passed['baseline']<=passed['candidate']
resolved=sorted(passed['candidate']-passed['baseline'])
assert resolved==['coincident_track_endpoints_keep_the_width_disk']
cost=read(E/'comparison-v1/result.json');assert cost['complete'] and cost['all_previously_materialized_features_identical']
assert not cost['unexpected_report_differences'] and len(cost['malformed_cli'])==32
assert len(cost['geometry_additions']['hvp109a'])==5 and not cost['geometry_additions']['cparti'] and not cost['geometry_additions']['ordinary64']
assert len(cost['independent_full_board_disk_checks'])==15
adverse=[r for r in cost['paired_summary'] if r['median_elapsed_ratio']>1 or any(p[k]>0 for p in r['all_pairs'] for k in ['allocation_calls_delta','allocated_bytes_delta','retained_requested_bytes_delta_delta','peak_extra_requested_bytes_delta'])]
sizes=cost['sizes'];size_delta={k:sizes['candidate'][k]-sizes['baseline'][k] for k in ['cli_bytes','probe_bytes']}
retention_rationale=(
    'Retain for explicit malformed-source failure and exact coincident-disk completion, above performance, memory and size in the governing ladder. '
    'The unchanged ordinary64 instrumented samples show slower median loading; the unchanged CPArti samples require more allocation calls and bytes. '
    'All adverse observations and artifact growth are retained. Previously materialized output is preserved and both original full-board resource gates pass. '
    'The additional HVP109A geometry changes the completed workload. No speedup or lower-resource claim is made.')
migration={
    'source_failure':'Required start/end/width fields are validated exactly; missing, malformed, duplicated, extra-valued or negative widths return a typed TrackLoadError through the existing anyhow Result instead of omitted/defaulted/truncated geometry. Shared exhaustion is preserved as BudgetExceeded. No partial board is published.',
    'coincident_tracks':'Exact rational endpoint equality selects the existing native circle constructor with radius width/2. Five formerly omitted HVP109A tracks now form certified disks; finite coincidence of distinct source endpoints remains an explicit failure.',
    'other_positive_track_failures':'Valid positive source that the legacy finite adapter cannot construct is an explicit unsupported/native-geometry failure; it is not counted as certified geometric completion.',
    'version':'Readiness CHECK_IMPLEMENTATION_VERSION 20 to 21. All raw selected CLI report differences are recorded and confined to this field on the measured corpus.',
    'legacy_tests':'Two complete original fixtures are byte-identical. One now requires the exact disk; the incomplete fixture requires typed MissingField(end) and separately preserves its pad/via/zone checks. Their original failed expectations remain in candidate v1.',
    'archived_carrier':'The unchanged 26-test native-stroke file expects failed regions for some malformed/budget cases; the new public contract returns a load error instead. Its complete result and first failures remain visible.'}
limitations=[
    'Noncoincident tracks still use the legacy finite rectangular adapter. General round caps, exact source-coordinate fidelity and extreme noncoincident strokes remain open; local predicate certainty does not certify that source fidelity.',
    'The new typed failure boundary covers straight-track start/end/width and their construction. Other object families, layer/net validation, the full error taxonomy, end-to-end provenance, and structured machine-readable CLI errors remain open.',
    'Input parsing, exact coefficient growth, total work, heap bytes and cancellation have no new internal bound. Shared work-budget checks preserve existing denial, but do not bound parsing. Runs use external time/address-space limits; allocator requests and process RSS do not measure expression depth or prove a heap-byte bound.',
    'HVP109A loads now include five additional disks. Their costs are retained as changed-workload observations, not a same-contract speed comparison. Instrumented matched samples for unchanged inputs do not establish uninstrumented production latency or a universal performance claim.',
    'The archived native-stroke tests stop at their first failed assertion; no credit is claimed for later unexecuted checks. Their remaining failures and the rejected 35-file integration remain preserved.',
    'The published-package user path, minimum compiler, complete pre-push/CI matrix, all-property audit and measured coverage remain incomplete. No push was performed; KiCad loading has no affected C/JS/WASM binding consumer in this patch.',
    'The inherited physical artifact audit remains failed/incomplete with 27093 missing paths. Fresh paths are audited independently without a whole-history or zero-loss claim.',
    'prepare_candidate_v2.py printed only its two test-path variables in the short terminal summary; its result.json correctly records all five changed paths and the complete source hashes. The inaccurate short summary is preserved, not used as a source inventory.'
]
next_steps=[
    'Continue ordinary strict native-stroke completion and exact source fidelity using the preserved original regressions and unchanged workload gates; do not restore the rejected integration wholesale.',
    'Extend visible failures and provenance through the remaining modeling/format boundaries, then complete clean external distribution, release/workflow validation and the remaining properties.'
]
decision={'status':'kept: typed track-load failures and exact coincident disks preserve full-board completion',
    'main_parent_commit':a['main_parent_commit'],'implementation_sha256':a['implementation_sha256'],
    'owning_layer':'Hyperdrc KiCad source adapter; existing geometry::circle_region owns disk construction',
    'properties':['P1','P6','P8','P9','P17','P18','P19','P20','P25','P26','P30','P35','P36','P37','P39'],
    'contracts':['contract-v2.md','contract-v3.md','contract-v4.md','cost-contract-v2.md'],
    'validation':validation,'new_public_tests':39,'full_native_suite_passed':1653,'separate_original_workload_gates_passed':3,
    'invalid_source_baseline':before['invalid_source_categories'],'typed_invalid_source_candidate':after['invalid_source_categories'],
    'actual_invalid_cli_cases_passed':32,'original_native_regressions':{'baseline':before['original_regressions'],'candidate':after['original_regressions'],'newly_completed':resolved,'previous_passes_preserved':True},
    'independent_full_board_disk_observations':15,'new_hvp109a_disks':5,'all_previously_materialized_features_identical':True,
    'comparison':'comparison-v1/result.json','all_cost_rows':cost['paired_summary'],'adverse_cost_rows':adverse,'artifact_size_delta_bytes':size_delta,
    'retention_rationale':retention_rationale,
    'raw_report_differences':cost['raw_report_differences'],'intentional_migration':migration,
    'preserved_failed_attempts':{'candidate-v1':['two silent-omission expectations','HVP109A load rejection in both original board gates','ignored Result in via test'],
        'candidate-v2':['missing imports prevented unit/workload compilation; 39 public tests and docs passed']},
    'track_load_failures_admitted':True,'general_exact_stroke_fidelity_claimed':False,'performance_improvement_claimed':False,
    'published_package_pipeline_executed':False,'full_plan_complete':False,'native_disposition_preserved':True,
    'independently_reimplemented_archived_path':a['independently_reimplemented_archived_path'],
    'limitations':limitations,'next':next_steps}
dump(E/'decision-v1.json',decision)
lines=['# Typed KiCad track failures and coincident disks','',
    'Retained a six-file change after current-revision reproduction and complete final HyperDRC qualification. The KiCad adapter returns typed source, unsupported-geometry, native-geometry, or budget failures through the existing loader Result. It publishes no partial board. Exact zero width remains empty after all required source fields validate.','',
    'The original HVP109A workload exposed five positive-width tracks with exactly coincident endpoints. They now reuse the native circle constructor. Independent Python Fraction extraction and 15 strict→approximate→strict checks establish their exact centers, radii, areas, point membership and native bounds. Previously materialized feature metadata and finite report coordinate bits remain identical across all measured inputs. This is a deliberate geometry correction and version-21 evidence migration, not a performance-only patch.','',
    'Final validation: 39 new public tests; 1,653 tests in the complete native run; all three original full-board, copper-neck and Gerber gates separately pass; warning-denied all-target Clippy, docs and formatting pass. The 39 focused tests overlap the full run. All 96 malformed-source observations now fail typed; 32 actual CLI cases reject input even with --allow-findings and emit no success report.','',
    f'The unchanged original 26 stroke regressions move from {before["original_regressions"]["passed"]} passed/{before["original_regressions"]["failed"]} failed to {after["original_regressions"]["passed"]} passed/{after["original_regressions"]["failed"]} failed. Only the coincident-track disk test is newly completed. Failed-region versus load-error expectations are explicitly different contracts; remaining failures stay archived.','',
    'Five alternating pairs retain both load phases, all allocation requests, retained/peak requested bytes, process resources, output snapshots and raw CLI reports. Only unchanged-output fixtures support matched comparisons. HVP109A includes five new disks; its ratios are descriptive changed-workload observations. No speedup is claimed.','',
    '| Input | Phase | Matched contract | Median candidate/baseline time | Allocation-call deltas (all pairs) |',
    '| --- | --- | --- | ---: | --- |']
for row in cost['paired_summary']:
    lines.append(f'| {row["fixture"]} | {row["phase"]} | {row["matched_completed_contract"]} | {row["median_elapsed_ratio"]:.4f} | '+', '.join(str(p['allocation_calls_delta']) for p in row['all_pairs'])+' |')
lines+=['',f'Observed CLI size delta: {size_delta["cli_bytes"]:+d} bytes; allocator-probe delta: {size_delta["probe_bytes"]:+d} bytes. Every adverse row is retained in decision-v1.json and comparison-v1/result.json.','',retention_rationale,'',
    'The first candidate rejected the HVP109A disks and failed the original workload gates. The second introduced the disk construction but missed private-test imports. Both full attempts remain frozen. The final target reruns all affected checks; no completed Cargo target was reopened.','',
    'Limits and remaining work:','',*['- '+s for s in limitations],'',*['- '+s for s in next_steps]]
(E/'progress-v1.md').write_text('\n'.join(lines)+'\n')
print(json.dumps({'new_public_tests':39,'full_native_tests':1653,'original_gates':3,'typed_invalid_observations':96,'native_disks':5,'full_plan_complete':False}),flush=True)
