"""Record kernel completion separately from unchanged source-adapter gaps."""
import hashlib,json,re
from pathlib import Path
W=Path(__file__).resolve().parent;ROOT=W.parents[2]
E=ROOT/'docs/evidence/2026-10-17-native-stroke-completion'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
def test_counts(p):
    rows=re.findall(r'test result: (?:ok|FAILED)\. (\d+) passed; (\d+) failed; (\d+) ignored;',Path(p).read_text())
    return {name:sum(int(row[i]) for row in rows) for i,name in enumerate(['passed','failed','ignored'])}
def test_names(p,status):return set(re.findall(r'^test (\S+) \.\.\. '+status+r'$',Path(p).read_text(),re.M))
a=read(E/'application-v1.json');assert a['format_exit_code']==0 and a['all_final_inputs_match_qualified_sources']
assert all(sha(ROOT/p)==h for p,h in a['implementation_sha256'].items())
validation={}
for folder,expected in [('candidate-validation-v1',['original-curve-regressions']),('candidate-validation-v2',[]),('candidate-validation-v4',['curve-all','circuit-all','clippy']),('consumer-validation-v1',['circuit-all'])]:
    result=read(E/folder/'result.json')
    assert result['complete'] and result['unexpected_exits']==expected
    rows=[]
    for row in read(E/folder/'commands.json'):
        for suffix in ['stdout','stderr','resources']:assert sha(E/folder/(row['name']+'.'+suffix))==row[suffix+'_sha256']
        resources=(E/folder/(row['name']+'.resources')).read_text()
        measured={key:re.search(pattern,resources).group(1) for key,pattern in {
            'user_seconds':r'User time \(seconds\): ([\d.]+)',
            'system_seconds':r'System time \(seconds\): ([\d.]+)',
            'maximum_rss_kib':r'Maximum resident set size \(kbytes\): (\d+)'}.items()}
        warnings=[s for s in (E/folder/(row['name']+'.stderr')).read_text().splitlines() if s.startswith('warning:')]
        rows.append({'name':row['name'],'command':row['command'],'exit_code':row['exit_code'],'elapsed_seconds':row['elapsed_seconds'],
            **test_counts(E/folder/(row['name']+'.stdout')),'warnings':warnings,'process_resources_including_any_build':measured})
    validation[folder]=rows
consumer={r['name']:r for r in validation['consumer-validation-v1']}
assert len(consumer)==9
assert {n:r['exit_code'] for n,r in consumer.items() if r['exit_code']}=={'circuit-all':101}
assert all(r['failed']==0 for n,r in consumer.items() if n!='circuit-all')
assert (consumer['circuit-all']['passed'],consumer['circuit-all']['failed'])==(341,7)
assert consumer['text-fabrication']['passed']==2 and not consumer['circuit-docs']['warnings']
final={r['name']:r for r in validation['candidate-validation-v4']}
assert {n:r['exit_code'] for n,r in final.items() if r['exit_code']}=={'curve-all':124,'circuit-all':101,'clippy':101}
incomplete_curve_attempt=final['curve-all']
assert incomplete_curve_attempt['failed']==0
for name in ['format','curve-all','circuit-all','clippy','native-cli']:final[name]=consumer[name]
reuse=read(E/'v4-qualification-reuse-v2/result.json')
assert reuse['complete'] and len(reuse['reused_completed_observations'])==9
text_baseline=read(E/'circuit-baseline-v1/result.json')
assert text_baseline['baseline_compile_failure_reproduced'] and text_baseline['exit_code']==101
before_fabrication=(W/'candidate-source-v4/crates/hypercircuit/tests/fabrication.rs').read_text()
after_fabrication=(ROOT/'crates/hypercircuit/tests/fabrication.rs').read_text()
assert before_fabrication.split('    let rejected = layout',1)[0]==after_fabrication.split('    // A selected but invalid font',1)[0]
text_consumer={
    'status':'kept: complete the typed text-constructor migration in public PCB materialization',
    'owning_layer':'Hypercircuit materialization adapter',
    'baseline':'The unchanged b1373812 snapshot fails compilation because a Result<CurveRegion2, ValidationError> is passed to a transform expecting CurveRegion2.',
    'baseline_evidence':'circuit-baseline-v1/result.json',
    'migration':'A selected invalid font or missing glyph now returns GeometryMaterializationError::Construction with its typed source. It cannot publish a partial successful report with a TextFontRejected omission. Absence of an optional production font policy remains an explicit TextArtwork omission.',
    'certainty':'Use truetype_text_with_context and the existing consume_geometry receipt accumulator before rotation and translation. Selecting APPROXIMATE_512 alone does not downgrade successful exact text.',
    'public_regressions':'Preserve every original valid fabrication assertion; test invalid font and missing B after valid A under STRICT -> APPROXIMATE_512 -> STRICT, retain the typed error source, then recover with valid text and Certified evidence.',
    'validation':'consumer-validation-v1',
    'same_completed_baseline_performance_comparison_available':False,
    'performance_limitation':'Current baseline does not compile this consumer. Complete native test/Clippy/doc command resource observations and built artifacts are retained; no text-path speedup or before/after allocation claim is made.'}
built_text_artifacts=[]
for line in (E/'consumer-validation-v1/circuit-build.stdout').read_text().splitlines():
    if not line.startswith('{'):continue
    row=json.loads(line)
    if row.get('reason')=='compiler-artifact' and row['target']['name']=='hypercircuit' and row.get('executable'):
        path=Path(row['executable'])
        built_text_artifacts.append({'path':str(path),'sha256':sha(path),'bytes':path.stat().st_size,
            'target_kind':row['target']['kind'],'test_profile':row['profile']['test']})
assert built_text_artifacts
text_consumer['built_artifacts']=built_text_artifacts
assert len(final)==14 and all(r['exit_code']==0 and r['failed']==0 for n,r in final.items() if n!='circuit-all')
assert final['circuit-all']['exit_code']==101 and final['circuit-all']['failed']==7
snapshot_result=read(E/'snapshot-requalification-v1/result.json')
assert snapshot_result['complete'] and snapshot_result['all_seven_prior_failures_replayed_successfully']
snapshot_rows=[]
for row in read(E/'snapshot-requalification-v1/commands.json'):
    assert row['exit_code']==0
    for suffix in ['stdout','stderr','resources']:assert sha(E/'snapshot-requalification-v1'/(row['name']+'.'+suffix))==row[suffix+'_sha256']
    snapshot_rows.append({'name':row['name'],'command':row['command'],'exit_code':row['exit_code'],
        'elapsed_seconds':row['elapsed_seconds'],**test_counts(E/'snapshot-requalification-v1'/(row['name']+'.stdout'))})
assert len(snapshot_rows)==5
assert [(r['name'],r['passed'],r['failed']) for r in snapshot_rows if r['name'] in ['easyduino','workflow']]==[('easyduino',9,0),('workflow',9,0)]
failed_target_counts=re.findall(r'test result: FAILED\. (\d+) passed; (\d+) failed;', (E/'consumer-validation-v1/circuit-all.stdout').read_text())
assert failed_target_counts==[('3','6'),('8','1')]
unaffected_circuit_passes=consumer['circuit-all']['passed']-sum(int(r[0]) for r in failed_target_counts)
assert unaffected_circuit_passes==330
metadata=read(E/'metadata-migration-v2/result.json')
assert metadata['complete'] and metadata['geometry_findings_and_other_artifacts_unchanged']
assert len(metadata['fixtures'])==7 and all(sha(ROOT/r['fixture'])==r['actual_sha256'] for r in metadata['fixtures'])
metadata_migration={
    'status':'kept: synchronize seven expected artifacts with committed readiness implementation version 21',
    'evidence':'metadata-migration-v2/result.json','differences':'artifact-mismatches-v1/result.json',
    'old_versions':{'six_easyduino_reports':20,'manufacturing_release_drc_evidence':1},'new_version':21,
    'independent_derivation':'Exact check-version byte substitutions, recomputed DRC length/SHA-256 and manifest core SHA-256, and independent ZIP reconstruction reproduce actual output byte-for-byte. All other report fields, findings, geometry, fabrication outputs and entry metadata are unchanged.',
    'fixtures':metadata['fixtures'],'test_bodies_and_original_source_fixtures_unchanged':True,
    'requalification':'snapshot-requalification-v1/result.json','replayed_targets':snapshot_rows,
    'original_cargo_suite_preserved':{'passed':341,'failed':7,'exit_code':101},
    'final_corpus_coverage':{'unchanged_other_target_passes':unaffected_circuit_passes,'complete_replayed_target_passes':18,'covered_configured_tests':348},
    'whole_cargo_suite_relabelled_as_success':False}
assert final['stroke-regressions']['passed']==10
assert all(final[n]['passed']==1 for n in ['full-board','copper-neck','gerber'])
assert not final['clippy']['warnings'] and not final['docs']['warnings']
for name in ['full-board','copper-neck','gerber']:
    combined='\n'.join((E/'candidate-validation-v4'/(name+'.'+suffix)).read_text() for suffix in ['stdout','stderr'])
    assert 'skipping complex project fixture' not in combined
gate_report=(E/'candidate-validation-v4/full-board.stdout').read_text()
baseline_gate_report=(ROOT/'docs/evidence/2026-10-17-kicad-track-failures/candidate-validation-v3/full-board.stdout').read_text()
for board in ['CPArti FPGA dev board.kicad_pcb','HVP109A.kicad_pcb']:assert board in gate_report
gate_counts=lambda text:re.findall(r'(\d+) violation\(s\) found, (\d+) waived:',text)
assert gate_counts(gate_report)==gate_counts(baseline_gate_report)==[('12','0'),('201','0')]
failed_ui=read(E/'ui-validation-v1/result.json')
assert failed_ui['complete'] and failed_ui['unexpected_exits']==['ui-tests','ui-clippy','ui-wasm']
failed_ui_rows=read(E/'ui-validation-v1/commands.json')
assert len(failed_ui_rows)==3 and all(r['exit_code']==101 for r in failed_ui_rows)
assert all('current package believes it\'s in a workspace when it\'s not' in (E/'ui-validation-v1'/(r['name']+'.stderr')).read_text() for r in failed_ui_rows)
assert read(E/'ui-isolation-v1/result.json')['all_final_v6_inputs_byte_identical']
ui=read(E/'ui-validation-v2/result.json');assert ui['complete'] and not ui['unexpected_exits']
ui_rows=[]
for row in read(E/'ui-validation-v2/commands.json'):
    assert row['exit_code']==0
    for suffix in ['stdout','stderr','resources']:assert sha(E/'ui-validation-v2'/(row['name']+'.'+suffix))==row[suffix+'_sha256']
    ui_rows.append({'name':row['name'],'command':row['command'],'exit_code':row['exit_code'],
        'elapsed_seconds':row['elapsed_seconds'],**test_counts(E/'ui-validation-v2'/(row['name']+'.stdout'))})
assert len(ui_rows)==3 and ui_rows[0]['passed']==42 and ui_rows[0]['failed']==0 and ui_rows[0]['ignored']==0
before=E/'reproduction-v2/curve_regressions.stdout';after=E/'candidate-validation-v4/stroke-regressions.stdout'
baseline_curve=test_counts(before);assert baseline_curve=={'passed':6,'failed':2,'ignored':0}
old_passes=test_names(before,'ok');new_passes=test_names(after,'ok')
assert old_passes<=new_passes
resolved=sorted(test_names(before,'FAILED')&new_passes);assert len(resolved)==2
baseline_added=test_counts(E/'baseline-contracts-v1/contracts.stdout')
assert baseline_added=={'passed':1,'failed':1,'ignored':0}
baseline_drc=test_counts(E/'reproduction-v2/drc_regressions.stdout');assert baseline_drc=={'passed':5,'failed':21,'ignored':0}
rejected=test_counts(E/'candidate-validation-v1/original-curve-regressions.stdout')
assert rejected=={'passed':4,'failed':4,'ignored':0}
cost=read(E/'comparison-v1/result.json');assert cost['complete'] and not cost['candidate_incomplete_cases']
assert cost['exact_scalar_serialization_errors']==0 and not any(cost['raw_full_board_report_differences'].values())
assert len(cost['all_samples'])==240 and len(cost['paired_summary'])==24
adverse=[r for r in cost['paired_summary'] if r['median_elapsed_ratio']>1 or any(p[k]>0 for p in r['all_pairs'] for k in ['allocation_calls_delta','allocated_bytes_delta','retained_requested_bytes_delta_delta','peak_extra_requested_bytes_delta'])]
size_delta={k:cost['sizes']['candidate'][k]-cost['sizes']['baseline'][k] for k in ['cli_bytes','probe_bytes']}
completed=[r for r in cost['paired_summary'] if r['matched_completed_semantic_contract']]
expanded=[r for r in cost['paired_summary'] if r['candidate_always_completes'] and not r['baseline_always_completes']]
structure=read(E/'structure-metrics-v1/result.json');assert structure['complete'] and structure['phase_observations']==240
cost_review=read(E/'cost-review-v1.json');assert cost_review['complete']
assert cost_review['comparison_sha256']==sha(E/'comparison-v1/result.json') and cost_review['structure_sha256']==sha(E/'structure-metrics-v1/result.json')
assert cost_review['runner_sha256']==sha(E/'review_cost_v1.py')
projects=read(E/'project-reports-v1/result.json');assert projects['complete'] and projects['all_report_values_identical']
migration={
    'representation':'Certified distinct single-line butt/round strokes now retain two native parallels and two endpoint caps before regularization. Boundary subdivision/order and equivalent scalar expression forms may differ from earlier general arrangement output. Mathematical area, bounds and point-set semantics have independent exact oracles; artifact equality is separate.',
    'unchanged_domain':'Positive half-width, connectivity and corner-style validation remain before the branch. Unresolved endpoint distinctness, square caps and multiple spans retain their existing general route. No global area formula, source parser or finite projection changes.',
    'archived_components':'The private helper and eight original regressions are byte-identical archived components, independently evaluated on the current stack. The full rejected 35-file integration remains rejected.',
    'source_adapter':'KiCad noncoincident tracks still use the legacy finite rectangular adapter. This kernel change does not claim exact round-cap source ingress or resolve the 21 observed original DRC failures.',
    'typed_text_consumer':text_consumer['migration'],
    'readiness_expected_artifacts':metadata_migration['independent_derivation'],
    'report_artifacts':'Selected CPArti and HVP109A CLI JSON reports are compared in both standalone board-file and complete extracted-project contexts, independently of native boundary snapshots. The full-board test visits both projects in one process; CLI observations use separate processes. No report migration is intended in this kernel patch.'}
limitations=[
    'The input contract is a single certified distinct line with positive half-width and butt/round caps. General square, multispan and unsupported symbolic completion are not expanded. Finite-grid membership is an independent finite oracle, not a proof of all points or all source expressions.',
    'The retained parallels and certified cap radius/sweep preserve construction evidence used by existing kernel consumers. This does not establish complete end-to-end provenance for every transformation, serialization, import or public binding, or a universal P15/P19 closure proof.',
    'Native KiCad source fidelity and the finite line_polygon adapter remain open. The full original 26-test DRC reproduction is retained at 5 passes and 21 failures; two old expectations use the superseded failed-region carrier. No credit is assigned to later unexecuted assertions.',
    'Existing work-budget denial/recovery and zero-refinement constructions are exercised. Retained output serialization supplies expression-tree depth/node and integer/rational coefficient-size observations; it expands shared expressions and omits caches, so it does not measure unique live graph nodes or peak intermediate growth. No new internal total-work, heap-byte, expression-growth or cancellation bound is implemented. External containment, allocator requests and process RSS do not prove those bounds.',
    'The measured operation includes exact source/path construction, stroke, area, bounds, native output, reconstruction, independent point queries and snapshot Value materialization. It excludes process startup, JSON encoding/file IO and final model destruction. Instrumented samples do not establish uninstrumented production latency or a universal speedup.',
    'Ratios for baseline incomplete cases are changed-completion observations, not faster successful operations. All unfavorable time/allocation/retention/size rows are retained. Cache warming may strengthen facts; no identical cold/warm failure-history requirement is imposed.',
    'Affected Hypercurve, CSGRS, HyperDRC and Hypercircuit native consumers are exercised. Nine passing v4 observations retain byte-identical dependency inputs; the dependency proof is in v4-qualification-reuse-v2. The final v6 snapshot adds the typed text consumer, its acceptance tests and seven independently derived readiness-metadata expectations to the five unchanged stroke files. The original Hypercircuit Cargo suite remains recorded as 341 passes, seven artifact failures and exit 101; complete replays of both affected targets pass 18 tests, alongside 330 unaffected passing tests. No failed command is relabelled as success. The direct demo consumer is covered by its release test suite, strict release Clippy and release WASM build. This is not a browser interaction test. No direct stroke_path binding exposure was found in the root C/JS/WASM APIs. Published-package consumer, minimum compiler, complete pre-push/CI debug combinations, coverage and all-property validation remain open. No push is performed.',
    'The inherited physical artifact audit remains failed/incomplete with 27093 missing paths. New physical paths are audited separately without an inode-deduplicated, whole-history or zero-loss claim.'
]
next_steps=[
    'Use the retained exact kernel construction to evaluate a narrow KiCad noncoincident-track adapter change, preserving typed source/load errors, complete source fixtures and unchanged full-board workload limits.',
    'Continue the exact finite-adapter and ordinary strict-success frontier, then end-to-end certainty/provenance, clean external distribution and the remaining properties and workflow validation.'
]
rationale=('Retain for new certified diagonal and symbolic downstream completion while preserving the original strict-success cases and workload gates. '
    'The area-only candidate is rejected because it loses previously available proof completion. The retained construction uses existing line, arc, contour and regularization machinery at the curve layer. '
    'The downstream text consumer also completes the current typed constructor migration, preserving policy, certainty and errors. This is a completeness change with explicit boundary and text-failure migrations, not a performance-only patch; no universal speedup is claimed.')
failed_attempts={
    'reproduction-v1':'Missing hyperreal extern prevented test compilation; no tests executed.',
    'reproduction-v2':'All original curve/DRC tests executed; the following direct diagnostic failed compilation because a LineSeg2-to-Segment2 From implementation does not exist. Corrected in direct-observation-v2.',
    'candidate-v1':'Area-only circular shortcut loses two previously passing cases and leaves both diagonal failures; rejected with 4 passes and 4 failures.',
    'contracts-observation-v1':'Wrong dependency search directory prevented compilation; corrected v2 passes both added contracts.',
    'candidate-preparation-v3':'Formatting-only comparison against standalone rustfmt failed. Preserved source was not built; corrected v4 records Cargo-formatted inputs.',
    'cost-preflight-v1':'Missing borrow on owned arc radius_squared prevented compilation; corrected v2 preflight passes. Preflight timing is excluded from benchmark evidence.',
    'candidate-validation-v4 curve-all':'External 1800-second combined build/test timeout after 2272 passing test lines and no assertion failure; compilation took 7 minutes 57 seconds. The complete suite is requalified from the same source in a fresh target with compilation and execution bounded separately. Every test, policy and original board-operation limit is retained.',
    'candidate-validation-v4 circuit-all and clippy':'Both fail on the text consumer type mismatch; the unchanged baseline reproduces that exact compiler error. The typed migration restores compilation and strict Clippy. The subsequent complete suite exposes seven stale artifact expectations, whose independent audit and complete target replays are recorded separately.',
    'consumer-validation-v1 circuit-all':'Complete run has 341 passes and seven stale readiness-metadata artifact failures. All fields and archive entries are independently audited before the explicit expectation migration; both unmodified test targets are replayed completely.',
    'metadata-migration-v1':'Verification expected spaced JSON tokens in the compact archive DRC evidence. No fixture was modified; corrected v2 independently verifies all bytes and digests.',
    'qualification-tail-v3':'Stops before dependent work because consumer-v1 retains the seven snapshot failures. Tail v4 requires their complete replay before UI or measurement.',
    'ui-validation-v1 and qualification-tail-v4':'Both complete Hypercircuit target replays pass, then all three demo commands fail before compilation because the nested snapshot discovers the outer workspace. The tail stops before measurements. Live and isolated Cargo metadata establish the harness-location cause; an identical external source copy and fresh target requalify the unchanged demo commands in ui-validation-v2. No manifest, lock, test or production input is changed.',
    'qualification-tail-v1 and v2':'Both prerequisite guards stop before dependent build or cost work because v4 has curve-all, circuit-all and clippy failures. Terminal session observations are retained. Tail v3 subsequently stops on the stale artifact expectations; tail v4 requires their complete replay before demo and measurements.'}
decision={'status':'kept: exact native single-line construction preserves downstream proof completion',
    'main_parent_commit':a['main_parent_commit'],'implementation_sha256':a['implementation_sha256'],
    'owning_layer':'Hypercurve single-line stroke construction; existing LineSeg2, CircularArc2, Contour2 and CurveRegion2 regularization own geometric invariants',
    'properties':['P1','P2','P3','P4','P6','P8','P9','P12','P13','P14','P15','P16','P17','P18','P19','P20','P25','P27','P35','P36','P37','P39'],
    'contracts':['contract-v1.md','contract-v2.md','contract-v3.md','cost-contract-v1.md','validation-requalification-contract-v1.md','circuit-contract-v1.md','metadata-contract-v1.md','ui-isolation-contract-v1.md'],
    'construction_review':'construction-review-v1.md',
    'validation':validation,'demo_validation':ui_rows,'incomplete_combined_curve_attempt':incomplete_curve_attempt,
    'failed_demo_validation':failed_ui_rows,'demo_workspace_isolation':'ui-isolation-v1/result.json',
    'complete_curve_requalification':{'directory':'consumer-validation-v1','command':'curve-all','exit_code':0},
    'readiness_metadata_migration':metadata_migration,
    'unchanged_dependency_qualification':'v4-qualification-reuse-v2/result.json',
    'typed_text_consumer_migration':text_consumer,
    'original_curve_regressions':{'baseline':baseline_curve,'candidate':{'passed':8,'failed':0,'ignored':0},'newly_completed':resolved,'previous_passes_preserved':True},
    'added_contract_regressions':{'baseline':baseline_added,'candidate':{'passed':2,'failed':0,'ignored':0}},
    'original_drc_reproduction':baseline_drc,'source_adapter_completion_claimed':False,
    'rejected_area_candidate':rejected,'separate_original_workload_gates_passed':3,
    'full_board_gate_fixtures_observed':['CPArti FPGA dev board.kicad_pcb','HVP109A.kicad_pcb'],
    'full_board_gate_report_counts':gate_counts(gate_report),'fixture_skip_diagnostics':False,
    'comparison':'comparison-v1/result.json','matched_completed_case_cap_phase_rows':len(completed),'expanded_completion_case_cap_phase_rows':len(expanded),
    'retained_expression_structure':'structure-metrics-v1/result.json',
    'cost_and_artifact_review':'cost-review-v1.json','reviewed_completion':cost_review['completion'],
    'all_cost_rows':cost['paired_summary'],'adverse_cost_rows':adverse,'artifact_size_delta_bytes':size_delta,
    'artifact_differences':cost['artifact_differences'],'repeated_snapshot_differences':cost['repeated_snapshot_differences'],
    'standalone_board_file_report_differences':cost['raw_full_board_report_differences'],'intentional_migration':migration,
    'complete_project_cli_reports':'project-reports-v1/result.json','complete_project_cli_observations':projects['reports'],
    'independently_admitted_archived_components_sha256':a['independently_admitted_archived_components_sha256'],
    'preserved_failed_attempts':failed_attempts,'retention_rationale':rationale,'limitations':limitations,'next':next_steps,
    'single_line_kernel_admitted':True,'general_exact_kicad_stroke_fidelity_claimed':False,'performance_improvement_claimed':False,
    'full_archived_integration_admitted':False,'native_disposition_preserved':True,'published_package_pipeline_executed':False,'full_plan_complete':False}
dump(E/'decision-v1.json',decision)
lines=['# Exact single-line strokes remain consumable','',rationale,'',
    'Current-revision reproduction finds six of the original eight curve regressions passing and two unable to certify diagonal analytic area. The additional translated/reversed symbolic regression also fails; its extreme-coordinate companion already passes. Final construction passes all eight unchanged archived regressions and both additional contracts. Test bodies stop at the first failure; baseline coverage beyond that point is not credited.','',
    'The private native helper and original eight tests are byte-identical archived components. Their independent admission does not reverse the rejection of the full 35-file integration. The earlier area-only candidate is rejected because it reduces the original suite to four passes.','',
    'The text consumer failed compilation on the unchanged baseline. It now passes the selected materialization context to the existing text constructor, consumes its certainty receipt, and propagates the typed error before transforms. The full valid fabrication fixture is preserved. Invalid selected fonts and missing glyphs now fail materialization instead of publishing omitted artwork as success; tests cover a valid prefix followed by a missing glyph and recovery under strict/approximate/strict policy. This failure-contract migration is documented on ProductionTextPolicy.','',
    '| Final command | Passed | Failed | Ignored | Exit |',
    '| --- | ---: | ---: | ---: | ---: |']
for r in final.values():lines.append(f'| {r["name"] + (" (v5; preserved snapshot failures)" if r["name"]=="circuit-all" else "")} | {r["passed"]} | {r["failed"]} | {r["ignored"]} | {r["exit_code"]} |')
for name in ['circuit-build','text-fabrication','curve-all-build','circuit-docs']:
    r=consumer[name];lines.append(f'| {r["name"]} | {r["passed"]} | {r["failed"]} | {r["ignored"]} | {r["exit_code"]} |')
for r in snapshot_rows:lines.append(f'| snapshot-{r["name"]} | {r["passed"]} | {r["failed"]} | {r["ignored"]} | {r["exit_code"]} |')
for r in ui_rows:lines.append(f'| {r["name"]} | {r["passed"]} | {r["failed"]} | {r["ignored"]} | {r["exit_code"]} |')
lines+=['','The first demo attempt stopped before compilation because the nested snapshot discovered the outer workspace. Offline Cargo metadata verified that the unchanged live demo and an identical external source copy each resolve their own workspace. The external copy passes all 42 tests, strict Clippy and the release WASM build in a fresh target. The three original failed commands and prerequisite stop remain preserved; no production manifest or lock was edited.']
lines+=['','The all-features row above is the complete consumer-validation-v1 execution. The earlier v4 combined build/test command timed out after 2272 passing test lines without an assertion failure and remains preserved as incomplete. V4 Hypercircuit and combined Clippy also failed on the independently reproduced text-consumer type mismatch. The final target builds first and then runs the unchanged complete curve test command, with the same 1800-second containment on each stage. Nine completed v4 observations have byte-identical dependency inputs, verified separately; the final snapshot differs in two Hypercircuit source/test files and seven expected artifacts. The consumer suite subsequently exposed stale readiness versions in six reports and one archive: its 341 passes/seven failures remain recorded. An independent byte/digest audit justifies the metadata migration; complete unchanged easyduino and workflow replays pass all 18 tests, including the formerly unreached assertions. Together with 330 unaffected successes, these cover all 348 configured Hypercircuit tests. No nonzero Cargo exit is relabelled as a passing command.','',
    'Configuration totals and focused tests overlap; they are not additive unique coverage. Full-board worker CPU remains capped at 120 seconds and the copper-neck check body at 60 seconds. Complete original archives and gate assertions are unchanged.','',
    'Five alternating pairs per six exact inputs and two cap styles retain 240 phase observations. Independent integer point-to-segment formulas, analytic area and extent formulas check both the original and rebuilt regions. All exact scalar snapshots, native segments, classifications, report outputs and measured artifact differences are retained.','',
    '| Input | Cap | Phase | Both complete | Median candidate/baseline time | Allocation-call deltas |',
    '| --- | --- | --- | --- | ---: | --- |']
for r in cost['paired_summary']:
    lines.append(f'| {r["label"]} | {r["cap"]} | {r["phase"]} | {r["matched_completed_semantic_contract"]} | {r["median_elapsed_ratio"]:.4f} | '+', '.join(str(p['allocation_calls_delta']) for p in r['all_pairs'])+' |')
lines+=['',f'CLI size delta: {size_delta["cli_bytes"]:+d} bytes. Allocator probe delta: {size_delta["probe_bytes"]:+d} bytes. All adverse rows remain in decision-v1.json and the complete comparison record. Ratios on incomplete baseline workloads are not speedup evidence.','',
    f'The candidate certifies all 120 measured phases; the baseline certifies 80. On rows that both complete, the largest median paired time increase is {(cost_review["largest_median_paired_elapsed_ratio_on_matched_rows"]-1)*100:.2f}%, and the largest median requested-allocation-byte increase is {cost_review["largest_median_requested_allocation_byte_increase_percent_on_matched_rows"]:.3f}%. These adverse butt-stroke observations are retained. The measured serialized expression metrics never increase, but are not unique live graphs or peak intermediate growth. All repeated snapshots are identical. The artifact review finds only the declared boundary/expression migration and strengthened analytic-area proofs; all other recorded facts and independent point-query results agree. The retention decision prioritizes certified completion and makes no universal performance claim.','',
    'The selected CPArti and HVP109A JSON reports are unchanged in both standalone board-file and complete project-archive contexts. These are separate observations; the full-board library gate runs both projects in one process. The source importer is not changed: the original DRC stroke suite remains a preserved baseline observation of five passes and 21 failures, to be addressed independently.','',
    'Representation migration: '+migration['representation'],'',
    'Limits and remaining work:','',*['- '+s for s in limitations],'',*['- '+s for s in next_steps]]
(E/'progress-v1.md').write_text('\n'.join(lines)+'\n')
print(json.dumps({'single_line_kernel_admitted':True,'original_curve_tests':8,'added_contracts':2,'original_workload_gates':3,
    'matched_completed_rows':len(completed),'expanded_completion_rows':len(expanded),'full_plan_complete':False}),flush=True)
