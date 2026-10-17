"""Record the complete integration's disposition and the alternative's actual limits."""
import hashlib
import json
from pathlib import Path
import re

E=Path(__file__).resolve().parent
ROOT=E.parents[2]


def sha(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def read(path):return json.loads(Path(path).read_text())

assert not (E/'decision-v1.json').exists() and not (E/'progress-v1.md').exists()
application=read(E/'application-v1.json')
assert application['pending_native_paths_remaining']==0 and application['format_exit_code']==0
candidate=read(E/'candidate-validation-v1/result.json')
committed=read(E/'committed-validation-v1/result.json')
wasm=read(E/'wasm-validation-v1/result.json')
probe=read(E/'committed-proposed-regressions-v1/result.json')
assert candidate['complete'] and set(candidate['failed_commands'])=={'full-board','copper-neck'}
assert committed['complete'] and not committed['failed_commands']
assert wasm['complete'] and not wasm['unexpected_exits'] and wasm['real_js_and_compiled_typescript_consumers_passed']
assert probe['complete'] and probe['counts']['ignored']==0
assert probe['counts']['passed']+probe['counts']['failed']==26
counts={}
for folder in ['candidate-validation-v1','committed-validation-v1','wasm-validation-v1']:
    for row in read(E/folder/'commands.json'):
        for suffix in ['stdout','stderr','resources']:
            assert sha(E/folder/(row['name']+'.'+suffix))==row[suffix+'_sha256']
        text=(E/folder/(row['name']+'.stdout')).read_text()
        matches=re.findall(r'test result: (?:ok|FAILED)\. (\d+) passed; (\d+) failed; (\d+) ignored;',text)
        if matches:
            numbers=[tuple(map(int,match)) for match in matches]
            counts[folder+'/'+row['name']]={name:sum(values[i] for values in numbers) for i,name in enumerate(['passed','failed','ignored'])}|{'groups':len(numbers)}
assert counts['candidate-validation-v1/native-curve-regressions']['passed']==8
assert counts['candidate-validation-v1/native-drc-regressions']['passed']==26
gates={}
for variant in ['candidate','committed']:
    rows={row['name']:row for row in read(E/(variant+'-validation-v1')/'commands.json')}
    gates[variant]={}
    for name,pattern,limit,metric in [
      ('full-board',r'took (\d+(?:\.\d+)?)s of worker time',120,'test-worker thread CPU seconds'),
      ('copper-neck',r'copper neck regression fixture took (\d+(?:\.\d+)?)s',60,'check-body wall seconds'),
      ('gerber',None,None,'original smoke assertions')]:
        executed=counts[variant+'-validation-v1/'+name]
        assert executed['passed']+executed['failed']==1 and executed['ignored']==0
        text=(E/(variant+'-validation-v1')/(name+'.stderr')).read_text()
        assert 'skipping complex project' not in text
        match=re.search(pattern,text) if pattern else None
        gates[variant][name]={'exit_code':rows[name]['exit_code'],'passed':rows[name]['exit_code']==0,
          'metric':metric,'unchanged_limit_seconds':limit,
          'reported_assertion_seconds':float(match[1]) if match else None,
          'whole_supervised_command_seconds':rows[name]['elapsed_seconds'],
          'full_original_fixtures_available':True}
assert gates['candidate']['full-board']['reported_assertion_seconds']>120
assert gates['candidate']['copper-neck']['reported_assertion_seconds']>60
assert all(row['passed'] for row in gates['committed'].values())
preflight=read(E/'fixture-preflight-v1/result.json')
assert preflight['complete'] and preflight['all_original_archives_and_entries_present']
for path,digest in application['rejected_native_sha256'].items():
    assert sha(E/'pending-source-v1'/path)==digest
assert all(sha(ROOT/path)==digest for path,digest in application['restored_native_sha256'].items())
assert all(not (ROOT/path).exists() for path in application['removed_added_paths'])
# The independent next-boundary WASM build may overlap correctness checks,
# but its completion must precede even a conservative lower bound on gate start.
followup=ROOT/'target/properties/wasm-mesh-failure-review/wasm-validation-v1/result.json'
followup_native=ROOT/'target/properties/wasm-mesh-failure-review/native-validation-v1/result.json'
followup_record,followup_native_record=read(followup),read(followup_native)
assert followup_record['complete'] and followup_native_record['complete']
followup_completed=max(followup.stat().st_mtime,followup_native.stat().st_mtime)
committed_rows={row['name']:row for row in read(E/'committed-validation-v1/commands.json')}
isolation={}
for name in ['full-board','copper-neck','gerber']:
    resource_file=E/'committed-validation-v1'/(name+'.resources')
    lower_start=resource_file.stat().st_mtime-committed_rows[name]['elapsed_seconds']
    assert followup_completed<lower_start,(name,followup_completed,lower_start)
    isolation[name]={'followup_completion_precedes_gate_start':True,
                     'conservative_gap_seconds':lower_start-followup_completed}
timing_isolation={'followup_result':str(followup),'followup_result_sha256':sha(followup),
                  'followup_native_result':str(followup_native),'followup_native_result_sha256':sha(followup_native),
                  'followup_completed_epoch':followup_completed,'gates':isolation,
                  'interpretation':'Resource-file end time minus the whole supervised duration is earlier than the actual gate start; the independent WASM phase completed before that bound.'}
(E/'followup-timing-isolation-v1.json').write_text(json.dumps(timing_isolation,indent=2)+'\n')
status='rejected: complete 35-file native integration fails unchanged workload contract; candidate preserved, committed sources restored'
record={
 'status':status,'user_direction':'Evaluate the 35 earlier native edits, choose whether to retain them, then continue the properties plan.',
 'base_commit':application['main_parent_commit'],'contract':'contract-v1.md','source_review':'source-review-v1.md',
 'scope':'The submitted complete integration is rejected. This is not a conclusion that every individual idea is unsound or that the legacy implementation meets the proposed new claims.',
 'owning_layers':['Hypercurve exact stroke construction, retained evidence, bounds/winding and arrangement','Hyperdrc source ingress, native stroke geometry, report projection and failure-preserving handoff'],
 'properties':['P1','P2','P3','P4','P5','P6','P8','P9','P10','P11','P12','P13','P14','P15','P16','P17','P18','P19','P20','P25','P26','P27','P28','P29','P30','P35','P36','P37'],
 'candidate':{'native_curve_regressions_passed':8,'native_drc_regressions_passed':26,'commands':candidate['commands'],
              'failed_commands':candidate['failed_commands'],'gates':gates['candidate'],
              'full_qualification_claimed':False,'reason':'Either failed original timing assertion blocks retention of the complete integration under its declared workload contract.'},
 'committed_alternative':{'commands':committed['commands'],'failed_commands':committed['failed_commands'],'gates':gates['committed'],
              'wasm_commands':wasm['commands'],'new_js_contract_tests_passed':12,
              'actual_typescript_compiled_and_executed':True,'prior_consumers_preserved':True},
 'independent_followup_build_isolation':timing_isolation,
 'test_counts_by_command':counts,'proposed_regressions_on_committed_libraries':probe,
 'known_gap_interpretation':'The unchanged proposed regression file was independently compiled against the exact qualified committed libraries and executed in full. Its failures remain failures. Passing legacy tests or workload gates does not certify source-coordinate fidelity, round track caps, extreme finite stroke support or construction-failure retention addressed by the proposal.',
 'regression_replay_limit':'Each failed test stops at its first assertion. Several budget/policy-named tests fail first on geometry construction or area; this does not establish that their later budget/policy assertions also failed.',
 'performance_interpretation':'The two representations perform different geometric work. No latency ratio, speedup or performance-only correctness claim is made. Original assertion outcomes are categorical admission evidence; whole supervised-command durations include setup, compilation and evidence bookkeeping and are not substitutes for assertion metrics.',
 'evidence_limits':'Candidate focused regressions and original gates are current measurements. Rejection stops full-candidate admission; prior broad suites and cost trials are not credited as fresh qualification. Committed-source tests, lint/docs, real WASM consumers and full proposed DRC regression replay qualify only their stated scopes. Report finding counts are not independent set oracles or full report equality.',
 'resource_limits':'External 1800-second command and 8-GiB native/64-GiB WASM per-process address-space limits. No internal work, heap-byte or cancellation bound is added.',
 'disposition':{'tracked_files_restored':29,'saved_additions_removed':6,'complete_candidate_files_preserved':35,
                'pending_native_paths_remaining':0,'candidate_snapshot':'pending-source-v1/',
                'tracked_patch':'preparation-v1/tracked.patch','untracked_patch':'preparation-v1/untracked.patch'},
 'rejected_native_sha256':application['rejected_native_sha256'],'restored_native_sha256':application['restored_native_sha256'],
 'removed_added_paths':application['removed_added_paths'],'implementation_sha256':{},
 'native_candidate_admitted':False,'disposition_complete':True,'optimization_claimed':False,
 'historical_artifact_audit_passed':False,'historical_preservation_complete':False,
 'published_package_pipeline_executed':False,'full_pre_push_matrix_executed':False,'push_performed':False,'full_plan_complete':False,
 'next':['Use the preserved failing source-fidelity and construction-failure regressions for smaller independently qualified changes; retain the original workload gates.',
         'Continue typed public failures, ordinary strict completion, end-to-end evidence and clean external distribution under the properties contract.']}
(E/'decision-v1.json').write_text(json.dumps(record,indent=2)+'\n')
lines=['# Disposition of the 35 pending native edits','',
 '**Rejected as a complete integration.** The 29 tracked files are restored to commit `99b88f707b04053de7f40a86d6d029e3c07460d9`; the six added source/test files are removed from production after exact preservation. All 35 candidate files remain under [pending-source-v1](pending-source-v1/), with [tracked](preparation-v1/tracked.patch) and [untracked](preparation-v1/untracked.patch) patches. No native edit remains indefinitely pending in the worktree.','',
 'This implements the user’s request for a retain-or-reject decision under [the declared acceptance contract](contract-v1.md). The [current source review](source-review-v1.md) identifies real accuracy, evidence and failure-handling improvements. The eight new curve regressions and all 26 new DRC regressions pass. Those benefits do not make the complete integration satisfy its unchanged workload contract.','',
 '| Original gate | Candidate result | Committed alternative | Unchanged criterion |',
 '| --- | --- | --- | --- |']
for name in ['full-board','copper-neck','gerber']:
 c=gates['candidate'][name];b=gates['committed'][name]
 value=(f"{c['reported_assertion_seconds']:.9f} seconds; failed" if c['reported_assertion_seconds'] is not None else 'passed' if c['passed'] else 'failed')
 criterion=f"< {c['unchanged_limit_seconds']} {c['metric']}" if c['unchanged_limit_seconds'] else 'original smoke assertions'
 lines.append(f"| {name} | {value} | {'passed' if b['passed'] else 'failed'} | {criterion} |")
lines+=['','The test source, policies, complete archives and geometry resolution are unchanged. Four independent unzip extractions verify both complete board entries in both snapshots. Gates run without other builds from this task. Candidate failures and complete logs are retained in [candidate validation](candidate-validation-v1/commands.json). Whole command times are separate from the asserted worker-CPU/check-body metrics. The representations do different work, so these observations establish no speedup or performance-only optimization.','',
 'The restored configuration passed the [native matrix](committed-validation-v1/commands.json), including root and curve feature combinations, DRC tests and all three original gates, all-target Clippy with warnings denied, docs with warnings denied and formatting. The [real WASM matrix](wasm-validation-v1/commands.json) passed all twelve new STL/error tests, both earlier JavaScript consumers, the prior TypeScript check, the compiled-and-executed new TypeScript consumer, and WASM Clippy. Main-tree formatting passed after byte-identical restoration.','',
 '| Native check | Passed | Failed | Existing ignored |','| --- | ---: | ---: | ---: |']
for name in ['root-all','root-minimal','curve-default','curve-all','native-other-tests']:
 r=counts['committed-validation-v1/'+name]
 lines.append(f"| {name} | {r['passed']} | {r['failed']} | {r['ignored']} |")
lines+=['','Counts overlap across feature configurations and are not a coverage percentage.','',
 f"The complete unchanged proposed DRC regression file was also [replayed against the committed libraries](committed-proposed-regressions-v1/result.json): **{probe['counts']['passed']} passed, {probe['counts']['failed']} failed, {probe['counts']['ignored']} ignored**. These failures are preserved as unmet mathematical claims, not hidden by the legacy suite. Rejection is not a correctness repair and does not certify the legacy handling of exact track tokens, round caps, extreme finite strokes or construction failures.",'',
 'A failed test stops at its first assertion. Several budget/policy-named tests fail first on construction or area, so their later budget/policy assertions are not independently established as failures.','',
 'Failed proposed regressions:','']
lines.extend('- `'+name+'`' for name in probe['failed_tests'])
lines+=['','The next useful work is smaller independently qualified repairs using these preserved regressions and the original full-scale gates. No individual optimization is assigned the integration’s costs without a controlled comparison. The complete candidate, original hard fixtures and all adverse results remain available.','',
 'Only the raw `current_work` ledger value changes; bytes and decoded keys outside it stay bound to revision `9025de62`. The historical artifact audit remains failed/incomplete, retaining 947,270 earlier expectations and 27,093 missing historical paths. New physical paths are hashed independently; no whole-history audit or zero-loss claim is made. Full distribution, pre-push, internal resource-bound, coverage and other properties work remains open. The full goal continues.','',
 'See [the disposition decision](decision-v1.json), [exact restoration](application-v1.json), [retained logs](collection-v1.json) and [checkpoint](checkpoint-v1.json).']
(E/'progress-v1.md').write_text('\n'.join(lines)+'\n')
print(json.dumps({'status':status,'gates':gates,'proposed_regressions_on_committed':probe['counts'],'pending_native_paths':0}),flush=True)
