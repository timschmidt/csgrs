"""Summarize observed stages without treating changed geometry as an optimization."""
import hashlib
import json
import re
from pathlib import Path

W = Path(__file__).resolve().parent
O = W / 'stage-observations-v1'

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

result = read(O / 'result.json')
assert result['complete'] and result['command_failures'] == []
commands = read(O / 'commands.json')
assert len(commands) == 6 and all(r['exit_code'] == 0 for r in commands)
for row in commands:
    for suffix in ['stdout', 'stderr', 'resources']:
        assert sha(O / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
protocol = read(O / 'protocol.json')
assert all(sha(p) == h for p, h in protocol['read_only_sha256'].items())
for source in protocol['sources'].values():
    assert all(sha(Path(source['source']) / p) == h for p, h in source['inputs'].items())
observations = {}
for label in ['baseline', 'candidate']:
    for fixture in ['cparti', 'hvp109a']:
        name = label + '-' + fixture + '-groups'
        events = [json.loads(line) for line in (O / (name + '.stdout')).read_text().splitlines()]
        assert events[-1]['complete']
        stages = {r['stage']: r['elapsed_seconds'] for r in events if 'elapsed_seconds' in r}
        neck = {n: t for n, t in stages.items() if n.endswith(':min-copper-neck')}
        sanity = {n: t for n, t in stages.items() if n not in neck and n not in ['load', 'copper-layers']}
        findings = read(O / (name + '.findings.json'))
        counts = {r['stage']: len(r['findings']) for r in findings['findings']}
        assert counts == {r['stage']: r['findings'] for r in events if isinstance(r.get('findings'), int)}
        by_check = {}
        for row in findings['findings']:
            for finding in row['findings']:
                by_check[finding['check']] = by_check.get(finding['check'], 0) + 1
        gaps = []
        previous = None
        for line in (O / (name + '.stderr')).read_text().splitlines():
            match = re.match(r'TRACE ([0-9.]+).*min-copper-neck: layer=([^ ]+) island=(\d+) exterior_vertices=(\d+) holes=(\d+)', line)
            if match:
                event = {'at': float(match[1]), 'layer': match[2], 'island': int(match[3]),
                         'exterior_vertices': int(match[4]), 'holes': int(match[5])}
                if previous is not None:
                    assert previous['layer'] == event['layer']
                    gaps.append(previous | {'seconds_to_next_trace': event['at'] - previous['at']})
                previous = event
            elif previous is not None and line.startswith('END ') and ':min-copper-neck ' in line:
                timestamp = float(line.split()[1])
                gaps.append(previous | {'seconds_to_next_trace': timestamp - previous['at']})
                previous = None
        assert previous is None
        resources = (O / (name + '.resources')).read_text()
        rss = int(re.search(r'Maximum resident set size \(kbytes\): (\d+)', resources)[1])
        observations[name] = {
            'source_load_seconds': stages['load'], 'native_aggregation_seconds': stages['copper-layers'],
            'layer_checks_seconds': sum(sanity.values()), 'neck_checks_seconds': sum(neck.values()),
            'individual_stages_seconds': stages, 'findings_per_stage': counts, 'findings_by_check': by_check,
            'findings_sha256': sha(O / (name + '.findings.json')), 'peak_rss_kib': rss,
            'complete_process_body_seconds': events[-1]['process_body_seconds'],
            'neck_islands_traced': len(gaps),
            'largest_neck_trace_gaps': sorted(gaps, key=lambda r: r['seconds_to_next_trace'], reverse=True)[:12],
            'neck_trace_gap_counts': {str(threshold): sum(g['seconds_to_next_trace'] >= threshold for g in gaps)
                                      for threshold in [0.001, 0.01, 0.1, 1.0]},
            'instrumentation': 'CPArti candidate includes debugger pauses and a three-second perf sample during aggregation, plus one debugger pause during neck. All rows include stage and trace logging.' if name == 'candidate-cparti-groups' else 'Stage and trace logging only.'
        }
backtrace = (W / 'aggregation-backtrace-v2/stdout').read_text()
assert 'CurveIntersectionBatchCache>::circular_support_relation' in backtrace
assert 'aggregate_native_regions' in backtrace
neck_trace = (W / 'neck-backtrace-v1/stdout').read_text()
assert 'retained_loop_query_bounds' in neck_trace and 'offset_for_check' in neck_trace
assert 'could not open as an executable' in (W / 'aggregation-backtrace-v1/stderr').read_text()
profile = read(W / 'aggregation-profile-v1/result.json')
assert all(r['exit_code'] == 0 for r in profile['commands'])
assert sha(W / 'aggregation-profile-v1/perf.data') == profile['perf_data_sha256']
assert '598 samples' in (W / 'aggregation-profile-v1/record.stderr').read_text()
decision = {
    'status': 'observed: native aggregation dominates the rejected exact-track workload; separate neck cost remains',
    'main_parent_commit': protocol['main_commit'], 'source_changes_applied': False,
    'implementation_sha256': {}, 'observations': observations,
    'owning_layer_findings': [
        'The full layer-sanity application group includes copper aggregation. CPArti candidate attribution assigns most of that group to aggregation, not to the public layer_sanity function alone.',
        'The verified aggregation stack enters CurveRegionBooleanContext unary candidate pairs and CurveIntersectionBatchCache::circular_support_relation. Current code scans all retained support-pair entries with arithmetic Point2 equality. This is a concrete optimization candidate, not a measured fraction of whole-stage time.',
        'Later perf samples include scalar interval/conversion and retained-ray classification work. Call-graph unwinding is incomplete; 598 cycle samples with zero reported losses do not establish reliable inclusive percentages or a single cause.',
        'The verified neck stack enters native offset regularization and retained-loop bounds. The independent neck resource limit remains a separate admission requirement; faster source aggregation alone cannot make it pass.'
    ],
    'retained_failed_attempts': {
        'aggregation-backtrace-v1': 'Debugger exited zero but could not load the executable across the sandbox namespace. Its two unknown frames are not a usable attribution.',
        'aggregation-backtrace-v2': 'Explicit verified executable and host sysroot resolve the native stack. The debugger still warns about different PID namespaces; this is one observed stack, not exhaustive thread coverage.',
        'backtrace-v3': 'Stage guard failed before creating output or attaching: aggregation had already completed. No snapshot receives credit.'
    },
    'native_runtime_admitted': False, 'full_archived_integration_admitted': False,
    'full_plan_complete': False, 'full_pre_push_matrix_executed': False,
    'original_workload_gates_unchanged': True, 'published_package_pipeline_executed': False,
    'limitations': [
        'Diagnostic stage timings compare different represented source sets, carry instrumentation and use one process per board. They are not speedup claims or a replacement for the original full-project gates.',
        'All returned findings, including geometry uncertainty, remain in raw JSON. Certified local evaluation receipts do not establish exact source intent, topology of every report or certification of readiness heuristics.',
        'The public checks are executed in application order; earlier stages warm later queries. Cold-neck mode is implemented in the probe but was not executed here. E36 retains the separately executed cold-neck failure.',
        'Baseline libraries come from completed E35 consumer validation and candidate libraries from completed E36 v4 CLI validation. Source manifests, full library hashes, exact flags and toolchain are retained; different feature histories and geometry preclude a performance-only comparison.',
        'No new kernel code, cache, internal resource bound or provenance mechanism is implemented. External containment and process RSS are not heap-byte or total-work proofs.',
        'Historical artifact audit remains failed/incomplete with 27093 missing paths. Only this new diagnostic evidence is added to the physical manifest.'
    ],
    'next': [
        'Reproduce a growing batch of native circular supports against the unchanged kernel. Compare exact support-key indexing with existing linear lookup while preserving ordered pairs, independent exact intersection oracles, symbolic fallback, uncertainty, policy admissibility, budget recovery and output determinism.',
        'Measure allocations, retained state and cold/warm lookup costs over the same completed input/output contract. Retain unfavorable cases; no indexed cache is admitted from this stack sample alone.',
        'Investigate native loop-bound reuse and neck construction evidence separately. Any repair must preserve exact geometry and prove its own acceptance tests before replaying both original full-project resource gates.',
        'Continue the complete properties plan, typed failures, ordinary strict completion and shipped user-path validation. E36 and the original full native bundle remain unadmitted.'
    ]
}
assert not (W / 'decision-v1.json').exists()
(W / 'decision-v1.json').write_text(json.dumps(decision, indent=2) + '\n')
lines = ['# E37: full-project cost attribution', '',
         'This read-only diagnostic identifies the next owning-layer experiments. No',
         'production source changes are admitted and the E36 rejection remains in force.', '',
         '| Source | Project | Load (s) | Native aggregation (s) | Layer checks (s) | Neck checks (s) |',
         '| --- | --- | ---: | ---: | ---: | ---: |']
for name, r in observations.items():
    label, fixture, _ = name.split('-')
    lines.append(f"| {label} | {fixture} | {r['source_load_seconds']:.3f} | {r['native_aggregation_seconds']:.3f} | {r['layer_checks_seconds']:.3f} | {r['neck_checks_seconds']:.3f} |")
lines += ['', 'These rows are diagnostic, not optimization benchmarks: baseline rectangles',
          'and candidate capsules represent different source sets. CPArti candidate',
          'timing includes brief debugger pauses and a three-second sampled profile.',
          'Each board runs in a separate process; the original full gate runs both.', '',
          'Native aggregation accounts for most of the expensive layer-sanity group.',
          'A resolved stack catches the batch circle-support cache scanning retained',
          'entries through arithmetic point equality. Later scalar and retained-ray',
          'samples show other work as well. The profile has 598 cycle samples and no',
          'reported loss, but incomplete call-graph unwinding prevents reliable',
          'inclusive attribution. An exact rational support-key index is an experiment',
          'to measure, not an optimization already established by this evidence.', '',
          'Neck checks separately enter native offset regularization and exact loop',
          'bounds. Their 60-second original limit still requires an independent repair.',
          'The raw findings retain uncertainty and readiness qualification. No local',
          'Certified receipt promotes sampled reports or historical source intent.', '',
          'The first stack attempt could not load symbols. Its zero exit code is not',
          'credited as useful attribution. The corrected attempt names the verified',
          'executable explicitly; its namespace warning remains recorded. A later',
          'aggregation-stage guard rejected an attempted snapshot before attachment.', '',
          'Both original full project archives, exact board bytes, source manifests,',
          'libraries, probe source, commands, findings and raw observations are retained.',
          'Completed Cargo targets were read only. The historical artifact audit still',
          'has 27,093 missing paths. No whole-history recovery, full CI completion,',
          'published-package validation or complete-properties claim is made.', '']
(W / 'progress-v1.md').write_text('\n'.join(lines))
print(json.dumps({'complete': True, 'observations': len(observations), 'source_changes_applied': False}), flush=True)
