"""Describe the closed memo-copy inventory without inferring timing or heap cost."""
import json

from leaf_memo_common_v2 import E, PREP, SUITES, dump, sha, verify


LEAVES = ['Int', 'One', 'Ratio', 'Pi', 'InvPi', 'Tau', 'Sqrt2', 'Sqrt3']


def main():
    destination = E / 'interpretation-v1.json'
    assert not destination.exists()
    prep = json.loads((PREP / 'protocol.json').read_text())
    verify(prep, full=True)
    assessment = json.loads((E / 'assessment-v2.json').read_text())
    assert assessment['assessor_sha256'] == sha(E / 'assess_v2.py')
    assert all(sha(E / p) == h for p, h in assessment['artifacts_sha256'].items())
    assert assessment['all_memo_event_and_shape_partitions_match']
    assert assessment['all_old_cache_and_reader_counts_match_parent']
    assert assessment['all_counts_repeat_in_reversed_order']
    scope = json.loads((E / 'scope-v2.json').read_text())
    assert scope['buckets'][:8] == LEAVES
    phases = assessment['classified_phases']
    assert len(phases) == 68
    inventory = []
    observed = set()
    for row in phases:
        payloads = row['memo_payloads']
        for kind, modes in payloads.items():
            for events in modes.values():
                for counts in events.values():
                    if counts['events']:
                        observed.add(kind)
                    if kind not in LEAVES:
                        continue
                    # This expectation follows the existing leaf constructors;
                    # zero Int/Ratio coefficients may yield an empty polynomial.
                    assert counts['none'] == counts['opaque'] == counts['other_closed'] == 0
                    assert counts['atoms'] == 0
                    if kind in ['Sqrt2', 'Sqrt3']:
                        assert counts['one_radical_term'] == counts['events']
                        assert counts['radicals'] == counts['terms'] == counts['events']
                    else:
                        assert counts['one_rational_term'] + counts['empty'] == counts['events']
                        assert counts['terms'] == counts['one_rational_term']
                        assert counts['radicals'] == 0
                        if kind not in ['Int', 'Ratio']:
                            assert counts['empty'] == 0
        counts = {}
        for mode in scope['modes']:
            counts[mode] = {}
            for event in scope['events']:
                counts[mode][event] = {
                    'exact_leaf_events': sum(payloads[kind][mode][event]['events'] for kind in LEAVES),
                    'other_constant_events': payloads['OtherConstant'][mode][event]['events'],
                    'other_node_events': payloads['OtherNode'][mode][event]['events'],
                    'exact_leaf_terms': sum(payloads[kind][mode][event]['terms'] for kind in LEAVES),
                    'exact_leaf_radicals': sum(payloads[kind][mode][event]['radicals'] for kind in LEAVES),
                }
        inventory.append({'suite': row['suite'], 'phase': row['phase'], 'counts': counts})
    summaries = {}
    for suite in SUITES:
        own = [row for row in inventory if row['suite'] == suite]
        assert len(own) == {'rational': 24, 'quadratic': 32, 'integer': 12}[suite]
        summaries[suite] = {
            'phases': len(own),
            'phases_with_exact_leaf_stores': sum(any(row['counts'][mode]['memo_store']['exact_leaf_events']
                for mode in scope['modes']) for row in own),
            'phases_with_exact_leaf_hits': sum(any(row['counts'][mode]['memo_hit']['exact_leaf_events']
                for mode in scope['modes']) for row in own),
            'counts_per_diagnostic_process': {
                mode: {event: {field: sum(row['counts'][mode][event][field] for row in own)
                    for field in own[0]['counts'][mode][event]}
                    for event in scope['events']} for mode in scope['modes']},
        }
    has_leaf_copies = any(summary['phases_with_exact_leaf_stores'] for summary in summaries.values())
    result = {
        'status': ('exact-leaf-memo-copies-observed-cost-and-replacement-open' if has_leaf_copies
                   else 'no-exact-leaf-memo-copies-observed-in-these-workloads'),
        'interpreter_sha256': sha(__file__),
        'assessment_sha256': sha(E / 'assessment-v2.json'),
        'failure_v1_sha256': sha(E / 'failure-v1.json'),
        'preparation_sha256': sha(PREP / 'protocol.json'),
        'commands_passed': 29,
        'scalar_tests_passed': 884,
        'fixed_count_processes': 12,
        'distinct_phases': 68,
        'all_public_counts_witnesses_and_parent_cache_events_preserved': True,
        'both_reversed_passes_match_exactly': True,
        'all_observed_exact_leaf_payloads_match_existing_leaf_constructors': True,
        'observed_exact_leaf_kinds': [kind for kind in LEAVES if kind in observed],
        'observed_memo_kinds': [kind for kind in scope['buckets'] if kind in observed],
        'unobserved_memo_kinds': [kind for kind in scope['buckets'] if kind not in observed],
        'per_suite_summary': summaries,
        'all_phase_inventory': inventory,
        'interpretation': [
            'These are counts immediately before the existing local memo store/hit clones. They identify the copied payload kinds and vector lengths.',
            'Each suite summary sums its distinct phases from one diagnostic process. The reversed repeat is checked for equality, not added to the summary or pooled with prior runs.',
            'The first eight buckets are exact immutable leaves that cannot publish persistent Laurent certificates. Other constants and other nodes are reported separately.',
            'The independent controls exercise all constant kinds, both retained/original modes, six shapes, zero-budget replay, original-pass retry and graph reclamation. Workload zeros remain zeros.',
            'The failed V1 assertion confused a process-wide constant approximation cache with the fresh node cache. V2 changes only that assertion; all old files and binaries remain retained.',
        ],
        'next_step': ('Audit a compact memo representation restricted to the observed exact-leaf kinds: '
                      + ', '.join(kind for kind in LEAVES if kind in observed)
                      + '. Preserve cache-before-memo-before-budget order, first-visit charging, zero-budget replay and nonleaf ownership; qualify any implementation and measure its complete costs separately.'
                      if has_leaf_copies else 'Close this workload hypothesis without constructing a leaf-memo runtime candidate.'),
        'limits': [
            'Event counts and vector lengths are not timing shares, allocation counts, byte savings or evidence of a speedup.',
            'No causal claim is made about previous scalar or native regressions, and no native limit changes.',
            'The fixed samples are closed. No extension, filtering, replacement or pooling is authorized by this interpretation.',
            'This diagnostic does not admit a runtime change or discharge the broader 39 properties, 14 targets and crate future work.',
        ],
        'new_timing_or_allocation_samples': 0,
        'new_native_gates_run': False,
        'runtime_candidate': False,
        'runtime_admitted': False,
        'plan_complete': False,
    }
    dump(destination, result)
    print(json.dumps({'status': result['status'], 'observed_kinds': result['observed_memo_kinds'],
                      'suite_summary': summaries, 'runtime_admitted': False}), flush=True)


if __name__ == '__main__':
    main()
