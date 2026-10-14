"""Join the closed costs to prior memo coverage without collecting new samples."""
import gzip
import json
import statistics

from pi_memo_cost_common_v1 import COST_PREP, E, ROOT, sha, dump, verify_costs
from pi_memo_cost_rows_v1 import parse


CONTRASTS = [('main', 'candidate'), ('borrowed', 'candidate'), ('main', 'borrowed')]
FIELDS = ['allocation_calls', 'allocated_bytes', 'retained_bytes_delta', 'peak_bytes_delta']
PINS = {
    'scalar-cost-assessment-v1.json': '3575f3d36dda070c0aa2e8f27eebe28643aa50d27c9e74810825c678a3e26a4e',
    'scalar-costs-v1/result.json': '7d091f823e5a038749742b919336064c50d3b7fb7036cf1b1bd9a37502ba50ff',
    'qualification-assessment-v2.json': 'd62ca3da61ee94bc7d159cc38f415265555f678bd386c8bee4dd6e5c176c9a3e',
    'scalar-cost-source-review-v1.json': 'c590b97f4c367e294bf521406e2ab47ed84663f11972e9540cee1a5bb2f2c7f2',
}
COVERAGE = ROOT / 'docs/evidence/2026-10-14-local-memo-leaf-coverage/interpretation-v1.json'
COVERAGE_SHA = '1980e56169995ac8379f3322247b1e86fd23d881dc651e393ea8fc6f6175015a'


def main():
    destination = E / 'scalar-cost-interpretation-v1.json'
    assert not destination.exists()
    assert all(sha(E / path) == digest for path, digest in PINS.items())
    assert sha(COVERAGE) == COVERAGE_SHA
    prep = json.loads((COST_PREP / 'protocol.json').read_text())
    verify_costs(prep, full=True)
    assessment = json.loads((E / 'scalar-cost-assessment-v1.json').read_text())
    assert assessment['status'] == 'closed-three-group-scalar-cost-comparison-complete'
    assert assessment['assessor_sha256'] == sha(E / 'assess_scalar_costs_v1.py')
    assert assessment['result_sha256'] == PINS['scalar-costs-v1/result.json']
    assert all(sha(E / path) == digest for path, digest in assessment['artifacts_sha256'].items())
    assert assessment['all_witnesses_and_representations_match']
    assert assessment['all_allocation_fields_repeat']
    assert assessment['all_original_group_allocation_fields_match_closed_history']
    assert not assessment['nonrepeating_allocation_fields']
    assert not assessment['historical_allocation_differences']
    out = E / 'scalar-costs-v1'
    result = json.loads((out / 'result.json').read_text())
    assert result['complete'] and result['commands'] == 133
    assert result['fixed_cost_processes'] == 108 and result['preflight_processes'] == 18
    assert all(sha(out / path) == digest for path, digest in result['artifact_sha256'].items())
    assert {p.name for p in out.iterdir() if p.is_file()} == set(result['artifact_sha256']) | {'result.json'}
    binaries = json.loads((out / 'binaries.json').read_text())
    libraries = json.loads((out / 'libraries.json').read_text())
    assert len(binaries) == 18
    assert all(sha(r['path']) == r['sha256'] for r in binaries.values())
    assert all(sha(r['path']) == r['sha256'] for group in libraries.values() for r in group.values())
    commands = json.loads((out / 'commands.json').read_text())
    assert len(commands) == 133 and all(c['exit_code'] == 0 for c in commands)
    trials = json.loads(gzip.decompress((out / 'trials.json.gz').read_bytes()))
    assert len(trials) == 126
    identities, raw_rows = {}, {}
    for trial in trials:
        stdout = gzip.decompress((out / (trial['command_name'] + '.stdout.gz')).read_bytes()).decode()
        rows, identity = parse(trial['suite'], stdout, trial['count'])
        assert rows == trial['rows'] and trial['identity_matches_reference']
        assert identity == identities.setdefault((trial['suite'], trial['count']), identity)
        if not trial['preflight']:
            for row in rows:
                if 'name' in row:
                    key = (trial['suite'], trial['mode'], trial['round'], trial['group'], row['name'])
                    assert key not in raw_rows
                    raw_rows[key] = row
    timing, memory = {}, {}
    for mode, values, target in [('time', assessment['timing_rows'], timing),
                                  ('memory', assessment['memory_rows'], memory)]:
        assert len(values) == 204
        for row in values:
            left, right = row['baseline_group'], row['candidate_group']
            key = (left, right, row['suite'], row['name'])
            assert (left, right) in CONTRASTS and key not in target
            target[key] = row
            a, b = ([raw_rows[row['suite'], mode, i, g, row['name']] for i in range(1, 7)]
                    for g in [left, right])
            assert all(r['iterations'] == row['iterations'] for r in a + b)
            if mode == 'time':
                x, y = ([r['nanos'] for r in v] for v in [a, b])
                ratios = [j / i for i, j in zip(x, y, strict=True)]
                assert x == row['baseline_ns'] and y == row['candidate_ns']
                assert row['paired_ratios'] == ratios
                assert row['median_paired_ratio'] == statistics.median(ratios)
                assert row['baseline_median_ns'] == statistics.median(x)
                assert row['candidate_median_ns'] == statistics.median(y)
                assert row['ratio_of_medians'] == statistics.median(y) / statistics.median(x)
                assert row['candidate_higher_pairs'] == sum(j > i for i, j in zip(x, y, strict=True))
                assert row['candidate_lower_pairs'] == sum(j < i for i, j in zip(x, y, strict=True))
            else:
                assert list(row['fields']) == FIELDS
                for field, data in row['fields'].items():
                    x, y = ([r[field] for r in v] for v in [a, b])
                    assert x == data['baseline'] and y == data['candidate']
                    assert len(set(x)) == len(set(y)) == 1
                    assert data['paired_deltas'] == [j - i for i, j in zip(x, y, strict=True)]
                    assert data['baseline_median'] == statistics.median(x)
                    assert data['candidate_median'] == statistics.median(y)
                    assert data['candidate_higher_pairs'] == sum(j > i for i, j in zip(x, y, strict=True))
                    assert data['candidate_lower_pairs'] == sum(j < i for i, j in zip(x, y, strict=True))
    assert set(timing) == set(memory)
    coverage = json.loads(COVERAGE.read_text())
    assert coverage['observed_exact_leaf_kinds'] == ['Pi']
    phases = {(r['suite'], r['phase']): r for r in coverage['all_phase_inventory']}
    assert len(phases) == 68
    assert all({k[2:] for k in timing if k[:2] == pair} == set(phases) for pair in CONTRASTS)
    joined = []
    for (suite, name), row in phases.items():
        counts = row['counts']
        stores = sum(c['memo_store']['exact_leaf_events'] for c in counts.values())
        hits = sum(c['memo_hit']['exact_leaf_events'] for c in counts.values())
        assert (stores, hits) in [(0, 0), (4096, 12288)]
        key = ('borrowed', 'candidate', suite, name)
        deltas = {f: memory[key]['fields'][f]['paired_deltas'][0] for f in FIELDS}
        expected = dict(zip(FIELDS, [-1000, -64000, 0, -64] if stores else [0, 0, 0, 0], strict=True))
        assert deltas == expected, (suite, name, deltas)
        joined.append({'suite': suite, 'name': name, 'prior_count_iterations': 4096,
                       'prior_pi_memo_stores': stores, 'prior_pi_memo_hits': hits,
                       'memory_iterations': 1000, 'candidate_minus_borrowed': deltas,
                       'candidate_vs_borrowed_timing': timing[key]})
    assert sum(r['prior_pi_memo_stores'] > 0 for r in joined) == 9
    summaries = []
    for left, right in CONTRASTS:
        keys = [k for k in timing if k[:2] == (left, right)]
        rows = []
        for key in keys:
            t = timing[key]
            deltas = {f: memory[key]['fields'][f]['paired_deltas'][0] for f in FIELDS}
            rows.append({k: t[k] for k in ['suite', 'name', 'median_paired_ratio', 'ratio_of_medians',
                                         'paired_ratios', 'candidate_higher_pairs', 'candidate_lower_pairs']} |
                        {'memory_deltas_per_1000': deltas})
        sizes = [r for r in assessment['binary_sizes']
                 if (r['baseline_group'], r['candidate_group']) == (left, right)]
        assert len(rows) == 68 and len(sizes) == 6
        for r in sizes:
            a, b = (binaries[g + '-' + r['probe']]['bytes'] for g in [left, right])
            assert r['baseline_bytes'] == a and r['candidate_bytes'] == b
            assert r['candidate_minus_baseline_bytes'] == b - a
        summaries.append({'baseline_group': left, 'candidate_group': right, 'timing_rows': 68,
            'higher_all_six': sum(r['candidate_higher_pairs'] == 6 for r in rows),
            'lower_all_six': sum(r['candidate_lower_pairs'] == 6 for r in rows),
            'higher_at_least_five': sum(r['candidate_higher_pairs'] >= 5 for r in rows),
            'higher_median': sum(r['median_paired_ratio'] > 1 for r in rows),
            'memory_increase_rows': sum(any(d > 0 for d in r['memory_deltas_per_1000'].values()) for r in rows),
            'memory_decrease_rows': sum(any(d < 0 for d in r['memory_deltas_per_1000'].values()) for r in rows),
            'all_memory_fields_equal_rows': sum(all(d == 0 for d in r['memory_deltas_per_1000'].values()) for r in rows),
            'larger_binaries': sum(r['candidate_minus_baseline_bytes'] > 0 for r in sizes),
            'all_phase_rows': rows, 'binary_sizes': sizes})
    assert [(s['higher_all_six'], s['lower_all_six'], s['higher_at_least_five'], s['higher_median'],
             s['memory_increase_rows'], s['all_memory_fields_equal_rows']) for s in summaries] == [
                 (5, 28, 11, 17, 20, 8), (0, 4, 9, 28, 0, 59), (6, 25, 10, 15, 20, 8)]
    dump(destination, {
        'status': 'pi-memo-allocation-benefit-confirmed-scalar-costs-block-retention',
        'interpreter_sha256': sha(__file__), 'inputs_sha256': PINS,
        'prior_memo_coverage_sha256': COVERAGE_SHA,
        'commands_passed': 133, 'fixed_cost_processes': 108, 'preflight_processes': 18,
        'comparison_rows': {'timing': 204, 'memory': 204, 'binary_size': 18},
        'all_raw_scalar_rows_reparsed': True, 'all_witnesses_and_representations_match': True,
        'all_allocation_fields_repeat': True, 'old_group_allocations_match_closed_history': True,
        'complete_contrasts': summaries, 'complete_phase_coverage_join': joined,
        'pi_memo_allocation_benefit': {
            'phases': 9, 'iterations_per_phase': 1000, 'allocation_calls_saved_each_phase': 1000,
            'requested_bytes_saved_each_phase': 64000, 'peak_bytes_saved_each_phase': 64,
            'retained_bytes_change_each_phase': 0, 'other_59_phases_all_fields_equal_to_parent': True},
        'decision': [
            'Do not advance this Pi memo source to broader consumer costs or native gates. Preserve it as a rejected runtime-admission candidate with a confirmed allocation reduction.',
            'Performance precedes memory in properties.md. Compared with the borrowed-integer parent, 28 phase median paired ratios are higher and nine phases are higher in five of six pairs. None is higher in all six; four phases are lower in all six. This mixed set does not establish a performance improvement.',
            'Rational radical-Laurent first/warm/cloned median paired ratios are 1.308610, 1.308871 and 1.329171 versus the parent, with 5/5/4 higher pairs. Integer canonical-boundary first/warm ratios are 1.185037 and 1.076351, each with five higher pairs. Keep every sixth/opposite-direction observation; no timing cause is established.',
            'The nine allocation-saving phases exactly match the prior Pi memo store coverage. Each saves one allocation and 64 requested bytes per operation, plus 64 peak bytes per 1000-input phase, with unchanged retained bytes. The prior count run used 4096 inputs and is not pooled with the 1000-input allocation run.',
            'Pi parameter transport first/warm/cloned is higher in 4/4/5 pairs versus the parent despite those allocation savings. The only all-six timing improvements are opaque-linear construction, quadratic Pi-transport warm/cloned and mixed-field-decline construction; none had Pi memo copies in the earlier coverage run. Do not attribute those improvements to eliminated Pi copies.',
            'Against main, five phases remain higher in all six pairs and 20 phases increase at least one allocation field. The new source removes the parent\'s allocation-call/requested-byte increases in opaque-linear and opaque-cancellation cloned phases, but their retained/peak increases and timing tradeoffs remain.',
            'All six new scalar probes are 688 to 712 bytes larger than the parent. All 882 all-feature and 794 default-feature scalar results passed earlier, including all 11 P4 controls in each configuration. Correctness and layout preservation do not waive cost gates.',
            'Close this fixed sample set. Any future design needs a fresh source audit and a distinct protocol; preserve the failed V1 setup, V2 source, all binaries, qualification and every cost observation.'
        ],
        'limits': assessment['limits'] + [
            'This is an admission judgment over all fixed observations, not a significance test or proof that the implementation is universally slower.',
            'The assessment artifact was present after its session handle expired. This interpreter independently verifies its pinned hashes, reparses raw rows and recomputes timing and memory comparisons; it does not invent a recovered assessor process exit code.',
            'Broader library costs, complete native reports, original 120-second worker CPU and 60-second check-body wall gates remain open for any future combined runtime. No gate was rerun for this rejected source.',
            'The complete 39-property, 14-target and crate-future-work plan remains open.'
        ],
        'all_sources_and_prior_artifacts_unchanged': True,
        'new_samples': 0, 'broader_library_costs_opened': False, 'new_native_gates_run': False,
        'runtime_candidate': True, 'runtime_admitted': False, 'plan_complete': False,
    })
    print(json.dumps({'status': 'pi-memo-cost-interpretation-complete', 'all_raw_rows_verified': True,
                      'comparisons': 426, 'phase_coverage_join': 68, 'allocation_saving_phases': 9,
                      'runtime_admitted': False}), flush=True)


if __name__ == '__main__':
    main()
