"""Validate counter conservation, reference parity and observed construction coverage."""
import gzip
import json
from pathlib import Path
import re

from common_v1 import E, OLD, PREP, sha, dump, verify


def records(data):
    return [json.loads(line) for line in data.splitlines() if line.startswith(b'{')]


def main():
    destination = E / 'assessment-v1.json'
    assert not destination.exists()
    prep = json.loads((PREP / 'protocol.json').read_text())
    verify(prep, full=True)
    out = E / 'diagnostic-v1'
    result = json.loads((out / 'result.json').read_text())
    assert result['complete'] and result['commands'] == 7 and result['trace_rows'] == 64
    assert all(sha(out / p) == digest for p, digest in result['artifact_sha256'].items())
    protocol = json.loads((out / 'protocol.json').read_text())
    assert protocol['preparation_sha256'] == sha(PREP / 'protocol.json')
    assert protocol['review_sha256'] == sha(E / 'source-review-v1.json')
    assert protocol['runner_sha256'] == sha(E / 'measure_v1.py')
    assert protocol['scope_sha256'] == sha(E / 'scope-v1.json')
    binaries = json.loads((out / 'binaries.json').read_text())
    libraries = json.loads((out / 'libraries.json').read_text())
    assert all(sha(r['path']) == r['sha256'] for r in [*binaries.values(), *libraries.values()])
    commands = json.loads((out / 'commands.json').read_text())
    assert [r['name'] for r in commands] == ['clippy', 'build-library', 'build-tests', 'library-tests', 'link-probe', 'trace-1', 'trace-2']
    assert all(r['exit_code'] == 0 and r['cwd'] == prep['source'] for r in commands)
    raw = {}
    for command in commands:
        for stream in ['stdout', 'stderr']:
            path = out / (command['name'] + '.' + stream + '.gz')
            assert sha(path) == command[stream + '_sha256']
            raw[command['name'] + '.' + stream] = gzip.decompress(path.read_bytes())
    assert b'test result: ok. 875 passed; 0 failed; 0 ignored;' in raw['library-tests.stdout']
    names = set(re.findall(rb'^test (\S+) \.\.\. ok$', raw['library-tests.stdout'], re.M))
    old_raw = gzip.decompress((OLD / 'library-costs-v1/candidate-scalar-library.stdout.gz').read_bytes())
    old_names = set(re.findall(rb'^test (\S+) \.\.\. ok$', old_raw, re.M))
    assert len(names) == 875 and len(old_names) == 868 and old_names <= names
    assert len(names - old_names) == 7
    assert all(b'persistent_laurent_cache_tests::' in n for n in names - old_names)
    assert any(b'diagnostic_cache_counters_observe_publication_and_zero_work_replay' in n for n in names)
    old_rows = records(gzip.decompress((OLD / 'construction-trace-v1/trace-1-full-cache.stdout.gz').read_bytes()))
    gold_trace = [r for r in old_rows if 'trace_phase' in r]
    gold_identity = [r for r in old_rows if 'fixture' in r or 'witness' in r]
    samples = json.loads((out / 'samples.json').read_text())
    assert [s['repetition'] for s in samples] == [1, 2]
    event_names = re.findall(r'^    "([a-z_]+)",$', (E / 'cache_trace_v1.rs').read_text(), re.M)
    assert len(event_names) == len(set(event_names)) == 27
    for sample in samples:
        name = 'trace-' + str(sample['repetition'])
        command = next(r for r in commands if r['name'] == name)
        assert command['command'] == ['taskset', '-c', '0', binaries['probe']['path'], '4096']
        rows = records(raw[name + '.stdout'])
        trace = [r for r in rows if 'trace_phase' in r]
        assert sample['rows'] == trace and len(trace) == 32
        assert [{k: v for k, v in r.items() if k != 'cache_events'} for r in trace] == gold_trace
        assert [r for r in rows if 'fixture' in r or 'witness' in r] == gold_identity
        for row in trace:
            c = row['cache_events']
            assert set(c) == set(event_names) and all(isinstance(v, int) and v >= 0 for v in c.values())
            assert c['project_entry'] == c['project_retained_success'] + c['project_retry_original'] + c['project_decline_without_reuse']
            assert c['retained_lookup'] == c['retained_visit']
            assert c['retained_visit'] + c['original_visit'] == c['retained_hit'] + c['local_memo_hit'] + c['normalizer_work_visit'] + c['normalizer_budget_decline']
            assert c['store_attempt'] == c['closed_copy_attempt']
            assert c['closed_copy_attempt'] == c['closed_copy_success'] + c['closed_copy_declined_opaque']
            assert c['closed_copy_success'] == c['store_published'] + c['store_already_present']
            assert c['evaluation_cell_allocated'] == c['evaluation_cell_published'] + c['evaluation_cell_race_lost']
    assert samples[0]['rows'] == samples[1]['rows']
    trace = samples[0]['rows']
    construction = [{'phase': r['trace_phase'], 'events': r['cache_events']} for r in trace if r['trace_phase'].endswith('_construction')]
    assert len(construction) == 8
    focus_names = ['negated_scaled_surd_construction', 'multiradical_square_construction']
    focus = [r for r in construction if r['phase'] in focus_names]
    assert len(focus) == 2
    totals = {name: sum(r['cache_events'][name] for r in trace) for name in event_names}
    assert all(totals[name] > 0 for name in ['retained_lookup', 'retained_hit', 'store_published', 'evaluation_cell_allocated'])
    zeros = [r['phase'] for r in construction if not any(r['events'].values())]
    focus_zero = all(not any(r['events'].values()) for r in focus)
    allocation_sizes = sorted({r['cache_events']['evaluation_cell_requested_bytes'] // r['cache_events']['evaluation_cell_allocated']
                               for r in trace if r['cache_events']['evaluation_cell_allocated']})
    assert len(allocation_sizes) == 1
    for r in trace:
        c = r['cache_events']
        assert c['evaluation_cell_requested_bytes'] == c['evaluation_cell_allocated'] * allocation_sizes[0]
    assessment = {
        'status': 'no_cache_activity_in_expensive_construction_windows' if focus_zero else 'cache_activity_observed_in_expensive_construction_windows',
        'assessor_sha256': sha(__file__), 'result_sha256': sha(out / 'result.json'),
        'commands': 7, 'format_passed': True, 'all_features_all_targets_clippy_warnings_denied': True,
        'scalar_tests_passed': 875, 'prior_scalar_tests_preserved': 868,
        'existing_persistent_cache_tests_passed': 6, 'new_counter_sanity_test_passed': True,
        'diagnostic_processes': 2, 'trace_rows': 64, 'event_fields_per_row': 27,
        'all_public_arithmetic_counts_exact_witnesses_and_inventories_preserved': True,
        'all_event_counts_repeat_exactly': True, 'all_counter_conservation_checks_passed': True,
        'construction_rows': construction, 'zero_event_construction_phases': zeros,
        'both_expensive_construction_windows_have_no_observed_cache_activity': focus_zero,
        'all_phase_totals_one_process': totals,
        'observed_evaluation_cell_allocation_request_bytes': allocation_sizes[0],
        'interpretation_limits': [
            'Current-thread diagnostic counts cover the named insertion sites and eight quadratic fixture families at 4096 inputs; they are not a universal absence theorem.',
            'Counters allocate no Rust heap memory while recording, but instrumentation adds instructions and changes the binary. Its durations are not cost comparisons.',
            'Requested evaluation-cell bytes cover only that Box allocation; payload vectors, scalar owners, full live memory and RSS are not measured.',
            'Counts cannot establish a source of elapsed-time regression. The original native failure and all closed library costs remain intact.'
        ],
        'runtime_candidate': False, 'runtime_admitted': False, 'plan_complete': False,
    }
    dump(destination, assessment)
    print(json.dumps({k: assessment[k] for k in ['status', 'scalar_tests_passed', 'trace_rows',
                                               'zero_event_construction_phases', 'runtime_admitted']}), flush=True)


if __name__ == '__main__':
    main()
