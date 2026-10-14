"""Retain every hardware window and compare only fully available fixed pairs."""
import gzip
import json
from pathlib import Path
import statistics

from common_v2 import E, OLD, sha, dump, verify


def records(data):
    return [json.loads(line) for line in data.splitlines() if line.startswith(b'{')]


def usable(row):
    before, after = row['before'], row['after']
    if row['errors'] or row['thread_cpu_ns'] is None or before is None or after is None:
        return False
    enabled = after['enabled_ns'] - before['enabled_ns']
    running = after['running_ns'] - before['running_ns']
    return enabled > 0 and enabled == running and before['user_cycles'] == before['user_instructions'] == 0


def metrics(row):
    assert usable(row)
    assert row['thread_cpu_ns'] > 0 and row['wall_ns'] > 0
    assert row['after']['user_cycles'] > 0 and row['after']['user_instructions'] > 0
    return {'user_cycles': row['after']['user_cycles'],
            'user_instructions': row['after']['user_instructions'],
            'thread_cpu_ns': row['thread_cpu_ns'], 'wall_ns': row['wall_ns']}


def main():
    destination = E / 'assessment-v2.json'
    assert not destination.exists()
    out = E / 'run-v2'
    result = json.loads((out / 'result.json').read_text())
    assert result['complete'] and result['commands'] == 27 and result['fixed_processes'] == 18
    assert all(sha(out / p) == h for p, h in result['artifact_sha256'].items())
    protocol = json.loads((out / 'protocol.json').read_text())
    verify(protocol, full=True)
    assert protocol['processes'] == 18 and protocol['inputs_each_process'] == 4096
    assert len(protocol['orders']) == 6
    for label in ['main', 'full-cache', 'no-persistent']:
        assert all(sum(order[position] == label for order in protocol['orders']) == 2 for position in range(3))
    old = (OLD / 'construction-trace-v1/probe.rs').read_text()
    new = (out / 'probe.rs').read_text()
    marker = 'fn inputs(index: usize)'
    assert old[old.index(marker):] == new[new.index(marker):]
    binaries = json.loads((out / 'binaries.json').read_text())
    assert all(sha(r['path']) == r['sha256'] for r in binaries.values())
    obj = json.loads((out / 'c-object.json').read_text())
    assert sha(obj['path']) == obj['sha256']
    assert all(sha(p) == h for p, h in json.loads((out / 'c-dependencies.json').read_text()).items())
    commands = json.loads((out / 'commands.json').read_text())
    expected = ['format', 'compile-c', 'clippy-controls', 'clippy-probe', 'build-controls',
                'link-main', 'link-full-cache', 'link-no-persistent', 'counter-controls']
    expected += ['sample-' + str(i) + '-' + label for i, order in enumerate(protocol['orders'], 1) for label in order]
    assert [r['name'] for r in commands] == expected
    assert all(r['exit_code'] == 0 for r in commands)
    raw = {}
    for command in commands:
        for stream in ['stdout', 'stderr']:
            path = out / (command['name'] + '.' + stream + '.gz')
            assert sha(path) == command[stream + '_sha256']
            raw[command['name'] + '.' + stream] = gzip.decompress(path.read_bytes())
    controls = records(raw['counter-controls.stdout'])
    assert len(controls) == 1 and controls[0] == json.loads((out / 'controls.json').read_text())
    control = controls[0]
    assert control['passed'] and control['both_descriptors_closed'] and control['normal_and_unwind_disable_verified']
    assert all(usable(control[k]) for k in ['empty', 'busy', 'replay'])
    assert not usable(control['unavailable'])
    assert control['busy']['after']['user_instructions'] > control['empty']['after']['user_instructions'] + 100000
    reference = records(gzip.decompress((OLD / 'construction-trace-v1/trace-1-full-cache.stdout.gz').read_bytes()))
    gold = [r for r in reference if 'witness' in r or 'fixture' in r]
    phases = [r['trace_phase'] for r in reference if 'trace_phase' in r]
    assert len(gold) == len(phases) == 32
    samples = json.loads((out / 'samples.json').read_text())
    assert len(samples) == 18
    declared_order = [(i, position, label) for i, order in enumerate(protocol['orders'], 1)
                      for position, label in enumerate(order, 1)]
    assert [(s['round'], s['position'], s['variant']) for s in samples] == declared_order
    invalid = []
    for sample, command in zip(samples, commands[9:], strict=True):
        label = sample['variant']
        assert command['command'] == ['taskset', '-c', '0', binaries[label]['path'], '4096']
        rows = records(raw[command['name'] + '.stdout'])
        assert [r for r in rows if 'witness' in r or 'fixture' in r] == gold
        hardware = [r for r in rows if 'hardware_phase' in r]
        assert sample['rows'] == hardware and [r['hardware_phase'] for r in hardware] == phases
        for row in hardware:
            assert row['iterations'] == 4096
            for kind in ['empty_window', 'work_window']:
                window = row[kind]
                assert usable(window) == window['usable']
                if not usable(window):
                    invalid.append({'round': sample['round'], 'variant': label, 'phase': row['hardware_phase'],
                                    'window': kind, 'observation': window})
                else:
                    metrics(window)
    assert len(invalid) == result['unusable_windows']
    comparisons, overhead = [], []
    for phase in phases:
        own = {label: [next(r for r in sample['rows'] if r['hardware_phase'] == phase)
                       for sample in sorted(samples, key=lambda s: s['round']) if sample['variant'] == label]
               for label in ['main', 'full-cache', 'no-persistent']}
        for label, rows in own.items():
            empty = [metrics(r['empty_window']) if usable(r['empty_window']) else None for r in rows]
            work = [metrics(r['work_window']) if usable(r['work_window']) else None for r in rows]
            field_ratios = None
            if all(x is not None for x in empty + work):
                field_ratios = {key: statistics.median(x[key] for x in empty) / statistics.median(x[key] for x in work)
                                for key in empty[0]}
            overhead.append({'variant': label, 'phase': phase, 'empty': empty, 'work': work,
                             'empty_to_work_ratio_of_medians': field_ratios, 'overhead_subtracted': False})
        for left, right in [('main', 'full-cache'), ('main', 'no-persistent'), ('no-persistent', 'full-cache')]:
            pair = []
            for round_index, (a, b) in enumerate(zip(own[left], own[right], strict=True), 1):
                valid = all(usable(r[k]) for r in [a, b] for k in ['empty_window', 'work_window'])
                pair.append({'round': round_index, 'all_four_windows_usable': valid,
                             'baseline_work': a['work_window'], 'candidate_work': b['work_window'],
                             'ratios': {k: metrics(b['work_window'])[k] / v for k, v in metrics(a['work_window']).items()} if valid else None})
            aggregate = None
            if all(r['all_four_windows_usable'] for r in pair):
                aggregate = {}
                for metric in metrics(pair[0]['baseline_work']):
                    a = [metrics(r['baseline_work'])[metric] for r in pair]
                    b = [metrics(r['candidate_work'])[metric] for r in pair]
                    aggregate[metric] = {'baseline': a, 'candidate': b,
                                         'ratio_of_medians': statistics.median(b) / statistics.median(a),
                                         'paired_ratios': [y / x for x, y in zip(a, b, strict=True)],
                                         'candidate_higher_pairs': sum(y > x for x, y in zip(a, b, strict=True)),
                                         'candidate_lower_pairs': sum(y < x for x, y in zip(a, b, strict=True))}
            comparisons.append({'baseline': left, 'candidate': right, 'phase': phase, 'pairs': pair,
                                'all_six_pairs_usable': all(r['all_four_windows_usable'] for r in pair),
                                'aggregate_only_if_all_six_usable': aggregate})
    assert len(comparisons) == len(overhead) == 96
    dump(E / 'comparisons-v2.json', comparisons)
    dump(E / 'empty-window-accounting-v2.json', overhead)
    assessment = {'status': 'hardware-diagnostic-complete' if not invalid else 'hardware-diagnostic-complete-with-unusable-windows',
                  'assessor_sha256': sha(__file__), 'result_sha256': sha(out / 'result.json'),
                  'comparisons_sha256': sha(E / 'comparisons-v2.json'),
                  'empty_window_accounting_sha256': sha(E / 'empty-window-accounting-v2.json'),
                  'fixed_processes': 18, 'work_windows': 576, 'empty_windows': 576,
                  'unusable_windows': invalid, 'comparison_rows': 96,
                  'full_six_pair_comparison_rows': sum(r['all_six_pairs_usable'] for r in comparisons),
                  'all_witnesses_and_representation_inventories_match': True,
                  'counter_controls_passed': True, 'all_prior_sources_and_libraries_unchanged': True,
                  'limits': [
                      'A distinct instrumented driver measures the original bodies using unchanged libraries. It does not reproduce the machine code of either earlier timing-only driver.',
                      'Hardware counts include surrounding clock/measurement instructions; separately recorded empty windows are retained without subtraction.',
                      'User-only cycles/instructions and thread CPU/wall cover different boundaries. Their ratios are not calibrated CPU frequency.',
                      'Raw scheduling times are retained. No scaled estimates, discarded observations, replacement runs or pooled historical timings are used.',
                      'These descriptive comparisons cannot erase earlier adverse costs, establish a universal runtime claim or admit a runtime candidate.'
                  ],
                  'runtime_candidate': False, 'runtime_admitted': False, 'plan_complete': False}
    dump(destination, assessment)
    print(json.dumps({k: v for k, v in assessment.items() if k not in ['unusable_windows', 'limits']}), flush=True)


if __name__ == '__main__':
    main()
