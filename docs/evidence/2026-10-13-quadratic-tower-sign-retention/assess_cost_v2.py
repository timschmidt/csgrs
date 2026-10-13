"""Audit every fixed compaction cost row, including adverse and stronger facts."""
import hashlib
import json
from pathlib import Path
import statistics

from measure_v2 import BLOCKS, FAMILIES, PHASES, IDENTITY_FIELDS, FACT_FIELDS, source_state, verify_sources, read_rows

E = Path(__file__).resolve().parent
OUT = E / 'cost-v2'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    destination = E / 'cost-assessment-v2.json'
    assert not destination.exists()
    protocol = json.loads((OUT / 'protocol.json').read_text())
    result = json.loads((OUT / 'result.json').read_text())
    assert result['complete'] and result['fixed_samples_closed']
    assert sha(E / 'measure_v2.py') == protocol['runner_sha256']
    assert sha(Path(__file__)) == protocol['assessor_sha256']
    assert sha(E / 'cost-metadata-v2.json') == protocol['metadata_update_sha256']
    assert sha(E / 'cost-failure-v1.json') == protocol['prior_guard_failure_sha256']
    assert sha(E / 'compaction_cost_probe_v1.rs') == protocol['probe_sha256']
    for name, key in [('correctness-v1.json', 'correctness_sha256'), ('qualification-v5/result.json', 'qualification_result_sha256'),
                      ('preparation-v4/protocol.json', 'preparation_sha256'), ('baseline-v1/protocol.json', 'baseline_protocol_sha256')]:
        assert sha(E / name) == protocol[key]
    assert all(sha(E / name) == digest for name, digest in protocol['source_manifests_sha256'].items())
    verify_sources(source_state())
    assert protocol['fixed_block_order'] == BLOCKS and protocol['families'] == FAMILIES and protocol['phases'] == PHASES
    assert protocol['time_inputs'] == 4096 and protocol['memory_inputs'] == 1000 and protocol['preflight_inputs'] == 42
    assert protocol['preflight_processes'] == result['preflight_processes'] == 4
    assert protocol['measured_processes'] == result['measured_processes'] == 20
    libraries = json.loads((OUT / 'libraries.json').read_text())
    binaries = json.loads((OUT / 'binaries.json').read_text())
    assert list(libraries) == list(protocol['variants']) == ['baseline', 'candidate']
    for variant, item in protocol['variants'].items():
        row = libraries[variant]
        assert sha(Path(row['path'])) == row['sha256'] and Path(row['path']).stat().st_size == row['bytes']
        assert Path(row['path']).is_relative_to(Path(item['target']))
        assert Path(row['manifest_path']) == Path(item['source']) / 'crates/hyperreal/Cargo.toml'
        assert row['features'] == libraries['baseline']['features'] and not row['profile']['test']
    assert all(sha(Path(row['path'])) == row['sha256'] and Path(row['path']).stat().st_size == row['bytes'] for row in binaries.values())
    assert all(sha(Path(name)) == digest for entries in json.loads((OUT / 'build-artifacts.json').read_text()).values() for name, digest in entries.items())
    commands = json.loads((OUT / 'commands.json').read_text())
    labels = [variant + '-' + mode for variant in libraries for mode in ['time', 'memory']]
    expected_trials = [(mode, block, variant) for mode in ['time', 'memory'] for block, order in enumerate(BLOCKS, 1) for variant in order]
    assert [row['name'] for row in commands] == ['build-baseline', 'build-candidate'] + ['link-' + label for label in labels] + ['preflight-' + label for label in labels] + [f'{mode}-{block}-{variant}' for mode, block, variant in expected_trials]
    assert len(commands) == 30
    for command in commands:
        assert command['exit_code'] == 0
        for stream in ['stdout', 'stderr']:
            assert sha(OUT / (command['name'] + '.' + stream)) == command[stream + '_sha256']
    preflights = json.loads((OUT / 'preflights.json').read_text())
    assert [row['label'] for row in preflights] == labels
    identity = None
    for preflight in preflights:
        label = preflight['label']
        rows = read_rows((OUT / ('preflight-' + label + '.stdout')).read_text(), 42, label.endswith('memory'))
        assert rows == preflight['rows']
        current = [{field: row[field] for field in IDENTITY_FIELDS} for row in rows if 'witness' in row]
        if identity is None:
            identity = current
        assert current == identity
    trials = json.loads((OUT / 'trials.json').read_text())
    assert [(row['mode'], row['block'], row['variant']) for row in trials] == expected_trials
    for trial in trials:
        assert trial['name'] == f"{trial['mode']}-{trial['block']}-{trial['variant']}"
        count = protocol['time_inputs'] if trial['mode'] == 'time' else protocol['memory_inputs']
        assert trial['rows'] == read_rows((OUT / (trial['name'] + '.stdout')).read_text(), count, trial['mode'] == 'memory')
    timing, memory, facts = [], [], []
    for mode in ['time', 'memory']:
        selected = [trial for trial in trials if trial['mode'] == mode]
        identity = None
        for trial in selected:
            current = [{field: row[field] for field in IDENTITY_FIELDS} for row in trial['rows'] if 'witness' in row]
            if identity is None:
                identity = current
            assert current == identity
        for name in [family + '_' + phase for family in FAMILIES for phase in PHASES]:
            values = {variant: [next(row for row in trial['rows'] if row.get('name') == name) for trial in selected if trial['variant'] == variant] for variant in libraries}
            assert all(len(rows) == 5 for rows in values.values())
            if mode == 'time':
                left = [row['nanos'] for row in values['baseline']]
                right = [row['nanos'] for row in values['candidate']]
                assert all(value > 0 for value in left + right)
                ratios = [y / x for x, y in zip(left, right, strict=True)]
                timing.append({'name': name, 'baseline_ns': left, 'candidate_ns': right,
                               'median_ratio': statistics.median(right) / statistics.median(left),
                               'block_ratios': ratios, 'median_block_ratio': statistics.median(ratios),
                               'slower_blocks': sum(y > x for x, y in zip(left, right, strict=True)),
                               'faster_blocks': sum(y < x for x, y in zip(left, right, strict=True)),
                               'baseline_range_ns': [min(left), max(left)], 'candidate_range_ns': [min(right), max(right)]})
            else:
                fields = {}
                for field in ['allocation_calls', 'allocated_bytes', 'retained_bytes_delta', 'peak_bytes_delta']:
                    left = [row[field] for row in values['baseline']]
                    right = [row[field] for row in values['candidate']]
                    delta = [y - x for x, y in zip(left, right, strict=True)]
                    fields[field] = {'baseline': left, 'candidate': right, 'block_deltas': delta,
                                     'median_delta': statistics.median(delta), 'increased_blocks': sum(value > 0 for value in delta)}
                memory.append({'name': name, 'fields': fields})
        for name in [family + '_' + phase for family in FAMILIES for phase in PHASES[1:]]:
            values = {variant: [next(row for row in trial['rows'] if row.get('witness') == name) for trial in selected if trial['variant'] == variant] for variant in libraries}
            fields = {}
            for field in FACT_FIELDS:
                left = [row[field] for row in values['baseline']]
                right = [row[field] for row in values['candidate']]
                assert all(y >= x for x, y in zip(left, right, strict=True)), (mode, name, field)
                fields[field] = {'baseline': left, 'candidate': right, 'strengthened_blocks': sum(y > x for x, y in zip(left, right, strict=True))}
            facts.append({'mode': mode, 'name': name, 'fields': fields})
    assert len(timing) == len(memory) == 60 and len(facts) == 90
    sizes = [{'mode': mode, 'baseline_bytes': binaries['baseline-' + mode]['bytes'], 'candidate_bytes': binaries['candidate-' + mode]['bytes'],
              'delta_bytes': binaries['candidate-' + mode]['bytes'] - binaries['baseline-' + mode]['bytes']} for mode in ['time', 'memory']]
    assessment = {'decision': 'fixed-direct-compaction-costs-complete-await-retention-review', 'assessor_sha256': sha(Path(__file__)),
                  'sampling': 'Fifteen families, four phases, five fixed paired blocks, two modes. Four 42-input preflights; twenty processes with 4096 timing or 1000 allocation inputs. Separate source-bound Cargo targets; all builds ended before probes.',
                  'timing_rows': timing, 'memory_rows': memory, 'fact_rows': facts, 'binary_sizes': sizes,
                  'slower_in_all_five_blocks': [row['name'] for row in timing if row['slower_blocks'] == 5],
                  'faster_in_all_five_blocks': [row['name'] for row in timing if row['faster_blocks'] == 5],
                  'memory_rows_with_any_nonzero_delta': [row['name'] for row in memory if any(any(value != 0 for value in field['block_deltas']) for field in row['fields'].values())],
                  'mathematical_witnesses_match': True, 'facts_never_weaker_in_paired_counts': True,
                  'limits': [protocol['limits'], 'Every pair/range and adverse row is retained. Timing ratios are descriptive, without significance or application-speedup claims.',
                             'Exact value witnesses are checked by the existing tower reducer; this is not an independent implementation or universal proof.',
                             'Scalar/direct-solver qualification and these costs do not pass the existing native application gates or complete the full plan.'],
                  'artifacts_sha256': {str(path.relative_to(E)): sha(path) for path in sorted(OUT.iterdir()) if path.is_file()},
                  'sources_main_and_bound_artifacts_unchanged': True, 'runtime_admitted': False, 'plan_complete': False}
    destination.write_text(json.dumps(assessment, indent=2) + '\n')
    print(json.dumps({key: assessment[key] for key in ['decision', 'slower_in_all_five_blocks', 'faster_in_all_five_blocks', 'memory_rows_with_any_nonzero_delta', 'binary_sizes']}))


if __name__ == '__main__':
    main()
