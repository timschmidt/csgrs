"""Verify the closed baseline/candidate sample set and every paired contrast."""
import hashlib
import json
from pathlib import Path
import statistics

from qualify_v1 import verify as verify_sources

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'cost-v1'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    destination = E / 'cost-assessment-v1.json'
    assert not destination.exists()
    preparation = json.loads((E / 'preparation-v1/protocol.json').read_text())
    protocol = json.loads((OUT / 'protocol.json').read_text())
    qualification = json.loads((E / 'correctness-v1.json').read_text())
    result = json.loads((OUT / 'result.json').read_text())
    trials = json.loads((OUT / 'trials.json').read_text())
    commands = json.loads((OUT / 'commands.json').read_text())
    libraries = json.loads((OUT / 'libraries.json').read_text())
    binaries = json.loads((OUT / 'binaries.json').read_text())
    verify_sources(preparation)
    assert qualification['all_22_commands_passed'] and qualification['all_recorded_layouts_match']
    assert result['complete'] and result['mathematical_witnesses_and_inventories_identical']
    assert len(trials) == result['measured_processes'] == protocol['measured_processes'] == 60
    assert sha(E / 'measure_v1.py') == protocol['runner_sha256']
    assert sha(Path(__file__)) == protocol['assessor_sha256']
    assert sha(E / 'correctness-v1.json') == protocol['correctness_sha256']
    assert sha(E / 'qualification-v1/result.json') == protocol['qualification_result_sha256']
    assert sha(E / 'preparation-v1/protocol.json') == protocol['preparation_sha256']
    comparisons = protocol['comparisons']
    assert comparisons == [{'reference': 'baseline', 'variant': 'candidate'}]
    variants = list(preparation['variants'])
    assert variants == ['baseline', 'candidate']
    assert len(commands) == 86
    assert [r['name'] for r in commands[:2]] == ['build-' + v for v in variants]
    assert all(r['name'].startswith('link-') for r in commands[2:14])
    assert all(r['name'].startswith('preflight-') for r in commands[14:26])
    expected_measurements = [f'{suite}-{mode}-{block}-{variant}'
                             for suite in protocol['suites'] for mode in protocol['modes']
                             for block, order in enumerate(protocol['fixed_block_order'], 1) for variant in order]
    assert [r['name'] for r in commands[26:]] == expected_measurements
    for command in commands[14:26]:
        label = command['name'].removeprefix('preflight-')
        assert command['command'] == ['timeout', '--signal=TERM', '--kill-after=10s', '1800',
            'taskset', '-c', str(protocol['cpu']), binaries[label]['path'], str(protocol['preflight_inputs'])]
    for command, trial in zip(commands[26:], trials):
        label = f'{trial["variant"]}-{trial["suite"]}-{trial["mode"]}'
        count = protocol['time_inputs' if trial['mode'] == 'time' else 'memory_inputs']
        assert command['command'] == ['timeout', '--signal=TERM', '--kill-after=10s', '1800',
            'taskset', '-c', str(protocol['cpu']), '/usr/bin/time', '-v', binaries[label]['path'], str(count)]
    for command in commands:
        assert command['exit_code'] == 0
        for stream in ['stdout', 'stderr']:
            assert sha(OUT / (command['name'] + '.' + stream)) == command[stream + '_sha256']
    assert set(libraries) == set(variants)
    for variant, row in preparation['variants'].items():
        library = libraries[variant]
        assert sha(Path(library['path'])) == library['sha256']
        assert Path(library['path']).is_relative_to(Path(row['cost_target']))
        assert Path(library['manifest_path']) == Path(row['source']) / 'crates/hyperreal/Cargo.toml'
        assert library['features'] == libraries['baseline']['features']
        assert library['target']['kind'] == ['lib'] and not library['profile']['test']
    assert set(binaries) == {f'{v}-{suite}-{mode}' for v in variants for suite in protocol['suites'] for mode in protocol['modes']}
    assert all(sha(Path(row['path'])) == row['sha256'] for row in binaries.values())
    for suite, item in protocol['historical_probe_inputs'].items():
        assert sha(ROOT / item['path']) == item['sha256'] == sha(OUT / 'probes' / (suite + '.rs'))
        for mode in protocol['modes']:
            preflight_identity = None
            for variant in variants:
                raw = (OUT / f'preflight-{variant}-{suite}-{mode}.stdout').read_text()
                current = [row for row in map(json.loads, raw.splitlines()) if 'fixture' in row or 'witness' in row]
                if preflight_identity is None:
                    preflight_identity = current
                assert current == preflight_identity
            selected = [trial for trial in trials if trial['suite'] == suite and trial['mode'] == mode]
            assert len(selected) == 10
            identity = None
            for block, order in enumerate(protocol['fixed_block_order'], 1):
                assert [trial['variant'] for trial in selected if trial['block'] == block] == order
            for trial in selected:
                name = f'{suite}-{mode}-{trial["block"]}-{trial["variant"]}'
                assert trial['rows'] == list(map(json.loads, (OUT / (name + '.stdout')).read_text().splitlines()))
                current = [row for row in trial['rows'] if 'fixture' in row or 'witness' in row]
                if identity is None:
                    identity = current
                assert current == identity
    assert len(result['summary']) == 136
    timing, memory, sizes = [], [], []
    for row in result['summary']:
        assert set(row['variants']) == set(variants)
        for variant, fields in row['variants'].items():
            selected = [trial for trial in trials if trial['suite'] == row['suite'] and trial['mode'] == row['mode'] and trial['variant'] == variant]
            assert [trial['block'] for trial in selected] == [1, 2, 3, 4, 5]
            for field, values in fields.items():
                samples = [next(entry[field] for entry in trial['rows'] if entry.get('name') == row['name']) for trial in selected]
                assert samples == values['samples'] and statistics.median(samples) == values['median']
        for contrast in comparisons:
            reference, variant = contrast['reference'], contrast['variant']
            base, candidate = row['variants'][reference], row['variants'][variant]
            common = {'comparison': variant + '_vs_' + reference, 'reference': reference, 'variant': variant,
                      'suite': row['suite'], 'name': row['name']}
            if row['mode'] == 'time':
                left, right = base['nanos']['samples'], candidate['nanos']['samples']
                assert len(left) == len(right) == 5
                ratios = [y / x for x, y in zip(left, right)]
                timing.append(common | {'reference_ns': left, 'candidate_ns': right,
                    'reference_median_ns': statistics.median(left), 'candidate_median_ns': statistics.median(right),
                    'ratio_of_medians': statistics.median(right) / statistics.median(left),
                    'block_ratios': ratios, 'median_block_ratio': statistics.median(ratios),
                    'slower_blocks': sum(y > x for x, y in zip(left, right)),
                    'faster_blocks': sum(y < x for x, y in zip(left, right)),
                    'reference_range_ns': [min(left), max(left)], 'candidate_range_ns': [min(right), max(right)]})
            else:
                fields = {}
                for field in base:
                    left, right = base[field]['samples'], candidate[field]['samples']
                    assert len(left) == len(right) == 5
                    delta = [y - x for x, y in zip(left, right)]
                    fields[field] = {'reference': left, 'candidate': right,
                        'reference_median': statistics.median(left), 'candidate_median': statistics.median(right),
                        'block_deltas': delta, 'median_block_delta': statistics.median(delta),
                        'increased_blocks': sum(y > x for x, y in zip(left, right)),
                        'decreased_blocks': sum(y < x for x, y in zip(left, right))}
                memory.append(common | {'fields': fields})
    assert len(timing) == len(memory) == 68
    names = [c['variant'] + '_vs_' + c['reference'] for c in comparisons]
    for contrast in comparisons:
        reference, variant = contrast['reference'], contrast['variant']
        for suite in protocol['suites']:
            for mode in protocol['modes']:
                base, candidate = binaries[f'{reference}-{suite}-{mode}'], binaries[f'{variant}-{suite}-{mode}']
                sizes.append(contrast | {'comparison': variant + '_vs_' + reference, 'suite': suite, 'mode': mode,
                    'reference_bytes': base['bytes'], 'candidate_bytes': candidate['bytes'],
                    'delta_bytes': candidate['bytes'] - base['bytes']})
    slower = {name: [r for r in timing if r['comparison'] == name and r['slower_blocks'] == 5] for name in names}
    memory_increases = {name: [r for r in memory if r['comparison'] == name and any(f['increased_blocks'] for f in r['fields'].values())] for name in names}
    assessment = {
        'status': 'fixed-paired-costs-complete-await-review', 'assessor_sha256': sha(Path(__file__)),
        'sampling': 'Three unchanged suites, separate time/allocation modes, five predeclared paired blocks; 4096 timing inputs or 1000 allocation inputs per phase. Twelve preflights precede 60 measurements after all builds finish. No samples are added to any earlier experiment.',
        'comparisons': comparisons, 'qualification': qualification['qualification'], 'layouts': qualification['layouts'],
        'all_mathematical_witnesses_and_representation_inventories_match': True,
        'timing_rows': timing, 'memory_rows': memory, 'binary_sizes': sizes,
        'slower_in_all_five_blocks': slower, 'memory_rows_with_any_increase': memory_increases,
        'limits': [
            'The comparison uses a new closed sample set. It neither extends previous samples nor establishes native workload performance.',
            'Ratios, directions and ranges are descriptive; they establish neither statistical significance nor an application speedup. Every adverse row is retained.',
            'Requested Rust heap counters exclude native C heaps, allocator bookkeeping and stack frames. Equal private type sizes do not establish equal total memory or eventual reclamation.',
            'Native qualification/cost controls, full reports, production CLI size, original full-fixture limits and the complete workflow matrix remain open. No dependency or layout change is part of this candidate.',
        ],
        'evidence_sha256': {str(p.relative_to(E)): sha(p) for p in [OUT / n for n in ['protocol.json', 'commands.json', 'trials.json', 'result.json', 'libraries.json', 'binaries.json']] + [E / 'correctness-v1.json']},
        'source_parent_main_and_artifacts_unchanged': True, 'runtime_admitted': False, 'plan_complete': False,
    }
    destination.write_text(json.dumps(assessment, indent=2) + '\n')
    print(json.dumps({'rows_per_contrast': 68, 'all_five_slower_rows': {k: len(v) for k, v in slower.items()},
                      'memory_rows_with_any_increase': {k: len(v) for k, v in memory_increases.items()}}))


if __name__ == '__main__':
    main()
