"""Retain every fixed native cost and receipt for the zero-radical candidate."""
import gzip
import hashlib
import json
from pathlib import Path
import statistics
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'native-controls-v1'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    destination = E / 'native-controls-assessment-v1.json'
    assert not destination.exists()
    result = json.loads((OUT / 'result.json').read_text())
    protocol = json.loads((OUT / 'protocol.json').read_text())
    preparation = json.loads((E / 'preparation-v1/protocol.json').read_text())
    commands = json.loads((OUT / 'commands.json').read_text())
    binaries = json.loads((OUT / 'binaries.json').read_text())
    libraries = json.loads((OUT / 'libraries.json').read_text())
    trials = json.loads((OUT / 'trials.json').read_text())
    historical = json.loads((OUT / 'historical-comparison.json').read_text())
    assert result['complete'] and result['all_variant_identities_match']
    assert historical['all_231_historical_witnesses_identical']
    assert sha(E / 'measure_native_v1.py') == protocol['runner_sha256']
    assert sha(Path(__file__)) == protocol['assessor_sha256']
    assert sha(E / 'native-validation-v1/result.json') == protocol['validation_sha256']
    assert sha(E / 'native-assessment-v1.json') == protocol['native_assessment_sha256']
    assert sha(E / 'scalar-decision-v1.json') == protocol['scalar_decision_sha256']
    assert sha(E / 'preparation-v1/protocol.json') == protocol['preparation_sha256']
    assert protocol['fixed_block_order'] == [
        ['baseline', 'candidate'], ['candidate', 'baseline'], ['candidate', 'baseline'],
        ['baseline', 'candidate'], ['baseline', 'candidate'],
    ]
    assert protocol['final_abort_blocks'] == protocol['fixed_block_order'][:3]
    assert protocol['time_inputs_per_family'] == 8192 and protocol['memory_inputs_per_family'] == 1000
    assert protocol['preflight_inputs'] == 42
    assert list(libraries) == ['baseline', 'candidate'] and len(binaries) == 8
    for name, item in protocol['historical_probe_inputs'].items():
        assert sha(ROOT / item['path']) == sha(OUT / 'probes' / name) == item['sha256'] == protocol['probe_sha256'][name]
    historical_path = ROOT / 'docs/evidence/2026-10-09-hyperdrc-kicad-pad-sources/cost/rows.json'
    assert sha(historical_path) == historical['historical_sha256']
    expected_trials = [(mode, index, variant) for mode in ['abort', 'time', 'memory', 'broad']
                       for index, order in enumerate(protocol['final_abort_blocks'] if mode == 'abort'
                                                     else protocol['fixed_block_order'], 1) for variant in order]
    assert [(row['mode'], row['index'], row['variant']) for row in trials] == expected_trials
    assert [row['name'] for row in commands] == (
        ['build-libraries-baseline', 'build-libraries-candidate']
        + ['build-' + variant + '-' + mode for variant in libraries for mode in ['time', 'memory', 'abort', 'broad']]
        + ['preflight-' + variant + '-' + mode for variant in libraries for mode in ['time', 'memory']]
        + [f'{mode}-{index}-{variant}' for mode, index, variant in expected_trials])
    assert sources() == preparation['main_source_sha256']
    assert all(sha(ROOT / name) == digest for name, digest in preparation['pending_native_sha256'].items())
    parent = json.loads(gzip.decompress((E / 'preparation-v1/parent-source-hashes.json.gz').read_bytes()))
    assert all(sha(Path(preparation['parent_source']) / name) == digest for name, digest in parent.items())
    for variant, row in preparation['variants'].items():
        assert all(sha(Path(row['source']) / name) == digest
                   for name, digest in (parent | row['source_changes_sha256']).items())
        for name, library in libraries[variant].items():
            assert sha(Path(library['path'])) == library['sha256']
            assert Path(library['path']).is_relative_to(Path(protocol['targets'][variant]))
            if name != 'serde_json':
                assert Path(library['manifest_path']) == Path(row['source']) / 'crates' / name / 'Cargo.toml'
            assert library['features'] == libraries['baseline'][name]['features']
    assert all(sha(Path(row['path'])) == row['sha256'] for row in binaries.values())
    for trial in trials:
        name = f"{trial['mode']}-{trial['index']}-{trial['variant']}"
        raw = (OUT / (name + '.stdout')).read_text()
        if trial['mode'] == 'broad':
            parsed = []
            for line in raw.splitlines():
                if line.startswith('operation_'):
                    label, iterations, nanos, witness = line.split('\t')
                    parsed.append({'name': label, 'iterations': int(iterations), 'elapsed_ns': int(nanos), 'witness': witness})
            assert [{key: value for key, value in row.items() if key != 'elapsed_ns'} for row in parsed] == json.loads(historical_path.read_text())
        else:
            parsed = [json.loads(line) for line in raw.splitlines()]
        assert parsed == trial['rows']
    for command in commands:
        assert command['exit_code'] == 0
        for stream in ['stdout', 'stderr']:
            assert sha(OUT / (command['name'] + '.' + stream)) == command[stream + '_sha256']
    assert len(trials) == 36
    assert len([command for command in commands if command['name'].startswith('preflight-')]) == 4
    for mode in ['time', 'memory', 'broad', 'abort']:
        selected = [trial for trial in trials if trial['mode'] == mode]
        orders = protocol['final_abort_blocks'] if mode == 'abort' else protocol['fixed_block_order']
        assert len(selected) == len(orders) * 2
        canonical = None
        for index, order in enumerate(orders, 1):
            assert [trial['variant'] for trial in selected if trial['index'] == index] == order
        for trial in selected:
            if mode == 'broad':
                identity = [{key: value for key, value in row.items() if key != 'elapsed_ns'} for row in trial['rows']]
                assert len(identity) == 231
            elif mode == 'abort':
                identity = trial['rows']
                assert len(identity) == 210
            else:
                identity = [row for row in trial['rows'] if 'witness' in row or 'fixture' in row]
                assert len(identity) == 48
            if canonical is None:
                canonical = identity
            assert identity == canonical
    variants = {}
    for variant in ['candidate']:
        timing, memory, broad = [], [], []
        for mode in ['time', 'memory', 'broad']:
            selected = [trial for trial in trials if trial['mode'] == mode]
            for name in [row['name'] for row in selected[0]['rows'] if 'name' in row]:
                rows = {}
                for key in ['baseline', variant]:
                    indexed = sorted([trial for trial in selected if trial['variant'] == key], key=lambda row: row['index'])
                    assert [trial['index'] for trial in indexed] == [1, 2, 3, 4, 5]
                    rows[key] = [next(row for row in trial['rows'] if row.get('name') == name) for trial in indexed]
                if mode in ['time', 'broad']:
                    field = 'nanos' if mode == 'time' else 'elapsed_ns'
                    samples = {key: [row[field] for row in values] for key, values in rows.items()}
                    ratios = [candidate / baseline for baseline, candidate in zip(samples['baseline'], samples[variant])]
                    record = {'name': name, 'samples_ns': samples, 'block_ratios': ratios,
                              'median_block_ratio': statistics.median(ratios),
                              'ratio_of_medians': statistics.median(samples[variant]) / statistics.median(samples['baseline']),
                              'ranges_ns': {key: [min(values), max(values)] for key, values in samples.items()},
                              'slower_blocks': sum(value > 1 for value in ratios),
                              'faster_blocks': sum(value < 1 for value in ratios)}
                    (timing if mode == 'time' else broad).append(record)
                else:
                    fields = ['allocation_calls', 'allocated_bytes', 'retained_bytes_delta', 'peak_bytes_delta']
                    samples = {key: {field: [row[field] for row in values] for field in fields} for key, values in rows.items()}
                    deltas = {field: [candidate - baseline for baseline, candidate in zip(samples['baseline'][field], samples[variant][field])] for field in fields}
                    memory.append({'name': name, 'samples': samples, 'block_deltas': deltas,
                                   'median_block_deltas': {field: statistics.median(values) for field, values in deltas.items()}})
        assert len(timing) == len(memory) == 48 and len(broad) == 231
        variants[variant] = {
            'arc_timing': timing, 'arc_memory': memory, 'broad_timing': broad,
            'arc_slower_in_all_five_blocks': [row for row in timing if row['slower_blocks'] == 5],
            'broad_slower_in_all_five_blocks': [row for row in broad if row['slower_blocks'] == 5],
            'memory_median_increases': [row for row in memory if any(value > 0 for value in row['median_block_deltas'].values())],
            'memory_increases_in_any_sample': [row for row in memory if any(value > 0 for values in row['block_deltas'].values() for value in values)],
            'probe_binary_sizes': {mode: {'baseline_bytes': binaries['baseline-' + mode]['bytes'],
                                          'candidate_bytes': binaries[variant + '-' + mode]['bytes'],
                                          'byte_delta': binaries[variant + '-' + mode]['bytes'] - binaries['baseline-' + mode]['bytes']}
                                   for mode in ['time', 'memory', 'broad', 'abort']},
        }
    assessment = {
        'decision': 'fixed-native-controls-recorded-await-complete-reports-and-tradeoff-review',
        'assessor_sha256': sha(Path(__file__)), 'variants': variants,
        'broad_and_arc_variant_witnesses_identical': True, 'historical_comparison': historical,
        'final_coordinate_abort': {'fresh_processes': 6, 'rows_each': 210, 'complete_receipts_identical': True,
                                   'scope': 'Six families, seven contexts and five active/repeated/cleared-prepared/reenabled/cleared-final stages. The unchanged probe checks final-coordinate availability around the public queries.'},
        'limits': [
            'Five fixed baseline/candidate blocks describe this run; no confidence intervals, universal regression bounds or extension of prior samples is claimed. Every adverse sample is retained.',
            'Construction, first, warm and cloned queries are separate rows. First uses fresh graphs within a process that can share constants; cloned query timing excludes cloning.',
            'Allocation counters cover requested Rust bytes, excluding native C heaps. Retained and peak bytes are phase-relative, not eventual process reclamation.',
            'The historical cost-probe abort is attached to an intermediate. Effective final-coordinate abortion is checked separately by the unchanged 210-row receipt probe.',
            'All 231 broad rows are preserved historical witnesses, not new proofs. Full report bytes and original native limits are separate requirements.',
            'The baseline already includes the isolated endpoint changes. These costs cannot erase the inherited endpoint trial tradeoffs or establish a speedup over main.',
        ],
        'artifacts_sha256': {str(path.relative_to(E)): sha(path) for path in sorted(OUT.rglob('*')) if path.is_file()},
        'source_parent_main_and_artifacts_unchanged': True, 'runtime_admitted': False, 'plan_complete': False,
    }
    destination.write_text(json.dumps(assessment, indent=2) + '\n')
    print(json.dumps({variant: {'arc_all_five_slower': len(row['arc_slower_in_all_five_blocks']),
                               'broad_all_five_slower': len(row['broad_slower_in_all_five_blocks']),
                               'memory_increased_rows': len(row['memory_median_increases'])}
                      for variant, row in variants.items()}))


if __name__ == '__main__':
    main()
