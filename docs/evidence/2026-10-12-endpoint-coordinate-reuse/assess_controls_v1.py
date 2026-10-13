"""Retain every fixed paired cost, receipt and broad-control observation."""
import gzip
import hashlib
import json
from pathlib import Path
import statistics
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'controls-v1'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    result = json.loads((OUT / 'result.json').read_text())
    protocol = json.loads((OUT / 'protocol.json').read_text())
    preparation = json.loads((E / 'preparation.json').read_text())
    commands = json.loads((OUT / 'commands.json').read_text())
    binaries = json.loads((OUT / 'binaries.json').read_text())
    libraries = json.loads((OUT / 'libraries.json').read_text())
    trials = json.loads((OUT / 'trials.json').read_text())
    historical = json.loads((OUT / 'historical-comparison.json').read_text())
    assert result['complete'] and result['all_variant_identities_match']
    assert sha(E / 'measure_v1.py') == protocol['runner_sha256']
    assert sources() == preparation['main_source_sha256']
    baseline = json.loads(gzip.decompress((E / 'baseline-source-hashes.json.gz').read_bytes()))
    for variant in ['baseline', 'candidate']:
        expected = baseline | (preparation['source_changes_sha256'] if variant == 'candidate' else {})
        assert all(sha(Path(protocol['sources'][variant]) / path) == value for path, value in expected.items())
        assert all(sha(Path(row['path'])) == row['sha256'] for row in libraries[variant].values())
    assert all(sha(Path(row['path'])) == row['sha256'] for row in binaries.values())
    for command in commands:
        assert command['exit_code'] == 0
        for stream in ['stdout', 'stderr']:
            assert sha(OUT / (command['name'] + '.' + stream)) == command[stream + '_sha256']
    assert len(trials) == 36
    timing, memory, broad = [], [], []
    for mode in ['time', 'memory', 'broad']:
        selected = [trial for trial in trials if trial['mode'] == mode]
        assert len(selected) == 10
        for name in [row['name'] for row in selected[0]['rows'] if 'name' in row]:
            rows = {}
            for variant in ['baseline', 'candidate']:
                indexed = sorted([trial for trial in selected if trial['variant'] == variant], key=lambda row: row['index'])
                assert [trial['index'] for trial in indexed] == [1, 2, 3, 4, 5]
                rows[variant] = [next(row for row in trial['rows'] if row.get('name') == name) for trial in indexed]
            if mode in ['time', 'broad']:
                field = 'nanos' if mode == 'time' else 'elapsed_ns'
                samples = {variant: [row[field] for row in values] for variant, values in rows.items()}
                ratios = [c / b for b, c in zip(samples['baseline'], samples['candidate'])]
                record = {'name': name, 'samples_ns': samples, 'paired_ratios': ratios,
                          'median_paired_ratio': statistics.median(ratios),
                          'ratio_of_medians': statistics.median(samples['candidate']) / statistics.median(samples['baseline']),
                          'slower_pairs': sum(value > 1 for value in ratios), 'faster_pairs': sum(value < 1 for value in ratios)}
                (timing if mode == 'time' else broad).append(record)
            else:
                fields = ['allocation_calls', 'allocated_bytes', 'retained_bytes_delta', 'peak_bytes_delta']
                samples = {variant: {field: [row[field] for row in values] for field in fields} for variant, values in rows.items()}
                deltas = {field: [c - b for b, c in zip(samples['baseline'][field], samples['candidate'][field])] for field in fields}
                memory.append({'name': name, 'samples': samples, 'paired_deltas': deltas,
                               'median_paired_deltas': {field: statistics.median(values) for field, values in deltas.items()}})
    assert len(timing) == len(memory) == 48 and len(broad) == 231
    abort = [trial for trial in trials if trial['mode'] == 'abort']
    assert len(abort) == 6 and all(len(trial['rows']) == 210 for trial in abort)
    assert all(trial['rows'] == abort[0]['rows'] for trial in abort)
    sizes = {mode: {'baseline_bytes': binaries['baseline-' + mode]['bytes'],
                    'candidate_bytes': binaries['candidate-' + mode]['bytes'],
                    'byte_delta': binaries['candidate-' + mode]['bytes'] - binaries['baseline-' + mode]['bytes']}
             for mode in ['time', 'memory', 'broad', 'abort']}
    assessment = {
        'decision': 'fixed-controls-recorded-requires-native-report-and-tradeoff-review',
        'arc_timing': timing, 'arc_memory': memory, 'broad_timing': broad,
        'arc_slower_in_all_five_pairs': [row for row in timing if row['slower_pairs'] == 5],
        'broad_slower_in_all_five_pairs': [row for row in broad if row['slower_pairs'] == 5],
        'memory_increases': [row for row in memory if any(value > 0 for value in row['median_paired_deltas'].values())],
        'broad_and_arc_variant_witnesses_identical': True, 'historical_comparison': historical,
        'final_coordinate_abort': {'fresh_processes': 6, 'rows_each': 210, 'complete_receipts_identical': True,
                                   'scope': 'Six families, seven contexts and five active/repeated/cleared-prepared/reenabled/cleared-final stages. Availability is checked on the final query coordinates before and after each public query sequence.'},
        'probe_binary_sizes': sizes,
        'limits': [
            'All five pairs are retained for every row, including adverse controls. Five paired samples describe this run and are not confidence intervals or universal regression bounds.',
            'Construction, first, warm and cloned queries are separate rows. First uses fresh fixture graphs in a process with potentially shared constants. Clone timing excludes cloning.',
            'Allocation counters cover requested Rust bytes, not allocator metadata or native C heaps. Retained bytes include output vectors. Time binaries have no counting allocator.',
            'The historical cost-probe abort is attached to an intermediate value. Its label does not establish final-coordinate abortion; the separate 210-row receipt probe checks that behavior directly.',
            'A focused arc gain is not application speedup evidence. Broad unchanged controls and both original native limits remain part of the admission decision.',
            'The 231 rows are historical capability witnesses, not 231 new proofs, and their full-application report counterpart is checked separately.',
        ],
        'artifacts_sha256': {str(path.relative_to(E)): sha(path) for path in sorted(OUT.rglob('*')) if path.is_file()},
        'main_runtime_unchanged': True, 'runtime_admitted': False, 'plan_complete': False,
    }
    destination = E / 'controls-assessment-v1.json'
    assert not destination.exists()
    destination.write_text(json.dumps(assessment, indent=2) + '\n')
    print(json.dumps({'arc_all_five_slower': [(row['name'], row['median_paired_ratio']) for row in assessment['arc_slower_in_all_five_pairs']],
                      'broad_all_five_slower': [(row['name'], row['median_paired_ratio']) for row in assessment['broad_slower_in_all_five_pairs']],
                      'memory_increases': len(assessment['memory_increases']), 'historical_witnesses_identical': historical['all_231_historical_witnesses_identical'], 'sizes': sizes}))


if __name__ == '__main__':
    main()
