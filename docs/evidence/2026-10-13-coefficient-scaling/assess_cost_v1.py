"""Verify every fixed cost row and retain each candidate's tradeoffs separately."""
import gzip
import hashlib
import json
from pathlib import Path
import statistics
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'cost-v1'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    assert not (E / 'cost-assessment-v1.json').exists()
    preparation = json.loads((E / 'preparation-v1/protocol.json').read_text())
    protocol = json.loads((OUT / 'protocol.json').read_text())
    validation = json.loads((E / 'validation-v1/result.json').read_text())
    result = json.loads((OUT / 'result.json').read_text())
    trials = json.loads((OUT / 'trials.json').read_text())
    libraries = json.loads((OUT / 'libraries.json').read_text())
    binaries = json.loads((OUT / 'binaries.json').read_text())
    assert validation['complete'] and result['complete'] and len(trials) == 90
    assert result['mathematical_witnesses_and_inventories_identical']
    assert sha(E / 'measure_v1.py') == protocol['runner_sha256']
    assert sha(E / 'validation-v1/result.json') == protocol['validation_result_sha256']
    assert sha(E / 'preparation-v1/protocol.json') == protocol['preparation_sha256']
    commands = json.loads((OUT / 'commands.json').read_text())
    for command in commands:
        assert command['exit_code'] == 0
        for stream in ['stdout', 'stderr']:
            assert sha(OUT / (command['name'] + '.' + stream)) == command[stream + '_sha256']
    assert len([row for row in commands if row['name'].startswith('preflight-')]) == 18
    original = json.loads(gzip.decompress((E / 'preparation-v1/parent-source-hashes.json.gz').read_bytes()))
    assert all(sha(Path(preparation['parent_source']) / name) == digest for name, digest in original.items())
    for variant, row in preparation['variants'].items():
        assert all(sha(Path(row['source']) / name) == digest for name, digest in (original | row['source_changes_sha256']).items())
        library = libraries[variant]
        assert sha(Path(library['path'])) == library['sha256']
        assert Path(library['path']).is_relative_to(Path(row['cost_target']))
        assert Path(library['manifest_path']) == Path(row['source']) / 'crates/hyperreal/Cargo.toml'
        assert library['features'] == libraries['baseline']['features']
    assert all(sha(Path(row['path'])) == row['sha256'] for row in binaries.values())
    assert sources() == preparation['main_source_sha256']
    assert all(sha(ROOT / name) == digest for name, digest in preparation['pending_native_sha256'].items())
    for suite in protocol['suites']:
        for mode in protocol['modes']:
            selected = [trial for trial in trials if trial['suite'] == suite and trial['mode'] == mode]
            assert len(selected) == 15
            identity = None
            for block, order in enumerate(protocol['fixed_block_order'], 1):
                assert [trial['variant'] for trial in selected if trial['block'] == block] == order
            for trial in selected:
                current = [row for row in trial['rows'] if 'fixture' in row or 'witness' in row]
                if identity is None:
                    identity = current
                assert current == identity
    timing, memory, sizes = [], [], []
    for row in result['summary']:
        base = row['variants']['baseline']
        for variant in ['right', 'both']:
            candidate = row['variants'][variant]
            if row['mode'] == 'time':
                left, right = base['nanos']['samples'], candidate['nanos']['samples']
                assert len(left) == len(right) == 5
                ratios = [y / x for x, y in zip(left, right)]
                timing.append({'variant': variant, 'suite': row['suite'], 'name': row['name'],
                               'baseline_ns': left, 'candidate_ns': right,
                               'baseline_median_ns': statistics.median(left),
                               'candidate_median_ns': statistics.median(right),
                               'median_ratio': statistics.median(right) / statistics.median(left),
                               'block_ratios': ratios, 'median_block_ratio': statistics.median(ratios),
                               'slower_blocks': sum(y > x for x, y in zip(left, right)),
                               'faster_blocks': sum(y < x for x, y in zip(left, right)),
                               'baseline_range_ns': [min(left), max(left)],
                               'candidate_range_ns': [min(right), max(right)]})
            else:
                fields = {}
                for field in base:
                    left, right = base[field]['samples'], candidate[field]['samples']
                    assert len(left) == len(right) == 5
                    delta = [y - x for x, y in zip(left, right)]
                    fields[field] = {'baseline': left, 'candidate': right,
                                     'baseline_median': statistics.median(left), 'candidate_median': statistics.median(right),
                                     'block_deltas': delta, 'median_block_delta': statistics.median(delta),
                                     'increased_blocks': sum(y > x for x, y in zip(left, right))}
                memory.append({'variant': variant, 'suite': row['suite'], 'name': row['name'], 'fields': fields})
    assert len(timing) == len(memory) == 136
    for variant in ['right', 'both']:
        for suite in protocol['suites']:
            for mode in protocol['modes']:
                base = binaries[f'baseline-{suite}-{mode}']
                candidate = binaries[f'{variant}-{suite}-{mode}']
                sizes.append({'variant': variant, 'suite': suite, 'mode': mode,
                              'baseline_bytes': base['bytes'], 'candidate_bytes': candidate['bytes'],
                              'delta_bytes': candidate['bytes'] - base['bytes']})
    adverse_timing = {variant: [row for row in timing if row['variant'] == variant and row['slower_blocks'] == 5]
                      for variant in ['right', 'both']}
    adverse_memory = {variant: [row for row in memory if row['variant'] == variant and
                                any(field['median_block_delta'] > 0 for field in row['fields'].values())]
                      for variant in ['right', 'both']}
    assessment = {
        'decision': 'fixed-scalar-costs-complete-await-variant-retention-review',
        'assessor_sha256': sha(Path(__file__)),
        'sampling': 'Three unchanged suites, two resource modes, five fixed three-variant blocks; 4096 time inputs or 1000 allocation inputs. Eighteen preflights precede 90 measurement processes. All builds are terminal before probes.',
        'checks': 'Four new tests pass on the unchanged baseline; each candidate passes 867 scalar library results, 28 doctests, strict Clippy, formatting and warning-denied docs. Focused test totals overlap the full suite.',
        'all_mathematical_witnesses_and_representation_inventories_match': True,
        'timing_rows': timing, 'memory_rows': memory, 'binary_sizes': sizes,
        'slower_in_all_five_blocks': adverse_timing,
        'any_memory_field_with_positive_median_block_delta': adverse_memory,
        'limits': [
            'Block ratios pair each candidate with the baseline in the same fixed block; this is a three-variant design, not additional samples for any earlier experiment.',
            'Ratios and directions are descriptive, without statistical-significance or application-speedup claims. Every range and adverse row is retained.',
            'Requested Rust allocations exclude native C heaps; retained and peak counters are phase-relative and do not prove eventual reclamation.',
            'Probe binary sizes are not production executable size. Native consumers, full reports, broader/default/feature qualification and original limits remain open.',
        ],
        'evidence_sha256': {str(path.relative_to(E)): sha(path) for path in
                           [OUT / 'protocol.json', OUT / 'commands.json', OUT / 'trials.json', OUT / 'result.json',
                            OUT / 'libraries.json', OUT / 'binaries.json', E / 'correctness-v1.json']},
        'source_parent_main_and_artifacts_unchanged': True, 'runtime_admitted': False, 'plan_complete': False,
    }
    (E / 'cost-assessment-v1.json').write_text(json.dumps(assessment, indent=2) + '\n')
    print(json.dumps({'timing_rows_per_candidate': 68, 'memory_rows_per_candidate': 68,
                      'all_five_slower_rows': {key: len(rows) for key, rows in adverse_timing.items()},
                      'memory_rows_with_any_increase': {key: len(rows) for key, rows in adverse_memory.items()}}))


if __name__ == '__main__':
    main()
