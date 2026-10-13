"""Verify the fixed sign observations and distinguish proof loss from contradiction."""
import gzip
import hashlib
import json
from pathlib import Path
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'observation-v2'
PRIOR = ROOT / 'docs/evidence/2026-10-13-coefficient-zero-radicals'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    destination = E / 'assessment-v2.json'
    assert not destination.exists()
    protocol = json.loads((OUT / 'protocol.json').read_text())
    result = json.loads((OUT / 'result.json').read_text())
    observations = json.loads((OUT / 'observations.json').read_text())
    commands = json.loads((OUT / 'commands.json').read_text())
    libraries = json.loads((OUT / 'libraries.json').read_text())
    binaries = json.loads((OUT / 'binaries.json').read_text())
    assert result['collection_complete'] and len(observations) == protocol['fresh_processes'] == 24
    assert sha(E / 'observe_v2.py') == protocol['runner_sha256']
    assert sha(Path(__file__)) == protocol['assessor_sha256']
    assert sha(E / 'probe_v2.rs') == protocol['probe_sha256']
    assert sha(E / 'failure-v1.json') == protocol['prior_compile_failure_sha256']
    assert sha(PRIOR / 'preparation-v1/protocol.json') == protocol['prior_preparation_sha256']
    assert sha(PRIOR / 'decision-v1.json') == protocol['prior_rejection_sha256']
    assert all(sha(PRIOR / name) == digest for name, digest in protocol['prior_terminal_results_sha256'].items())
    assert protocol['variant_order'] == list(protocol['variants']) == ['main', 'endpoint']
    assert result['observation_processes'] == len(observations)
    assert result['process_failures'] == [row['name'] for row in observations if row['exit_code'] != 0]
    assert result['invalid_output'] == [row['name'] for row in observations if row['parse_error'] is not None]
    assert result['complete'] == (not result['process_failures'] and not result['invalid_output'])
    assert protocol['cases'] == [{'family': family, 'shape': shape, 'scale': scale}
                                 for family in ['quadratic', 'nested', 'biquadratic']
                                 for shape in ['linear', 'square'] for scale in [1, -7]]
    for variant, item in protocol['variants'].items():
        expected = json.loads(gzip.decompress((OUT / (variant + '-source-hashes.json.gz')).read_bytes()))
        assert len(expected) == item['source_count']
        assert all(sha(Path(item['source']) / name) == digest for name, digest in expected.items())
        if variant == 'main':
            assert sources() == expected
        library = libraries[variant]
        assert sha(Path(library['path'])) == library['sha256']
        assert Path(library['path']).is_relative_to(Path(item['target']))
        assert Path(library['manifest_path']) == Path(item['source']) / 'crates/hyperreal/Cargo.toml'
        assert library['features'] == libraries['main']['features']
        assert sha(Path(binaries[variant]['path'])) == binaries[variant]['sha256']
    parent = json.loads(gzip.decompress((OUT / 'parent-source-hashes.json.gz').read_bytes()))
    assert all(sha(Path(protocol['parent_source']) / name) == digest for name, digest in parent.items())
    assert all(sha(ROOT / name) == digest for name, digest in protocol['pending_native_sha256'].items())
    assert all(sha(Path(path)) == digest for entries in json.loads((OUT / 'build-artifacts.json').read_text()).values()
               for path, digest in entries.items())
    names = [f'{variant}-{index:02d}-{case["family"]}-{case["shape"]}-{case["scale"]}'
             for variant in protocol['variant_order'] for index, case in enumerate(protocol['cases'], 1)]
    assert [row['name'] for row in observations] == names
    assert [row['name'] for row in commands] == ['build-main', 'build-endpoint', 'link-main', 'link-endpoint'] + names
    assert all(command['exit_code'] == 0 for command in commands[:4])
    for observation, (variant, case) in zip(observations, [(variant, case) for variant in protocol['variant_order']
                                                         for case in protocol['cases']], strict=True):
        assert observation['variant'] == variant and observation['case'] == case
    for command in commands:
        for stream in ['stdout', 'stderr']:
            assert sha(OUT / (command['name'] + '.' + stream)) == command[stream + '_sha256']
    summaries = {}
    for variant in protocol['variant_order']:
        selected = [row for row in observations if row['variant'] == variant]
        summary = {key: [] for key in ['process_failures', 'uncovered', 'contradictions', 'equality_unproven',
                                      'immediate_weakenings', 'zero_precision_weakenings', 'original_weakenings',
                                      'repeated_weakenings', 'original_clone_weakenings',
                                      'compact_clone_weakenings', 'compact_query_weakenings', 'rows']}
        for observation in selected:
            name, row = observation['name'], observation['row']
            assert observation['exit_code'] == next(command['exit_code'] for command in commands if command['name'] == name)
            if observation['exit_code'] != 0 or observation['parse_error'] is not None:
                summary['process_failures'].append(observation)
                continue
            assert json.loads((OUT / (name + '.stdout')).read_text()) == row
            assert {key: row[key] for key in observation['case']} == observation['case']
            assert row['delta_warmed'] == 'positive'
            assert row['delta_before'] in ['positive', 'negative', 'zero', 'unknown']
            assert isinstance(row['compact_covered'], bool)
            expected = 'negative' if row['scale'] < 0 else 'positive'
            fields = ['original_immediate', 'original_zero', 'original_clone_zero']
            if row['compact_covered']:
                fields += ['compact_immediate', 'compact_zero', 'compact_512', 'clone_zero', 'repeated_zero', 'original_after']
            assert all(row[field] in ['positive', 'negative', 'zero', 'unknown'] for field in fields)
            wrong = [field for field in fields if row[field] not in [expected, 'unknown']]
            if row['delta_before'] not in ['positive', 'unknown']:
                wrong.append('delta_before')
            if row['original_zero'] == expected and row['original_clone_zero'] == 'unknown':
                summary['original_clone_weakenings'].append(name)
            if row['compact_covered']:
                assert isinstance(row['repeated_covered'], bool)
                assert row['difference_tower_sign'] in ['positive', 'negative', 'zero', 'unknown']
                assert row['all_definite_signs_agree'] == (not any(field != 'delta_before' for field in wrong))
                if row['difference_tower_sign'] not in ['zero', 'unknown']:
                    wrong.append('difference_tower_sign')
                if row['difference_tower_sign'] == 'unknown':
                    summary['equality_unproven'].append(name)
                if row['original_immediate'] == expected and row['compact_immediate'] == 'unknown':
                    summary['immediate_weakenings'].append(name)
                if row['original_zero'] == expected and row['compact_zero'] == 'unknown':
                    summary['zero_precision_weakenings'].append(name)
                if row['original_zero'] == expected and row['original_after'] == 'unknown':
                    summary['original_weakenings'].append(name)
                if (row['clone_zero'] == 'unknown'
                        and any(row[field] == expected for field in ['compact_immediate', 'compact_zero', 'compact_512'])):
                    summary['compact_clone_weakenings'].append(name)
                earlier = False
                for field in ['compact_immediate', 'compact_zero', 'compact_512']:
                    if earlier and row[field] == 'unknown':
                        summary['compact_query_weakenings'].append({'name': name, 'field': field})
                    earlier |= row[field] == expected
                if (row['repeated_covered'] and row['repeated_zero'] == 'unknown'
                        and any(row[field] == expected for field in ['compact_immediate', 'compact_zero', 'compact_512'])):
                    summary['repeated_weakenings'].append(name)
            else:
                summary['uncovered'].append(name)
            if wrong:
                summary['contradictions'].append({'name': name, 'fields': wrong, 'row': row})
            summary['rows'].append(row)
        summaries[variant] = summary
    assessment = {
        'decision': 'fixed-sign-transfer-observations-recorded-await-source-review',
        'assessor_sha256': sha(Path(__file__)), 'sources': summaries,
        'scope': 'Twelve fresh processes per source, using the same fixed probe, compiler and all-feature scalar configuration. Current main and endpoint baseline remain separate observations.',
        'limits': [
            'A covered equal reconstruction may still need source analysis to establish why a previously available sign is not carried. This observation does not assume a cause.',
            'Unknown is not a contradictory sign. Uncovered compaction returns None and leaves the original value available.',
            'Zero and -512 are requested precision limits, not a claim of zero computation. The explicit warming request is -1100.',
            'This is not universal cache, abort, concurrency, performance, error-bound or full native qualification. All original geometry failures and the full properties/future-work plan remain open.',
        ],
        'artifacts_sha256': {str(path.relative_to(E)): sha(path) for path in sorted(OUT.iterdir()) if path.is_file()},
        'source_main_libraries_and_binaries_unchanged': True, 'runtime_admitted': False, 'plan_complete': False,
    }
    destination.write_text(json.dumps(assessment, indent=2) + '\n')
    print(json.dumps({variant: {key: len(values) for key, values in summary.items()} for variant, summary in summaries.items()}))


if __name__ == '__main__':
    main()
