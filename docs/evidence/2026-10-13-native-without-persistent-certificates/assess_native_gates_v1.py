"""Audit the closed original-gate run without substituting process wall time."""
import gzip
import json
from pathlib import Path
import re

from common_v1 import E, PREP, sha, dump, verify


def main():
    destination = E / 'native-gates-assessment-v1.json'
    assert not destination.exists()
    out = E / 'native-gates-v1'
    prep = json.loads((PREP / 'protocol.json').read_text())
    verify(prep, full=True)
    result = json.loads((out / 'result.json').read_text())
    assert result['complete'] and result['commands'] == 10
    assert all(sha(out / p) == h for p, h in result['artifact_sha256'].items())
    protocol = json.loads((out / 'protocol.json').read_text())
    assert protocol['runner_sha256'] == sha(E / 'native_gates_v1.py')
    assert protocol['scope_sha256'] == sha(E / 'native-gates-scope-v1.json')
    assert protocol['preparation_sha256'] == sha(PREP / 'protocol.json')
    assert protocol['library_result_sha256'] == sha(E / 'library-costs-v1/result.json')
    assert protocol['library_interpretation_sha256'] == sha(E / 'library-interpretation-v1.json')
    binary = json.loads((out / 'binary.json').read_text())
    assert sha(binary['path']) == binary['sha256']
    assert Path(binary['path']).is_relative_to(Path(prep['variants']['candidate']['target']))
    selection = json.loads((out / 'allocator-selection.json').read_text())
    assert selection['verified'] and selection['target'] == 'mi_malloc_aligned'
    assert selection['binary_sha256'] == binary['sha256']
    assert binary['allocator_dependency_features'] == {'mimalloc': [], 'libmimalloc_sys': []}
    commands = json.loads((out / 'commands.json').read_text())
    expected = ['fixture-archive-1', 'fixture-archive-2', 'build-test', 'nm-demangled',
                'nm-raw', 'global-alloc', 'relocations', 'allocator-contracts', 'full-board', 'copper-neck']
    assert [r['name'] for r in commands] == expected
    assert all(r['exit_code'] == 0 for r in commands[:8])
    streams = {}
    for row in commands:
        streams[row['name']] = {}
        assert row['cwd'] == protocol['source']
        for stream in ['stdout', 'stderr']:
            path = out / (row['name'] + '.' + stream + '.gz')
            assert sha(path) == row[stream + '_sha256']
            streams[row['name']][stream] = gzip.decompress(path.read_bytes()).decode()
    assert '2 passed; 0 failed; 0 ignored;' in streams['allocator-contracts']['stdout']
    # Depending on log configuration, report progress can use stderr.
    both_board = ''.join(streams['full-board'].values())
    findings = [int(n) for n in re.findall(r'finished run \((\d+) active finding', both_board)]
    assert findings == [12, 197] and both_board.count('finished emit report') == 2
    rows = []
    for row, gate in zip(commands[-2:], result['gates'], strict=True):
        name = row['name']
        assert gate['name'] == name
        assert row['command'] == ['/usr/bin/time', '-v', binary['path'], gate['test'],
                                  '--exact', '--test-threads=1', '--nocapture']
        raw = ''.join(streams[name].values())
        assert 'skipping complex project fixture' not in raw
        assert 'running 1 test' in raw
        passed = row['exit_code'] == 0
        assert row['exit_code'] in [0, 101] and gate['passed'] == passed
        summary = '1 passed; 0 failed; 0 ignored;' if passed else '0 passed; 1 failed; 0 ignored;'
        assert summary in raw
        limit = 120 if name == 'full-board' else 60
        pattern = (r'KiCad smoke suite took ([0-9.]+)s of worker time' if name == 'full-board'
                   else r'complex project copper neck regression fixture took ([0-9.]+)s')
        match = re.search(pattern, raw)
        seconds = float(match[1]) if match else None
        assert gate['limit_seconds'] == limit and gate['assertion_seconds'] == seconds
        if passed:
            assert seconds is None
        else:
            assert seconds is not None and seconds >= limit
        rows.append({'name': name, 'test': gate['test'], 'passed': passed,
                     'assertion_metric': 'worker thread CPU' if name == 'full-board' else 'check-body wall',
                     'unchanged_limit_seconds': limit, 'printed_assertion_seconds': seconds,
                     'process_wall_seconds': row['process_wall_seconds'],
                     'process_wall_is_not_assertion_metric': True,
                     'stdout_sha256': row['stdout_sha256'], 'stderr_sha256': row['stderr_sha256']})
    passed = all(r['passed'] for r in rows)
    assert passed == result['candidate_original_gates_passed']
    assessment = {
        'status': 'native-viability-passed-with-library-costs-open' if passed else 'native-viability-failed',
        'assessor_sha256': sha(__file__), 'result_sha256': sha(out / 'result.json'),
        'protocol_sha256': sha(out / 'protocol.json'), 'binary': binary,
        'commands': len(commands), 'allocator_contract_tests_passed': 2,
        'allocator_selection': selection, 'original_gates': rows,
        'both_original_gates_passed': passed, 'fixture_findings': findings,
        'fixture_archives': protocol['fixtures'], 'fixture_skips': 0,
        'internal_passing_durations_available': False,
        'single_invocation_each_no_paired_native_speedup_claim': True,
        'complete_report_byte_identity_established': False,
        'wider_stack_qualification_complete': False,
        'library_costs_resolved': False,
        'all_prior_source_and_closed_library_evidence_preserved': True,
        'runtime_admitted': False, 'plan_complete': False,
    }
    dump(destination, assessment)
    print(json.dumps({k: assessment[k] for k in ['status', 'commands', 'original_gates',
                                               'both_original_gates_passed', 'runtime_admitted']}))


if __name__ == '__main__':
    main()
