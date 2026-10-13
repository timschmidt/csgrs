"""Qualify frozen counters, then observe one original complete-board run.

The diagnostic adds synchronization and branch overhead. Counts measure
executed work, not allocations, elapsed shares or an optimization's savings.
All commands run sequentially in one fresh diagnostic Cargo target.
"""
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
PREP = E / 'preparation-v1'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def verify(preparation, protocol, binary=None):
    assert sha(E / 'prepare_v1.py') == preparation['runner_sha256']
    for name, digest in preparation['inputs_sha256'].items():
        assert sha(E / name) == digest, name
    assert sha(E / 'diagnose_v1.py') == protocol['runner_sha256']
    assert sha(E / 'assess_v1.py') == protocol['assessor_sha256']
    assert sha(PREP / 'protocol.json') == protocol['preparation_sha256']
    assert sha(PREP / 'parent-source-hashes.json.gz') == preparation['parent_hashes_sha256']
    assert sha(PREP / 'source-hashes.json.gz') == preparation['source_hashes_sha256']
    assert sha(PREP / 'reference-source-hashes.json.gz') == preparation['reference_hashes_sha256']
    assert sources() == preparation['main_source_sha256']
    assert all(sha(ROOT / p) == h for p, h in preparation['pending_native_sha256'].items())
    for label, directory in [('parent-source', preparation['parent_source']), ('source', preparation['source']), ('reference-source', preparation['reference_source'])]:
        entries = json.loads(gzip.decompress((PREP / (label + '-hashes.json.gz')).read_bytes()))
        assert all(sha(Path(directory) / p) == h for p, h in entries.items())
    if binary is not None:
        assert sha(Path(binary['path'])) == binary['sha256']


def parse_counts(raw):
    names = ['invalid', 'unknown', 'zero', 'negative', 'positive', 'nonzero_unsigned', 'nonzero_zero_sign']
    rows = []
    for line in raw.splitlines():
        fields = line.split('\t')
        if fields[0] == 'BOUND_SIGN_FALLBACK':
            assert len(fields) == 8
            index, starts, unknown, negative, zero, positive, unfinished = map(int, fields[1:])
            assert index == len(rows) and 0 <= index < 7
            results = [unknown, negative, zero, positive]
            assert all(n >= 0 for n in [starts, *results, unfinished])
            assert sum(results) + unfinished == starts
            rows.append({'bound_tag': names[index], 'starts': starts,
                         'results_unknown_negative_zero_positive': results, 'unfinished': unfinished})
    assert len(rows) == 7
    total = sum(r['starts'] for r in rows)
    assert total > 0
    eligible = sum(rows[i]['starts'] for i in [2, 3, 4])
    agreeing = rows[2]['results_unknown_negative_zero_positive'][2] + rows[3]['results_unknown_negative_zero_positive'][1] + rows[4]['results_unknown_negative_zero_positive'][3]
    declined = sum(rows[i]['results_unknown_negative_zero_positive'][0] for i in [2, 3, 4])
    eligible_unfinished = sum(rows[i]['unfinished'] for i in [2, 3, 4])
    contradictory = eligible - agreeing - declined - eligible_unfinished
    return {'rows': rows, 'started': total, 'completed': sum(sum(r['results_unknown_negative_zero_positive']) for r in rows),
            'unfinished': sum(r['unfinished'] for r in rows), 'eligible_known_sign': eligible,
            'eligible_agreeing': agreeing, 'eligible_fallback_declined': declined,
            'eligible_unfinished': eligible_unfinished, 'eligible_contradictory': contradictory,
            'anomalous_nonzero_zero_sign': rows[6]['starts']}


def main():
    preparation = json.loads((PREP / 'protocol.json').read_text())
    source, target = Path(preparation['source']), Path(preparation['target'])
    assert not target.exists() and not Path(preparation['reference_target']).exists()
    out = E / 'diagnostic-v1'
    out.mkdir(exist_ok=False)
    env = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
           'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never',
           'CARGO_TARGET_DIR': str(target), 'RUSTDOCFLAGS': '-D warnings'}
    scalar = ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--lib']
    commands = [
        ('format', ['cargo', 'fmt', '--all', '--', '--check'], None),
        ('parent-reference', scalar + ['bound_sign_reference_tests', '--', '--test-threads=1'], 3),
        ('candidate-reference', scalar + ['bound_sign_reference_tests', '--', '--test-threads=1'], 3),
        ('counter-calibration', scalar + ['bound_sign_diagnostic::tests', '--', '--test-threads=1'], 3),
        ('counter-sites', scalar + ['bound_sign_site_tests', '--', '--test-threads=1'], 5),
        ('sign-regressions', scalar + ['quadratic_tower_sign_retention_tests', '--', '--test-threads=1'], 11),
        ('cache-frontiers', scalar + ['persistent_laurent_cache_tests', '--', '--test-threads=1'], 6),
        ('scalar-library', scalar + ['--', '--test-threads=1'], 885),
        ('scalar-doctests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--doc', '--', '--test-threads=1'], 28),
        ('endpoint-regressions', ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--all-features', '--lib', 'endpoint_proof_tests', '--', '--test-threads=1'], 5),
        ('strict-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal', '-p', 'hypersolve', '-p', 'hypercurve', '-p', 'hyperdrc', '--all-features', '--all-targets', '--', '-D', 'warnings'], None),
        ('diagnostic-docs', ['cargo', 'doc', '--locked', '-p', 'hyperreal', '--all-features', '--no-deps'], None),
        ('build-board', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--all-features', '--lib', '--no-run', '--message-format=json'], None),
    ]
    protocol = {
        'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
        'assessor_sha256': sha(E / 'assess_v1.py'),
        'preparation_sha256': sha(PREP / 'protocol.json'),
        'source': str(source), 'target': str(target), 'environment': env,
        'compiler': subprocess.check_output(['rustc', '-vV'], text=True),
        'inherited_rustflags': os.environ.get('RUSTFLAGS'),
        'commands': commands, 'command_timeout_seconds': 1800,
        'original_worker_cpu_limit_seconds': 120,
        'gate_test': 'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite',
        'counter_scope': 'Existing root-bound tag at the final exact-sign quadratic fallback, crossed with the unchanged computed result. No fallback bypass, new arithmetic or proof publication.',
        'coverage_limits': 'Instrumentation changes execution cost. Metadata and result counts are not heap allocations or time shares. No estimated runtime improvement or runtime admission follows from these counts.',
        'runtime_admitted': False,
    }
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    phases, tests, binary = [], [], None

    def run(name, command, expected=None, gate=False):
        verify(preparation, protocol, binary)
        wrapped = ['timeout', '--signal=TERM', '--kill-after=10s', '1800', *command]
        print(json.dumps({'start': name}), flush=True)
        command_source = Path(preparation['reference_source']) if name == 'parent-reference' else source
        command_env = env | ({'CARGO_TARGET_DIR': preparation['reference_target']} if name == 'parent-reference' else {})
        started = time.monotonic()
        with (out / (name + '.stdout')).open('w') as stdout, (out / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(wrapped, cwd=command_source, env=os.environ | command_env, stdout=stdout, stderr=stderr).returncode
        row = {'name': name, 'command': wrapped, 'cwd': str(command_source), 'environment': command_env, 'exit_code': code,
               'seconds': time.monotonic() - started,
               'stdout_sha256': sha(out / (name + '.stdout')), 'stderr_sha256': sha(out / (name + '.stderr'))}
        phases.append(row)
        (out / 'commands.json').write_text(json.dumps(phases, indent=2) + '\n')
        print(json.dumps(row), flush=True)
        verify(preparation, protocol, binary)
        assert code == 0 or (gate and code == 101), (name, code)
        raw = (out / (name + '.stdout')).read_text()
        if expected is not None:
            counts = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;', raw)
            assert counts == [(str(expected), '0', '0')], (name, counts)
            tests.append({'name': name, 'passed': expected, 'failed': 0, 'ignored': 0})
            (out / 'tests.json').write_text(json.dumps(tests, indent=2) + '\n')
        return code, raw

    for name, command, expected in commands:
        code, raw = run(name, command, expected)
    events = [json.loads(line) for line in raw.splitlines() if line.startswith('{')]
    artifacts = [row for row in events if row.get('reason') == 'compiler-artifact'
                 and row.get('executable') and row['target']['name'] == 'hyperdrc'
                 and row['target']['kind'] == ['lib'] and row['profile']['test']]
    assert len(artifacts) == 1
    artifact = artifacts[0]
    path = Path(artifact['executable'])
    assert path.is_relative_to(target)
    assert Path(artifact['manifest_path']) == source / 'crates/hyperdrc/Cargo.toml'
    binary = {'path': str(path), 'sha256': sha(path), 'bytes': path.stat().st_size,
              'manifest_path': artifact['manifest_path'], 'features': artifact['features'],
              'package_id': artifact['package_id'], 'target': artifact['target'], 'profile': artifact['profile']}
    (out / 'binary.json').write_text(json.dumps(binary, indent=2) + '\n')
    code, raw = run('full-board', [str(path), protocol['gate_test'], '--exact', '--test-threads=1', '--nocapture'], gate=True)
    both = raw + (out / 'full-board.stderr').read_text()
    assert 'skipping complex project fixture test' not in both
    assert both.count('hyperdrc: finished emit report') == 2
    assert [int(x) for x in re.findall(r'finished run \((\d+) active finding', both)] == [12, 197]
    elapsed = re.search(r'KiCad smoke suite took ([0-9.]+)s of worker time', both)
    if code == 101:
        assert elapsed and float(elapsed[1]) >= 120 and '0 passed; 1 failed' in raw
    else:
        assert '1 passed; 0 failed' in raw
    counts = parse_counts(both)
    (out / 'counts.json').write_text(json.dumps(counts, indent=2) + '\n')
    verify(preparation, protocol, binary)
    result = {'complete': True, 'tests': tests, 'phases': phases,
              'original_worker_cpu_limit_seconds': 120,
              'instrumented_worker_cpu_seconds': float(elapsed[1]) if elapsed else None,
              'original_gate_passed': code == 0, 'both_reports_emitted': True,
              'finding_counts': [12, 197], 'counts': counts,
              'source_parent_main_and_binary_unchanged': True,
              'runtime_admitted': False, 'plan_complete': False}
    (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'complete': True, 'original_gate_passed': code == 0,
                      'counts': counts}), flush=True)


if __name__ == '__main__':
    main()
