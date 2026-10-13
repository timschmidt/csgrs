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
PREP = E / 'preparation-v2'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def verify(preparation, protocol, binary=None):
    assert sha(E / 'prepare_v2.py') == preparation['runner_sha256']
    failure = json.loads((E / 'preparation-failure-v1.json').read_text())
    failed_hashes = E / 'preparation-v1/failed-source-hashes.json.gz'
    assert sha(failed_hashes) == failure['source_hashes_sha256']
    failed_entries = json.loads(gzip.decompress(failed_hashes.read_bytes()))
    assert all(sha(Path(failure['failed_source']) / p) == h for p, h in failed_entries.items())
    for name, digest in failure['inputs_sha256'].items():
        assert sha(E / name) == digest, name
    for name, digest in preparation['inputs_sha256'].items():
        assert sha(E / name) == digest, name
    assert sha(E / 'diagnose_v2.py') == protocol['runner_sha256']
    assert sha(E / 'assess_v2.py') == protocol['assessor_sha256']
    assert sha(PREP / 'protocol.json') == protocol['preparation_sha256']
    assert sha(PREP / 'parent-source-hashes.json.gz') == preparation['parent_hashes_sha256']
    assert sha(PREP / 'source-hashes.json.gz') == preparation['source_hashes_sha256']
    assert sources() == preparation['main_source_sha256']
    assert all(sha(ROOT / p) == h for p, h in preparation['pending_native_sha256'].items())
    for label, directory in [('parent-source', preparation['parent_source']), ('source', preparation['source'])]:
        entries = json.loads(gzip.decompress((PREP / (label + '-hashes.json.gz')).read_bytes()))
        assert all(sha(Path(directory) / p) == h for p, h in entries.items())
    if binary is not None:
        assert sha(Path(binary['path'])) == binary['sha256']


def parse_counts(raw):
    inputs, rows = [], []
    for line in raw.splitlines():
        fields = line.split('\t')
        if fields[0] == 'SAME_RADICAND_INPUT':
            assert len(fields) == 3
            inputs.append(dict(zip(['calls', 'narrow_inputs'], map(int, fields[1:]))))
        elif fields[0] == 'SAME_RADICAND_RESULT':
            assert len(fields) == 7 and fields[1] in ['0', '1']
            rows.append({'equal': bool(int(fields[1])), 'branch_entries': int(fields[2]),
                         'completed_by_output_radical_count': list(map(int, fields[3:]))})
    assert len(inputs) == 1 and len(rows) == 2
    assert [row['equal'] for row in rows] == [False, True]
    counts = inputs[0]
    assert counts['calls'] > 0 and 0 <= counts['narrow_inputs'] <= counts['calls']
    assert sum(row['branch_entries'] for row in rows) == counts['narrow_inputs']
    for row in rows:
        assert row['branch_entries'] >= 0
        assert all(value >= 0 for value in row['completed_by_output_radical_count'])
        assert sum(row['completed_by_output_radical_count']) == row['branch_entries']
    assert rows[1]['completed_by_output_radical_count'][2:] == [0, 0]
    return counts | {'comparisons': rows}


def main():
    preparation = json.loads((PREP / 'protocol.json').read_text())
    source, target = Path(preparation['source']), Path(preparation['target'])
    assert not target.exists()
    out = E / 'diagnostic-v2'
    out.mkdir(exist_ok=False)
    env = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
           'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never',
           'CARGO_TARGET_DIR': str(target), 'RUSTDOCFLAGS': '-D warnings'}
    scalar = ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--lib']
    commands = [
        ('format', ['cargo', 'fmt', '--all', '--', '--check'], None),
        ('counter-calibration', scalar + ['same_radicand_diagnostic::tests', '--', '--test-threads=1'], 2),
        ('counter-sites', scalar + ['same_radicand_site_tests', '--', '--test-threads=1'], 3),
        ('sign-regressions', scalar + ['quadratic_tower_sign_retention_tests', '--', '--test-threads=1'], 11),
        ('cache-frontiers', scalar + ['persistent_laurent_cache_tests', '--', '--test-threads=1'], 6),
        ('scalar-library', scalar + ['--', '--test-threads=1'], 879),
        ('scalar-doctests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--doc', '--', '--test-threads=1'], 28),
        ('endpoint-regressions', ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--all-features', '--lib', 'endpoint_proof_tests', '--', '--test-threads=1'], 5),
        ('strict-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal', '-p', 'hypersolve', '-p', 'hypercurve', '-p', 'hyperdrc', '--all-features', '--all-targets', '--', '-D', 'warnings'], None),
        ('diagnostic-docs', ['cargo', 'doc', '--locked', '-p', 'hyperreal', '--all-features', '--no-deps'], None),
        ('build-board', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--all-features', '--lib', '--no-run', '--message-format=json'], None),
    ]
    protocol = {
        'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
        'assessor_sha256': sha(E / 'assess_v2.py'),
        'preparation_sha256': sha(PREP / 'protocol.json'),
        'source': str(source), 'target': str(target), 'environment': env,
        'compiler': subprocess.check_output(['rustc', '-vV'], text=True),
        'inherited_rustflags': os.environ.get('RUSTFLAGS'),
        'commands': commands, 'command_timeout_seconds': 1800,
        'original_worker_cpu_limit_seconds': 120,
        'gate_test': 'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite',
        'counter_scope': 'All coefficient multiplier entries; narrow input shapes have one radical and no opaque atoms on each side. Existing comparison branches supply equal/unequal outcomes without new scalar comparisons; successful returns supply stored radical-vector lengths.',
        'coverage_limits': 'Instrumentation changes execution cost. Product and stored-output counts are not heap allocations or time shares. No estimated runtime improvement or runtime admission follows from these counts.',
        'runtime_admitted': False,
    }
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    phases, tests, binary = [], [], None

    def run(name, command, expected=None, gate=False):
        verify(preparation, protocol, binary)
        wrapped = ['timeout', '--signal=TERM', '--kill-after=10s', '1800', *command]
        print(json.dumps({'start': name}), flush=True)
        started = time.monotonic()
        with (out / (name + '.stdout')).open('w') as stdout, (out / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(wrapped, cwd=source, env=os.environ | env, stdout=stdout, stderr=stderr).returncode
        row = {'name': name, 'command': wrapped, 'cwd': str(source), 'exit_code': code,
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
