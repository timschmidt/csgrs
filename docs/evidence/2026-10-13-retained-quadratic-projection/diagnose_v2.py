"""Qualify frozen counters, then observe one original complete-board run.

The diagnostic adds synchronization and context overhead. Counts measure
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
    assert sha(E / 'preparation-failure-v1.json') == preparation['preparation_failure_sha256']
    failure = json.loads((E / 'preparation-failure-v1.json').read_text())
    failed_hashes = E / 'preparation-v1/failed-source-hashes.json.gz'
    assert sha(failed_hashes) == failure['source_hashes_sha256']
    failed_entries = json.loads(gzip.decompress(failed_hashes.read_bytes()))
    assert all(sha(Path(failure['failed_source']) / p) == h for p, h in failed_entries.items())
    assert sha(E / 'diagnose_v2.py') == protocol['runner_sha256'] == preparation['diagnostic_runner_sha256']
    assert sha(E / 'assess_v2.py') == protocol['assessor_sha256'] == preparation['assessor_sha256']
    assert sha(E / 'retained_projection_diagnostic.rs') == preparation['helper_input_sha256']
    assert sha(E / 'retained_projection_site_tests.rs') == preparation['site_tests_input_sha256']
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
    queries, reads, copies = [], [], []
    for line in raw.splitlines():
        fields = line.split('\t')
        if fields[0] == 'RETAINED_PROJECTION_QUERY':
            assert len(fields) == 6
            queries.append(dict(zip(['started', 'succeeded', 'declined', 'unwound', 'original_retries'], map(int, fields[1:]))))
        elif fields[0] == 'RETAINED_PROJECTION_READ':
            assert len(fields) == 5
            reads.append({'role': fields[1]} | dict(zip(['missing_cell', 'missing_form', 'completed_copies'], map(int, fields[2:]))))
        elif fields[0] == 'RETAINED_PROJECTION_COPY':
            assert len(fields) == 7
            copies.append({'role': fields[1], 'shape': fields[2]} | dict(zip(['copies', 'term_entries', 'radical_entries', 'nonempty_radical_vectors'], map(int, fields[3:]))))
    assert len(queries) == 1 and len(reads) == 2 and len(copies) == 12
    assert [row['role'] for row in reads] == ['root', 'recursive']
    shapes = ['empty', 'rational', 'quadratic', 'multiple_radicals', 'nonzero_power', 'multiple_powers']
    assert [(r['role'], r['shape']) for r in copies] == [(role, shape) for role in ['root', 'recursive'] for shape in shapes]
    query = queries[0]
    assert query['started'] > 0 and query['unwound'] == 0
    assert query['started'] == query['succeeded'] + query['declined']
    assert 0 <= query['original_retries'] <= query['started']
    assert sum(reads[0][name] for name in ['missing_cell', 'missing_form', 'completed_copies']) == query['started']
    for read in reads:
        same_role = [row for row in copies if row['role'] == read['role']]
        assert sum(row['copies'] for row in same_role) == read['completed_copies']
        for row in same_role:
            assert all(row[name] >= 0 for name in ['copies', 'term_entries', 'radical_entries', 'nonempty_radical_vectors'])
            assert row['nonempty_radical_vectors'] <= row['term_entries']
            if row['shape'] == 'empty':
                assert row['term_entries'] == row['radical_entries'] == row['nonempty_radical_vectors'] == 0
            elif row['shape'] in ['rational', 'quadratic', 'multiple_radicals', 'nonzero_power']:
                assert row['term_entries'] == row['copies']
            if row['shape'] == 'rational':
                assert row['radical_entries'] == row['nonempty_radical_vectors'] == 0
            elif row['shape'] == 'quadratic':
                assert row['radical_entries'] == row['nonempty_radical_vectors'] == row['copies']
            elif row['shape'] == 'multiple_radicals':
                assert row['radical_entries'] >= 2 * row['copies']
                assert row['nonempty_radical_vectors'] == row['copies']
            elif row['shape'] == 'multiple_powers':
                assert row['term_entries'] >= 2 * row['copies']
    return {'queries': query, 'reads': reads, 'copies': copies}


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
        ('counter-calibration', scalar + ['retained_projection_diagnostic::tests', '--', '--test-threads=1'], 4),
        ('counter-sites', scalar + ['retained_projection_site_tests', '--', '--test-threads=1'], 5),
        ('sign-regressions', scalar + ['quadratic_tower_sign_retention_tests', '--', '--test-threads=1'], 11),
        ('cache-frontiers', scalar + ['persistent_laurent_cache_tests', '--', '--test-threads=1'], 6),
        ('scalar-library', scalar + ['--', '--test-threads=1'], 883),
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
        'counter_scope': 'Only existing cache accessors and original retries within exact quadratic projection. Address-only thread context separates root and recursive reads. Completed to_polynomial copies are counted at the return site; shape fields inspect retained vector lengths and term powers only.',
        'coverage_limits': 'Instrumentation changes execution cost. Shape and copied-entry counts are not heap allocations or time shares. No estimated runtime improvement or runtime admission follows from these counts.',
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
                      'queries': counts['queries'], 'reads': counts['reads']}), flush=True)


if __name__ == '__main__':
    main()
