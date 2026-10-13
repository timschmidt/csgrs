"""Qualify the frozen endpoint/P4 combination sequentially in a fresh target.

The existing complete-board and copper-neck assertions run separately after
all other tests. Their original fixtures, limits and metrics are unchanged.
All timings here are whole subprocess wall observations; only assertion output
can establish either internal gate measurement. This is not a paired benchmark.
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


def verify(preparation, protocol, binaries):
    assert sources() == preparation['main_source_sha256']
    assert all(sha(ROOT / p) == h for p, h in preparation['pending_native_sha256'].items())
    assert sha(E / 'prepare_v1.py') == preparation['runner_sha256']
    assert sha(E / 'qualify_v1.py') == protocol['runner_sha256'] == preparation['qualification_runner_sha256']
    assert sha(E / 'assess_qualification_v1.py') == protocol['assessor_sha256'] == preparation['qualification_assessor_sha256']
    assert sha(PREP / 'protocol.json') == protocol['preparation_sha256']
    assert sha(PREP / 'parent-source-hashes.json.gz') == preparation['parent_hashes_sha256']
    assert sha(PREP / 'source-hashes.json.gz') == preparation['source_hashes_sha256']
    for label, directory in [('parent-source', preparation['parent_source']), ('source', preparation['source'])]:
        entries = json.loads(gzip.decompress((PREP / (label + '-hashes.json.gz')).read_bytes()))
        assert all(sha(Path(directory) / p) == h for p, h in entries.items())
    assert all(sha(Path(row['path'])) == row['sha256'] for row in binaries.values())


def counts(raw):
    matches = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;', raw)
    assert matches and all(int(row[1]) == 0 for row in matches)
    return {'passed': sum(int(row[0]) for row in matches), 'failed': 0,
            'ignored': sum(int(row[2]) for row in matches), 'groups': len(matches)}


def main():
    preparation = json.loads((PREP / 'protocol.json').read_text())
    source, target = Path(preparation['source']), Path(preparation['target'])
    assert not target.exists()
    out = E / 'qualification-v1'
    out.mkdir(exist_ok=False)
    env = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
           'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never',
           'CARGO_TARGET_DIR': str(target), 'RUSTDOCFLAGS': '-D warnings'}
    gates = [
        {'label': 'full-board', 'test': 'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite',
         'limit_seconds': 120, 'metric': 'worker CPU'},
        {'label': 'copper-neck', 'test': 'zz_complex_project_performance_tests::min_copper_neck_width_completes_on_copper_layers',
         'limit_seconds': 60, 'metric': 'check body wall time'},
    ]
    skips = [arg for gate in gates for arg in ['--skip', gate['test']]]
    cargo_test = ['cargo', 'test', '--locked', '--release']
    commands = [
        ('format', ['cargo', 'fmt', '--all', '--', '--check'], None),
        ('build-scalar-tests', cargo_test + ['-p', 'hyperreal', '--all-features', '--lib', '--no-run', '--message-format=json'], None),
        ('sign-regressions', ['{hyperreal}', 'quadratic_tower_sign_retention_tests', '--test-threads=1'], [11, 0, 1]),
        ('cache-frontiers', ['{hyperreal}', 'persistent_laurent_cache_tests', '--test-threads=1'], [6, 0, 1]),
        ('scalar-library', ['{hyperreal}', '--test-threads=2'], [874, 0, 1]),
        ('scalar-doctests', cargo_test + ['-p', 'hyperreal', '--all-features', '--doc'], [28, 0, 1]),
        ('strict-stack-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal', '-p', 'hypersolve', '-p', 'hypercurve', '-p', 'hyperdrc', '--all-features', '--all-targets', '--', '-D', 'warnings'], None),
        ('solver-tests', cargo_test + ['-p', 'hypersolve', '--all-features', '--', '--test-threads=2'], [974, 0, 10]),
        ('endpoint-regressions', cargo_test + ['-p', 'hypercurve', '--all-features', '--lib', 'endpoint_proof_tests', '--', '--test-threads=1'], [5, 0, 1]),
        ('curve-tests', cargo_test + ['-p', 'hypercurve', '--all-features', '--', '--test-threads=2'], [2494, 33, 51]),
        ('drc-tests-except-separate-gates', cargo_test + ['-p', 'hyperdrc', '--all-features', '--', '--test-threads=2', *skips], [1644, 0, 10]),
        ('stack-docs', ['cargo', 'doc', '--locked', '-p', 'hyperreal', '-p', 'hypersolve', '-p', 'hypercurve', '-p', 'hyperdrc', '--all-features', '--no-deps'], None),
        ('build-board-tests', cargo_test + ['-p', 'hyperdrc', '--all-features', '--lib', '--no-run', '--message-format=json'], None),
    ]
    protocol = {
        'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
        'assessor_sha256': sha(E / 'assess_qualification_v1.py'),
        'preparation_sha256': sha(PREP / 'protocol.json'),
        'source': str(source), 'target': str(target), 'environment': env,
        'compiler': subprocess.check_output(['rustc', '-vV'], text=True),
        'cargo': subprocess.check_output(['cargo', '-V'], text=True),
        'inherited_rustflags': os.environ.get('RUSTFLAGS'),
        'commands': commands, 'gates': gates, 'command_timeout_seconds': 1800,
        'test_count_basis': 'Endpoint parent 863 scalar library tests plus eleven admitted tests. Other suites retain their previous inventories. Focused counts overlap full suites.',
        'gate_duration_note': 'Process wall includes setup and is not worker CPU or copper-neck check-body wall. Passing assertions need not print their internal duration.',
        'native_runtime_admitted': False,
    }
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    phases, tests, binaries = [], [], {}

    def run(name, command, expected=None, gate=False):
        verify(preparation, protocol, binaries)
        command = [binaries[arg[1:-1]]['path'] if arg.startswith('{') and arg.endswith('}') else arg for arg in command]
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
        verify(preparation, protocol, binaries)
        assert code == 0 or (gate and code == 101), (name, code)
        raw = (out / (name + '.stdout')).read_text()
        if expected is not None:
            actual = counts(raw)
            assert [actual['passed'], actual['ignored'], actual['groups']] == expected, (name, actual, expected)
            tests.append({'name': name} | actual)
            (out / 'tests.json').write_text(json.dumps(tests, indent=2) + '\n')
        return raw

    for name, command, expected in commands:
        raw = run(name, command, expected)
        if name in ['build-scalar-tests', 'build-board-tests']:
            package = 'hyperreal' if name == 'build-scalar-tests' else 'hyperdrc'
            events = [json.loads(line) for line in raw.splitlines() if line.startswith('{')]
            artifacts = [event for event in events if event.get('reason') == 'compiler-artifact'
                         and event.get('executable') and event['target']['name'] == package
                         and event['target']['kind'] == ['lib'] and event['profile']['test']]
            assert len(artifacts) == 1
            artifact = artifacts[0]
            assert Path(artifact['manifest_path']) == source / 'crates' / package / 'Cargo.toml'
            binary = Path(artifact['executable'])
            assert binary.is_relative_to(target)
            binaries[package] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size,
                                 'manifest_path': artifact['manifest_path'], 'features': artifact['features'],
                                 'package_id': artifact['package_id'], 'target': artifact['target'], 'profile': artifact['profile']}
            (out / 'binaries.json').write_text(json.dumps(binaries, indent=2) + '\n')
    for gate in gates:
        raw = run(gate['label'], ['/usr/bin/time', '-v', '{hyperdrc}', gate['test'],
                                 '--exact', '--test-threads=1', '--nocapture'], gate=True)
        both = raw + (out / (gate['label'] + '.stderr')).read_text()
        assert 'skipping complex project fixture test' not in both
        assert '1 passed' in raw or '1 failed' in raw
        if gate['label'] == 'full-board':
            assert both.count('hyperdrc: finished emit report') == 2
    verify(preparation, protocol, binaries)
    result = {'complete': True, 'all_passed': all(row['exit_code'] == 0 for row in phases),
              'tests': tests, 'phases': phases, 'source_parent_main_and_binaries_unchanged': True,
              'native_runtime_admitted': False, 'plan_complete': False}
    (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'complete': True, 'all_passed': result['all_passed'], 'tests': tests}), flush=True)


if __name__ == '__main__':
    main()
