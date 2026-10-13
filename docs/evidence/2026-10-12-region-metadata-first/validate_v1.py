"""Validate the frozen metadata-order candidate and its original native gates.

Commands run sequentially against a dedicated Cargo target. Existing tests and
fixtures are unchanged. Native timing assertions retain their original limits.
"""
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
WORK = ROOT / 'target/region-metadata-first'
SOURCE = WORK / 'source'
BASE = ROOT / 'target/small-integer-certificates/source'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    out = E / 'validation-v1'
    out.mkdir(exist_ok=False)
    preparation = json.loads((E / 'preparation.json').read_text())
    original = json.loads(gzip.decompress((E / 'baseline-source-hashes.json.gz').read_bytes()))
    current = original | preparation['source_changes_sha256']
    env = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
           'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never',
           'CARGO_TARGET_DIR': str(WORK / 'target'), 'RUSTDOCFLAGS': '-D warnings'}
    commands = [
        ('format', ['cargo', 'fmt', '--all', '--', '--check']),
        ('metadata-regressions', ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--all-features', '--lib', 'metadata_order_tests', '--', '--test-threads=1']),
        ('strict-consumer-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hypercurve', '-p', 'hyperdrc', '--all-features', '--all-targets', '--', '-D', 'warnings']),
        ('curve-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--all-features', '--', '--test-threads=2']),
        ('drc-regressions', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--all-features', '--test', 'native_linear_strokes', '--', '--test-threads=1']),
        ('curve-docs', ['cargo', 'doc', '--locked', '-p', 'hypercurve', '--all-features', '--no-deps']),
        ('build-board-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--all-features', '--lib', '--no-run', '--message-format=json']),
    ]
    gates = [
        {'label': 'full-board', 'test': 'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite', 'limit_seconds': 120, 'metric': 'worker CPU'},
        {'label': 'copper-neck', 'test': 'zz_complex_project_performance_tests::min_copper_neck_width_completes_on_copper_layers', 'limit_seconds': 60, 'metric': 'check wall time'},
    ]
    protocol = {'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
                'preparation_sha256': sha(E / 'preparation.json'),
                'commands': commands, 'environment': env, 'gates': gates,
                'runtime_admitted': False}
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    phases = []

    def verify():
        assert sources() == preparation['main_source_sha256']
        assert all(sha(SOURCE / name) == digest for name, digest in current.items())
        assert all(sha(BASE / name) == digest for name, digest in original.items())
        assert sha(Path(__file__)) == protocol['runner_sha256']

    def run(name, command, allow_failure=False):
        verify()
        started = time.monotonic()
        with (out / (name + '.stdout')).open('w') as stdout, (out / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(command, cwd=SOURCE, env=os.environ | env,
                                  stdout=stdout, stderr=stderr).returncode
        verify()
        row = {'name': name, 'command': command, 'exit_code': code, 'seconds': time.monotonic() - started,
               'sources_unchanged': True, 'stdout_sha256': sha(out / (name + '.stdout')),
               'stderr_sha256': sha(out / (name + '.stderr'))}
        phases.append(row)
        (out / 'result.json').write_text(json.dumps({'phases': phases, 'complete': False}, indent=2) + '\n')
        print(json.dumps(row), flush=True)
        if code and not allow_failure:
            raise SystemExit(code)
        return (out / (name + '.stdout')).read_text()

    for name, command in commands:
        log = run(name, command)
    events = [json.loads(line) for line in log.splitlines() if line.startswith('{')]
    artifacts = [event for event in events if event.get('reason') == 'compiler-artifact'
                 and event.get('executable') and event['target']['name'] == 'hyperdrc']
    assert len(artifacts) == 1
    artifact = artifacts[0]
    assert Path(artifact['manifest_path']) == SOURCE / 'crates/hyperdrc/Cargo.toml'
    binary = Path(artifact['executable'])
    assert binary.is_relative_to(WORK / 'target')
    identity = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size,
                'manifest_path': artifact['manifest_path'], 'features': artifact['features']}
    (out / 'binary.json').write_text(json.dumps(identity, indent=2) + '\n')
    for gate in gates:
        assert sha(binary) == identity['sha256']
        log = run(gate['label'], ['/usr/bin/time', '-v', str(binary), gate['test'], '--exact',
                                 '--test-threads=1', '--nocapture'], allow_failure=True)
        both = log + (out / (gate['label'] + '.stderr')).read_text()
        assert 'skipping complex project fixture test' not in both
        assert '1 passed' in log or '1 failed' in log
        if gate['label'] == 'full-board':
            assert both.count('hyperdrc: finished emit report') == 2
    (out / 'result.json').write_text(json.dumps({'phases': phases, 'complete': True,
                                               'all_passed': all(row['exit_code'] == 0 for row in phases),
                                               'runtime_admitted': False}, indent=2) + '\n')


if __name__ == '__main__':
    main()
