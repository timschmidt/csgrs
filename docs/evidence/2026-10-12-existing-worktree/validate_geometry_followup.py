"""Run geometry checks omitted after the recorded library-test failure.

The earlier runner stops at the first failing Cargo invocation. This separate
phase retains that failure and runs the remaining integration and documentation
checks against exactly the same frozen sources. It does not relax either board
runtime limit or claim the repository's complete pre-push matrix.
"""
import json
import os
from pathlib import Path
import subprocess
import time

from validate import E, ROOT, sha, sources


def main():
    predecessor = E / 'geometry-layout-validation'
    previous = json.loads((predecessor / 'result.json').read_text())
    assert len(previous['phases']) == 3
    assert all(p['source_unchanged'] for p in previous['phases'])
    assert previous['phases'][-1]['name'] == 'drc-tests'
    assert previous['phases'][-1]['exit_code'] == 101
    initial = json.loads((predecessor / 'protocol.json').read_text())['sources_sha256']
    assert sources() == initial
    out = E / 'geometry-followup-validation'
    out.mkdir(exist_ok=False)
    commands = [
        ('drc-integrations', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--all-features', '--test', '*', '--', '--test-threads=2'], {}),
        ('doctests', ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '-p', 'hyperdrc', '--all-features', '--doc'], {}),
        ('docs', ['cargo', 'doc', '--locked', '-p', 'hypercurve', '-p', 'hyperdrc', '--all-features', '--no-deps'], {'RUSTDOCFLAGS': '-D warnings'}),
    ]
    env = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
           'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never'}
    protocol = {'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                'runner_sha256': sha(Path(__file__)),
                'source_helper_sha256': sha(E / 'validate.py'),
                'predecessor_result_sha256': sha(predecessor / 'result.json'),
                'commands': commands, 'environment': env, 'sources_sha256': initial,
                'scope': __doc__}
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    results = []
    for name, command, extra in commands:
        assert sources() == initial
        started = time.monotonic()
        with (out / (name + '.stdout')).open('w') as stdout, (out / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(command, cwd=ROOT, env=os.environ | env | extra,
                                  stdout=stdout, stderr=stderr).returncode
        unchanged = sources() == initial
        result = {'name': name, 'exit_code': code, 'seconds': time.monotonic() - started,
                  'source_unchanged': unchanged, 'stdout_sha256': sha(out / (name + '.stdout')),
                  'stderr_sha256': sha(out / (name + '.stderr'))}
        results.append(result)
        complete = len(results) == len(commands) and all(r['exit_code'] == 0 and r['source_unchanged'] for r in results)
        (out / 'result.json').write_text(json.dumps({'phases': results, 'complete': complete}, indent=2) + '\n')
        print(json.dumps(result), flush=True)
        assert unchanged
        if code:
            raise SystemExit(code)


if __name__ == '__main__':
    main()
