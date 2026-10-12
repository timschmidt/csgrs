"""Validate both retained log streams and execute the omitted copper-neck gate.

The original board process completed and failed its unchanged 120-second
assertion. Its runner then searched stdout for progress written to stderr.
Preserve that failure and every original artifact; do not rerun the board or
weaken either runtime assertion to repair a log-accounting error.
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
WORK = ROOT / 'target/persistent-laurent-cache'
SOURCE = WORK / 'candidate/source'
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources

sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    out = E / 'native-gate-v1-followup'
    out.mkdir(exist_ok=False)
    previous = E / 'native-gate-v1'
    result = json.loads((previous / 'result.json').read_text())
    assert len(result['phases']) == 4 and not result['complete']
    assert all(p['exit_code'] == 0 for p in result['phases'][:3])
    assert result['phases'][-1]['name'] == 'full-board'
    assert result['phases'][-1]['exit_code'] == 101
    for phase in result['phases']:
        assert phase['sources_unchanged']
        for suffix in ['stdout', 'stderr']:
            assert sha(previous / (phase['name'] + '.' + suffix)) == phase[suffix + '_sha256']
    stdout = (previous / 'full-board.stdout').read_text()
    stderr = (previous / 'full-board.stderr').read_text()
    assert stdout.count('hyperdrc: finished emit report') == 0
    assert stderr.count('hyperdrc: finished emit report') == 2
    assert '1 failed' in stdout
    assert '159.882472765s of worker time' in stderr
    assert 'skipping complex project fixture test' not in stdout + stderr
    preparation = json.loads((E / 'preparation.json').read_text())
    original = json.loads(gzip.decompress((E / 'initial-source-hashes.json.gz').read_bytes()))
    delta = json.loads((E / 'validation-v1/protocol.json').read_text())['source_changes_sha256']
    current = original | delta
    protocol = json.loads((previous / 'protocol.json').read_text())
    identity = json.loads((previous / 'binary.json').read_text())
    binary = Path(identity['path'])
    assert sha(binary) == identity['sha256']
    command = ['/usr/bin/time', '-v', str(binary), protocol['gates'][1]['test'], '--exact', '--test-threads=1', '--nocapture']
    correction = {'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
                  'original_runner_sha256': sha(E / 'native_gate_v1.py'),
                  'original_result_sha256': sha(previous / 'result.json'),
                  'original_runner_failure': 'AssertionError at native_gate_v1.py:95: assert log.count("hyperdrc: finished emit report") == 2',
                  'source_data_valid': True, 'original_full_board_worker_cpu_seconds': 159.882472765,
                  'original_full_board_limit_seconds': 120,
                  'command': command, 'environment': protocol['environment'],
                  'binary_sha256': identity['sha256'], 'runtime_admitted': False}
    (out / 'protocol.json').write_text(json.dumps(correction, indent=2) + '\n')

    def verify():
        assert sources() == preparation['main_source_sha256']
        assert all(sha(SOURCE / p) == expected for p, expected in current.items())
        assert sha(binary) == identity['sha256']

    verify()
    started = time.monotonic()
    with (out / 'copper-neck.stdout').open('w') as stdout_file, (out / 'copper-neck.stderr').open('w') as stderr_file:
        code = subprocess.run(command, cwd=SOURCE, env=os.environ | protocol['environment'],
                              stdout=stdout_file, stderr=stderr_file).returncode
    verify()
    log = (out / 'copper-neck.stdout').read_text() + (out / 'copper-neck.stderr').read_text()
    assert 'skipping complex project fixture test' not in log
    assert '1 passed' in log or '1 failed' in log
    result = {'exit_code': code, 'seconds': time.monotonic() - started,
              'sources_unchanged': True, 'binary_unchanged': True,
              'stdout_sha256': sha(out / 'copper-neck.stdout'),
              'stderr_sha256': sha(out / 'copper-neck.stderr'), 'complete': True,
              'original_full_board_gate_passed': False, 'runtime_admitted': False}
    (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
