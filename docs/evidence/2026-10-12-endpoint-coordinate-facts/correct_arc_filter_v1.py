"""Run the missed arc regression without modifying the executed diagnostic."""
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
SOURCE = ROOT / 'target/endpoint-coordinate-facts/diagnostic-v1/source'
PARENT = ROOT / 'docs/evidence/2026-10-12-region-metadata-first'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    out = E / 'arc-filter-correction-v1'
    out.mkdir(exist_ok=False)
    prior = json.loads((E / 'diagnostic-v1/protocol.json').read_text())
    parent = json.loads((PARENT / 'preparation.json').read_text())
    expected = json.loads(gzip.decompress((PARENT / 'baseline-source-hashes.json.gz').read_bytes()))
    expected |= parent['source_changes_sha256'] | prior['source_changes_sha256']
    protocol = {'runner_sha256': sha(Path(__file__)), 'prior_protocol_sha256': sha(E / 'diagnostic-v1/protocol.json'),
                'reason': 'The original segment::tests filter matched zero tests. The function is in segment::policy_cache_tests. The original zero-test logs and failed assessment remain unchanged.',
                'expected_test_count': 1}
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    phases = []

    def verify():
        assert sources() == parent['main_source_sha256']
        assert all(sha(SOURCE / name) == digest for name, digest in expected.items())
        assert sha(Path(__file__)) == protocol['runner_sha256']

    def run(name, command):
        verify()
        start = time.monotonic()
        with (out / (name + '.stdout')).open('w') as stdout, (out / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(command, cwd=SOURCE, env=os.environ | prior['environment'], stdout=stdout, stderr=stderr).returncode
        verify()
        row = {'name': name, 'command': command, 'exit_code': code, 'seconds': time.monotonic() - start,
               'stdout_sha256': sha(out / (name + '.stdout')), 'stderr_sha256': sha(out / (name + '.stderr'))}
        phases.append(row)
        (out / 'result.json').write_text(json.dumps({'complete': False, 'phases': phases}, indent=2) + '\n')
        assert code == 0
        return (out / (name + '.stdout')).read_text()

    log = run('build', ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--all-features', '--lib', '--no-run', '--message-format=json'])
    events = [json.loads(line) for line in log.splitlines() if line.startswith('{')]
    artifacts = [row for row in events if row.get('reason') == 'compiler-artifact' and row.get('executable') and row['target']['name'] == 'hypercurve']
    assert len(artifacts) == 1
    artifact = artifacts[0]
    assert Path(artifact['manifest_path']) == SOURCE / 'crates/hypercurve/Cargo.toml'
    binary = Path(artifact['executable'])
    assert binary.is_relative_to(Path(prior['environment']['CARGO_TARGET_DIR']))
    identity = {'path': str(binary), 'sha256': sha(binary), 'manifest_path': artifact['manifest_path'], 'features': artifact['features']}
    log = run('arc-regression', [str(binary), 'segment::policy_cache_tests::sweep_membership_uses_a_decisive_half_plane', '--exact', '--test-threads=1'])
    assert 'test result: ok. 1 passed; 0 failed; 0 ignored;' in log
    assert sha(binary) == identity['sha256']
    result = {'complete': True, 'phases': phases, 'binary': identity, 'tests_passed': 1, 'source_unchanged': True}
    (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
