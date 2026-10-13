"""Freeze current-main source bytes and run the sign-retention regressions.

The previous native source manifest supplies only the file inventory; every
copied byte comes from current main. Three experiment-only files are absent
from main and excluded. All main scalar/consumer sources and 35 pending native
entries are included. No prior experimental runtime is inherited.
"""
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tarfile
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
Q = ROOT / 'docs/evidence/2026-10-13-quadratic-tower-sign-transfer'
OUT = E / 'baseline-v1'
WORK = ROOT / 'target/quadratic-tower-sign-retention'
SOURCE = WORK / 'baseline/source'
TARGET = WORK / 'baseline/qualification-target'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    OUT.mkdir(exist_ok=False)
    SOURCE.mkdir(parents=True, exist_ok=False)
    guard = sources()
    previous = json.loads((Q / 'observation-v2/protocol.json').read_text())
    assert guard == json.loads(gzip.decompress((Q / 'observation-v2/main-source-hashes.json.gz').read_bytes()))
    pending = previous['pending_native_sha256']
    inventory_file = ROOT / 'docs/evidence/2026-10-13-coefficient-zero-radicals/preparation-v1/parent-source-hashes.json.gz'
    inventory = json.loads(gzip.decompress(inventory_file.read_bytes()))
    absent = sorted(name for name in inventory if not (ROOT / name).is_file())
    assert absent == ['crates/hypercurve/src/bezier_region/metadata_order_tests.rs',
                      'crates/hypercurve/src/segment/endpoint_proof_tests.rs',
                      'crates/hyperreal/src/computable/node/persistent_laurent_cache_tests.rs']
    names = sorted((set(inventory) - set(absent)) | set(guard) | set(pending))
    original = {name: sha(ROOT / name) for name in names}
    for name in names:
        destination = SOURCE / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / name, destination)
    facade = 'crates/hyperreal/src/real/arithmetic/quadratic_tower_sign.rs'
    test = 'crates/hyperreal/src/real/arithmetic/quadratic_tower_sign_retention_tests.rs'
    assert test not in original
    prefix = '#[cfg(test)]\nmod quadratic_tower_sign_retention_tests;\n\n'
    (SOURCE / facade).write_text(prefix + (ROOT / facade).read_text())
    shutil.copyfile(E / Path(test).name, SOURCE / test)
    snapshot = original | {name: sha(SOURCE / name) for name in [facade, test]}
    for name, entries in [('original', original), ('baseline', snapshot)]:
        (OUT / (name + '-source-hashes.json.gz')).write_bytes(gzip.compress((json.dumps(entries, sort_keys=True) + '\n').encode(), mtime=0))
    with tarfile.open(OUT / 'test-delta.tar.gz', 'w:gz') as archive:
        for name in [facade, test]:
            archive.add(SOURCE / name, arcname=name)
    env = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
           'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never', 'CARGO_TARGET_DIR': str(TARGET)}
    protocol = {'scope': __doc__, 'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
                'runner_sha256': sha(Path(__file__)), 'test_sha256': sha(E / Path(test).name),
                'observation_decision_sha256': sha(Q / 'decision-v1.json'),
                'inventory_file_sha256': sha(inventory_file), 'excluded_experiment_files': absent,
                'source': str(SOURCE), 'target': str(TARGET), 'original_source_count': len(original),
                'baseline_source_count': len(snapshot), 'main_source_sha256': guard, 'pending_native_sha256': pending,
                'source_changes_sha256': {name: snapshot[name] for name in [facade, test]},
                'test_delta_sha256': sha(OUT / 'test-delta.tar.gz'), 'environment': env,
                'compiler': subprocess.check_output(['rustc', '-vV'], text=True),
                'new_test_functions': 9, 'expected_regression_failures': 6, 'expected_control_passes': 3,
                'runtime_unchanged_after_removing_test_module': (SOURCE / facade).read_text().removeprefix(prefix) == (ROOT / facade).read_text(),
                'runtime_admitted': False}
    assert protocol['runtime_unchanged_after_removing_test_module']
    (OUT / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    commands = []
    executable = None

    def verify():
        assert sha(Path(__file__)) == protocol['runner_sha256']
        assert sha(E / Path(test).name) == protocol['test_sha256']
        assert sha(Q / 'decision-v1.json') == protocol['observation_decision_sha256']
        assert sources() == guard
        assert all(sha(ROOT / name) == digest for name, digest in original.items())
        assert all(sha(SOURCE / name) == digest for name, digest in snapshot.items())
        assert all(sha(ROOT / name) == digest for name, digest in pending.items())
        if executable:
            assert sha(Path(executable['path'])) == executable['sha256']

    def run(name, command, expected=0):
        verify()
        wrapped = ['timeout', '--signal=TERM', '--kill-after=10s', '1800', *command]
        print(json.dumps({'start': name}), flush=True)
        started = time.monotonic()
        with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(wrapped, cwd=SOURCE, env=os.environ | env, stdout=stdout, stderr=stderr).returncode
        row = {'name': name, 'command': wrapped, 'cwd': str(SOURCE), 'exit_code': code, 'expected_exit_code': expected,
               'seconds': time.monotonic() - started, 'stdout_sha256': sha(OUT / (name + '.stdout')),
               'stderr_sha256': sha(OUT / (name + '.stderr'))}
        commands.append(row)
        (OUT / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
        print(json.dumps({key: row[key] for key in ['name', 'exit_code', 'seconds']}), flush=True)
        verify()
        assert code == expected, name
        return (OUT / (name + '.stdout')).read_text()

    run('fmt', ['cargo', 'fmt', '--all', '--', '--check'])
    raw = run('build', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--lib', '--no-run', '--message-format=json'])
    matches = [row for row in map(json.loads, raw.splitlines()) if row.get('reason') == 'compiler-artifact'
               and row['target']['name'] == 'hyperreal' and row.get('executable')]
    assert len(matches) == 1
    artifact = matches[0]
    binary = Path(artifact['executable'])
    assert binary.is_relative_to(TARGET) and artifact['profile']['test']
    assert Path(artifact['manifest_path']) == SOURCE / 'crates/hyperreal/Cargo.toml'
    executable = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size,
                  'manifest_path': artifact['manifest_path'], 'features': artifact['features'], 'profile': artifact['profile']}
    (OUT / 'binary.json').write_text(json.dumps(executable, indent=2) + '\n')
    listed = run('list', [str(binary), 'quadratic_tower_sign_retention_tests', '--list'])
    tests = [line.removesuffix(': test') for line in listed.splitlines() if line.endswith(': test')]
    assert len(tests) == 9
    raw = run('regressions', [str(binary), 'quadratic_tower_sign_retention_tests', '--nocapture', '--test-threads=1'], 101)
    results = re.findall(r'^test (\S+) \.\.\. (ok|FAILED)$', raw, re.MULTILINE)
    assert sorted(name for name, _ in results) == sorted(tests)
    failed = [name for name, status in results if status == 'FAILED']
    passed = [name for name, status in results if status == 'ok']
    assert len(failed) == 6 and len(passed) == 3
    assert all(name.endswith('_retains_sign') for name in failed)
    verify()
    result = {'complete': True, 'regression_failures': failed, 'control_passes': passed,
              'source_and_main_unchanged': True, 'runtime_admitted': False,
              'limits': 'Nine focused tests. This does not qualify runtime or establish universal sign, abort, cache or resource behavior.'}
    (OUT / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
