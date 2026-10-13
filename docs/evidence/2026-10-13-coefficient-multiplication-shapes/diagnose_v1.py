"""Observe coefficient operand shapes without changing the original arithmetic.

A separate frozen source and Cargo target retain the endpoint candidate. Fixed
integer counters inspect vector lengths only. The original complete-board test,
120-second worker CPU assertion, every query and cache operation remain intact.
Diagnostic reporting happens after the original assertion through a drop guard.
"""
import difflib
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
PARENT = ROOT / 'docs/evidence/2026-10-12-endpoint-coordinate-reuse'
PROFILE = ROOT / 'docs/evidence/2026-10-13-endpoint-candidate-profile/assessment-v1.json'
WORK = ROOT / 'target/coefficient-multiplication-shapes'
SOURCE = WORK / 'source'
TARGET = WORK / 'target'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def replace(text, before, after):
    assert text.count(before) == 1, before
    return text.replace(before, after)


def main():
    out = E / 'diagnostic-v1'
    out.mkdir(exist_ok=False)
    WORK.mkdir(exist_ok=False)
    parent = json.loads((PARENT / 'preparation.json').read_text())
    expected = json.loads(gzip.decompress((PARENT / 'baseline-source-hashes.json.gz').read_bytes())) | parent['source_changes_sha256']
    original = Path(parent['candidate_source'])
    guard = parent['main_source_sha256']
    pending = json.loads((ROOT / 'docs/evidence/2026-10-12-existing-worktree/geometry-validation-closure/result.json').read_text())['pending_native_sha256']
    assert len(expected) == 10130 and len(pending) == 35
    assert all(sha(original / name) == digest for name, digest in expected.items())
    assert sources() == guard and all(sha(ROOT / name) == digest for name, digest in pending.items())
    with (out / 'copy.log').open('w') as log:
        subprocess.run(['cp', '-a', '--reflink=auto', str(original), str(SOURCE)], stdout=log, stderr=subprocess.STDOUT, check=True)
    helper = 'crates/hyperreal/src/coefficient_shape_diagnostic.rs'
    lib = 'crates/hyperreal/src/lib.rs'
    algebra = 'crates/hyperreal/src/computable/node/algebra.rs'
    app = 'crates/hyperdrc/src/app.rs'
    shutil.copyfile(E / 'coefficient_shape_diagnostic.rs', SOURCE / helper)
    path = SOURCE / lib
    path.write_text(replace(path.read_text(), 'mod trace;', 'pub mod coefficient_shape_diagnostic;\n\nmod trace;'))
    path = SOURCE / algebra
    signature = '    fn multiply(self, other: &Self) -> Option<Self> {'
    path.write_text(replace(path.read_text(), signature, signature + '\n        crate::coefficient_shape_diagnostic::record(\n            self.radicals.len(), self.atoms.len(), other.radicals.len(), other.atoms.len(),\n        );'))
    path = SOURCE / app
    begin = ('    pub(crate) fn complex_project_zip_kicad_board_completes_smoke_check_suite() {\n'
             '        let _geometry_guard = super::COMPLEX_PROJECT_GEOMETRY_TEST_LOCK\n'
             '            .lock()\n'
             '            .unwrap_or_else(std::sync::PoisonError::into_inner);')
    path.write_text(replace(path.read_text(), begin, begin + '\n        let _coefficient_shape_report = hyperreal::coefficient_shape_diagnostic::Report::begin();'))
    with (out / 'prepare-format.log').open('w') as log:
        for name in [lib, app]:
            subprocess.run(['rustfmt', '--edition', '2024', str(SOURCE / name)], stdout=log, stderr=subprocess.STDOUT, check=True)
    changes = {name: sha(SOURCE / name) for name in [helper, lib, algebra, app]}
    current = {name: sha(SOURCE / name) for name in sorted(set(expected) | {helper})}
    assert {name for name, digest in current.items() if expected.get(name) != digest} == set(changes)
    with tarfile.open(out / 'instrumentation.tar.gz', 'w:gz') as archive:
        for name in changes:
            archive.add(SOURCE / name, arcname=name)
    patch = ''.join(''.join(difflib.unified_diff(
        (original / name).read_text().splitlines(True) if name in expected else [],
        (SOURCE / name).read_text().splitlines(True), 'a/' + name, 'b/' + name)) for name in changes)
    (out / 'instrumentation.patch').write_text(patch)
    (out / 'parent-source-hashes.json.gz').write_bytes(gzip.compress((json.dumps(expected, sort_keys=True) + '\n').encode(), mtime=0))
    env = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
           'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never', 'CARGO_TARGET_DIR': str(TARGET)}
    protocol = {'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
                'helper_input_sha256': sha(E / 'coefficient_shape_diagnostic.rs'),
                'previous_goal_turn_classification': 'progress',
                'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
                'parent_preparation_sha256': sha(PARENT / 'preparation.json'),
                'profile_assessment_sha256': sha(PROFILE),
                'parent_source': str(original), 'source': str(SOURCE), 'target': str(TARGET),
                'parent_source_count': len(expected), 'source_changes_sha256': changes,
                'main_source_sha256': guard, 'pending_native_sha256': pending,
                'environment': env, 'inherited_rustflags': os.environ.get('RUSTFLAGS'),
                'compiler': subprocess.check_output(['rustc', '-vV'], text=True),
                'shapes': ['rational', 'radical', 'opaque', 'mixed'],
                'shape_definition': 'Presence of radical/opaque vectors only. Radical, opaque and mixed rows may also have a nonzero rational constant; no rational value is queried.',
                'row_fields': ['calls', 'left_radical_length_sum', 'left_atom_length_sum', 'right_radical_length_sum', 'right_atom_length_sum'],
                'original_worker_cpu_limit_seconds': 120, 'command_timeout_seconds': 1800,
                'limits': 'Counters include every active process thread and add synchronization overhead. Operand frequencies and length sums are not allocation counts, durations or a speedup prediction. No arithmetic or runtime admission change is made.',
                'runtime_admitted': False}
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    commands, tests = [], []
    binary = None
    binary_sha = None

    def verify():
        assert sha(Path(__file__)) == protocol['runner_sha256']
        assert sha(E / 'coefficient_shape_diagnostic.rs') == protocol['helper_input_sha256']
        assert sources() == guard
        assert all(sha(ROOT / name) == digest for name, digest in pending.items())
        assert all(sha(original / name) == digest for name, digest in expected.items())
        assert all(sha(SOURCE / name) == digest for name, digest in current.items())
        if binary is not None:
            assert sha(binary) == binary_sha

    def run(name, command, allowed=(0,), passed=None):
        verify()
        print(json.dumps({'start': name}), flush=True)
        started = time.monotonic()
        command = ['timeout', '--signal=TERM', '--kill-after=10s', '1800', *command]
        with (out / (name + '.stdout')).open('w') as stdout, (out / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(command, cwd=SOURCE, env=os.environ | env,
                                  stdout=stdout, stderr=stderr).returncode
        row = {'name': name, 'command': command, 'exit_code': code, 'seconds': time.monotonic() - started,
               'stdout_sha256': sha(out / (name + '.stdout')), 'stderr_sha256': sha(out / (name + '.stderr'))}
        commands.append(row)
        (out / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
        print(json.dumps(row), flush=True)
        verify()
        assert code in allowed, name
        raw = (out / (name + '.stdout')).read_text()
        if passed is not None:
            counts = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;', raw)
            assert counts == [(str(passed), '0', '0')], (name, counts)
            tests.append({'name': name, 'passed': passed, 'failed': 0, 'ignored': 0})
        return code, raw

    run('format', ['cargo', 'fmt', '--all', '--', '--check'])
    scalar = ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--lib']
    run('counter-calibration', scalar + ['coefficient_shape_diagnostic::tests', '--', '--test-threads=1'], passed=2)
    run('cache-frontiers', scalar + ['persistent_laurent_cache_tests', '--', '--test-threads=1'], passed=6)
    run('endpoint-regressions', ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--all-features',
                                 '--lib', 'endpoint_proof_tests', '--', '--test-threads=1'], passed=5)
    run('strict-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal', '-p', 'hypercurve',
                          '-p', 'hyperdrc', '--all-features', '--all-targets', '--', '-D', 'warnings'])
    _, raw = run('build-board', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--all-features',
                                 '--lib', '--no-run', '--message-format=json'])
    events = [json.loads(line) for line in raw.splitlines() if line.startswith('{')]
    artifacts = [row for row in events if row.get('reason') == 'compiler-artifact'
                 and row.get('executable') and row['target']['name'] == 'hyperdrc']
    assert len(artifacts) == 1
    artifact = artifacts[0]
    assert Path(artifact['manifest_path']) == SOURCE / 'crates/hyperdrc/Cargo.toml'
    binary = Path(artifact['executable'])
    assert binary.is_relative_to(TARGET)
    binary_sha = sha(binary)
    (out / 'binary.json').write_text(json.dumps({'path': str(binary), 'sha256': binary_sha,
        'bytes': binary.stat().st_size, 'manifest_path': artifact['manifest_path'],
        'features': artifact['features'], 'package_id': artifact['package_id']}, indent=2) + '\n')
    code, raw = run('full-board', [str(binary), 'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite',
                                  '--exact', '--test-threads=1', '--nocapture'], allowed=(0, 101))
    combined = raw + (out / 'full-board.stderr').read_text()
    assert 'skipping complex project fixture test' not in combined
    assert combined.count('hyperdrc: finished emit report') == 2
    assert '12 active finding(s)' in combined and '197 active finding(s)' in combined
    elapsed = re.search(r'took ([0-9.]+)s of worker time', combined)
    if code == 101:
        assert elapsed and '0 passed; 1 failed' in combined and float(elapsed[1]) >= 120
    else:
        assert '1 passed; 0 failed' in combined
    rows = []
    for line in combined.splitlines():
        if not line.startswith('COEFFICIENT_SHAPE\t'):
            continue
        _, left, right, *values = line.split('\t')
        assert len(values) == 5
        rows.append({'left': left, 'right': right, **dict(zip(protocol['row_fields'], map(int, values)))})
    assert len(rows) == 16
    assert {(row['left'], row['right']) for row in rows} == {(a, b) for a in protocol['shapes'] for b in protocol['shapes']}
    assert sum(row['calls'] for row in rows) > 0
    (out / 'rows.json').write_text(json.dumps(rows, indent=2) + '\n')
    verify()
    result = {'complete': True, 'tests': tests, 'full_board_exit_code': code,
              'original_worker_cpu_limit_seconds': 120,
              'instrumented_worker_cpu_seconds': float(elapsed[1]) if elapsed else None,
              'both_reports_emitted': True, 'finding_counts': [12, 197],
              'shape_rows': 16, 'coefficient_multiplications': sum(row['calls'] for row in rows),
              'source_parent_and_main_unchanged': True, 'runtime_admitted': False}
    (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
