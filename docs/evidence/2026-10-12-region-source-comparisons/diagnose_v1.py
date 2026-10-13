"""Observe source metadata around unchanged duplicate-loop geometry queries.

The previous goal turn made progress through five committed checkpoints. This
separate diagnostic forks the frozen integer-leaf parent, without the rejected
coefficient merge. Every original geometry comparison, conditional source
comparison, validation ordering and error remains. Additional observations read
only integer provenance; counters retain no expression owners. Timers and
counters perturb execution. Only exiting-thread counters are collected.
"""
import difflib
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
PARENT = ROOT / 'docs/evidence/2026-10-12-small-integer-certificates'
BASE = ROOT / 'target/small-integer-certificates/source'
WORK = ROOT / 'target/region-source-comparisons/diagnostic-v1'
SOURCE = WORK / 'source'
TARGET = WORK / 'target'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    out = E / 'diagnostic-v1'
    out.mkdir(exist_ok=False)
    original = json.loads(gzip.decompress((PARENT / 'baseline-source-hashes.json.gz').read_bytes()))
    delta = json.loads((PARENT / 'validation-v1/protocol.json').read_text())['source_changes_sha256']
    baseline = original | delta
    guard = json.loads((PARENT / 'preparation.json').read_text())['main_source_sha256']
    assert sources() == guard
    assert all(sha(BASE / name) == expected for name, expected in baseline.items())
    WORK.mkdir(parents=True, exist_ok=False)
    with (out / 'copy.log').open('w') as log:
        subprocess.run(['cp', '-a', '--reflink=auto', str(BASE), str(SOURCE)],
                       stdout=log, stderr=subprocess.STDOUT, check=True)
    region = 'crates/hypercurve/src/bezier_region.rs'
    helper = 'crates/hypercurve/src/bezier_region/source_comparison_diagnostic.rs'
    path = SOURCE / region
    text = path.read_text()
    before = 'mod point_location;'
    assert text.count(before) == 1
    text = text.replace(before, before + '\nmod source_comparison_diagnostic;')
    before = '            boundary_loop.fragments == candidate.fragments\n                && boundary_loop.arrangement_sources == candidate.arrangement_sources'
    assert text.count(before) == 1
    text = text.replace(before, '            source_comparison_diagnostic::fragments_equal(boundary_loop, candidate)\n                && boundary_loop.arrangement_sources == candidate.arrangement_sources')
    path.write_text(text)
    shutil.copyfile(E / 'source_comparison_diagnostic.rs', SOURCE / helper)
    with (out / 'prepare-format.log').open('w') as log:
        subprocess.run(['rustfmt', '--edition', '2024', str(path)], stdout=log,
                       stderr=subprocess.STDOUT, check=True)
    changed = [region, helper]
    current = {name: sha(SOURCE / name) for name in sorted(set(baseline) | {helper})}
    assert {name for name, value in current.items() if baseline.get(name) != value} == set(changed)
    with tarfile.open(out / 'instrumentation.tar.gz', 'w:gz') as archive:
        for name in changed:
            archive.add(SOURCE / name, arcname=name)
    patch = ''.join(''.join(difflib.unified_diff(
        (BASE / name).read_text().splitlines(True) if name in baseline else [],
        (SOURCE / name).read_text().splitlines(True), 'a/' + name, 'b/' + name)) for name in changed)
    (out / 'instrumentation.patch').write_text(patch)
    env = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
           'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never', 'CARGO_TARGET_DIR': str(TARGET)}
    test_command = ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--all-features', '--lib']
    commands = [
        ('format', ['cargo', 'fmt', '--all', '--', '--check']),
        ('diagnostic-regressions', test_command + ['source_comparison_diagnostic::tests', '--', '--test-threads=1']),
        ('provenance-constructor', test_command + ['bezier_region::tests::certified_boundary_constructors_validate_arrangement_sources', '--', '--exact', '--test-threads=1']),
        ('reused-source-regression', test_command + ['bezier_region::tests::retained_region_constructor_rejects_reused_arrangement_sources_across_loops', '--', '--exact', '--test-threads=1']),
        ('duplicate-boundary-regression', test_command + ['bezier_region::tests::native_boundary_loops_convert_into_unified_region_validation', '--', '--exact', '--test-threads=1']),
        ('strict-curve-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hypercurve', '--all-features', '--all-targets', '--', '-D', 'warnings']),
        ('build-board-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--all-features', '--lib', '--no-run', '--message-format=json']),
    ]
    protocol = {
        'scope': __doc__, 'previous_goal_turn': 'progress',
        'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'runner_sha256': sha(Path(__file__)),
        'parent_preparation_sha256': sha(PARENT / 'preparation.json'),
        'parent_delta_sha256': sha(PARENT / 'validation-v1/protocol.json'),
        'prior_profile_assessment_sha256': sha(ROOT / 'docs/evidence/2026-10-12-integer-leaf-profile/assessment-v1.json'),
        'only_instrumented_sources': {name: current[name] for name in changed},
        'instrumentation_archive_sha256': sha(out / 'instrumentation.tar.gz'),
        'instrumentation_patch_sha256': sha(out / 'instrumentation.patch'),
        'commands': commands, 'environment': env,
        'metadata_kinds': ['both_absent', 'left_absent', 'right_absent', 'equal', 'unequal'],
        'row_contract': 'Metadata kind, equal fragment counts, geometry equality, call count, and geometry-query wall nanoseconds. Original geometry equality executes once. Extra metadata observation executes afterward; the original conditional metadata check remains.',
        'unchanged_worker_limit_seconds': 120, 'runtime_admitted': False,
    }
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    results = []

    def verify():
        assert sources() == guard
        assert all(sha(SOURCE / name) == expected for name, expected in current.items())
        assert all(sha(BASE / name) == expected for name, expected in baseline.items())

    def run(name, command, allow_failure=False):
        verify()
        started = time.monotonic()
        with (out / (name + '.stdout')).open('w') as stdout, (out / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(command, cwd=SOURCE, env=os.environ | env, stdout=stdout, stderr=stderr).returncode
        verify()
        row = {'name': name, 'command': command, 'exit_code': code, 'seconds': time.monotonic() - started,
               'sources_unchanged': True, 'stdout_sha256': sha(out / (name + '.stdout')),
               'stderr_sha256': sha(out / (name + '.stderr'))}
        results.append(row)
        (out / 'result.json').write_text(json.dumps({'phases': results, 'complete': False}, indent=2) + '\n')
        print(json.dumps(row), flush=True)
        if code and not allow_failure:
            raise SystemExit(code)
        return (out / (name + '.stdout')).read_text()

    for name, command in commands:
        log = run(name, command)
    events = [json.loads(line) for line in log.splitlines() if line.startswith('{')]
    artifacts = [row for row in events if row.get('reason') == 'compiler-artifact'
                 and row.get('executable') and row['target']['name'] == 'hyperdrc']
    assert len(artifacts) == 1
    artifact = artifacts[0]
    assert Path(artifact['manifest_path']) == SOURCE / 'crates/hyperdrc/Cargo.toml'
    binary = Path(artifact['executable'])
    assert binary.is_relative_to(TARGET)
    identity = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size,
                'manifest_path': artifact['manifest_path'], 'features': artifact['features']}
    (out / 'binary.json').write_text(json.dumps(identity, indent=2) + '\n')
    log = run('full-board', ['/usr/bin/time', '-v', str(binary), 'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite', '--exact', '--test-threads=1', '--nocapture'], allow_failure=True)
    combined = log + (out / 'full-board.stderr').read_text()
    assert sha(binary) == identity['sha256']
    assert 'skipping complex project fixture test' not in combined
    assert combined.count('hyperdrc: finished emit report') == 2
    assert '1 passed' in log or '1 failed' in log
    rows = []
    for line in combined.splitlines():
        if line.startswith('REGION_SOURCE_COMPARISON '):
            kind, same_length, equal, calls, nanos = [int(value) for value in line.split()[1:]]
            rows.append({'metadata_kind': protocol['metadata_kinds'][kind],
                         'same_fragment_count': bool(same_length), 'geometry_equal': bool(equal),
                         'calls': calls, 'geometry_nanos': nanos})
    assert rows
    (out / 'counters.json').write_text(json.dumps(rows, indent=2) + '\n')
    (out / 'result.json').write_text(json.dumps({'phases': results, 'complete': True,
                                               'rows': len(rows), 'observed_comparisons': sum(row['calls'] for row in rows),
                                               'timing_is_perturbed': True, 'runtime_admitted': False}, indent=2) + '\n')


if __name__ == '__main__':
    main()
