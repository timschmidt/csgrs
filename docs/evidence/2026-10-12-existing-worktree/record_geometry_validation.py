"""Close the source-frozen geometry validation without admitting its runtime.

Keep the completed logs, both original runtime failures, and a reconstructible
copy of the 35 pending native files. The archive is a candidate checkpoint, not
an accepted performance change or a replacement for the original board files.
"""
import gzip
import io
import json
from pathlib import Path
import re
import subprocess
import tarfile

from validate import E, ROOT, sha, sources


def main():
    out = E / 'geometry-validation-closure'
    out.mkdir(exist_ok=False)
    groups = ['geometry-layout-validation', 'geometry-followup-validation']
    initial = json.loads((E / groups[0] / 'protocol.json').read_text())['sources_sha256']
    assert sources() == initial
    artifacts = {}
    for group in groups:
        protocol = json.loads((E / group / 'protocol.json').read_text())
        assert protocol['sources_sha256'] == initial
        result = json.loads((E / group / 'result.json').read_text())
        assert len(result['phases']) == 3
        for phase in result['phases']:
            assert phase['source_unchanged']
            expected = 101 if phase['name'] == 'drc-tests' else 0
            assert phase['exit_code'] == expected
            for suffix in ['stdout', 'stderr']:
                p = E / group / (phase['name'] + '.' + suffix)
                assert sha(p) == phase[suffix + '_sha256']
        for p in sorted((E / group).iterdir()):
            assert p.is_file()
            artifacts[str(p.relative_to(E))] = sha(p)

    counts = {}
    for group, phase, expected in [
        (groups[0], 'curve-tests', [50, 2485, 0, 33]),
        (groups[0], 'drc-tests', [1, 1560, 2, 0]),
        (groups[1], 'drc-integrations', [6, 81, 0, 0]),
        (groups[1], 'doctests', [3, 4, 0, 0]),
    ]:
        log = (E / group / (phase + '.stdout')).read_text()
        rows = re.findall(r'test result: (\w+)\. (\d+) passed; (\d+) failed; (\d+) ignored;', log)
        values = [len(rows)] + [sum(int(row[i]) for row in rows) for i in [1, 2, 3]]
        assert values == expected
        counts[phase] = dict(zip(['summaries', 'passed', 'failed', 'ignored'], values))
    log = (E / groups[0] / 'drc-tests.stdout').read_text()
    worker = float(re.search(r'KiCad smoke suite took ([0-9.]+)s of worker time', log).group(1))
    neck = float(re.search(r'copper neck regression fixture took ([0-9.]+)s', log).group(1))
    assert worker > 120 and neck > 60
    assert 'skipping complex project fixture test' not in log

    entries = subprocess.check_output(['git', 'status', '--porcelain=v1', '-z', '--untracked-files=all', '--', 'crates/hypercurve', 'crates/hyperdrc'], cwd=ROOT).decode().split('\0')
    pending = {entry[3:]: entry[:2] for entry in entries if entry}
    assert len(pending) == 35
    assert set(pending.values()) == {' M', '??'}
    before = {name: sha(ROOT / name) for name in sorted(pending)}
    archive_path = out / 'pending-native-sources.tar.gz'
    with archive_path.open('xb') as raw:
        with gzip.GzipFile(fileobj=raw, mode='wb', mtime=0, filename='') as compressed:
            with tarfile.open(fileobj=compressed, mode='w') as archive:
                for name in sorted(pending):
                    data = (ROOT / name).read_bytes()
                    info = tarfile.TarInfo(name)
                    info.size = len(data)
                    info.mode = 0o644
                    archive.addfile(info, io.BytesIO(data))
    with tarfile.open(archive_path, 'r:gz') as archive:
        import hashlib
        restored = {item.name: hashlib.sha256(archive.extractfile(item).read()).hexdigest()
                    for item in archive.getmembers()}
    assert restored == before
    patch = subprocess.check_output(['git', 'diff', '--binary', '--', *sorted(pending)], cwd=ROOT)
    (out / 'pending-native-tracked.patch').write_bytes(patch)
    assert sources() == initial
    assert before == {name: sha(ROOT / name) for name in pending}
    record = {
        'decision': 'Correctness and documentation checks complete in this scope; native runtime admission fails both original full-board cost gates.',
        'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'runner_sha256': sha(Path(__file__)),
        'test_counts': counts,
        'strict_release_all_feature_all_target_clippy_passed': True,
        'warning_denied_docs_passed': True,
        'workspace_formatting': 'Passed in line-layout-fix/result.json on the same runtime sources.',
        'failed_gates': [
            {'test': 'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite',
             'metric': 'worker CPU seconds', 'observed': worker, 'unchanged_limit': 120},
            {'test': 'zz_complex_project_performance_tests::min_copper_neck_width_completes_on_copper_layers',
             'metric': 'check wall seconds', 'observed': neck, 'unchanged_limit': 60},
        ],
        'pending_native_status': pending,
        'pending_native_sha256': before,
        'pending_native_archive_sha256': sha(archive_path),
        'pending_native_tracked_patch_sha256': sha(out / 'pending-native-tracked.patch'),
        'artifacts_sha256': artifacts,
        'source_unchanged': True,
        'runtime_admitted': False,
        'plan_complete': False,
        'limitations': [
            'These are test-result totals; no exhaustive coverage, correctness proof, matched speedup, memory or executable-size claim is made.',
            'The 33 curve ignores remain explicit. The repository complete pre-push workflow matrix has not run.',
            'The archive preserves a pending candidate, including its known runtime failures. Its exact source geometry and original full-board fixtures remain required.',
            'Historical scalar prototype guards predate authorized main changes. Combined scalar/native qualification requires a new baseline.',
        ],
        'next_experiment': 'Evaluate reuse of successful graph-free normal forms in shared subexpressions. Preserve all exact facts, opaque-atom ownership, failures and original budgets; prior root-only cache diagnostics do not establish a speedup or safe recursive reuse.',
    }
    (out / 'result.json').write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps({'result_sha256': sha(out / 'result.json'), 'counts': counts,
                      'pending_native_files': len(pending), 'runtime_admitted': False}), flush=True)


if __name__ == '__main__':
    main()
