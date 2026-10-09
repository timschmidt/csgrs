"""Admit only Point2 documentation and regressions after rejecting shortcuts."""
import hashlib
import json
from pathlib import Path
import re
import subprocess
import tarfile

root = Path(__file__).resolve().parent
out = root / 'point-equality-regressions'
work = Path('/tmp/drc-point-equality-regressions/source')
baseline_work = Path('/tmp/drc-normal-form-ownership-isolated/source')
read = lambda p: json.loads(p.read_text())
digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
info = lambda p: dict(bytes=p.stat().st_size, sha256=digest(p))
source = read(out / 'sources.json')
baseline = read(root / 'normal-form-ownership-isolated/sources.json')
assert source['unchanged'] and source['non_test_executable_source_unchanged']
assert all(digest(work / p) == h for p, h in source['source_hashes'].items())
assert all(digest(baseline_work / p) == h for p, h in baseline['source_hashes'].items())
changes = {p for p, h in source['source_hashes'].items() if baseline['source_hashes'].get(p) != h}
assert changes == {'crates/hypercurve/src/point.rs', 'crates/hypercurve/src/point/equality_tests.rs'}
with tarfile.open(out / 'source-delta.tar.gz') as archive:
    assert set(archive.getnames()) == changes
    for name in changes:
        assert hashlib.sha256(archive.extractfile(name).read()).hexdigest() == source['source_hashes'][name]
doc = '''///
/// Equality proves matching coordinates; `false` can also mean that symbolic
/// equality could not be established. Convert to [`CurvePoint2`](crate::CurvePoint2)
/// and use [`coincides_with`](crate::CurvePoint2::coincides_with) for a fallible
/// geometric comparison.
'''
hook = '#[cfg(test)]\nmod equality_tests;\n\n'
point = (work / 'crates/hypercurve/src/point.rs').read_text()
assert point.count(doc) == 1 and point.count(hook) == 1
assert point.replace(doc, '').replace(hook, '') == (baseline_work / 'crates/hypercurve/src/point.rs').read_text()
assert all(Path(p).read_bytes() == (work / p).read_bytes() for p in changes)
native = read(root / 'normal-form-ownership-candidate/sources.json')
unchanged_other_sources = [p for p, h in native['source_hashes'].items()
                           if p != 'crates/hypercurve/src/point.rs' and digest(Path(p)) == h]
assert len(unchanged_other_sources) == len(native['source_hashes']) - 1
status = read(out / 'status.json')
assert [r['name'] for r in status] == ['format', 'point-tests', 'public-curve-point', 'curve-clippy', 'curve-doc']
assert all(r['exit_code'] == 0 for r in status)
assert source['environment']['RUSTDOCFLAGS'] == '-D warnings'
tests = {}
for phase, expected in [('point-tests', 14), ('public-curve-point', 7)]:
    result = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored', (out / (phase + '.log')).read_text())
    assert result == [(str(expected), '0', '0')]
    tests[phase] = expected
oracle = read(root / 'point-equality-oracle.json')
assert oracle['point_pairs'] == 15000 and oracle['comparison_assertions'] == 45000
record = dict(
    decision='kept',
    scope='Only the independent equality regression module and five documentation lines are retained. All six measured Point2 shortcut implementations were rejected; no non-test executable Rust statements change.',
    production_baseline_revision='73d162ed0',
    previous_diagnostic='../diagnostics-25.json',
    source_changes=sorted(changes),
    non_test_executable_source_unchanged=True,
    other_native_candidate_source_cargo_fixture_hashes_unchanged=len(unchanged_other_sources),
    validation=dict(commands=5, tests=tests, total_passed=21, ignored=0,
                    strict_release_all_target_clippy=True, strict_release_rustdoc=True,
                    source_hashes_verified=len(source['source_hashes']), source_delta_archive_verified=True),
    independent_oracle=dict(path='../point-equality-oracle.json', generator='../point_equality_oracle.py',
                            point_pairs=15000, forward_reverse_cloned_assertions=45000,
                            coverage='Three arbitrary-real origin families, four nonzero exact rational scales including 1e400 and 1e-400, before/after explicit numerical preparation. Additional cases cover exact 1e-40 separations under overlapping enclosures, unproved trigonometric equality, abort state, and four concurrent readers with low/high/low precision preparation.'),
    rejected_optimization_evidence=['../diagnostics-' + str(i) + '.json' for i in range(20, 26)],
    limitations='Focused point tests and public CurvePoint consumers only; this is not the full curve/workspace/pre-push matrix. No board or report replay was needed or run for these documentation/test-only edits, and no optimization or application speedup is claimed. Other native-source prototypes remain uncommitted and unadmitted; the original full-board runtime gate still fails for that larger candidate. All 39 properties, 14 targets and crate future work remain in scope.',
    compiler=subprocess.check_output(['rustc', '-Vv'], text=True),
)
record['artifacts'] = {p.name:info(p) for p in sorted(out.iterdir()) if p.is_file() and p.name != 'manifest.json'}
for name in ['analyze_point_equality_regressions.py', 'point-equality-oracle.json', 'point_equality_oracle.py']:
    record['artifacts']['../' + name] = info(root / name)
(out / 'manifest.json').write_text(json.dumps(record, indent=2) + '\n')
print('Isolated documentation and regressions admitted:', tests, '; runtime code unchanged.')
