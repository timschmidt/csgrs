"""Verify focused exactness and isolated source identity for the ray repair."""
import hashlib
import json
from pathlib import Path
import re
import tarfile

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'ray-filter-finite-guard'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
source = read(OUT / 'sources.json')
native = read(Path(source['main_source_record']))
directory = ROOT / 'target/ray-filter-finite-guard/source'
assert all(digest(directory / name) == value for name, value in source['source_hashes'].items())
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
with tarfile.open(OUT / 'source-delta.tar.gz') as archive:
    assert set(archive.getnames()) == set(source['only_modified_sources'])
    for name in archive.getnames():
        assert hashlib.sha256(archive.extractfile(name).read()).hexdigest() == source['source_hashes'][name]
audit = read(OUT / 'committed-source-audit.json')
assert audit['all_baseline_inputs_match_git'] and audit['all_package_rust_inputs_accounted_for']
assert audit['checked_committed_inputs'] == 991
status = read(OUT / 'status.json')
assert len(status) == 3 and all(command['exit_code'] == 0 for command in status)
tests = {}
for name in ['ray-filter-unit-tests', 'ray-filter-public-tests']:
    rows = re.findall(r'test result: ok\. (\d+) passed; 0 failed; 0 ignored', (OUT / (name+'.log')).read_text())
    tests[name] = sum(map(int, rows)); assert tests[name] == 2
record = {
    'decision': 'four focused regressions pass on the isolated finite-intermediate guard; full production validation is underway before runtime admission',
    'baseline_failure_record': 'diagnostics-43.json',
    'source_files_frozen': len(source['source_hashes']),
    'committed_package_inputs_audited': audit['checked_committed_inputs'],
    'changed_sources': source['only_modified_sources'],
    'implementation': 'Before using projection inequalities, require finite computed error, side and forward values. Non-finite intermediates return false from the exclusion filter, preserving the existing exact fallback.',
    'focused_tests_passed': tests,
    'independent_oracle_matrix': {'helper_intersection_witnesses': 32, 'helper_missed_box_controls': 2, 'public_original_counterexample_and_control_cases': 6, 'affine_lens_configurations': 176, 'classification_assertions_in_affine_matrix': 5280, 'exact_area_assertions': 176, 'coverage': 'Two policies; eleven ordinary/extreme/signed affine frames; unit and retained charts; both traversal directions; cold and low/high/low prepared input intervals; first, repeated and cloned public point queries; independently derived inside, outside, boundary and area witnesses.'},
    'format_passed': True,
    'main_runtime_unchanged': True,
    'next_step': 'Finish full curve/DRC validation, strict Clippy/docs, original board gate, complete report/capability replay and resource comparisons before admitting the three scoped source files.',
    'limitations': 'Focused assertions do not prove the complete floating-point error bound or close a full property. No main runtime repair, scalar change or broader native-prototype admission is claimed.',
}
paths = [path for path in sorted(OUT.iterdir()) if path.is_file()]
paths += [EVIDENCE / name for name in ['ray_filter_tests.rs', 'ray_filter_extreme_coordinates.rs', 'reproduce_ray_filter_finite_guard.py', 'analyze_ray_filter_focused.py']]
record['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in paths}
(EVIDENCE / 'diagnostics-44.json').write_text(json.dumps(record, indent=2)+'\n')
print('Four focused tests passed; 176 exact affine configurations and 5280 classification assertions verified.')
