"""Observe original endpoint retries on the complete native board fixture."""
import difflib
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'arc-endpoint-retry-diagnostic'
WORK = ROOT / 'target/arc-endpoint-retry-diagnostic'
SOURCE, TARGET = WORK / 'source', WORK / 'target'
BASE = ROOT / 'target/laurent-memo-sharing/baseline/source'
CACHE = ROOT / 'target/laurent-root-memo-omission/candidate/target'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
native_record = EVIDENCE / 'native-exactness-gate/sources.json'
base_record = EVIDENCE / 'laurent-memo-sharing/baseline-sources.json'
native, base = read(native_record), read(base_record)
assert read(EVIDENCE / 'diagnostics-75.json')['decision'].startswith('not retained:')
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
assert all(digest(BASE / name) == value for name, value in base['source_hashes'].items())
OUT.mkdir()
WORK.mkdir()
for source, destination, label in [(BASE, SOURCE, 'source'), (CACHE, TARGET, 'target')]:
    with (OUT / ('copy-' + label + '.log')).open('w') as log:
        subprocess.run(['cp', '-a', '--reflink=auto', str(source), str(destination)], stdout=log, stderr=subprocess.STDOUT, check=True)
for name, value in native['source_hashes'].items():
    path = SOURCE / name
    path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(ROOT / name, path)
    assert digest(path) == value
old_helper = 'crates/hyperreal/src/computable/node/normal_form_memo_frontier_probe.rs'
assert (SOURCE / old_helper).is_file()
(SOURCE / old_helper).unlink()
segment = 'crates/hypercurve/src/segment.rs'
helper = 'crates/hypercurve/src/arc_endpoint_retry_diagnostic.rs'
path = SOURCE / segment
original = path.read_text()
before = '''        if self.contains_endpoint(point, policy) == Some(true) {
            return Classification::Decided(true);
        }
        let radius_delta = point.distance_squared(self.center()) - self.radius_squared();
        match is_zero(&radius_delta, policy) {
            Some(false) => Classification::Decided(false),
            Some(true) => self.contains_sweep_point_with_policy(point, policy),
            None => Classification::Uncertain(crate::UncertaintyReason::RealSign),
        }
'''
after = '''        let first_endpoint = self.contains_endpoint(point, policy);
        if first_endpoint == Some(true) {
            arc_endpoint_retry_diagnostic::observe(policy, first_endpoint, None, None);
            return Classification::Decided(true);
        }
        let radius_delta = point.distance_squared(self.center()) - self.radius_squared();
        let radius = is_zero(&radius_delta, policy);
        match radius {
            Some(false) => {
                arc_endpoint_retry_diagnostic::observe(policy, first_endpoint, Some(radius), None);
                Classification::Decided(false)
            }
            Some(true) => {
                // Inline the original sweep wrapper to observe its existing retry.
                let repeated = self.contains_endpoint(point, policy);
                arc_endpoint_retry_diagnostic::observe(policy, first_endpoint, Some(radius), Some(repeated));
                if repeated == Some(true) {
                    Classification::Decided(true)
                } else {
                    self.contains_non_endpoint_sweep_point(point, policy)
                }
            }
            None => {
                arc_endpoint_retry_diagnostic::observe(policy, first_endpoint, Some(radius), None);
                Classification::Uncertain(crate::UncertaintyReason::RealSign)
            }
        }
'''
assert original.count(before) == 1
assert '''        if self.contains_endpoint(point, policy) == Some(true) {
            return Classification::Decided(true);
        }

        self.contains_non_endpoint_sweep_point(point, policy)
''' in original
path.write_text(original.replace(before, after) + '\n#[path = "arc_endpoint_retry_diagnostic.rs"]\nmod arc_endpoint_retry_diagnostic;\n')
shutil.copyfile(EVIDENCE / 'arc_endpoint_retry_diagnostic.rs', SOURCE / helper)
ENV = os.environ | {
    'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
    'CARGO_NET_OFFLINE': 'true', 'CARGO_TARGET_DIR': str(TARGET),
}
with (OUT / 'prepare-format.log').open('w') as log:
    subprocess.run(['cargo', 'fmt', '--all'], cwd=SOURCE, env=ENV, stdout=log, stderr=subprocess.STDOUT, check=True)
expected = base['source_hashes'] | native['source_hashes']
paths = (set(expected) - {old_helper}) | {helper}
initial = {name: digest(SOURCE / name) for name in sorted(paths)}
assert {name for name, value in initial.items() if expected.get(name) != value} == {segment, helper}
compressed = OUT / 'source-hashes.json.gz'
compressed.write_bytes(gzip.compress((json.dumps(initial, indent=2) + '\n').encode(), mtime=0))
with tarfile.open(OUT / 'instrumentation.tar.gz', 'w:gz') as archive:
    for name in [segment, helper]:
        archive.add(SOURCE / name, arcname=name)
(OUT / 'instrumentation.patch').write_text(''.join(difflib.unified_diff(original.splitlines(True), path.read_text().splitlines(True), 'a/' + segment, 'b/' + segment)))
record = {
    'scope': 'Diagnostic-only endpoint retry counters on the unchanged complete native prototype. All original endpoint, radius and sweep operations execute in the original order; the sweep wrapper is inlined only to observe its existing retry. No optimization or board-gate relaxation.',
    'native_source_record': str(native_record), 'native_source_record_sha256': digest(native_record),
    'fallback_source_record': str(base_record), 'fallback_source_record_sha256': digest(base_record),
    'source_hashes_file': str(compressed), 'source_hashes_file_sha256': digest(compressed),
    'source_files': len(initial), 'only_instrumented_sources': [segment, helper],
    'removed_prior_test_only_probe': old_helper,
    'main_production_unchanged': True,
    'counter_layout': {'shape': [3, 2, 3, 4, 4], 'policy': ['exact_only', 'approximate_512_enabled', 'edge_preview'], 'bounded_exact_pass': [False, True], 'first_endpoint': ['false', 'true', 'unknown'], 'radius': ['false', 'true', 'unknown', 'not_run'], 'repeated_endpoint': ['false', 'true', 'unknown', 'not_run']},
    'limitations': 'Thread-local integer counters perturb code layout and timing. A certified negative endpoint result may permit mathematical reuse, but counts do not prove preservation of later cache-dependent decisions or predict a speedup. The native prototype remains unadmitted.',
}
(OUT / 'sources.json').write_text(json.dumps(record, indent=2) + '\n')
shutil.copyfile(__file__, OUT / 'reproduce.py')
status = []


def verify():
    assert all(digest(SOURCE / name) == value for name, value in initial.items())
    assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())


def run(name, command, allow_gate_failure=False):
    verify()
    print('start', name, flush=True)
    start = time.monotonic()
    with (OUT / (name + '.log')).open('w') as log:
        result = subprocess.run(command, cwd=SOURCE, env=ENV, stdout=log, stderr=subprocess.STDOUT, check=False)
    status.append({'name': name, 'command': command, 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start})
    (OUT / 'status.json').write_text(json.dumps(status, indent=2) + '\n')
    verify()
    print('finish', name, result.returncode, flush=True)
    if result.returncode and not allow_gate_failure:
        raise SystemExit(result.returncode)


run('format', ['cargo', 'fmt', '--all', '--', '--check'])
run('curve-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hypercurve', '--features', 'triangulation', '--all-targets', '--', '-D', 'warnings'])
run('point-regressions', ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--features', 'triangulation', '--lib', 'point::', '--', '--test-threads=2'])
run('segment-regressions', ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--features', 'triangulation', '--lib', 'segment::', '--', '--test-threads=2'])
run('arc-regressions', ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--features', 'triangulation', '--test', 'hypercurve_arc_bezier', '--test', 'hypercurve_circle_predicates', '--', '--test-threads=2'])
run('build-board-smoke', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--lib', '--no-run', '--message-format=json'])
executables = []
for line in (OUT / 'build-board-smoke.log').read_text().splitlines():
    if line.startswith('{'):
        event = json.loads(line)
        if event.get('reason') == 'compiler-artifact' and event.get('executable') and event['target']['name'] == 'hyperdrc':
            executables.append(event['executable'])
assert len(executables) == 1
(OUT / 'binary.json').write_text(json.dumps({'path': executables[0], 'sha256': digest(Path(executables[0]))}, indent=2) + '\n')
run('full-board-smoke', ['/usr/bin/time', '-v', '-o', str(OUT / 'full-board-smoke-time.log'), executables[0], 'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite', '--exact', '--test-threads=1', '--nocapture'], allow_gate_failure=True)
log = (OUT / 'full-board-smoke.log').read_text()
assert 'skipping complex project fixture test' not in log
assert log.count('hyperdrc: finished emit report') == 2
assert 'ARC_ENDPOINT_RETRY_DIAGNOSTIC {' in log
assert status[-1]['exit_code'] in [0, 101]
verify()
(OUT / 'source-verification.json').write_text(json.dumps({'all_instrumented_and_main_source_hashes_unchanged': True, 'candidate_files': len(initial), 'main_files': len(native['source_hashes']), 'scope': 'Diagnostic complete; original board gate outcome is retained separately and may fail.'}, indent=2) + '\n')
print('Full-fixture arc endpoint retry diagnostic complete; no optimization admission.', flush=True)
