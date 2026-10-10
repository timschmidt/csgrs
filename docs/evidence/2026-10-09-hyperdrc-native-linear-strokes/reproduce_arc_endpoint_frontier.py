"""Freeze and validate separate test probes for the production endpoint pair."""
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
OUT = EVIDENCE / 'arc-endpoint-frontier'
WORK = ROOT / 'target/arc-endpoint-frontier'
PRODUCTION = EVIDENCE / 'arc-endpoint-certificate'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
WORK.mkdir()
assert (OUT / 'oracle.json').is_file()
assert not (OUT / 'status.json').exists()
native = read(EVIDENCE / 'native-exactness-gate/sources.json')['source_hashes']
segment = 'crates/hypercurve/src/segment.rs'
policy = 'crates/hypercurve/src/policy.rs'
helper = 'crates/hypercurve/src/arc_endpoint_frontier_probe.rs'
cases = 'crates/hypercurve/src/arc_endpoint_cases.rs'
module = '\n#[cfg(test)]\n#[path = "arc_endpoint_frontier_probe.rs"]\nmod arc_endpoint_frontier_probe;\n'
observer = '''
#[cfg(test)]
pub(crate) fn arc_endpoint_probe_observe<T>(
    evaluate: impl FnOnce() -> T,
) -> (T, CurveCertainty) {
    let observation = OperationObservation::begin();
    let value = evaluate();
    (value, observation.finish())
}
'''
records, hashes, production = {}, {}, {}
for variant in ['baseline', 'candidate']:
    parent_path = PRODUCTION / (variant + '-sources.json')
    parent = read(parent_path)
    parent_map = Path(parent['source_hashes_file'])
    assert digest(parent_map) == parent['source_hashes_file_sha256']
    expected = json.loads(gzip.decompress(parent_map.read_bytes()))
    source = Path(parent['source_directory'])
    assert all(digest(source / name) == value for name, value in expected.items())
    production[variant] = (source, expected)
    work = WORK / variant
    work.mkdir()
    for source_dir, destination, label in [(source, work / 'source', 'source'), (Path(parent['target_directory']), work / 'target', 'target')]:
        with (OUT / f'copy-{variant}-{label}.log').open('w') as log:
            subprocess.run(['cp', '-a', '--reflink=auto', str(source_dir), str(destination)], stdout=log, stderr=subprocess.STDOUT, check=True)
    probe = work / 'source'
    for name, addition in [(segment, module), (policy, observer)]:
        (probe / name).write_text((probe / name).read_text() + addition)
    shutil.copyfile(EVIDENCE / 'arc_endpoint_frontier_probe.rs', probe / helper)
    shutil.copyfile(OUT / 'arc_endpoint_cases.rs', probe / cases)
    env = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CARGO_TARGET_DIR': str(work / 'target'), 'CARGO_NET_OFFLINE': 'true'}
    with (OUT / (variant + '-prepare-format.log')).open('w') as log:
        subprocess.run(['cargo', 'fmt', '--all'], cwd=probe, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
    paths = sorted(set(expected) | {helper, cases})
    current = {name: digest(probe / name) for name in paths}
    assert {name for name in paths if current[name] != expected.get(name)} == {segment, policy, helper, cases}
    assert (probe / segment).read_text() == (source / segment).read_text() + module
    hashes[variant] = current
    compressed = OUT / (variant + '-source-hashes.json.gz')
    compressed.write_bytes(gzip.compress((json.dumps(current, indent=2) + '\n').encode(), mtime=0))
    with tarfile.open(OUT / (variant + '-probe-source-delta.tar.gz'), 'w:gz') as archive:
        for name in [segment, policy, helper, cases]:
            archive.add(probe / name, arcname=name)
    patches = []
    for name in [segment, policy, helper, cases]:
        before = (source / name).read_text() if (source / name).exists() else ''
        patches.append(''.join(difflib.unified_diff(before.splitlines(True), (probe / name).read_text().splitlines(True), 'a/' + name, 'b/' + name)))
    (OUT / (variant + '-probe.patch')).write_text(''.join(patches))
    record = {
        'scope': 'Test-only membership/frontier receipts; runtime body is the separate frozen production variant.',
        'parent_source_record': str(parent_path), 'parent_source_record_sha256': digest(parent_path),
        'source_directory': str(probe), 'target_directory': str(work / 'target'),
        'source_hashes_file': str(compressed), 'source_hashes_file_sha256': digest(compressed),
        'source_files': len(current), 'test_only_additions': [segment, policy, helper, cases],
        'production_sources_unchanged': True, 'main_sources_unchanged': True,
    }
    records[variant] = record
    (OUT / (variant + '-sources.json')).write_text(json.dumps(record, indent=2) + '\n')
assert [name for name in hashes['baseline'] if hashes['baseline'][name] != hashes['candidate'][name]] == [segment]
shutil.copyfile(__file__, OUT / 'reproduce.py')
status = []


def verify():
    for variant, record in records.items():
        probe = Path(record['source_directory'])
        assert all(digest(probe / name) == value for name, value in hashes[variant].items())
        source, expected = production[variant]
        assert all(digest(source / name) == value for name, value in expected.items())
    assert all(digest(ROOT / name) == value for name, value in native.items())


def run(variant, label, command, timeout=1800):
    verify()
    record = records[variant]
    env = os.environ | {
        'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
        'CARGO_NET_OFFLINE': 'true', 'CARGO_TARGET_DIR': record['target_directory'],
    }
    start = time.monotonic()
    print('start', variant, label, flush=True)
    path = OUT / (variant + '-' + label + '.log')
    with path.open('w') as log:
        # GNU timeout kills the whole foreground process group, including tests.
        result = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', str(timeout), *command], cwd=record['source_directory'], env=env, stdout=log, stderr=subprocess.STDOUT, check=False)
    status.append({'variant': variant, 'name': label, 'command': command, 'timeout_seconds': timeout, 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start, 'log_sha256': digest(path)})
    (OUT / 'status.json').write_text(json.dumps(status, indent=2) + '\n')
    verify()
    print('finish', variant, label, result.returncode, flush=True)
    if result.returncode:
        raise SystemExit(result.returncode)


for variant in ['baseline', 'candidate']:
    run(variant, 'format', ['cargo', 'fmt', '--all', '--', '--check'])
    run(variant, 'curve-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hypercurve', '--features', 'triangulation', '--all-targets', '--', '-D', 'warnings'])
    for label in ['unresolved_and_aborted', 'matrix', 'shared_queries']:
        run(variant, label, ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--features', 'triangulation', '--lib', 'segment::arc_endpoint_frontier_probe::arc_endpoint_frontier_' + label, '--', '--exact', '--nocapture', '--test-threads=1'])
    run(variant, 'point-regressions', ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--features', 'triangulation', '--lib', 'point::', '--', '--test-threads=2'])
    run(variant, 'segment-regressions', ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--features', 'triangulation', '--lib', 'segment::tests::', '--', '--test-threads=2'])
    run(variant, 'arc-regressions', ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--features', 'triangulation', '--test', 'hypercurve_arc_bezier', '--test', 'hypercurve_circle_predicates', '--', '--test-threads=2'])
verify()
(OUT / 'source-verification.json').write_text(json.dumps({'all_probe_production_and_main_source_hashes_unchanged': True, 'probe_files_each': len(hashes['baseline']), 'production_files_each': len(production['baseline'][1]), 'main_files': len(native)}, indent=2) + '\n')
print('Focused endpoint probes complete; no resource or production admission claim.', flush=True)
