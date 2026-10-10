"""Freeze a clean native pair with only the previously measured endpoint change."""
import ast
import difflib
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tarfile

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'arc-endpoint-native'
WORK = ROOT / 'target/arc-endpoint-native'
BASE = ROOT / 'target/laurent-memo-sharing/baseline/source'
CACHE = ROOT / 'target/arc-endpoint-retry-diagnostic/target'
PRODUCTION = ROOT / 'target/arc-endpoint-certificate/baseline/source'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
native_path = EVIDENCE / 'native-exactness-gate/sources.json'
base_path = EVIDENCE / 'laurent-memo-sharing/baseline-sources.json'
extra_path = EVIDENCE / 'laurent-root-production/sources.json'
patch_script = EVIDENCE / 'prepare_arc_endpoint_certificate.py'
native, base, extra = read(native_path), read(base_path), read(extra_path)
assert read(EVIDENCE / 'diagnostics-82.json')['no_adaptive_extension_or_discarded_samples']
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
assert all(digest(BASE / name) == value for name, value in base['source_hashes'].items())
additional = extra['additional_source_hashes']
assert all(not name.endswith(('.rs', 'Cargo.toml', 'Cargo.lock')) and '.cargo/' not in name for name in additional)
assert all(digest(PRODUCTION / name) == value for name, value in additional.items())
snippets = {}
for node in ast.parse(patch_script.read_text()).body:
    if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
        if node.targets[0].id in ['before', 'after']:
            snippets[node.targets[0].id] = ast.literal_eval(node.value)
assert set(snippets) == {'before', 'after'}
segment = 'crates/hypercurve/src/segment.rs'
old_helper = 'crates/hyperreal/src/computable/node/normal_form_memo_frontier_probe.rs'
original = (ROOT / segment).read_text()
assert original.count(snippets['before']) == 1
expected = base['source_hashes'] | native['source_hashes'] | additional
del expected[old_helper]
OUT.mkdir()
WORK.mkdir()
maps = {}
for variant in ['baseline', 'candidate']:
    work = WORK / variant
    work.mkdir()
    for source, destination, label in [(BASE, work / 'source', 'source'), (CACHE, work / 'target', 'target')]:
        with (OUT / f'copy-{variant}-{label}.log').open('w') as log:
            subprocess.run(['cp', '-a', '--reflink=auto', str(source), str(destination)], stdout=log, stderr=subprocess.STDOUT, check=True)
    source = work / 'source'
    for origin, files in [(ROOT, native['source_hashes']), (PRODUCTION, additional)]:
        for name in files:
            destination = source / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(origin / name, destination)
    (source / old_helper).unlink()
    assert 'normal_form_memo_frontier_probe' not in (source / 'crates/hyperreal/src/computable/node/algebra.rs').read_text()
    assert all(digest(source / name) == value for name, value in expected.items())
    if variant == 'candidate':
        (source / segment).write_text(original.replace(snippets['before'], snippets['after']))
    hashes = {name: digest(source / name) for name in sorted(expected)}
    maps[variant] = hashes
    compressed = OUT / (variant + '-source-hashes.json.gz')
    compressed.write_bytes(gzip.compress((json.dumps(hashes, indent=2) + '\n').encode(), mtime=0))
    record = {
        'scope': 'Complete unadmitted native prototype; no diagnostic code added. ' + ('Original endpoint computation.' if variant == 'baseline' else 'Previously tested exact-negative endpoint certificate reuse only.'),
        'source_directory': str(source), 'target_directory': str(work / 'target'),
        'source_hashes_file': str(compressed), 'source_hashes_file_sha256': digest(compressed),
        'source_files': len(hashes), 'native_files': len(native['source_hashes']),
        'native_source_record': str(native_path), 'native_source_record_sha256': digest(native_path),
        'base_source_record': str(base_path), 'base_source_record_sha256': digest(base_path),
        'additional_input_record': str(extra_path), 'additional_input_record_sha256': digest(extra_path),
        'additional_non_rust_inputs': len(additional),
        'patch_script': str(patch_script), 'patch_script_sha256': digest(patch_script),
        'removed_prior_test_only_probe': old_helper,
        'runtime_delta_from_main_prototype': [] if variant == 'baseline' else [segment],
        'main_runtime_unchanged': True,
    }
    (OUT / (variant + '-sources.json')).write_text(json.dumps(record, indent=2) + '\n')
assert [name for name in expected if maps['baseline'][name] != maps['candidate'][name]] == [segment]
candidate = WORK / 'candidate/source' / segment
with tarfile.open(OUT / 'candidate-source-delta.tar.gz', 'w:gz') as archive:
    archive.add(candidate, arcname=segment)
(OUT / 'candidate.patch').write_text(''.join(difflib.unified_diff(original.splitlines(True), candidate.read_text().splitlines(True), 'a/' + segment, 'b/' + segment)))
plan = {
    'source_frozen_before_validation': True,
    'validation_each_variant': ['workspace fmt', 'strict all-target curve Clippy with triangulation', 'Point2 and segment unit filters with nonzero counts', 'arc/circle and native-stroke curve integration tests', 'native-stroke DRC integration tests', 'strict all-target DRC Clippy', 'build the unchanged DRC board-smoke executable'],
    'measurement': 'After all builds and focused checks finish, run exactly one unchanged complete board-smoke test per variant, baseline then candidate. No builds, tests or Rust edits overlap either measurement. Preserve the original 120-second worker CPU assertion and any failure.',
    'comparison': 'Record findings, reported worker CPU when available, process CPU, wall time, maximum RSS and executable bytes. A single observation per variant is descriptive, not a robust speedup estimate. This is not complete native report-byte replay or native admission.',
    'retention': 'Review alongside all production correctness/report gates and persistent broad-workload slowdowns in diagnostics-82. No further sample extension for the existing production replay binaries.',
}
(OUT / 'plan.json').write_text(json.dumps(plan, indent=2) + '\n')
shutil.copyfile(__file__, OUT / 'prepare.py')
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
print('Frozen native pair:', len(expected), 'inputs each; candidate differs only in segment.rs.', flush=True)
