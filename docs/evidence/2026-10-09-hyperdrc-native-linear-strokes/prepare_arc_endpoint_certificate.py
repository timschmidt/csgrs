"""Freeze original and certificate-reuse production sources without probe code."""
import difflib
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'arc-endpoint-certificate'
WORK = ROOT / 'target/arc-endpoint-certificate'
BASE = ROOT / 'target/laurent-memo-index/baseline/source'
CACHE = ROOT / 'target/contact-index-directed-width/target'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
parent_path = EVIDENCE / 'laurent-memo-index/baseline-sources.json'
parent = read(parent_path)
assert digest(Path(parent['source_hashes_file'])) == parent['source_hashes_file_sha256']
parent_hashes = json.loads(gzip.decompress(Path(parent['source_hashes_file']).read_bytes()))
native_path = Path(parent['main_source_record'])
native = read(native_path)
assert all(digest(BASE / name) == value for name, value in parent_hashes.items())
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
assert read(EVIDENCE / 'diagnostics-76.json')['observations']['exact_negative_before_successful_radius_query'] == 10446
OUT.mkdir()
WORK.mkdir()
old_helper = 'crates/hyperreal/src/computable/node/normal_form_memo_frontier_probe.rs'
algebra = 'crates/hyperreal/src/computable/node/algebra.rs'
segment = 'crates/hypercurve/src/segment.rs'
module = '\n#[cfg(test)]\nmod normal_form_memo_frontier_probe;\n'
baseline_algebra = (BASE / algebra).read_text()
assert baseline_algebra.endswith(module)
baseline_algebra = baseline_algebra[:-len(module)]
assert baseline_algebra == subprocess.check_output(['git', 'show', 'HEAD:' + algebra], cwd=ROOT, text=True)
original = (BASE / segment).read_text()
assert original == subprocess.check_output(['git', 'show', 'HEAD:' + segment], cwd=ROOT, text=True)
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
after = '''        let endpoint = self.contains_endpoint(point, policy);
        if endpoint == Some(true) {
            return Classification::Decided(true);
        }
        // Only an exact negative result certifies that neither endpoint matches.
        // Unknown and lossy-policy results retain the retry after the radius query.
        let certified_non_endpoint = endpoint == Some(false)
            && !policy.permits_approximate_512()
            && !policy.is_edge_preview();
        let radius_delta = point.distance_squared(self.center()) - self.radius_squared();
        match is_zero(&radius_delta, policy) {
            Some(false) => Classification::Decided(false),
            Some(true) if certified_non_endpoint => {
                self.contains_non_endpoint_sweep_point(point, policy)
            }
            Some(true) => self.contains_sweep_point_with_policy(point, policy),
            None => Classification::Uncertain(crate::UncertaintyReason::RealSign),
        }
'''
assert original.count(before) == 1
paths = sorted(set(parent_hashes) - {old_helper})
maps = {}
for variant in ['baseline', 'candidate']:
    work = WORK / variant
    work.mkdir()
    for source, destination, label in [(BASE, work / 'source', 'source'), (CACHE, work / 'target', 'target')]:
        with (OUT / f'copy-{variant}-{label}.log').open('w') as log:
            subprocess.run(['cp', '-a', '--reflink=auto', str(source), str(destination)], stdout=log, stderr=subprocess.STDOUT, check=True)
    source = work / 'source'
    (source / old_helper).unlink()
    (source / algebra).write_text(baseline_algebra)
    if variant == 'candidate':
        (source / segment).write_text(original.replace(before, after))
    env = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CARGO_NET_OFFLINE': 'true', 'CARGO_TARGET_DIR': str(work / 'target')}
    with (OUT / (variant + '-format.log')).open('w') as log:
        subprocess.run(['cargo', 'fmt', '--all', '--', '--check'], cwd=source, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
    hashes = {name: digest(source / name) for name in paths}
    changed = {name for name, value in hashes.items() if parent_hashes[name] != value}
    assert changed == ({algebra} if variant == 'baseline' else {algebra, segment})
    maps[variant] = hashes
    compressed = OUT / (variant + '-source-hashes.json.gz')
    compressed.write_bytes(gzip.compress((json.dumps(hashes, indent=2) + '\n').encode(), mtime=0))
    record = {
        'scope': 'Original admitted production runtime' if variant == 'baseline' else 'Local reuse of an exact negative arc-endpoint certificate; unknown, approximate and preview retries remain unchanged.',
        'parent_source_record': str(parent_path), 'parent_source_record_sha256': digest(parent_path),
        'main_source_record': str(native_path), 'main_source_record_sha256': digest(native_path),
        'source_directory': str(source), 'target_directory': str(work / 'target'),
        'source_hashes_file': str(compressed), 'source_hashes_file_sha256': digest(compressed),
        'source_files': len(hashes), 'removed_prior_test_only_probe': old_helper,
        'only_runtime_change_from_admitted_source': None if variant == 'baseline' else segment,
        'contains_no_diagnostic_rust': True, 'main_runtime_unchanged': True,
    }
    (OUT / (variant + '-sources.json')).write_text(json.dumps(record, indent=2) + '\n')
assert [name for name in paths if maps['baseline'][name] != maps['candidate'][name]] == [segment]
candidate = WORK / 'candidate/source' / segment
with tarfile.open(OUT / 'candidate-source-delta.tar.gz', 'w:gz') as archive:
    archive.add(candidate, arcname=segment)
(OUT / 'candidate.patch').write_text(''.join(difflib.unified_diff(original.splitlines(True), candidate.read_text().splitlines(True), 'a/' + segment, 'b/' + segment)))
shutil.copyfile(__file__, OUT / 'prepare.py')
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
print('Frozen production pair:', len(paths), 'inputs per variant; only segment.rs differs; no diagnostic Rust.', flush=True)
