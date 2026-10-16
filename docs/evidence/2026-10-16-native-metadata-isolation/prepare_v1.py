"""Copy current native inputs and isolate one predicate-order change."""
import difflib
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
ROOT = E.parents[2]
WORK = Path('/dev/shm/csgrs-native-metadata-isolation')
HEAD = '73472622dd4034b8b8777cb2a116c4020f6bb328'
PREVIOUS = E.parent / '2026-10-16-native-admission-recheck/checkpoint-v1.json'
RUNNER = E.parent / '2026-10-15-factored-product-proofs/validate_v8.py'


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def dump(path, data):
    path.write_text(json.dumps(data, indent=2) + '\n')


assert not WORK.exists() and not (E / 'preparation-v1.json').exists()
assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == HEAD
assert sha(RUNNER) == 'f3ee76657ed7c1e8792452ded7c2380c1ad03a41abb312d80f9adf68b4dc6f05'
spec = importlib.util.spec_from_file_location('guard', RUNNER)
guard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(guard)
main_sources = guard.sources()
previous = json.loads(PREVIOUS.read_text())
pending = previous['pending_native_sha256']
assert len(pending) == 35 and all(sha(ROOT / p) == h for p, h in pending.items())
assert sha(ROOT / 'docs/properties-status.json') == previous['ledger_after_sha256']
names = subprocess.check_output([
    'git', 'ls-files', '-z', '--cached', '--others', '--exclude-standard'
], cwd=ROOT).decode().split('\0')
excluded = ['docs/evidence/', 'crates/comparisons/',
            'crates/hyperreal/verification/dependencies/']
names = sorted({name for name in names if name and not name.startswith(tuple(excluded))
                and (not name.startswith('benchmarks/')
                     or name.startswith(('benchmarks/rust/', 'benchmarks/support/')))})
assert all((ROOT / name).is_file() and not (ROOT / name).is_symlink() for name in names)
original = {name: sha(ROOT / name) for name in names}
assert all(original.get(p) == h for p, h in pending.items())
WORK.mkdir()
region = 'crates/hypercurve/src/bezier_region.rs'
tests = 'crates/hypercurve/src/bezier_region/metadata_order_tests.rs'
assert tests not in original
before = ('            boundary_loop.fragments == candidate.fragments\n'
          '                && boundary_loop.arrangement_sources == candidate.arrangement_sources')
after = ('            boundary_loop.arrangement_sources == candidate.arrangement_sources\n'
         '                && boundary_loop.fragments == candidate.fragments')
sources = {}
hashes = {}
for variant in ['baseline', 'candidate']:
    source = WORK / variant / 'source'
    source.mkdir(parents=True)
    for name in names:
        target = source / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / name, target)
        assert target.stat().st_ino != (ROOT / name).stat().st_ino or target.stat().st_dev != (ROOT / name).stat().st_dev
    path = source / region
    text = path.read_text()
    assert text.count(before) == 1
    if variant == 'candidate':
        text = text.replace(before, after)
    anchor = '#[cfg(test)]\nmod tests;'
    assert text.count(anchor) == 1
    text = text.replace(anchor, anchor + '\n\n#[cfg(test)]\nmod metadata_order_tests;')
    path.write_text(text)
    shutil.copyfile(E / 'metadata_order_tests.rs', source / tests)
    with (E / ('format-' + variant + '-v1.log')).open('wb') as log:
        subprocess.run(['rustfmt', '--edition', '2024', str(path)],
                       stdout=log, stderr=subprocess.STDOUT, check=True)
    current = {name: sha(source / name) for name in [*names, tests]}
    changed = {name: h for name, h in current.items() if original.get(name) != h}
    assert set(changed) == {region, tests}
    with tarfile.open(E / (variant + '-delta-v1.tar.gz'), 'w:gz') as archive:
        for name in sorted(changed):
            archive.add(source / name, arcname=name)
    patch = ''.join(''.join(difflib.unified_diff(
        (ROOT / name).read_text().splitlines(True) if name in original else [],
        (source / name).read_text().splitlines(True), 'a/' + name, 'b/' + name))
        for name in sorted(changed))
    (E / (variant + '-v1.patch')).write_text(patch)
    sources[variant] = str(source)
    hashes[variant] = current
assert {p for p in hashes['baseline'] if hashes['baseline'][p] != hashes['candidate'][p]} == {region}
with gzip.open(E / 'source-hashes-v1.json.gz', 'wt') as stream:
    json.dump({'main_guard': main_sources, 'original': original, **hashes}, stream, sort_keys=True)
assert guard.sources() == main_sources
assert all(sha(ROOT / name) == h for name, h in original.items())
dump(E / 'preparation-v1.json', {
    'head': HEAD, 'previous_checkpoint': str(PREVIOUS.relative_to(ROOT)),
    'previous_checkpoint_sha256': sha(PREVIOUS), 'runner_sha256': sha(Path(__file__)),
    'sources': sources, 'source_hashes_sha256': sha(E / 'source-hashes-v1.json.gz'),
    'copied_original_files_per_variant': len(original),
    'copied_original_bytes_per_variant': sum((ROOT / p).stat().st_size for p in original),
    'excluded_prefixes': excluded,
    'benchmark_copy_scope': ['benchmarks/rust/', 'benchmarks/support/'],
    'pending_native_sha256': pending, 'main_sources_unchanged': True,
    'production_difference': 'Only duplicate-evidence conjunction operand order.',
    'identical_tests_sha256': hashes['baseline'][tests],
    'runtime_admitted': False, 'full_plan_complete': False,
})
print(json.dumps({'source_files_per_variant': len(original) + 1,
                  'production_changed_paths': [region], 'main_native_unchanged': 35}), flush=True)
