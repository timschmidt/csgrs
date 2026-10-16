"""Freeze the resource matrix before observing any candidate measurements."""
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
PHASE = 'resources-v1'
TARGET = Path('/dev/shm/csgrs-gear-airfoil-failures') / PHASE
BUILDS = {
    'baseline': ROOT / 'target/properties/hilbert-trimming/candidate-v2/release/deps',
    'candidate': ROOT / 'target/properties/gear-airfoil-failures/qualification-v1/release/deps',
}
CASES = ['involute_gear', 'cycloidal_gear', 'involute_rack', 'cycloidal_rack', 'airfoil_naca4']
PROBE = str((E / 'resource-probe/allocation_v1.rs').relative_to(ROOT))
assert not TARGET.exists() and not (E / PHASE).exists()


def dump(name, value):
    path = E / name
    assert not path.exists(), path
    path.write_text(json.dumps(value, indent=2) + '\n')


commands = [
    ['mkdir', ['mkdir', '-p', str(TARGET)]],
    ['allocation-driver-fmt', ['rustfmt', '--edition', '2024', '--check', PROBE]],
]
preserved = {}
for variant, build in BUILDS.items():
    files = [p for p in build.iterdir() if p.is_file() and p.suffix in {'.rlib', '.rmeta', '.so'}]
    files += [build / 'profile_primitives-a860b8425128ee3e', build / 'feature_pipeline-7fecdce69333ac9d']
    for path in sorted(files):
        assert path.is_file() and not path.is_symlink(), path
        with path.open('rb') as stream:
            preserved[str(path)] = hashlib.file_digest(stream, 'sha256').hexdigest()
    commands.append([f'allocation-build-{variant}', [
        'rustc', '--edition=2024', '-Copt-level=2', '-D', 'warnings', PROBE,
        '--extern', f'csgrs={build}/libcsgrs.rlib',
        '--extern', f'hypercurve={build}/libhypercurve-65c0df1f5cff6824.rlib',
        '-L', f'dependency={build}', '-o', str(TARGET / f'alloc-{variant}'),
    ]])

for temperature in ['cold', 'warm']:
    for case in CASES:
        for variant in BUILDS:
            commands.append([f'alloc-{temperature}-{case}-{variant}', [
                str(TARGET / f'alloc-{variant}'), case, temperature,
            ]])
    for pair in range(5):
        variants = ['baseline', 'candidate'] if pair % 2 == 0 else ['candidate', 'baseline']
        for variant in variants:
            commands.append([f'timing-{temperature}-{pair}-{variant}', [
                'env', 'CSGRS_BENCH_SAMPLES=1',
                'CSGRS_BENCH_WARMUP=' + ('0' if temperature == 'cold' else '1'),
                'CSGRS_BENCH_ITERATIONS=' + ('1' if temperature == 'cold' else '8'),
                f'CSGRS_BENCH_TEMPERATURE={temperature}', f'CSGRS_BENCH_SAMPLE_OFFSET={pair}',
                'CSGRS_BENCH_FILTER=' + ','.join('profile_primitives/constructor/' + case for case in CASES),
                str(BUILDS[variant] / 'profile_primitives-a860b8425128ee3e'),
            ]])
for variant, build in BUILDS.items():
    commands.append([f'catalog-{variant}', [
        'env', 'CSGRS_BENCH_SAMPLES=1', 'CSGRS_BENCH_WARMUP=0', 'CSGRS_BENCH_ITERATIONS=1',
        'CSGRS_BENCH_TEMPERATURE=cold', 'CSGRS_BENCH_SAMPLE_OFFSET=0',
        'CSGRS_BENCH_FILTER=feature/mesh_primitives/catalog,feature/profile_primitives/catalog',
        str(build / 'feature_pipeline-7fecdce69333ac9d'),
    ]])
assert len(commands) == 46
dump(PHASE + '.json', commands)
dump(PHASE + '-target.json', str(TARGET))
dump(PHASE + '-seed.json', None)
dump(PHASE + '-preserve.json', preserved)
print(json.dumps({'commands': len(commands), 'pinned_input_files': len(preserved)}))
