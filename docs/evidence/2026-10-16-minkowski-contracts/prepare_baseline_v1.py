"""Pin current libraries and the complete pre-edit diagnostic/cost phase."""
import hashlib
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OLD = E.parent / '2026-10-16-polyhedral-construction'
TARGET = Path('/dev/shm/csgrs-minkowski-contracts/baseline-v1')
BUILD = ROOT / 'target/properties/polyhedral-construction/cost-build-v1/release/deps'


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == 'adef9a64a91a7524006ba41527e427e2edb79fbe'
assert not TARGET.exists()
driver = (OLD / 'resource_probe_v1.rs').read_text().replace('use hypermesh::Point3;\n', '')
start, end = driver.index('fn build('), driver.index('fn record(')
driver = driver[:start] + '''fn build(case: &str) -> TriangleMesh {
    assert_eq!(case, "cube-minkowski");
    let outcome = solid::minkowski_sum_with_context(
        &solid::cube(r(2)).unwrap(), &solid::cube(r(3)).unwrap(),
        &GeometryContext::STRICT).unwrap();
    assert_eq!(outcome.certainty, GeometryCertainty::Certified);
    outcome.value
}
''' + driver[end:]
driver = driver.replace('    let serialized = record(&replay);', '''    replay.validate_closed_pwn(&GeometryContext::STRICT.mesh_context()).unwrap();
    let six_volume = replay.triangles.iter().fold(r(0), |sum, triangle| {
        let [a,b,c] = triangle.indices().map(|i| replay.positions[i].to_vector());
        sum + a.dot(&b.cross(&c))
    });
    assert_eq!(six_volume.exact_rational_normal_form(), r(750).exact_rational_normal_form());
    let serialized = record(&replay);''')
path = E / 'resource_probe_v1.rs'
assert not path.exists()
path.write_text(driver)
commands = [['mkdir', ['mkdir', '-p', str(TARGET)]]]
common = ['rustc', '--edition=2024', '-Copt-level=2', '-D', 'warnings',
          '--extern', f'csgrs={BUILD}/libcsgrs.rlib',
          '--extern', f'hypermesh={BUILD}/libhypermesh-98c5db38df7e7da0.rlib',
          '-L', f'dependency={BUILD}']
commands += [
    ['diagnostic-build', common + [str(OLD / 'baseline_probe_v1.rs'), '-o', str(TARGET / 'diagnostic')]],
    ['diagnostic', [str(TARGET / 'diagnostic'), str(TARGET / 'fixtures')]],
]
for kind in ['timing', 'allocation']:
    command = common + ['--check-cfg', 'cfg(allocation)', str(path), '-o', str(TARGET / kind)]
    if kind == 'allocation': command += ['--cfg', 'allocation']
    commands.append(['build-' + kind, command])
for kind, count in [('timing', 3), ('allocation', 1)]:
    for temperature in ['cold', 'warm']:
        for sample in range(count):
            name = f'{kind}-{temperature}-{sample}'
            commands.append([name, [str(TARGET / kind), 'cube-minkowski', temperature,
                                    str(TARGET / (name + '.json'))]])
preserved = {str(path): sha(path) for path in BUILD.iterdir()
             if path.is_file() and path.suffix in {'.rlib', '.rmeta', '.so'}}
for suffix, value in [('.json', commands), ('-target.json', str(TARGET)),
                      ('-seed.json', None), ('-preserve.json', preserved)]:
    path = E / ('baseline-v1' + suffix)
    assert not path.exists()
    path.write_text(json.dumps(value, indent=2) + '\n')
record = {'template_sha256': sha(OLD / 'resource_probe_v1.rs'),
          'diagnostic_sha256': sha(OLD / 'baseline_probe_v1.rs'),
          'resource_driver_sha256': sha(E / 'resource_probe_v1.rs'),
          'commands': len(commands), 'library_pins': len(preserved)}
(E / 'baseline-preparation-v1.json').write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps(record))
