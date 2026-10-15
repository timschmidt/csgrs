"""Capture unresolved line orientations using the exact Cargo dependency graph."""
from pathlib import Path
import difflib, hashlib, json, shutil

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
CORE = Path('/run/user/1000/csgrs-quotient-scalar-proofs/scalar-and-drill-v1/release')
TARGET = Path('/tmp/csgrs-quotient-scalar-proofs/diagnostic-v1')
SOURCE = Path('/tmp/csgrs-quotient-scalar-proofs/diagnostic-v1-source/hypermesh/src')
PROBE = ROOT/'docs/evidence/2026-10-15-strict-drilled-workflows/drill_probe_v1.rs'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
assert json.loads((E/'scalar-and-drill-v1/result.json').read_text())['passed']
assert not TARGET.exists() and not SOURCE.exists()
root_fingerprints = list(CORE.glob('.fingerprint/csgrs-*/lib-csgrs.json'))
assert len(root_fingerprints) == 1
root = json.loads(root_fingerprints[0].read_text())
assert json.loads(root['features']) == ['curve']
required = {name: fingerprint for _, name, _, fingerprint in root['deps']}
mapping = {}


def library(name):
    expected = required[name]
    candidates = [p for p in (CORE/'.fingerprint').glob(name+'-*/lib-'+name)
                  if int.from_bytes(bytes.fromhex(p.read_text().strip()), 'little') == expected]
    assert len(candidates) == 1, (name, candidates)
    fingerprint = candidates[0]
    suffix = fingerprint.parent.name[len(name)+1:]
    path = CORE/'deps'/f'lib{name}-{suffix}.rlib'
    assert path.is_file()
    metadata = json.loads(fingerprint.with_suffix('.json').read_text())
    mapping[name] = {'path': str(path), 'sha256': sha(path), 'cargo_fingerprint': expected,
                     'fingerprint_json': str(fingerprint.with_suffix('.json')),
                     'features': metadata['features']}
    if name == 'hyperphysics':
        required['hypersolve'] = next(value for _, dependency, _, value in metadata['deps'] if dependency == 'hypersolve')
    return path


library('hyperphysics')
shutil.copytree(ROOT/'crates/hypermesh/src', SOURCE)
source = SOURCE/'surface_arrangement.rs'
before = source.read_text()
needle = '    let side = decisions.decide(outcome, "surface arrangement retained line orientation")?;'
assert before.count(needle) == 1
insertion = """    if matches!(outcome, hyperlimit::PredicateOutcome::Unknown { .. }) {
        let abx = &edge[1].x - &edge[0].x;
        let aby = &edge[1].y - &edge[0].y;
        let acx = &point.x - &edge[0].x;
        let acy = &point.y - &edge[0].y;
        let det = &abx * &acy - &aby * &acx;
        eprintln!("unresolved_orientation_from={:?}", edge[0]);
        eprintln!("unresolved_orientation_to={:?}", edge[1]);
        eprintln!("unresolved_orientation_point={point:?}");
        eprintln!("unresolved_orientation_determinant={det:?}");
        eprintln!("{}", std::backtrace::Backtrace::force_capture());
    }
"""
after = before.replace(needle, insertion + needle)
source.write_text(after)
(E/'diagnostic-v1.patch').write_text(''.join(difflib.unified_diff(
    before.splitlines(True), after.splitlines(True),
    fromfile='crates/hypermesh/src/surface_arrangement.rs', tofile='diagnostic-v1-source/surface_arrangement.rs')))


def build(name, source, dependencies):
    command = ['rustc', '--crate-name', name, '--crate-type', 'rlib', '--edition=2024',
               '-C', 'opt-level=3', '-C', 'debuginfo=1', '-C', 'force-frame-pointers=yes',
               '-D', 'warnings', str(source), '-L', f'dependency={CORE}/deps',
               '-L', f'dependency={TARGET}', '-o', str(TARGET/f'lib{name}.rlib')]
    if name == 'csgrs': command += ['--cfg', 'feature="curve"']
    for dependency in dependencies:
        path = TARGET/f'lib{dependency}.rlib' if dependency in ['hypermesh', 'hyperphysics'] else library(dependency)
        command += ['--extern', f'{dependency}={path}']
    return command


commands = [
    ['prepare-target', ['mkdir', str(TARGET)]],
    ['native-build', build('hypermesh', SOURCE/'lib.rs', ['hyperreal', 'hyperlattice', 'hyperlimit', 'hypertri'])],
    ['physics-build', build('hyperphysics', ROOT/'crates/hyperphysics/src/lib.rs', ['hyperreal', 'hyperlattice', 'hyperlimit', 'hypersolve', 'hypermesh'])],
    ['root-build', build('csgrs', ROOT/'src/lib.rs', ['hyperreal', 'hyperlattice', 'hyperlimit', 'hypertri', 'hypercurve', 'hypermesh', 'hyperphysics', 'thiserror'])],
    ['probe-build', ['rustc', '--edition=2024', '-C', 'opt-level=3', '-C', 'debuginfo=1',
                     '-C', 'force-frame-pointers=yes', '-D', 'warnings', '--extern',
                     f'csgrs={TARGET}/libcsgrs.rlib', '-L', f'dependency={CORE}/deps',
                     '-L', f'dependency={TARGET}', str(PROBE), '-o', str(TARGET/'probe')]],
    ['drill-16-strict', [str(TARGET/'probe'), '16', 'strict']],
    ['drill-32-strict', [str(TARGET/'probe'), '32', 'strict']],
]
keep = json.loads((E/'scalar-and-drill-v1-preserve.json').read_text())
for p in [*CORE.rglob('*.rlib'), *CORE.rglob('*.so'), *SOURCE.rglob('*.rs'),
          PROBE, Path(__file__), *CORE.glob('.fingerprint/*/lib-*.json')]:
    keep[str(p)] = sha(p)
for suffix, value in [('.json', commands), ('-seed.json', None), ('-preserve.json', keep)]:
    p = E/('diagnostic-v1'+suffix)
    assert not p.exists()
    p.write_text(json.dumps(value, indent=2)+'\n')
(E/'diagnostic-dependencies-v1.json').write_text(json.dumps(mapping, indent=2)+'\n')
print(json.dumps({'phase': 'diagnostic-v1', 'libraries_pinned': len(mapping), 'commands': len(commands)}))
