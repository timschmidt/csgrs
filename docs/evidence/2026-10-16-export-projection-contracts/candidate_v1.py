"""Validate exact export projections on a fresh physical source snapshot and empty target."""
import difflib
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import resource
import shutil
import subprocess
import time

ROOT = Path('/home/tim/Documents/GitHub/workspace/csgrs')
E = ROOT / 'docs/evidence/2026-10-16-export-projection-contracts'
E24 = ROOT / 'docs/evidence/2026-10-16-physical-build-requalification'
WORK = ROOT / 'target/properties/export-projection-contracts'
SOURCE, TARGET = WORK / 'candidate-source-v1', WORK / 'candidate-target-v1'
OUT = WORK / 'candidate-validation-v1'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


baseline = WORK / 'baseline-v1'
assert json.loads((baseline / 'result.json').read_text())['sources_and_all_observed_libraries_unchanged']
rows = json.loads((baseline / 'commands.json').read_text())
assert len(rows) == 6 and all(row['exit_code'] == 0 for row in rows if row['name'].endswith('-compile'))
for name, counts in [('format', '9 passed; 3 failed;'), ('topology', '4 passed; 0 failed;'), ('acceptance', '3 passed; 6 failed;')]:
    assert counts in (baseline / (name + '-test.stdout')).read_text()
with gzip.open(baseline / 'source-hashes.json.gz', 'rt') as stream:
    original = json.load(stream)
assert all(sha(ROOT / name) == digest for name, digest in original.items())
for directory in [SOURCE, TARGET, OUT]:
    directory.mkdir(exist_ok=False)
for name, digest in original.items():
    dest = SOURCE / name
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(ROOT / name, dest)
    assert sha(dest) == digest

def replace_once(text, old, new):
    assert text.count(old) == 1, old
    return text.replace(old, new)

changed = {}
for name in ['src/io/mod.rs', 'src/io/obj.rs', 'src/io/stl.rs']:
    changed[name] = (SOURCE / name).read_text()

path = SOURCE / 'src/io/mod.rs'
text = path.read_text()
text = replace_once(text, '    /// A modeling constructor failed while importing a format record.',
                    '    /// A native modeling operation or validation failed at a format boundary.')
marker = '#[cfg(any(feature = "stl-io", feature = "dxf-io"))]\npub(crate) fn finite_triangle_normal('
helper = '/// Exact decimal spelling of an already finite binary64 boundary value.\n///\n/// Retain the legacy 17 fractional digits whenever sufficient. A dyadic\n/// significand times 2^e needs at most -e fractional decimal digits after its\n/// trailing binary zeros are removed; binary64 bounds this precision at 1074.\n#[cfg(any(feature = "obj-io", feature = "stl-io"))]\npub(crate) fn decimal_f64(value: f64) -> impl std::fmt::Display {\n    struct Decimal(f64);\n\n    impl std::fmt::Display for Decimal {\n        fn fmt(&self, f: &mut std::fmt::Formatter<\'_>) -> std::fmt::Result {\n            let bits = self.0.to_bits();\n            let exponent = ((bits >> 52) & 0x7ff) as i32;\n            let significand = (bits & ((1_u64 << 52) - 1))\n                | if exponent == 0 { 0 } else { 1_u64 << 52 };\n            let fractional_bits = if significand == 0 {\n                0\n            } else {\n                1075 - exponent.max(1) - significand.trailing_zeros() as i32\n            };\n            let precision = fractional_bits.max(17) as usize;\n            write!(f, "{:.precision$}", self.0)\n        }\n    }\n\n    Decimal(value)\n}\n\n'
text = replace_once(text, marker, helper + marker)
path.write_text(text)

path = SOURCE / 'src/io/obj.rs'
text = path.read_text()
text = replace_once(text, 'use super::{IoError, finite_f64, single_line_metadata, triangulate_planar_face};',
                    'use super::{IoError, decimal_f64, finite_f64, single_line_metadata, triangulate_planar_face};')
text = replace_once(text, '/// to finite binary64 values at this format boundary; topology validation is a\n/// separate operation.',
                    '/// to finite binary64 values and written as exact terminating decimals, so\n/// reimport preserves that projection. Topology validation is a separate operation.')
text = replace_once(text, '"v {:.17} {:.17} {:.17}\\n",', '"v {} {} {}\\n",')
for axis in 'xyz':
    old = f'            finite_f64(&point.{axis}, "OBJ", "vertex {axis}")?,'
    text = replace_once(text, old, f'            decimal_f64(finite_f64(&point.{axis}, "OBJ", "vertex {axis}")?),')
path.write_text(text)

path = SOURCE / 'src/io/stl.rs'
text = path.read_text()
text = replace_once(text, '    IoError, finite_f32, finite_f64, finite_triangle_normal, single_line_metadata,',
                    '    IoError, decimal_f64, finite_f32, finite_f64, finite_triangle_normal, single_line_metadata,')
text = replace_once(text, '/// Serialize native triangle geometry as ASCII STL.',
                    '/// Serialize native triangle geometry as ASCII STL.\n///\n/// Finite binary64 coordinates and computed normals use exact decimal spelling.\n/// Reimport through [`from_stl`] subsequently quantizes coordinates to binary32;\n/// its proof concerns that parsed geometry, not the original exact model.')
text = replace_once(text, '"  facet normal {:.17} {:.17} {:.17}\\n",', '"  facet normal {} {} {}\\n",')
text = replace_once(text, '            normal[0], normal[1], normal[2],',
                    '            decimal_f64(normal[0]), decimal_f64(normal[1]), decimal_f64(normal[2]),')
text = replace_once(text, '"      vertex {:.17} {:.17} {:.17}\\n",', '"      vertex {} {} {}\\n",')
for axis in 'xyz':
    old = f'                finite_f64(&point.{axis}, "STL", "vertex {axis}")?,'
    text = replace_once(text, old, f'                decimal_f64(finite_f64(&point.{axis}, "STL", "vertex {axis}")?),')
text = replace_once(text, '/// Serialize native triangle geometry as binary STL.',
                    '/// Serialize native triangle geometry as binary STL.\n///\n/// Coordinates are projected to finite binary32 values. Each encoded triangle\n/// is certified nondegenerate under the strict native policy before complete\n/// bytes are published or cached. Open triangle surfaces remain exportable;\n/// closed topology and fidelity before projection are separate claims.')
old = '            Ok(StlTriangle {\n                normal: Normal::new([normal[0] as f32, normal[1] as f32, normal[2] as f32]),\n                vertices: [point(&points[0])?, point(&points[1])?, point(&points[2])?],\n            })'
new = '            let vertices = [point(&points[0])?, point(&points[1])?, point(&points[2])?];\n            let projected = vertices.each_ref().map(|vertex| {\n                Point3::new(\n                    Real::try_from(vertex[0]).expect("finite_f32 established a finite value"),\n                    Real::try_from(vertex[1]).expect("finite_f32 established a finite value"),\n                    Real::try_from(vertex[2]).expect("finite_f32 established a finite value"),\n                )\n            });\n            if !hypermesh::Plane::points_are_nondegenerate(\n                &crate::GeometryContext::STRICT.mesh_context(),\n                &projected[0],\n                &projected[1],\n                &projected[2],\n            )\n            .map_err(|error| IoError::Construction {\n                format: "STL",\n                source: error.into(),\n            })?\n            .value\n            {\n                return Err(IoError::Geometry {\n                    format: "STL",\n                    detail: "degenerate triangle after binary32 projection".into(),\n                });\n            }\n            Ok(StlTriangle {\n                normal: Normal::new([normal[0] as f32, normal[1] as f32, normal[2] as f32]),\n                vertices,\n            })'
text = replace_once(text, old, new)
path.write_text(text)

test_name = 'tests/export_projection_contracts.rs'
shutil.copyfile(E / 'export_projection_contracts_v1.rs', SOURCE / test_name)
subprocess.run(['rustfmt', '--edition', '2024', '--config-path', str(SOURCE / '.rustfmt.toml'),
                *[str(SOURCE / name) for name in changed], str(SOURCE / test_name)], check=True)
assert sha(SOURCE / test_name) == sha(E / 'export_projection_contracts_v1.rs')
(OUT / 'production.patch').write_text(''.join(''.join(difflib.unified_diff(before.splitlines(keepends=True),
    (SOURCE / name).read_text().splitlines(keepends=True), fromfile='a/' + name, tofile='b/' + name)) for name, before in changed.items()))
inputs = original | {name: sha(SOURCE / name) for name in [*changed, test_name]}
with gzip.open(OUT / 'source-hashes.json.gz', 'wt') as stream:
    json.dump(inputs, stream, sort_keys=True)
scratch = WORK / 'candidate-scratch-v1'
scratch.mkdir(exist_ok=False)
environment = os.environ.copy()
for key in ['RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS', 'RUSTC_WRAPPER', 'RUSTC_WORKSPACE_WRAPPER', 'RUSTDOCFLAGS']:
    environment.pop(key, None)
environment.update({'CARGO_HOME': '/home/tim/.cargo', 'CARGO_TARGET_DIR': str(TARGET),
                    'CARGO_BUILD_JOBS': '2', 'CARGO_NET_OFFLINE': 'true', 'CARGO_INCREMENTAL': '0',
                    'TMPDIR': str(scratch), 'CARGO_TERM_COLOR': 'never', 'RUST_BACKTRACE': '0'})
probes = {'format': E24 / 'format_probe_v1.rs', 'topology': E24 / 'stl-topology-v1/probe_v1.rs'}
protocol = {'runner_sha256': sha(__file__), 'source': str(SOURCE), 'target': str(TARGET),
            'base_commit': 'c9410688b0c7854fba6c9a90f34cae5b3382318a',
            'source_hashes_sha256': sha(OUT / 'source-hashes.json.gz'),
            'target_initially_empty': not list(TARGET.iterdir()),
            'target_strategy': 'New empty physical ext4 target; no inherited build artifacts or overlays.',
            'mount': subprocess.check_output(['findmnt', '-T', str(TARGET), '-n', '-o', 'TARGET,FSTYPE,OPTIONS'], text=True),
            'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
            'cargo': subprocess.check_output(['cargo', '-V'], text=True),
            'probes': {name: {'path': str(path), 'sha256': sha(path)} for name, path in probes.items()},
            'environment': {k: environment[k] for k in ['CARGO_HOME','CARGO_TARGET_DIR','CARGO_BUILD_JOBS','CARGO_NET_OFFLINE','CARGO_INCREMENTAL','TMPDIR']},
            'elapsed_limit_seconds_per_command': 1800, 'address_space_limit_bytes_per_process': 8 * 1024**3,
            'native_runtime_admitted': False, 'full_plan_complete': False}
assert 'ext4' in protocol['mount'] and 'fuse' not in protocol['mount']
dump(OUT / 'protocol.json', protocol)
rows = []
retained = WORK / 'candidate-executables-v1'
retained.mkdir(exist_ok=False)


def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


def run(name, command, *, expected=0, docs=False):
    assert sha(__file__) == protocol['runner_sha256']
    assert all(sha(SOURCE / path) == digest for path, digest in inputs.items())
    print('START ' + name, flush=True)
    env = environment | ({'RUSTDOCFLAGS': '-D warnings'} if docs else {})
    start = time.monotonic()
    with (OUT / (name + '.stdout')).open('wb') as stdout, (OUT / (name + '.stderr')).open('wb') as stderr:
        result = subprocess.run(['/usr/bin/time', '-v', '-o', str(OUT / (name + '.resources')),
                                 'timeout', '--signal=TERM', '--kill-after=20s', '1800s', *command],
                                cwd=SOURCE, env=env, stdout=stdout, stderr=stderr,
                                preexec_fn=limits, start_new_session=True)
    assert all(sha(SOURCE / path) == digest for path, digest in inputs.items())
    paths = {Path(p) for p in re.findall(r'Running[^\n]*\(([^)]+)\)', (OUT / (name + '.stderr')).read_text(errors='replace'))}
    paths.update(Path(p) for p in re.findall(r'Running `([^`]+)`', (OUT / (name + '.stderr')).read_text(errors='replace')))
    if command[0] == 'rustc' and result.returncode == 0:
        paths.add(Path(command[command.index('-o') + 1]))
    if Path(command[0]).is_absolute() and Path(command[0]).is_relative_to(WORK):
        paths.add(Path(command[0]))
    binaries = {}
    for path in paths:
        path = path if path.is_absolute() else SOURCE / path
        digest = sha(path)
        copy = retained / digest / path.name
        copy.parent.mkdir(exist_ok=True)
        if not copy.exists():
            shutil.copy2(path, copy)
        assert sha(copy) == digest
        binaries[str(path)] = {'sha256': digest, 'retained': str(copy), 'bytes': path.stat().st_size}
    row = {'name': name, 'command': command, 'exit_code': result.returncode,
           'expected_exit_code': expected, 'elapsed_seconds': time.monotonic() - start,
           'binaries': binaries, 'sources_unchanged': True, 'rustdocflags': env.get('RUSTDOCFLAGS'),
           **{s + '_sha256': sha(OUT / (name + '.' + s)) for s in ['stdout', 'stderr', 'resources']}}
    rows.append(row)
    dump(OUT / 'commands.json', rows)
    print(json.dumps({k: row[k] for k in ['name','exit_code','expected_exit_code','elapsed_seconds']}), flush=True)
    return result.returncode


def cargo(action, *args):
    return ['cargo', action, '--manifest-path', str(SOURCE / 'Cargo.toml'), '--locked', '--release', '-p', 'csgrs', *args]


run('format', ['cargo', 'fmt', '--manifest-path', str(SOURCE / 'Cargo.toml'), '--all', '--', '--check'])
assert run('minimal-regression', cargo('test', '--no-default-features', '--features', 'obj-io,stl-io', '--test', 'export_projection_contracts', '--', '--test-threads=1')) == 0
deps = TARGET / 'release/deps'
bindings = {}
for crate in ['csgrs', 'hypermesh', 'hyperlattice']:
    paths = sorted(deps.glob('lib' + crate + '*.rlib'))
    assert len(paths) == 1, (crate, paths)
    copy = WORK / 'probe-libraries-v1' / paths[0].name
    copy.parent.mkdir(exist_ok=True)
    shutil.copy2(paths[0], copy)
    assert sha(paths[0]) == sha(copy)
    bindings[crate] = {'path': str(paths[0]), 'retained': str(copy), 'sha256': sha(copy)}
dump(OUT / 'probe-bindings.json', bindings)
remaining = set()
for name, source in probes.items():
    binary = WORK / (name + '-candidate-v1')
    command = ['rustc', '--edition=2024', '--test', '-C', 'opt-level=3', '-C', 'debug-assertions=off', '-L', 'dependency=' + str(deps)]
    for crate, record in bindings.items():
        command += ['--extern', crate + '=' + record['retained']]
    assert run('compile-' + name, [*command, str(source), '-o', str(binary)]) == 0
    run(name + '-probe', [str(binary), '--test-threads=1', '--nocapture'], expected=0)
    if name == 'format':
        text = (OUT / 'format-probe.stdout').read_text()
        assert '12 passed; 0 failed; 0 ignored;' in text
        assert set(re.findall(r'^test (\S+) \.\.\. FAILED$', text, re.MULTILINE)) == remaining
run('root-all', cargo('test', '--all-features', '--no-fail-fast', '--', '--test-threads=2'))
run('stl-only', cargo('test', '--no-default-features', '--features', 'stl-io', '--no-fail-fast', '--', '--test-threads=2'))
run('obj-only', cargo('test', '--no-default-features', '--features', 'obj-io', '--no-fail-fast', '--', '--test-threads=2'))
run('basic-shapes', cargo('run', '--no-default-features', '--features', 'curve,stl-io', '--example', 'basic_shapes'))
example_outputs = sorted((SOURCE / 'stl/examples/basic_shapes').glob('*.stl'))
assert {path.stem for path in example_outputs} == {'cube', 'sphere', 'cylinder', 'ellipsoid', 'torus'}
for path in example_outputs:
    assert path.read_bytes() == (ROOT / 'docs/evidence/2026-10-16-stl-native-validation/basic-shapes-output-v2' / path.name).read_bytes()
run('strict-clippy', cargo('clippy', '--all-features', '--all-targets', '--', '-D', 'warnings'))
run('stl-clippy', cargo('clippy', '--no-default-features', '--features', 'stl-io', '--all-targets', '--', '-D', 'warnings'))
run('basic-shapes-clippy', cargo('clippy', '--no-default-features', '--features', 'curve,stl-io', '--example', 'basic_shapes', '--', '-D', 'warnings'))
run('obj-clippy', cargo('clippy', '--no-default-features', '--features', 'obj-io', '--all-targets', '--', '-D', 'warnings'))
run('docs', cargo('doc', '--all-features', '--no-deps'), docs=True)
assert all(sha(row['path']) == row['sha256'] for row in protocol['probes'].values())
assert all(sha(row['retained']) == row['sha256'] for row in bindings.values())
record = {'complete': True, 'commands': len(rows),
          'unexpected_exits': [row['name'] for row in rows if row['exit_code'] != row['expected_exit_code']],
          'known_unfixed_format_failures': sorted(remaining), 'sources_unchanged': True,
          'main_sources_modified': False, 'native_runtime_admitted': False, 'full_plan_complete': False}
dump(OUT / 'result.json', record)
print(json.dumps(record), flush=True)
raise SystemExit(101 if record['unexpected_exits'] else 0)
