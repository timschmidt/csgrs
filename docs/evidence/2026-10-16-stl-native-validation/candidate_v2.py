"""Validate STL import on a fresh physical source snapshot and empty target."""
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
E = ROOT / 'docs/evidence/2026-10-16-stl-native-validation'
E24 = ROOT / 'docs/evidence/2026-10-16-physical-build-requalification'
WORK = ROOT / 'target/properties/stl-native-validation'
SOURCE, TARGET = WORK / 'candidate-source-v2', WORK / 'candidate-target-v2'
OUT = WORK / 'candidate-validation-v2'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


baseline = WORK / 'baseline-v1'
previous = json.loads((WORK / 'candidate-validation-v1/result.json').read_text())
assert previous['complete'] and previous['unexpected_exits'] == ['stl-only', 'stl-clippy']
assert json.loads((baseline / 'result.json').read_text())['sources_and_all_observed_libraries_unchanged']
baseline_rows = json.loads((baseline / 'commands.json').read_text())
assert len(baseline_rows) == 6
assert all(row['exit_code'] == (0 if row['name'].endswith('-compile') else 101) for row in baseline_rows)
for name, counts in [('format', '7 passed; 5 failed;'), ('topology', '2 passed; 2 failed;'), ('acceptance', '3 passed; 5 failed;')]:
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

stl_path = SOURCE / 'src/io/stl.rs'
before = stl_path.read_text()
after = before.replace('/// Import validated STL triangles into the native Hypermesh carrier.', '''/// Import STL triangles with certified closed-PWN topology into Hypermesh.
///
/// Both encodings parse coordinates as `f32`, then promote finite values exactly
/// to native scalars. Strict native validation proves nondegenerate faces and
/// balanced geometric edge multiplicities, retaining that proof for later
/// operations. Canonical empty geometry is also accepted. Disconnected, nested,
/// inverted and multiply wound closed surfaces need not be manifolds.
///
/// This certifies the parsed geometry, not fidelity to an earlier exact model
/// or analytic surface. Invalid topology returns [`IoError::Construction`] with
/// the native mesh error preserved as its cause; parsing errors remain IO errors.''')
assert after != before
assert after.count('    indexed.validate()?;\n') == 1
after = after.replace('    indexed.validate()?;\n', '')
assert after.count('        let points = face.vertices.map(|index| positions[index].clone());\n        triangle_normal(&points)?;\n') == 1
after = after.replace('        let points = face.vertices.map(|index| positions[index].clone());\n        triangle_normal(&points)?;\n', '')
assert after.count('    Ok(TriangleMesh::new(positions, triangles))') == 1
after = after.replace('    Ok(TriangleMesh::new(positions, triangles))', '''    let mesh = TriangleMesh::new(positions, triangles);
    if !(mesh.positions.is_empty() && mesh.triangles.is_empty()) {
        mesh.validate_closed_pwn(&crate::GeometryContext::STRICT.mesh_context())
            .map_err(|error| IoError::Construction {
                format: "STL",
                source: error.into(),
            })?;
    }
    Ok(mesh)''')
stl_path.write_text(after)
test_path = SOURCE / 'tests/stl_import_contracts.rs'
shutil.copyfile(E / 'stl_import_contracts_v1.rs', test_path)
subprocess.run(['rustfmt', '--edition', '2024', '--config-path', str(SOURCE / '.rustfmt.toml'), str(stl_path), str(test_path)], check=True)
assert sha(test_path) == sha(E / 'stl_import_contracts_v1.rs')
(OUT / 'production.patch').write_text(''.join(difflib.unified_diff(before.splitlines(keepends=True), stl_path.read_text().splitlines(keepends=True), fromfile='a/src/io/stl.rs', tofile='b/src/io/stl.rs')))
manifest = SOURCE / 'Cargo.toml'
manifest_before = manifest.read_text()
old_requirement = 'name = "basic_shapes"\nrequired-features = ["stl-io"]'
assert manifest_before.count(old_requirement) == 1
assert json.loads((WORK / 'example-baseline-v1/result.json').read_text())['exit_code'] == 1
manifest.write_text(manifest_before.replace(old_requirement, 'name = "basic_shapes"\nrequired-features = ["curve", "stl-io"]'))
(OUT / 'manifest.patch').write_text(''.join(difflib.unified_diff(manifest_before.splitlines(keepends=True), manifest.read_text().splitlines(keepends=True), fromfile='a/Cargo.toml', tofile='b/Cargo.toml')))
assert sha(stl_path) == sha(WORK / 'candidate-source-v1/src/io/stl.rs')
assert sha(test_path) == sha(WORK / 'candidate-source-v1/tests/stl_import_contracts.rs')
inputs = original | {'Cargo.toml': sha(manifest), 'src/io/stl.rs': sha(stl_path), 'tests/stl_import_contracts.rs': sha(test_path)}
with gzip.open(OUT / 'source-hashes.json.gz', 'wt') as stream:
    json.dump(inputs, stream, sort_keys=True)
scratch = WORK / 'candidate-scratch-v2'
scratch.mkdir(exist_ok=False)
environment = os.environ.copy()
for key in ['RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS', 'RUSTC_WRAPPER', 'RUSTC_WORKSPACE_WRAPPER', 'RUSTDOCFLAGS']:
    environment.pop(key, None)
environment.update({'CARGO_HOME': '/home/tim/.cargo', 'CARGO_TARGET_DIR': str(TARGET),
                    'CARGO_BUILD_JOBS': '2', 'CARGO_NET_OFFLINE': 'true', 'CARGO_INCREMENTAL': '0',
                    'TMPDIR': str(scratch), 'CARGO_TERM_COLOR': 'never', 'RUST_BACKTRACE': '0'})
probes = {'format': E24 / 'format_probe_v1.rs', 'topology': E24 / 'stl-topology-v1/probe_v1.rs'}
protocol = {'runner_sha256': sha(__file__), 'source': str(SOURCE), 'target': str(TARGET),
            'previous_candidate_result_sha256': sha(WORK / 'candidate-validation-v1/result.json'),
            'base_commit': '7a1f0c7e1f4f69499feb3b471f739dfb50f4776c',
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
retained = WORK / 'candidate-executables-v2'
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
assert run('minimal-regression', cargo('test', '--no-default-features', '--features', 'obj-io,stl-io', '--test', 'stl_import_contracts', '--', '--test-threads=1')) == 0
deps = TARGET / 'release/deps'
bindings = {}
for crate in ['csgrs', 'hypermesh', 'hyperlattice']:
    paths = sorted(deps.glob('lib' + crate + '*.rlib'))
    assert len(paths) == 1, (crate, paths)
    copy = WORK / 'probe-libraries-v2' / paths[0].name
    copy.parent.mkdir(exist_ok=True)
    shutil.copy2(paths[0], copy)
    assert sha(paths[0]) == sha(copy)
    bindings[crate] = {'path': str(paths[0]), 'retained': str(copy), 'sha256': sha(copy)}
dump(OUT / 'probe-bindings.json', bindings)
remaining = {'obj_small_coordinate_projection_survives_text', 'ascii_stl_small_coordinate_projection_survives_text', 'binary_stl_rejects_faces_collapsed_by_quantization'}
for name, source in probes.items():
    binary = WORK / (name + '-candidate-v2')
    command = ['rustc', '--edition=2024', '--test', '-C', 'opt-level=3', '-C', 'debug-assertions=off', '-L', 'dependency=' + str(deps)]
    for crate, record in bindings.items():
        command += ['--extern', crate + '=' + record['retained']]
    assert run('compile-' + name, [*command, str(source), '-o', str(binary)]) == 0
    run(name + '-probe', [str(binary), '--test-threads=1', '--nocapture'], expected=101 if name == 'format' else 0)
    if name == 'format':
        text = (OUT / 'format-probe.stdout').read_text()
        assert '9 passed; 3 failed; 0 ignored;' in text
        assert set(re.findall(r'^test (\S+) \.\.\. FAILED$', text, re.MULTILINE)) == remaining
run('root-all', cargo('test', '--all-features', '--no-fail-fast', '--', '--test-threads=2'))
run('stl-only', cargo('test', '--no-default-features', '--features', 'stl-io', '--no-fail-fast', '--', '--test-threads=2'))
run('basic-shapes', cargo('run', '--no-default-features', '--features', 'curve,stl-io', '--example', 'basic_shapes'))
run('strict-clippy', cargo('clippy', '--all-features', '--all-targets', '--', '-D', 'warnings'))
run('stl-clippy', cargo('clippy', '--no-default-features', '--features', 'stl-io', '--all-targets', '--', '-D', 'warnings'))
run('basic-shapes-clippy', cargo('clippy', '--no-default-features', '--features', 'curve,stl-io', '--example', 'basic_shapes', '--', '-D', 'warnings'))
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
