"""Qualify one OBJ correction on new physical artifacts after native timing gates."""
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
E24 = ROOT / 'docs/evidence/2026-10-16-physical-build-requalification'
WORK = Path('/dev/shm/csgrs-obj-index-contracts')
PREP = WORK / 'preparation-v2'
OUT = WORK / 'candidate-validation-v1'
SOURCE, TARGET = WORK / 'candidate-source-v1', WORK / 'candidate-target-v1'
BASE_WORK = Path('/dev/shm/csgrs-physical-build-requalification')


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


print('Waiting for original native timing gates to finish before candidate compilation.', flush=True)
while not (E24 / 'validation-v2/result.json').exists():
    time.sleep(1)
assert json.loads((E24 / 'validation-v2/result.json').read_text())['complete']
preparation = json.loads((PREP / 'preparation.json').read_text())
assert sha(ROOT / 'src/io/obj.rs') == preparation['base_obj_sha256']
assert sha(PREP / 'obj.rs') == preparation['candidate_obj_sha256']
assert sha(PREP / 'obj_export_failures.rs') == preparation['test_sha256']
with gzip.open(E24 / 'validation-v1/source-hashes.json.gz', 'rt') as stream:
    original = json.load(stream)
assert all(sha(BASE_WORK / 'source-v1' / name) == digest for name, digest in original.items())
SOURCE.mkdir(exist_ok=False)
TARGET.mkdir(exist_ok=False)
OUT.mkdir(exist_ok=False)
for name, digest in original.items():
    destination = SOURCE / name
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(BASE_WORK / 'source-v1' / name, destination)
    assert sha(destination) == digest
shutil.copyfile(PREP / 'obj.rs', SOURCE / 'src/io/obj.rs')
assert 'tests/obj_export_failures.rs' not in original
shutil.copyfile(PREP / 'obj_export_failures.rs', SOURCE / 'tests/obj_export_failures.rs')
inputs = original | {'src/io/obj.rs': sha(SOURCE / 'src/io/obj.rs'),
                     'tests/obj_export_failures.rs': sha(SOURCE / 'tests/obj_export_failures.rs')}
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
protocol = {'runner_sha256': sha(__file__), 'preparation_sha256': sha(PREP / 'preparation.json'),
            'source': str(SOURCE), 'target': str(TARGET), 'target_initially_empty': not list(TARGET.iterdir()),
            'target_strategy': 'New empty physical tmpfs; no inherited build artifacts or overlays.',
            'source_hashes_sha256': sha(OUT / 'source-hashes.json.gz'),
            'format_probe': str(E24 / 'format_probe_v1.rs'), 'format_probe_sha256': sha(E24 / 'format_probe_v1.rs'),
            'cost_probe': str(E24 / 'obj_cost_probe_v1.rs'), 'cost_probe_sha256': sha(E24 / 'obj_cost_probe_v1.rs'),
            'native_validation_completed_before_start': True,
            'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
            'cargo': subprocess.check_output(['cargo', '-V'], text=True),
            'environment': {k: environment[k] for k in ['CARGO_HOME','CARGO_TARGET_DIR','CARGO_BUILD_JOBS','CARGO_NET_OFFLINE','CARGO_INCREMENTAL','TMPDIR']},
            'elapsed_limit_seconds_per_command': 1800, 'address_space_limit_bytes_per_process': 8 * 1024**3,
            'runtime_admitted': False, 'full_plan_complete': False}
dump(OUT / 'protocol.json', protocol)
results = []
retained = WORK / 'candidate-executables-v1'
retained.mkdir(exist_ok=False)


def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


def run(name, command, *, expected=0, docs=False):
    assert sha(__file__) == protocol['runner_sha256']
    assert all(sha(SOURCE / p) == h for p, h in inputs.items())
    print('START ' + name, flush=True)
    start = time.monotonic()
    env = environment | ({'RUSTDOCFLAGS': '-D warnings'} if docs else {})
    with (OUT / (name + '.stdout')).open('wb') as stdout, (OUT / (name + '.stderr')).open('wb') as stderr:
        result = subprocess.run(['/usr/bin/time','-v','-o',str(OUT / (name + '.resources')),
                                 'timeout','--signal=TERM','--kill-after=20s','1800s', *command],
                                cwd=SOURCE,env=env,stdout=stdout,stderr=stderr,
                                preexec_fn=limits,start_new_session=True)
    assert all(sha(SOURCE / p) == h for p, h in inputs.items())
    paths = {Path(p) for p in re.findall(r'Running[^\n]*\(([^)]+)\)', (OUT / (name + '.stderr')).read_text(errors='replace'))}
    if command[0] == 'rustc' and result.returncode == 0:
        paths.add(Path(command[command.index('-o') + 1]))
    if Path(command[0]).is_absolute() and Path(command[0]).is_relative_to(WORK):
        paths.add(Path(command[0]))
    binaries = {}
    for path in paths:
        path = path if path.is_absolute() else SOURCE / path
        assert path.is_file()
        digest = sha(path)
        copy = retained / digest / path.name
        copy.parent.mkdir(exist_ok=True)
        if not copy.exists():
            shutil.copy2(path, copy)
        assert sha(copy) == digest
        binaries[str(path)] = {'sha256': digest, 'retained': str(copy), 'bytes': path.stat().st_size}
    row = {'name': name, 'command': command, 'binaries': binaries, 'exit_code': result.returncode,
           'expected_exit_code': expected, 'elapsed_seconds': time.monotonic()-start,
           'sources_unchanged': True, 'rustdocflags': env.get('RUSTDOCFLAGS'),
           **{suffix+'_sha256': sha(OUT / (name+'.'+suffix)) for suffix in ['stdout','stderr','resources']}}
    results.append(row)
    dump(OUT / 'commands.json', results)
    print(json.dumps({k:row[k] for k in ['name','exit_code','expected_exit_code','elapsed_seconds']}),flush=True)
    return result.returncode


def cargo(action, *args):
    return ['cargo', action, '--manifest-path', str(SOURCE / 'Cargo.toml'), '--locked', '--release', '-p', 'csgrs', *args]


run('format', ['cargo','fmt','--manifest-path',str(SOURCE / 'Cargo.toml'),'--all','--','--check'])
assert run('minimal-regression', cargo('test','--no-default-features','--features','obj-io,stl-io','--test','obj_export_failures','--','--test-threads=1')) == 0

# Compile both fixed probes with explicit physical library paths. The old target is only read.
bindings = {}
for variant, target in [('baseline', BASE_WORK / 'format-target-v1'), ('candidate', TARGET)]:
    deps = target / 'release/deps'
    chosen = {}
    for name in ['csgrs','hyperlattice','hypermesh']:
        paths = sorted(deps.glob('lib'+name+'*.rlib'))
        assert len(paths) == 1, (variant,name,paths)
        retained_library = WORK / 'probe-libraries-v1' / variant / paths[0].name
        retained_library.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(paths[0], retained_library)
        assert sha(paths[0]) == sha(retained_library)
        chosen[name] = {'path': str(paths[0]), 'sha256': sha(paths[0]), 'retained': str(retained_library)}
    bindings[variant] = chosen
    command = ['rustc','--edition=2024','-C','opt-level=3','-C','debug-assertions=off',
               '-L','dependency='+str(deps)]
    for name, record in chosen.items():
        command += ['--extern', name+'='+record['path']]
    binary = WORK / ('obj-cost-'+variant+'-v1')
    assert run('compile-cost-'+variant, [*command, str(E24 / 'obj_cost_probe_v1.rs'), '-o', str(binary)]) == 0
    if variant == 'candidate':
        probe = WORK / 'format-boundary-candidate-v1'
        assert run('compile-format-probe', [*command,'--test',str(E24 / 'format_probe_v1.rs'),'-o',str(probe)]) == 0
        run('format-probe', [str(probe),'--test-threads=1','--nocapture'], expected=101)
        stdout = (OUT / 'format-probe.stdout').read_text()
        expected_failures = {'obj_small_coordinate_projection_survives_text',
            'ascii_stl_small_coordinate_projection_survives_text',
            'binary_stl_small_representable_cube_composes',
            'ascii_stl_small_representable_cube_composes',
            'binary_stl_rejects_faces_collapsed_by_quantization'}
        observed = set(re.findall(r'^test (\S+) \.\.\. FAILED$',stdout,re.MULTILINE))
        assert observed == expected_failures and '7 passed; 5 failed; 0 ignored;' in stdout
    assert all(sha(r['path']) == r['sha256'] for r in chosen.values())
dump(OUT / 'probe-bindings.json', bindings)
run('root-all', cargo('test','--all-features','--no-fail-fast','--','--test-threads=2'))
run('obj-only', cargo('test','--no-default-features','--features','obj-io','--no-fail-fast','--','--test-threads=2'))
run('strict-clippy', cargo('clippy','--all-features','--all-targets','--','-D','warnings'))
run('obj-clippy', cargo('clippy','--no-default-features','--features','obj-io','--all-targets','--','-D','warnings'))
run('docs', cargo('doc','--all-features','--no-deps'),docs=True)
assert all(sha(SOURCE / p) == h for p,h in inputs.items())
assert sha(E24 / 'format_probe_v1.rs') == protocol['format_probe_sha256']
assert sha(E24 / 'obj_cost_probe_v1.rs') == protocol['cost_probe_sha256']
record = {'complete': True, 'commands': len(results),
          'unexpected_exits': [r['name'] for r in results if r['exit_code'] != r['expected_exit_code']],
          'known_unfixed_format_failures': sorted(expected_failures),
          'probe_binaries_sha256': {str(p):sha(p) for p in [WORK/'obj-cost-baseline-v1',WORK/'obj-cost-candidate-v1',WORK/'format-boundary-candidate-v1']},
          'sources_unchanged': True, 'main_sources_modified': False,
          'runtime_admitted': False,'full_plan_complete':False}
dump(OUT / 'result.json', record)
print(json.dumps(record),flush=True)
raise SystemExit(101 if record['unexpected_exits'] else 0)
