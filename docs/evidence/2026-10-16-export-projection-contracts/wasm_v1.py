"""Fresh-source real WASM consumers for the exact tested native candidate."""
import difflib
import gzip
import hashlib
import json
import os
from pathlib import Path
import resource
import shutil
import subprocess
import time

ROOT = Path('/home/tim/Documents/GitHub/workspace/csgrs')
E = ROOT / 'docs/evidence/2026-10-16-export-projection-contracts'
WORK = ROOT / 'target/properties/export-projection-contracts'
NATIVE = WORK / 'candidate-source-v1'
SOURCE, TARGET = WORK / 'wasm-source-v1', WORK / 'wasm-target-v1'
OUT, SCRATCH = WORK / 'wasm-validation-v1', WORK / 'wasm-scratch-v1'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


with gzip.open(WORK / 'candidate-validation-v1/source-hashes.json.gz', 'rt') as stream:
    original = json.load(stream)
assert all(sha(NATIVE / name) == digest for name, digest in original.items())
for directory in [SOURCE, TARGET, OUT, SCRATCH]:
    directory.mkdir(exist_ok=False)
for name, digest in original.items():
    dest = SOURCE / name
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(NATIVE / name, dest)
    assert sha(dest) == digest
script_name = 'scripts/test-wasm-bindings.sh'
script = SOURCE / script_name
before = script.read_text()
marker = 'node tests/wasm/geometry_certainty.cjs "${wasm_bindings_dir}/csgrs.js"\n'
assert before.count(marker) == 1
script.write_text(before.replace(marker, marker + 'node tests/wasm/export_projection.cjs "${wasm_bindings_dir}/csgrs.js"\n'))
test_name = 'tests/wasm/export_projection.cjs'
shutil.copyfile(E / 'export_projection_v2.cjs', SOURCE / test_name)
inputs = original | {name: sha(SOURCE / name) for name in [script_name, test_name]}
assert all(inputs[name] == digest for name, digest in original.items() if name != script_name)
with gzip.open(OUT / 'source-hashes.json.gz', 'wt') as stream:
    json.dump(inputs, stream, sort_keys=True)
(OUT / 'harness.patch').write_text(''.join(difflib.unified_diff(before.splitlines(keepends=True), script.read_text().splitlines(keepends=True), fromfile='a/' + script_name, tofile='b/' + script_name)))
tool_record = json.loads((WORK / 'wasm-tools-v1/result.json').read_text())
assert tool_record['exit_code'] == 0 and tool_record['version'] == 'wasm-bindgen 0.2.126'
bindgen = Path(tool_record['binary'])
assert sha(bindgen) == tool_record['binary_sha256']
env = os.environ.copy()
for key in ['RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS', 'RUSTC_WRAPPER', 'RUSTC_WORKSPACE_WRAPPER', 'RUSTDOCFLAGS']:
    env.pop(key, None)
env.update({'CARGO_HOME': '/home/tim/.cargo', 'CARGO_TARGET_DIR': str(TARGET),
            'CARGO_BUILD_JOBS': '2', 'CARGO_NET_OFFLINE': 'true', 'CARGO_INCREMENTAL': '0',
            'TMPDIR': str(SCRATCH), 'CARGO_TERM_COLOR': 'never', 'RUST_BACKTRACE': '0',
            'WASM_BINDGEN': str(bindgen)})
protocol = {'runner_sha256': sha(__file__), 'source': str(SOURCE), 'target': str(TARGET),
            'native_input_hashes_sha256': sha(WORK / 'candidate-validation-v1/source-hashes.json.gz'),
            'source_hashes_sha256': sha(OUT / 'source-hashes.json.gz'),
            'native_rust_cargo_inputs_unchanged': True, 'changed_relative_to_native': [script_name, test_name],
            'target_initially_empty': not list(TARGET.iterdir()),
            'mount': subprocess.check_output(['findmnt','-T',str(TARGET),'-n','-o','TARGET,FSTYPE,OPTIONS'], text=True),
            'rustc': subprocess.check_output(['rustc','-vV'], text=True),
            'node': subprocess.check_output(['node','--version'], text=True).strip(),
            'npm': subprocess.check_output(['npm','--version'], text=True).strip(),
            'wasm_bindgen': tool_record,
            'elapsed_limit_seconds_per_command': 1800,
            'address_space_limit_bytes': 64 * 1024**3,
            'containment_note': 'External elapsed and address-space limits; V8 needs large virtual address reservations. This is not an internal heap, work or cancellation contract.',
            'environment': {key: env[key] for key in ['CARGO_HOME','CARGO_TARGET_DIR','CARGO_BUILD_JOBS','CARGO_NET_OFFLINE','CARGO_INCREMENTAL','TMPDIR','WASM_BINDGEN']}}
assert 'ext4' in protocol['mount'] and 'fuse' not in protocol['mount']
dump(OUT / 'protocol.json', protocol)
rows = []


def limits():
    resource.setrlimit(resource.RLIMIT_AS, (64 * 1024**3, 64 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


def run(name, command):
    assert sha(__file__) == protocol['runner_sha256']
    assert all(sha(SOURCE / path) == digest for path, digest in inputs.items())
    assert sha(bindgen) == tool_record['binary_sha256']
    print('START ' + name, flush=True)
    start = time.monotonic()
    with (OUT / (name + '.stdout')).open('wb') as stdout, (OUT / (name + '.stderr')).open('wb') as stderr:
        result = subprocess.run(['/usr/bin/time','-v','-o',str(OUT / (name + '.resources')),
                                 'timeout','--signal=TERM','--kill-after=20s','1800s',*command],
                                cwd=SOURCE, env=env, stdout=stdout, stderr=stderr,
                                preexec_fn=limits, start_new_session=True)
    assert all(sha(SOURCE / path) == digest for path, digest in inputs.items())
    row = {'name': name, 'command': command, 'exit_code': result.returncode,
           'elapsed_seconds': time.monotonic() - start, 'sources_unchanged': True,
           **{suffix + '_sha256': sha(OUT / (name + '.' + suffix)) for suffix in ['stdout','stderr','resources']}}
    rows.append(row)
    dump(OUT / 'commands.json', rows)
    print(json.dumps(row), flush=True)
    return result.returncode


assert run('node-syntax', ['node','--check',str(SOURCE / test_name)]) == 0
assert run('install-locked-typescript', ['npm','ci','--ignore-scripts','--no-audit','--no-fund',
    '--prefix',str(SOURCE / 'tests/wasm'),'--cache',str(WORK / 'npm-cache-v1')]) == 0
package = json.loads((SOURCE / 'tests/wasm/node_modules/typescript/package.json').read_text())
assert package['version'] == '5.7.3'
assert run('real-js-and-typescript-consumers', ['bash',str(script)]) == 0
assert run('wasm-clippy', ['cargo','clippy','--locked','--profile','wasm','--target','wasm32-unknown-unknown',
    '--no-default-features','--features','wasm','--lib','--','-D','warnings']) == 0
artifacts = [TARGET / 'wasm32-unknown-unknown/wasm/csgrs.wasm',
             *sorted((TARGET / 'wasm-bindings-node').glob('*')),
             SOURCE / 'tests/wasm/node_modules/typescript/lib/tsc.js', bindgen]
records = {str(path): {'sha256': sha(path), 'bytes': path.stat().st_size} for path in artifacts if path.is_file()}
assert all(sha(SOURCE / path) == digest for path, digest in inputs.items())
dump(OUT / 'result.json', {'all_commands_passed': all(row['exit_code'] == 0 for row in rows),
    'command_count': len(rows), 'sources_unchanged': True, 'typescript': package['version'],
    'artifacts': records, 'existing_certainty_consumer_preserved': True,
    'typed_io_failure_receipt_not_claimed': True, 'full_distribution_validation_complete': False})
print('WASM_COMPLETE', flush=True)
