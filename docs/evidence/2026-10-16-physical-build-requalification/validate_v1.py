"""Rebuild current sources in a new physical target and retain all command outcomes."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import resource
import shutil
import subprocess
import sys
import time
import zipfile

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
WORK = Path('/dev/shm/csgrs-physical-build-requalification')
SOURCE, TARGET = WORK / 'source-v1', WORK / 'target-v1'
OUT = E / 'validation-v1'
BASE = '3b69af324a9642d1f71b9cc38de58b59a236f740'
PRIOR = E.parent / '2026-10-16-native-metadata-isolation'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


def gz(path):
    with gzip.open(path, 'rt') as stream:
        return json.load(stream)


assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == BASE
storage = json.loads((E / 'storage-v1.json').read_text())
assert storage['all_eight_identity_reads_passed'] and storage['executable_exit_code'] == 0
OUT.mkdir(exist_ok=False)
SOURCE.mkdir(exist_ok=False)
TARGET.mkdir(exist_ok=False)
assert not list(TARGET.iterdir()) and not TARGET.is_mount()
original = gz(PRIOR / 'source-hashes-v1.json.gz')['original']
inputs = {name: sha(ROOT / name) for name in original}
changed = [name for name in original if original[name] != inputs[name]]
assert set(changed) <= {'docs/properties-status.json'}, changed
pending = json.loads((PRIOR / 'checkpoint-v2.json').read_text())['pending_native_sha256']
assert all(inputs[p] == h for p, h in pending.items())
for name, digest in inputs.items():
    destination = SOURCE / name
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(ROOT / name, destination)
    assert sha(destination) == digest
    assert (destination.stat().st_dev, destination.stat().st_ino) != ((ROOT / name).stat().st_dev, (ROOT / name).stat().st_ino)
assert all(sha(ROOT / name) == digest for name, digest in inputs.items())
with gzip.open(OUT / 'source-hashes.json.gz', 'wt') as stream:
    json.dump(inputs, stream, sort_keys=True)
fixtures = []
for name, entry, expected, size in [
    ('CPArti FPGA dev board.zip', 'CPArti FPGA dev board.kicad_pcb', 'f57195393ff7a938af0a9651781ba13536d248cbb43f13a41739e2a97cf99e35', 8177693),
    ('HVP109A.zip', 'HVP109A.kicad_pcb', '986b7e26b60c83c97a851c403fb90a927c9018d30ad6b94c7bba5925aa99e28a', 5611517),
]:
    path = SOURCE / 'crates/hyperdrc/docs' / name
    assert sha(path) == expected
    with zipfile.ZipFile(path) as archive:
        assert archive.testzip() is None
        data = archive.read(entry)
        assert len(data) == size
        fixtures.append({'archive': str(path), 'sha256': expected, 'entry': entry,
                         'entry_sha256': hashlib.sha256(data).hexdigest(), 'entry_bytes': size})
old_commands = json.loads((PRIOR / 'candidate-v1.json').read_text())
commands = []
for name, old in old_commands:
    if name in ['overlay', 'fixtures', 'metadata-regressions', 'verify-lower']:
        continue
    command = [str(SOURCE / 'Cargo.toml') if arg.endswith('/candidate/source/Cargo.toml') else arg for arg in old]
    if name in ['strict-clippy', 'docs']:
        position = command.index('-p')
        command[position:position] = ['-p', 'csgrs']
    commands.append((name, command))
# Root composition checks run first, then the native consumers and their original gates.
order = ['format', 'root-all', 'root-minimal', 'curve-default', 'curve-all', 'strict-clippy',
         'native-other-tests', 'docs', 'full-board', 'copper-neck', 'gerber', 'native-cli']
commands.sort(key=lambda row: order.index(row[0]))
assert [row[0] for row in commands] == order
base_env = os.environ.copy()
for key in ['RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS', 'RUSTC_WRAPPER', 'RUSTC_WORKSPACE_WRAPPER', 'RUSTDOCFLAGS']:
    base_env.pop(key, None)
base_env.update({'CARGO_HOME': '/home/tim/.cargo', 'CARGO_TARGET_DIR': str(TARGET),
                 'CARGO_BUILD_JOBS': '2', 'CARGO_NET_OFFLINE': 'true', 'CARGO_INCREMENTAL': '0',
                 'RUST_BACKTRACE': '0', 'CARGO_TERM_COLOR': 'never'})
protocol = {'base_commit': BASE, 'runner_sha256': sha(Path(__file__)),
            'source': str(SOURCE), 'target': str(TARGET), 'target_initially_empty': True,
            'target_strategy': 'Physical tmpfs, no overlay and no build artifact seed.',
            'source_hashes_sha256': sha(OUT / 'source-hashes.json.gz'), 'source_files': len(inputs),
            'pending_native_sha256': pending, 'commands': commands, 'fixtures': fixtures,
            'environment': {k: base_env[k] for k in ['CARGO_HOME', 'CARGO_TARGET_DIR', 'CARGO_BUILD_JOBS', 'CARGO_NET_OFFLINE', 'CARGO_INCREMENTAL']},
            'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
            'cargo': subprocess.check_output(['cargo', '-V'], text=True),
            'hardware': subprocess.check_output(['lscpu'], text=True),
            'mount': subprocess.check_output(['findmnt', '-T', str(TARGET), '-o', 'TARGET,FSTYPE,OPTIONS'], text=True),
            'elapsed_limit_seconds_per_command': 1800, 'address_space_limit_bytes_per_process': 8 * 1024**3,
            'containment_is_not_an_internal_resource_contract': True,
            'metadata_order_candidate_included': False, 'runtime_admitted': False, 'full_plan_complete': False}
dump(OUT / 'protocol.json', protocol)
results = []
retained = WORK / 'executables-v1'
retained.mkdir()


def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


for name, command in commands:
    assert sha(Path(__file__)) == protocol['runner_sha256']
    print('START ' + name, flush=True)
    start = time.monotonic()
    stdout, stderr, resources = [OUT / (name + '.' + suffix) for suffix in ['stdout', 'stderr', 'resources']]
    with stdout.open('wb') as out, stderr.open('wb') as err:
        result = subprocess.run(['/usr/bin/time', '-v', '-o', str(resources),
                                 'timeout', '--signal=TERM', '--kill-after=20s', '1800s', *command],
                                cwd=SOURCE, env=base_env, stdout=out, stderr=err,
                                preexec_fn=limits, start_new_session=True)
    unchanged = all(sha(SOURCE / p) == h and sha(ROOT / p) == h for p, h in inputs.items())
    assert unchanged
    text = stderr.read_text(errors='replace')
    binary_paths = {Path(path) for path in re.findall(r'Running[^\n]*\(([^)]+)\)', text)}
    if name == 'native-cli' and result.returncode == 0:
        for line in stdout.read_text().splitlines():
            try:
                value = json.loads(line)
            except json.JSONDecodeError:
                continue
            if value.get('reason') == 'compiler-artifact' and value.get('executable'):
                binary_paths.add(Path(value['executable']))
    binaries = {}
    for path in binary_paths:
        path = path if path.is_absolute() else SOURCE / path
        assert path.is_relative_to(TARGET) and path.is_file(), path
        digest = sha(path)
        copy = retained / digest / path.name
        copy.parent.mkdir(exist_ok=True)
        if not copy.exists():
            shutil.copy2(path, copy)
        assert sha(copy) == digest
        binaries[str(path)] = {'sha256': digest, 'retained': str(copy), 'bytes': path.stat().st_size}
    row = {'name': name, 'command': command, 'exit_code': result.returncode,
           'elapsed_seconds': time.monotonic() - start, 'sources_unchanged': unchanged,
           'binaries': binaries, **{suffix + '_sha256': sha(OUT / (name + '.' + suffix)) for suffix in ['stdout', 'stderr', 'resources']}}
    results.append(row)
    dump(OUT / 'commands.json', results)
    print(json.dumps({'name': name, 'exit_code': row['exit_code'], 'seconds': row['elapsed_seconds'], 'retained_executables': len(binaries)}), flush=True)
record = {'complete': True, 'commands': len(results), 'passed': sum(r['exit_code'] == 0 for r in results),
          'failed': [r['name'] for r in results if r['exit_code'] != 0], 'sources_unchanged': True,
          'target_strategy': protocol['target_strategy'], 'runtime_admitted': False, 'full_plan_complete': False}
dump(OUT / 'result.json', record)
print(json.dumps(record), flush=True)
if record['failed']:
    sys.exit(101)
