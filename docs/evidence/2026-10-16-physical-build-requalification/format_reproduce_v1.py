"""Run public format acceptance probes against unchanged sources in another empty physical target."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import resource
import shutil
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
WORK = Path('/dev/shm/csgrs-physical-build-requalification')
SOURCE, TARGET = WORK / 'format-source-v1', WORK / 'format-target-v1'
OUT = E / 'format-reproduction-v1'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


# Root modeling qualification is complete. The continuation runner waits for
# this process before launching either original runtime gate.
prior = json.loads((E / 'validation-v1/commands.json').read_text())
assert all(any(row['name'] == name and row['exit_code'] == 0 for row in prior)
           for name in ['root-all', 'root-minimal'])
with gzip.open(E / 'validation-v1/source-hashes.json.gz', 'rt') as stream:
    inputs = json.load(stream)
assert all(sha(ROOT / p) == h and sha(WORK / 'source-v1' / p) == h for p, h in inputs.items())
SOURCE.mkdir(exist_ok=False)
TARGET.mkdir(exist_ok=False)
OUT.mkdir(exist_ok=False)
for name, digest in inputs.items():
    destination = SOURCE / name
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(WORK / 'source-v1' / name, destination)
    assert sha(destination) == digest
probe_name = 'tests/format_boundary_probe.rs'
assert probe_name not in inputs
shutil.copyfile(E / 'format_probe_v1.rs', SOURCE / probe_name)
inputs[probe_name] = sha(SOURCE / probe_name)
cost_name = 'examples/obj_export_cost.rs'
assert cost_name not in inputs
shutil.copyfile(E / 'obj_cost_probe_v1.rs', SOURCE / cost_name)
inputs[cost_name] = sha(SOURCE / cost_name)
with gzip.open(OUT / 'source-hashes.json.gz', 'wt') as stream:
    json.dump(inputs, stream, sort_keys=True)
command = ['cargo', 'test', '--manifest-path', str(SOURCE / 'Cargo.toml'), '--locked', '--release',
           '-p', 'csgrs', '--no-default-features', '--features', 'obj-io,stl-io',
           '--test', 'format_boundary_probe', '--', '--test-threads=1', '--nocapture']
environment = os.environ.copy()
for key in ['RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS', 'RUSTC_WRAPPER', 'RUSTC_WORKSPACE_WRAPPER', 'RUSTDOCFLAGS']:
    environment.pop(key, None)
environment.update({'CARGO_HOME': '/home/tim/.cargo', 'CARGO_TARGET_DIR': str(TARGET),
                    'CARGO_BUILD_JOBS': '2', 'CARGO_NET_OFFLINE': 'true', 'CARGO_INCREMENTAL': '0',
                    'CARGO_TERM_COLOR': 'never', 'RUST_BACKTRACE': '0'})
protocol = {'runner_sha256': sha(Path(__file__)), 'probe_sha256': sha(E / 'format_probe_v1.rs'),
            'contract_sha256': sha(E / 'format-contract-v1.md'), 'source': str(SOURCE), 'target': str(TARGET),
            'target_initially_empty': not list(TARGET.iterdir()), 'target_strategy': 'Direct physical tmpfs; no inherited build artifacts.',
            'command': command, 'source_hashes_sha256': sha(OUT / 'source-hashes.json.gz'),
            'new_test': probe_name, 'new_cost_example': cost_name, 'cost_probe_sha256': sha(E / 'obj_cost_probe_v1.rs'), 'production_source_changes': 0,
            'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
            'cargo': subprocess.check_output(['cargo', '-V'], text=True),
            'environment': {key: environment[key] for key in ['CARGO_HOME', 'CARGO_TARGET_DIR', 'CARGO_BUILD_JOBS', 'CARGO_NET_OFFLINE', 'CARGO_INCREMENTAL']},
            'elapsed_limit_seconds': 1800, 'address_space_limit_bytes_per_process': 8 * 1024**3,
            'runtime_admitted': False, 'full_plan_complete': False}
dump(OUT / 'protocol.json', protocol)


def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


start = time.monotonic()
with (OUT / 'probe.stdout').open('wb') as stdout, (OUT / 'probe.stderr').open('wb') as stderr:
    result = subprocess.run(['/usr/bin/time', '-v', '-o', str(OUT / 'probe.resources'),
                             'timeout', '--signal=TERM', '--kill-after=20s', '1800s', *command],
                            cwd=SOURCE, env=environment, stdout=stdout, stderr=stderr,
                            preexec_fn=limits, start_new_session=True)
assert all(sha(SOURCE / p) == h for p, h in inputs.items())
assert sha(Path(__file__)) == protocol['runner_sha256']
assert sha(E / 'format_probe_v1.rs') == protocol['probe_sha256']
binaries = {str(p): sha(p) for p in (TARGET / 'release/deps').glob('format_boundary_probe-*')
            if p.is_file() and os.access(p, os.X_OK)}
cost_command = ['cargo', 'build', '--manifest-path', str(SOURCE / 'Cargo.toml'), '--locked', '--release',
                '-p', 'csgrs', '--no-default-features', '--features', 'obj-io,stl-io',
                '--example', 'obj_export_cost']
with (OUT / 'cost-build.stdout').open('wb') as stdout, (OUT / 'cost-build.stderr').open('wb') as stderr:
    cost_result = subprocess.run(['/usr/bin/time', '-v', '-o', str(OUT / 'cost-build.resources'),
                                 'timeout', '--signal=TERM', '--kill-after=20s', '1800s', *cost_command],
                                cwd=SOURCE, env=environment, stdout=stdout, stderr=stderr,
                                preexec_fn=limits, start_new_session=True)
cost_binary = TARGET / 'release/examples/obj_export_cost'
assert all(sha(SOURCE / p) == h for p, h in inputs.items())
assert sha(E / 'obj_cost_probe_v1.rs') == protocol['cost_probe_sha256']
record = {'complete': True, 'exit_code': result.returncode, 'elapsed_seconds': time.monotonic() - start,
          'source_inputs_unchanged': True, 'binaries_sha256': binaries,
          **{suffix + '_sha256': sha(OUT / ('probe.' + suffix)) for suffix in ['stdout', 'stderr', 'resources']},
          'production_source_changes': 0, 'runtime_admitted': False, 'full_plan_complete': False,
          'cost_build_command': cost_command, 'cost_build_exit_code': cost_result.returncode,
          'cost_binary': str(cost_binary) if cost_result.returncode == 0 else None,
          'cost_binary_sha256': sha(cost_binary) if cost_result.returncode == 0 else None,
          'cost_measurements_performed': False,
          'cost_build_logs_sha256': {suffix: sha(OUT / ('cost-build.' + suffix)) for suffix in ['stdout', 'stderr', 'resources']}}
dump(OUT / 'result.json', record)
print(json.dumps(record), flush=True)
raise SystemExit(result.returncode or cost_result.returncode)
