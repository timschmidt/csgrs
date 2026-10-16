"""Build the repository-pinned binding generator in a new isolated tool root."""
import hashlib
import json
import os
from pathlib import Path
import resource
import subprocess

ROOT = Path('/home/tim/Documents/GitHub/workspace/csgrs')
E = ROOT / 'docs/evidence/2026-10-16-export-projection-contracts'
WORK = ROOT / 'target/properties/export-projection-contracts'
OUT = WORK / 'wasm-tools-v1'
TARGET = WORK / 'wasm-tools-target-v1'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


for path in [OUT, TARGET]:
    path.mkdir(exist_ok=False)
scratch = OUT / 'scratch'
scratch.mkdir()
environment = os.environ.copy()
for name in ['RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS', 'RUSTC_WRAPPER', 'RUSTC_WORKSPACE_WRAPPER']:
    environment.pop(name, None)
environment.update({'CARGO_HOME': '/home/tim/.cargo', 'CARGO_NET_OFFLINE': 'true',
                    'CARGO_BUILD_JOBS': '2', 'CARGO_INCREMENTAL': '0', 'TMPDIR': str(scratch),
                    'CARGO_TERM_COLOR': 'never'})
command = ['cargo', 'install', '--locked', '--offline', '--version', '0.2.126',
           'wasm-bindgen-cli', '--root', str(OUT / 'install'), '--target-dir', str(TARGET), '--jobs', '2']
protocol = {'runner_sha256': sha(__file__), 'command': command,
            'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
            'source_archive_sha256': sha('/home/tim/.cargo/registry/cache/index.crates.io-1949cf8c6b5b557f/wasm-bindgen-cli-0.2.126.crate'),
            'target_initially_empty': not list(TARGET.iterdir()),
            'elapsed_limit_seconds': 1800, 'address_space_limit_bytes_per_process': 8 * 1024**3}
(OUT / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
with (OUT / 'stdout').open('wb') as stdout, (OUT / 'stderr').open('wb') as stderr:
    result = subprocess.run(['/usr/bin/time', '-v', '-o', str(OUT / 'resources'),
                             'timeout', '--signal=TERM', '--kill-after=20s', '1800s', *command],
                            cwd=ROOT, env=environment, stdout=stdout, stderr=stderr,
                            preexec_fn=limits, start_new_session=True)
record = {'exit_code': result.returncode, 'command': command,
          **{name + '_sha256': sha(OUT / name) for name in ['stdout', 'stderr', 'resources']}}
if result.returncode == 0:
    binary = OUT / 'install/bin/wasm-bindgen'
    version = subprocess.check_output([str(binary), '--version'], text=True).strip()
    assert version == 'wasm-bindgen 0.2.126'
    record |= {'binary': str(binary), 'binary_sha256': sha(binary), 'version': version}
(OUT / 'result.json').write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps(record), flush=True)
raise SystemExit(result.returncode)
