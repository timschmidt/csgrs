"""Select the CLI required by Cargo.lock without changing qualified sources."""
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
TOOLS = Path('/tmp/csgrs-factored-product-proofs/wasm-tools-v1/install')
TARGET = Path('/tmp/csgrs-factored-product-proofs/wasm-bindings-v1')
assert json.loads((E / 'wasm-tools-v1/result.json').read_text())['passed']
assert subprocess.check_output([str(TOOLS / 'bin/wasm-bindgen'), '--version'],
                               text=True).strip() == 'wasm-bindgen 0.2.126'
commands = [
    ['prepare-target', ['mkdir', str(TARGET)]],
    ['wasm', ['env', 'WASM_BINDGEN=' + str(TOOLS / 'bin/wasm-bindgen'),
              'bash', 'scripts/test-wasm-bindings.sh']],
]
pins = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in (TOOLS / 'bin').iterdir() if path.is_file()}
contract = E / 'boundary-certainty-contract-v1.md'
pins[str(contract)] = hashlib.sha256(contract.read_bytes()).hexdigest()
for suffix, value in [('.json', commands), ('-seed.json', None),
                      ('-target.json', str(TARGET)), ('-preserve.json', pins)]:
    path = E / ('wasm-bindings-v1' + suffix)
    assert not path.exists()
    path.write_text(json.dumps(value, indent=2) + '\n')
sources = list(Path('/home/tim/.cargo/registry/src').glob('*/wasm-bindgen-cli-0.2.126'))
assert len(sources) == 1
with tarfile.open(E / 'wasm-cli-source-v1.tar.gz', 'x:gz') as archive:
    archive.add(sources[0], arcname='wasm-bindgen-cli-0.2.126')
metadata = {
    'required_cli': 'wasm-bindgen 0.2.126',
    'cli_source_archive_sha256': hashlib.sha256((E / 'wasm-cli-source-v1.tar.gz').read_bytes()).hexdigest(),
    'cli_binaries_sha256': pins,
    'node_version': subprocess.check_output(['node', '--version'], text=True).strip(),
    'typescript_version': subprocess.check_output([str(ROOT / 'tests/wasm/node_modules/.bin/tsc'),
                                                  '--version'], text=True).strip(),
    'failed_version_guard': 'qualification-v5/wasm.stderr',
    'source_changes_after_rust_qualification': False,
    'scope': 'Binding-only rerun after selecting the CLI version required by the unchanged lockfile. The original failed phase is retained; Rust checks are reused on identical source hashes.',
}
path = E / 'wasm-tools-metadata-v1.json'
assert not path.exists()
path.write_text(json.dumps(metadata, indent=2) + '\n')
print('Prepared real WASM/Node/TypeScript execution with locked CLI')
