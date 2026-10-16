"""Apply only validated exports and their real public consumer regressions."""
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
WORK = ROOT / 'target/properties/export-projection-contracts'
PARENT = E.parent / '2026-10-16-stl-native-validation'
BASE = 'c9410688b0c7854fba6c9a90f34cae5b3382318a'
NATIVE_PATHS = ['src/io/mod.rs','src/io/obj.rs','src/io/stl.rs','tests/export_projection_contracts.rs']
JS_PATHS = ['scripts/test-wasm-bindings.sh','tests/wasm/export_projection.cjs']


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def gz(path):
    with gzip.open(path, 'rt') as stream:
        return json.load(stream)


assert not (E / 'application-v1.json').exists()
assert subprocess.check_output(['git','rev-parse','HEAD'], cwd=ROOT, text=True).strip() == BASE
subprocess.run(['git','diff','--cached','--quiet'], cwd=ROOT, check=True)
qualification = read(WORK / 'candidate-validation-v2/result.json')
wasm = read(WORK / 'wasm-validation-v1/result.json')
cost = read(WORK / 'cost-v1/result.json')
fractional = read(WORK / 'fractional-cost-v1/result.json')
assert fractional['complete'] and fractional['runtime_processes'] == 5 and fractional['sample_rows'] == 40
assert fractional['library_inputs_unchanged'] and fractional['all_repeated_complete_outputs_equal']
assert qualification['complete'] and not qualification['unexpected_exits']
assert wasm['all_commands_passed'] and wasm['command_count'] == 4
assert cost['complete'] and cost['runtime_processes'] == 90 and cost['sample_rows'] == 180
assert cost['complete_input_and_output_comparisons_equal'] and cost['library_inputs_unchanged']
parent = read(PARENT / 'checkpoint-v3.json')
assert all(sha(ROOT / path) == digest for path, digest in parent['pending_native_sha256'].items())
baseline = gz(WORK / 'baseline-v1/source-hashes.json.gz')
assert all(sha(ROOT / path) == digest for path, digest in baseline.items())
assert not (ROOT / NATIVE_PATHS[-1]).exists() and not (ROOT / JS_PATHS[-1]).exists()
native_inputs = gz(WORK / 'candidate-validation-v2/source-hashes.json.gz')
wasm_inputs = gz(WORK / 'wasm-validation-v1/source-hashes.json.gz')
assert all(sha(WORK / 'candidate-source-v2' / path) == digest for path, digest in native_inputs.items())
assert all(sha(WORK / 'wasm-source-v1' / path) == digest for path, digest in wasm_inputs.items())
# WASM builds the library, not native integration tests. Its production Rust,
# manifests and every dependency input are byte-identical to native v2.
partition = set(JS_PATHS) | {'tests/export_projection_contracts.rs'}
assert all(wasm_inputs[path] == digest for path, digest in native_inputs.items() if path not in partition)
assert set(wasm_inputs) - set(native_inputs) == {JS_PATHS[-1]}
assert sha(WORK / 'wasm-source-v1/tests/export_projection_contracts.rs') == sha(E / 'export_projection_contracts_v1.rs')
assert sha(WORK / 'candidate-source-v2/tests/export_projection_contracts.rs') == sha(E / 'export_projection_contracts_v2.rs')
inputs = native_inputs | {name: wasm_inputs[name] for name in JS_PATHS}
with gzip.open(E / 'final-input-hashes-v1.json.gz', 'wt') as stream:
    json.dump(inputs, stream, sort_keys=True)
assert sha(ROOT / 'docs/properties-status.json') == parent['ledger_after_sha256']
retained = {}
for directory in ['baseline-v1','candidate-validation-v1','candidate-validation-v2',
                  'wasm-tools-v1','wasm-validation-v1','cost-v1','fractional-cost-v1']:
    destination = E / directory
    assert not destination.exists()
    for source in (WORK / directory).rglob('*'):
        if not source.is_file():
            continue
        if source.suffix not in {'.json','.gz','.stdout','.stderr','.resources','.patch','.rows','.data','.txt'} and source.name not in {'stdout','stderr','resources'}:
            continue
        output = destination / source.relative_to(WORK / directory)
        output.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, output)
        assert sha(source) == sha(output)
        retained[str(output.relative_to(E))] = {'original': str(source), 'sha256': sha(source)}
# Preserve the complete unchanged full-resolution example output, not only its checksum.
example_files = sorted((WORK / 'candidate-source-v2/stl/examples/basic_shapes').glob('*.stl'))
assert {path.stem for path in example_files} == {'cube','sphere','cylinder','ellipsoid','torus'}
for source in example_files:
    assert source.read_bytes() == (PARENT / 'basic-shapes-output-v2' / source.name).read_bytes()
    output = E / 'basic-shapes-output-v2' / source.name
    output.parent.mkdir(exist_ok=True)
    shutil.copy2(source, output)
    retained[str(output.relative_to(E))] = {'original': str(source), 'sha256': sha(source)}
for name in NATIVE_PATHS:
    shutil.copyfile(WORK / 'candidate-source-v2' / name, ROOT / name)
for name in JS_PATHS:
    shutil.copyfile(WORK / 'wasm-source-v1' / name, ROOT / name)
assert all(sha(ROOT / path) == digest for path, digest in inputs.items() if path != 'docs/properties-status.json')
command = ['cargo','fmt','--all','--','--check']
result = subprocess.run(command, cwd=ROOT, capture_output=True)
(E / 'main-format.stdout').write_bytes(result.stdout)
(E / 'main-format.stderr').write_bytes(result.stderr)
record = {'main_parent_commit': BASE,
          'implementation_sha256': {path: sha(ROOT / path) for path in NATIVE_PATHS + JS_PATHS},
          'candidate_input_hashes_sha256': sha(WORK / 'candidate-validation-v2/source-hashes.json.gz'),
          'wasm_input_hashes_sha256': sha(WORK / 'wasm-validation-v1/source-hashes.json.gz'),
          'final_input_hashes_sha256': sha(E / 'final-input-hashes-v1.json.gz'),
          'main_build_inputs_equal_tested_snapshots_except_status_ledger': True,
          'validation_partition': 'Native v2 validates final Rust, tests and all Cargo inputs. WASM v1 validates byte-identical production Rust and Cargo inputs plus final JS/harness. Only its uncompiled native integration test uses the v1 byte-decoder iteration spelling.',
          'format_command': command, 'format_exit_code': result.returncode,
          'format_stdout_sha256': sha(E / 'main-format.stdout'), 'format_stderr_sha256': sha(E / 'main-format.stderr'),
          'retained_completed_evidence': retained, 'pending_native_files_unchanged': 35,
          'native_runtime_admitted': False, 'published_package_pipeline_executed': False,
          'full_plan_complete': False}
(E / 'application-v1.json').write_text(json.dumps(record, indent=2) + '\n')
assert result.returncode == 0
assert all(sha(ROOT / path) == digest for path, digest in parent['pending_native_sha256'].items())
print('Applied six validated implementation/test/harness paths; main formatting passed.', flush=True)
