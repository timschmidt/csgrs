"""Apply the tested STL implementation and retain completed, reviewable evidence."""
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
WORK = ROOT / 'target/properties/stl-native-validation'
PARENT = E.parent / '2026-10-16-obj-index-validation'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


assert not (E / 'application-v2.json').exists()
assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == '7a1f0c7e1f4f69499feb3b471f739dfb50f4776c'
subprocess.run(['git', 'diff', '--cached', '--quiet'], cwd=ROOT, check=True)
qualification = read(WORK / 'candidate-validation-v2/result.json')
cost = read(WORK / 'cost-v2/result.json')
assert qualification['complete'] and not qualification['unexpected_exits']
assert cost['complete'] and cost['runtime_processes'] == 60 and cost['sample_rows'] == 120
assert cost['complete_input_and_output_comparisons_equal'] and cost['library_inputs_unchanged']
parent = read(PARENT / 'checkpoint-v1.json')
assert all(sha(ROOT / path) == digest for path, digest in parent['pending_native_sha256'].items())
with gzip.open(WORK / 'baseline-v1/source-hashes.json.gz', 'rt') as stream:
    baseline = json.load(stream)
assert all(sha(ROOT / path) == digest for path, digest in baseline.items())
assert not (ROOT / 'tests/stl_import_contracts.rs').exists()
with gzip.open(WORK / 'candidate-validation-v2/source-hashes.json.gz', 'rt') as stream:
    inputs = json.load(stream)
assert all(sha(WORK / 'candidate-source-v2' / path) == digest for path, digest in inputs.items())
assert sha(ROOT / 'docs/properties-status.json') == parent['ledger_after_sha256']
retained = {}
for directory in ['baseline-v1', 'example-baseline-v1', 'candidate-validation-v1', 'candidate-validation-v2', 'cost-v2']:
    destination = E / directory
    assert not destination.exists()
    for source in (WORK / directory).rglob('*'):
        if not source.is_file():
            continue
        # Executables remain at their recorded physical paths and are hashed in
        # the artifact manifest. Keep their logs, inputs and full output rows here.
        if source.suffix not in {'.json', '.gz', '.stdout', '.stderr', '.resources', '.patch', '.stl', '.rows'} and source.name not in {'stdout', 'stderr', 'resources'}:
            continue
        output = destination / source.relative_to(WORK / directory)
        output.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, output)
        assert sha(source) == sha(output)
        retained[str(output.relative_to(E))] = {'original': str(source), 'sha256': sha(source)}
example_files = sorted((WORK / 'candidate-source-v2/stl/examples/basic_shapes').glob('*.stl'))
assert {path.stem for path in example_files} == {'cube', 'sphere', 'cylinder', 'ellipsoid', 'torus'}
for source in example_files:
    output = E / 'basic-shapes-output-v2' / source.name
    output.parent.mkdir(exist_ok=True)
    shutil.copy2(source, output)
    retained[str(output.relative_to(E))] = {'original': str(source), 'sha256': sha(source)}
for name in ['Cargo.toml', 'src/io/stl.rs', 'tests/stl_import_contracts.rs']:
    shutil.copyfile(WORK / 'candidate-source-v2' / name, ROOT / name)
assert all(sha(ROOT / path) == digest for path, digest in inputs.items() if path != 'docs/properties-status.json')
command = ['cargo', 'fmt', '--all', '--', '--check']
result = subprocess.run(command, cwd=ROOT, capture_output=True)
(E / 'main-format.stdout').write_bytes(result.stdout)
(E / 'main-format.stderr').write_bytes(result.stderr)
record = {'main_parent_commit': '7a1f0c7e1f4f69499feb3b471f739dfb50f4776c',
          'implementation_sha256': {path: sha(ROOT / path) for path in ['Cargo.toml', 'src/io/stl.rs', 'tests/stl_import_contracts.rs']},
          'candidate_input_hashes_sha256': sha(WORK / 'candidate-validation-v2/source-hashes.json.gz'),
          'main_build_inputs_equal_tested_snapshot_except_status_ledger': True,
          'format_command': command, 'format_exit_code': result.returncode,
          'format_stdout_sha256': sha(E / 'main-format.stdout'), 'format_stderr_sha256': sha(E / 'main-format.stderr'),
          'retained_completed_evidence': retained, 'pending_native_files_unchanged': 35,
          'native_runtime_admitted': False, 'published_package_pipeline_executed': False,
          'full_plan_complete': False}
(E / 'application-v2.json').write_text(json.dumps(record, indent=2) + '\n')
assert result.returncode == 0
assert all(sha(ROOT / path) == digest for path, digest in parent['pending_native_sha256'].items())
print('Applied the tested STL source and eight regressions; main worktree formatting passed.')
