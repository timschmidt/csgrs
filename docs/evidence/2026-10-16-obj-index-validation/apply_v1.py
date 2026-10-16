"""Apply only the qualified OBJ source and regressions, then check the actual worktree."""
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
WORK = Path('/dev/shm/csgrs-obj-index-contracts')
PREP = WORK / 'preparation-v2'
PARENT = E.parent / '2026-10-16-physical-build-requalification'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


assert not (E / 'application-v1.json').exists()
assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == 'd44bada176878d8ee180ae21d729acbd488d7c90'
subprocess.run(['git', 'diff', '--cached', '--quiet'], cwd=ROOT, check=True)
qualification = read(WORK / 'candidate-validation-v1/result.json')
cost = read(WORK / 'cost-v1/result.json')
assert qualification['complete'] and not qualification['unexpected_exits']
assert cost['complete'] and cost['fresh_processes'] == 10 and cost['completed_rows'] == 30
assert cost['all_exact_output_bytes_equal'] and cost['all_retained_string_capacities_equal']
parent = read(PARENT / 'checkpoint-v1.json')
assert all(sha(ROOT / p) == h for p, h in parent['pending_native_sha256'].items())
preparation = read(PREP / 'preparation.json')
assert sha(ROOT / 'src/io/obj.rs') == preparation['base_obj_sha256']
assert sha(PREP / 'obj.rs') == preparation['candidate_obj_sha256']
assert sha(PREP / 'obj_export_failures.rs') == preparation['test_sha256']
assert not (ROOT / 'tests/obj_export_failures.rs').exists()
with gzip.open(WORK / 'candidate-validation-v1/source-hashes.json.gz', 'rt') as stream:
    inputs = json.load(stream)
assert all(sha(WORK / 'candidate-source-v1' / p) == h for p, h in inputs.items())
assert all(sha(ROOT / p) == h for p, h in inputs.items()
           if p not in ['src/io/obj.rs', 'tests/obj_export_failures.rs', 'docs/properties-status.json'])
assert sha(ROOT / 'docs/properties-status.json') == parent['ledger_after_sha256']

retained = {}
for directory in ['candidate-validation-v1', 'cost-v1']:
    destination = E / directory
    assert not destination.exists()
    for source in (WORK / directory).rglob('*'):
        if source.is_file():
            output = destination / source.relative_to(WORK / directory)
            output.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, output)
            assert sha(source) == sha(output)
            retained[str(output.relative_to(E))] = {'original': str(source), 'sha256': sha(source)}

for source, destination in [('obj.rs', 'src/io/obj.rs'), ('obj_export_failures.rs', 'tests/obj_export_failures.rs')]:
    shutil.copyfile(PREP / source, ROOT / destination)
assert all(sha(ROOT / p) == h for p, h in inputs.items() if p != 'docs/properties-status.json')
command = ['cargo', 'fmt', '--all', '--', '--check']
result = subprocess.run(command, cwd=ROOT, capture_output=True)
(E / 'main-format.stdout').write_bytes(result.stdout)
(E / 'main-format.stderr').write_bytes(result.stderr)
record = {
    'main_parent_commit': 'd44bada176878d8ee180ae21d729acbd488d7c90',
    'implementation_sha256': {p: sha(ROOT / p) for p in ['src/io/obj.rs', 'tests/obj_export_failures.rs']},
    'candidate_input_hashes_sha256': sha(WORK / 'candidate-validation-v1/source-hashes.json.gz'),
    'main_build_inputs_equal_tested_snapshot_except_status_ledger': True,
    'format_command': command, 'format_exit_code': result.returncode,
    'format_stdout_sha256': sha(E / 'main-format.stdout'), 'format_stderr_sha256': sha(E / 'main-format.stderr'),
    'retained_completed_evidence': retained,
    'pending_native_files_unchanged': 35, 'native_runtime_admitted': False,
    'published_package_pipeline_executed': False, 'full_plan_complete': False,
}
(E / 'application-v1.json').write_text(json.dumps(record, indent=2) + '\n')
assert result.returncode == 0
assert all(sha(ROOT / p) == h for p, h in parent['pending_native_sha256'].items())
print('Applied the exact qualified OBJ source and seven regressions; worktree formatting passed.')
