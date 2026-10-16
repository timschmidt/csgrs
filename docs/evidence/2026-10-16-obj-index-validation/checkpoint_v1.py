"""Pin the narrow OBJ correction and new physical evidence without rewriting history."""
import ast
import concurrent.futures
import gzip
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
ROOT = E.parents[2]
PARENT = E.parent / '2026-10-16-physical-build-requalification'
WORK = Path('/dev/shm/csgrs-obj-index-contracts')
BASE = 'd44bada176878d8ee180ae21d729acbd488d7c90'
PREFIX = str(E.relative_to(ROOT)) + '/'
sys.path.insert(0, str(PARENT))
from load_expectations_v1 import load


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def gz(path):
    with gzip.open(path, 'rt') as stream:
        return json.load(stream)


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True) + '\n')


def span(raw):
    start = re.search(rb'"current_work"\s*:\s*', raw).end()
    value, length = json.JSONDecoder().raw_decode(raw[start:].decode())
    return start, start + len(raw[start:].decode()[:length].encode()), value


assert not (E / 'checkpoint-v1.json').exists()
assert not (E / 'artifact-expectations-v1.json.gz').exists()
assert (E / 'progress-v1.md').is_file()
assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == BASE
subprocess.run(['git', 'diff', '--cached', '--quiet'], cwd=ROOT, check=True)
previous_path = PARENT / 'checkpoint-v1.json'
previous = read(previous_path)
assert all(sha(ROOT / p) == h for p, h in previous['evidence_files_sha256'].items())
assert sha(ROOT / previous['artifact_expectations']) == previous['artifact_expectations_sha256']
prior, inherited_missing = load(ROOT / previous['artifact_expectations'], allow_unqualified=True)
assert len(prior) == 874041 and len(inherited_missing) == 27093
pending = previous['pending_native_sha256']
assert len(pending) == 35 and all(sha(ROOT / p) == h for p, h in pending.items())
inherited = previous['inherited_implementation_sha256'] | previous['implementation_sha256']
extra, dependencies = previous['extra_unchanged_inputs_sha256'], previous['live_source_dependency_pins_sha256']
decision, application = read(E / 'decision-v1.json'), read(E / 'application-v1.json')
implementation = application['implementation_sha256']
assert decision['obj_index_fix_admitted'] and implementation == decision['implementation_sha256']
assert set(implementation) == {'src/io/obj.rs', 'tests/obj_export_failures.rs'}
assert all(sha(ROOT / p) == h for pins in [inherited | implementation, extra, dependencies] for p, h in pins.items())
inputs = gz(E / 'candidate-validation-v1/source-hashes.json.gz')
assert all(sha(WORK / 'candidate-source-v1' / p) == h for p, h in inputs.items())
assert all(sha(ROOT / p) == h for p, h in inputs.items() if p != 'docs/properties-status.json')
assert application['format_exit_code'] == 0
assert sha(E / 'main-format.stdout') == application['format_stdout_sha256']
assert sha(E / 'main-format.stderr') == application['format_stderr_sha256']
for path in E.rglob('*.py'):
    ast.parse(path.read_text(), filename=str(path))
for retention in [read(E / 'preparation-retention-v1.json')['files'], application['retained_completed_evidence']]:
    assert all(sha(E / name) == row['sha256'] == sha(row['original']) for name, row in retention.items())
qualification = read(E / 'candidate-validation-v1/result.json')
cost = read(E / 'cost-v1/result.json')
assert qualification['complete'] and not qualification['unexpected_exits']
assert cost['complete'] and cost['fresh_processes'] == 10 and cost['completed_rows'] == 30
assert cost['all_exact_output_bytes_equal'] and cost['all_retained_string_capacities_equal']
protocol = read(E / 'candidate-validation-v1/protocol.json')
assert sha(E / 'candidate_v1.py') == sha(WORK / 'candidate_v1.py') == protocol['runner_sha256']
assert sha(E / 'cost_v1.py') == sha(WORK / 'cost_v1.py') == read(E / 'cost-v1/protocol.json')['runner_sha256']
assert sha(E / 'candidate-validation-v1/source-hashes.json.gz') == protocol['source_hashes_sha256']
assert sha(protocol['format_probe']) == protocol['format_probe_sha256']
assert sha(protocol['cost_probe']) == protocol['cost_probe_sha256']
for row in read(E / 'candidate-validation-v1/commands.json'):
    for suffix in ['stdout', 'stderr', 'resources']:
        assert sha(E / 'candidate-validation-v1' / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
    assert all(sha(record['retained']) == record['sha256'] for record in row['binaries'].values())
for variant in read(E / 'candidate-validation-v1/probe-bindings.json').values():
    assert all(sha(record['retained']) == record['sha256'] for record in variant.values())
assert all(sha(p) == h for p, h in qualification['probe_binaries_sha256'].items())

# No inode-based memoization: every physical path is read separately.
paths = []
for path in WORK.rglob('*'):
    assert not path.is_symlink() and not path.is_mount(), path
    if path.is_file():
        paths.append(path)


def inspect(path):
    before = path.stat()
    digest = sha(path)
    after = path.stat()
    assert all(getattr(before, key) == getattr(after, key)
               for key in ['st_dev', 'st_ino', 'st_size', 'st_mtime_ns', 'st_ctime_ns']), path
    return str(path), digest


print('Hashing completed physical OBJ files:', len(paths), flush=True)
with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
    physical = dict(pool.map(inspect, paths, buffersize=128))
assert prior.keys().isdisjoint(physical)
expectations = {
    'schema_version': 3, 'kind': 'new physical OBJ evidence; inherited historical audit remains failed/incomplete',
    'parent_expectations': previous['artifact_expectations'],
    'parent_expectations_sha256': previous['artifact_expectations_sha256'],
    'inherited_expected_artifact_paths': len(prior), 'inherited_missing_artifact_paths': len(inherited_missing),
    'added_physical_artifacts_sha256': physical, 'new_physical_artifacts_verified': len(physical),
    'expected_artifact_paths': len(prior) + len(physical), 'inherited_artifacts_reverified': False,
    'complete_artifact_audit_passed': False, 'historical_preservation_complete': False,
    'scope': 'The completed OBJ physical namespace is hashed independently. All parent expectations, known nested-view failures and inherited missing paths remain unchanged. No whole-history verification or zero-loss claim.'}
with gzip.open(E / 'artifact-expectations-v1.json.gz', 'wt') as stream:
    json.dump(expectations, stream, sort_keys=True)
assert gz(E / 'artifact-expectations-v1.json.gz') == expectations
ledger = ROOT / 'docs/properties-status.json'
raw, before = ledger.read_bytes(), sha(ledger)
assert before == previous['ledger_after_sha256']
reference = subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT)
start, end, work = span(raw)
rstart, rend, _ = span(reference)
assert (raw[:start], raw[end:]) == (reference[:rstart], reference[rend:])
work['obj_index_validation'] = {
    'status': decision['status'], 'main_parent_commit': BASE,
    'contract': PREFIX + 'contract-v1.md', 'progress': PREFIX + 'progress-v1.md',
    'decision': PREFIX + 'decision-v1.json', 'checkpoint': PREFIX + 'checkpoint-v1.json',
    'artifact_expectations': PREFIX + 'artifact-expectations-v1.json.gz',
    'implementation_sha256': implementation, 'original_format_diagnostic': decision['original_format_diagnostic'],
    'cost': PREFIX + 'cost-v1/result.json', 'obj_index_fix_admitted': True,
    'native_runtime_admitted': False, 'pending_native_files_unchanged': 35,
    'published_package_pipeline_executed': False, 'full_plan_complete': False,
    'next': decision['next']}
updated = raw[:start] + json.dumps(work, indent=2).encode() + raw[end:]
ustart, uend, _ = span(updated)
assert (updated[:ustart], updated[uend:]) == (reference[:rstart], reference[rend:])
assert {k: v for k, v in json.loads(updated).items() if k != 'current_work'} == {
    k: v for k, v in json.loads(reference).items() if k != 'current_work'}
assert sha(ledger) == before
ledger.write_bytes(updated)
files = {str(path.relative_to(ROOT)): sha(path) for path in E.rglob('*') if path.is_file()}
record = {
    'status': decision['status'], 'checkpoint_kind': 'narrow OBJ source admission; historical artifact limitations retained',
    'previous_closure': str(previous_path.relative_to(ROOT)), 'previous_closure_sha256': sha(previous_path),
    'evidence_files_sha256': files, 'artifact_expectations': PREFIX + 'artifact-expectations-v1.json.gz',
    'artifact_expectations_sha256': sha(E / 'artifact-expectations-v1.json.gz'), 'artifact_expectations_schema': 3,
    'new_physical_artifacts_verified': len(physical), 'inherited_expected_artifact_paths': len(prior),
    'inherited_missing_artifact_paths': len(inherited_missing), 'complete_artifact_audit_passed': False,
    'historical_preservation_complete': False, 'implementation_sha256': implementation,
    'inherited_implementation_sha256': inherited, 'extra_unchanged_inputs_sha256': extra,
    'pending_native_sha256': pending, 'live_source_dependency_pins_sha256': dependencies,
    'ledger_before_sha256': before, 'ledger_after_sha256': sha(ledger), 'only_current_work_changed': True,
    'python_sources_parse': True, 'obj_index_fix_admitted': True, 'native_runtime_admitted': False,
    'full_plan_complete': False}
dump(E / 'checkpoint-v1.json', record)
print(json.dumps({k: record[k] for k in ['new_physical_artifacts_verified', 'only_current_work_changed', 'obj_index_fix_admitted']}), flush=True)
