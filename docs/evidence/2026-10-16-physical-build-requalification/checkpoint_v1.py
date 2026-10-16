"""Close fresh physical validation without reclassifying unqualified history."""
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
from load_expectations_v1 import load

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
WORK = Path('/dev/shm/csgrs-physical-build-requalification')
STL_WORK = Path('/dev/shm/csgrs-stl-import-contracts')
OLD = E.parent / '2026-10-16-native-metadata-isolation'
BASE = '3b69af324a9642d1f71b9cc38de58b59a236f740'
PREFIX = str(E.relative_to(ROOT)) + '/'


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
previous_path = OLD / 'checkpoint-v2.json'
previous = read(previous_path)
assert all(sha(ROOT / p) == h for p, h in previous['evidence_files_sha256'].items())
assert sha(ROOT / previous['artifact_expectations']) == previous['artifact_expectations_sha256']
prior, inherited_missing = load(ROOT / previous['artifact_expectations'], allow_unqualified=True)
assert len(prior) == 854555 and len(inherited_missing) == 27093
pending = previous['pending_native_sha256']
assert len(pending) == 35 and all(sha(ROOT / p) == h for p, h in pending.items())
inherited = previous['inherited_implementation_sha256'] | previous['implementation_sha256']
extra, dependencies = previous['extra_unchanged_inputs_sha256'], previous['live_source_dependency_pins_sha256']
assert all(sha(ROOT / p) == h for pins in [inherited, extra, dependencies] for p, h in pins.items())
inputs = gz(E / 'validation-v1/source-hashes.json.gz')
assert all(sha(ROOT / p) == h and sha(WORK / 'source-v1' / p) == h for p, h in inputs.items())
format_inputs = gz(E / 'format-reproduction-v1/source-hashes.json.gz')
assert all(sha(WORK / 'format-source-v1' / p) == h for p, h in format_inputs.items())
for runner, protocol_path in [
    ('storage_v1.py', 'storage-v1.json'),
    ('validate_v1.py', 'validation-v1/protocol.json'),
    ('validate_v2.py', 'validation-v2/protocol.json'),
    ('format_reproduce_v1.py', 'format-reproduction-v1/protocol.json'),
    ('replay_v1.py', 'report-replay-v1/protocol.json'),
]:
    assert sha(E / runner) == read(E / protocol_path)['runner_sha256']
format_protocol = read(E / 'format-reproduction-v1/protocol.json')
assert sha(E / 'format_probe_v1.rs') == format_protocol['probe_sha256']
assert sha(E / 'obj_cost_probe_v1.rs') == format_protocol['cost_probe_sha256']
assert sha(E / 'format-contract-v1.md') == format_protocol['contract_sha256']
for path in E.rglob('*.py'):
    ast.parse(path.read_text(), filename=str(path))

validation = read(E / 'validation-v2/result.json')
rows = read(E / 'validation-v2/commands.json')
assert validation['complete'] and validation['commands'] == len(rows) == 12
assert validation['failed'] == [r['name'] for r in rows if r['exit_code'] != 0]
assert read(E / 'report-replay-v1/result.json')['complete']
references = []
summary = []
for row in rows:
    directory = Path(row['log_directory'])
    for suffix in ['stdout', 'stderr', 'resources']:
        assert sha(directory / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
    for name, binary in row['binaries'].items():
        assert sha(binary['retained']) == binary['sha256']
        references.append({'command': row['name'], 'path': name, **binary})
    counts = [list(map(int, values)) for values in re.findall(
        r'test result: .*? (\d+) passed; (\d+) failed; (\d+) ignored;',
        (directory / (row['name'] + '.stdout')).read_text())]
    summary.append({k: row[k] for k in ['name', 'exit_code', 'elapsed_seconds']} | {
        'reported_test_counts': dict(zip(['passed', 'failed', 'ignored'],
                                         [sum(v) for v in zip(*counts)] if counts else [0, 0, 0])),
        'logs': str(directory.relative_to(ROOT)) + '/' + row['name']})
format_result = read(E / 'format-reproduction-v1/result.json')
assert format_result['complete'] and format_result['exit_code'] == 101
assert all(sha(p) == h for p, h in format_result['binaries_sha256'].items())
assert sha(format_result['cost_binary']) == format_result['cost_binary_sha256']
for suffix in ['stdout', 'stderr', 'resources']:
    assert sha(E / 'format-reproduction-v1' / ('probe.' + suffix)) == format_result[suffix + '_sha256']
format_stdout = (E / 'format-reproduction-v1/probe.stdout').read_text()
assert '4 passed; 8 failed; 0 ignored;' in format_stdout
format_failures = re.findall(r'^test (\S+) \.\.\. FAILED$', format_stdout, re.MULTILINE)
assert len(format_failures) == 8
stl_result = read(E / 'stl-topology-v1/baseline-tests-v1/result.json')
assert stl_result['complete'] and [r['exit_code'] for r in stl_result['commands']] == [0, 101]
stl_stdout = (E / 'stl-topology-v1/baseline-tests-v1/test.stdout').read_text()
assert '2 passed; 2 failed; 0 ignored;' in stl_stdout
stl_failures = re.findall(r'^test (\S+) \.\.\. FAILED$', stl_stdout, re.MULTILINE)
assert len(stl_failures) == 2
stl_retention = read(E / 'stl-topology-v1/retention.json')
assert all(sha(E / 'stl-topology-v1' / name) == row['sha256'] == sha(row['original'])
           for name, row in stl_retention['files'].items())
assert sha(STL_WORK / 'baseline-tests-v1/stl_import_contracts') == stl_result['binary_sha256']
dump(E / 'executable-references-v1.json', references)

# Each path is read independently; virtual inode identity is never used as a cache key.
paths = []
for directory in [WORK, STL_WORK]:
    for path in directory.rglob('*'):
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


print('Hashing completed physical build files:', len(paths), flush=True)
with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
    physical = dict(pool.map(inspect, paths, buffersize=128))
assert prior.keys().isdisjoint(physical)
expectations = {
    'schema_version': 3, 'kind': 'fresh physical scope; historical audit remains failed/incomplete',
    'parent_expectations': previous['artifact_expectations'],
    'parent_expectations_sha256': previous['artifact_expectations_sha256'],
    'inherited_expected_artifact_paths': len(prior), 'inherited_missing_artifact_paths': len(inherited_missing),
    'added_physical_artifacts_sha256': physical, 'new_physical_artifacts_verified': len(physical),
    'expected_artifact_paths': len(prior) + len(physical), 'inherited_artifacts_reverified': False,
    'new_scope_uses_empty_physical_build_targets': True, 'new_scope_uses_overlay_compiler_inputs': False,
    'complete_artifact_audit_passed': False, 'historical_preservation_complete': False,
    'scope': 'Only the fresh physical build namespace and separate STL topology reproduction namespace are hashed. Parent expectations, known nested-view failures and inherited missing paths remain unchanged. No whole-history verification or zero-loss claim.'}
with gzip.open(E / 'artifact-expectations-v1.json.gz', 'wt') as stream:
    json.dump(expectations, stream, sort_keys=True)
assert gz(E / 'artifact-expectations-v1.json.gz') == expectations
decision = {
    'status': 'fresh physical validation recorded; native admission and format corrections remain open',
    'base_commit': BASE, 'owning_layer': 'validation infrastructure and public format boundaries',
    'properties': ['P6', 'P18', 'P19', 'P26', 'P27', 'P35', 'P36', 'P37'],
    'validation': validation, 'commands': summary,
    'format_reproduction': {'passed': 4, 'failed': 8, 'failures': format_failures},
    'stl_topology_reproduction': {'passed': 2, 'failed': 2, 'failures': stl_failures},
    'report_replay': read(E / 'report-replay-v1/result.json'),
    'report_rows': read(E / 'report-replay-v1/commands.json'),
    'source_or_runtime_edits': 0, 'pending_native_files_unchanged': 35,
    'runtime_admitted': False, 'full_workflow_matrix_executed': False,
    'published_package_pipeline_executed': False, 'complete_artifact_audit_passed': False,
    'application_speedup_claimed': False, 'full_plan_complete': False,
    'next': 'Validate the minimal OBJ index correction against reproduced failures, positive composition controls and unchanged valid output. Keep native timing gates and all other format failures open.'}
dump(E / 'decision-v1.json', decision)
ledger = ROOT / 'docs/properties-status.json'
raw, before = ledger.read_bytes(), sha(ledger)
assert before == previous['ledger_after_sha256']
reference = subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT)
start, end, work = span(raw)
rstart, rend, _ = span(reference)
assert (raw[:start], raw[end:]) == (reference[:rstart], reference[rend:])
work['physical_build_requalification'] = {
    'status': decision['status'], 'baseline_commit': BASE,
    'contract': PREFIX + 'contract-v1.md', 'progress': PREFIX + 'progress-v1.md',
    'decision': PREFIX + 'decision-v1.json', 'checkpoint': PREFIX + 'checkpoint-v1.json',
    'artifact_expectations': PREFIX + 'artifact-expectations-v1.json.gz',
    'passed_commands': validation['passed'], 'failed_commands': validation['failed'],
    'format_reproduction': decision['format_reproduction'],
    'stl_topology_reproduction': decision['stl_topology_reproduction'],
    'complete_report_replay_passed': decision['report_replay']['passed'],
    'source_or_runtime_edits': 0, 'pending_native_files_unchanged': 35,
    'runtime_admitted': False, 'full_plan_complete': False, 'next': decision['next']}
work['validation_storage_integrity'] = {
    'status': 'fresh physical scope checked; nested-overlay history remains unqualified',
    'fresh_report': PREFIX + 'progress-v1.md',
    'historical_diagnostic': str(OLD.relative_to(ROOT)) + '/progress-v2.md',
    'scope': 'New targets began empty on direct physical tmpfs. This requalification does not retroactively validate prior compiler inputs or repair historical missing artifacts.',
    'complete_artifact_audit_passed': False, 'full_plan_complete': False}
updated = raw[:start] + json.dumps(work, indent=2).encode() + raw[end:]
ustart, uend, _ = span(updated)
assert (updated[:ustart], updated[uend:]) == (reference[:rstart], reference[rend:])
assert {k: v for k, v in json.loads(updated).items() if k != 'current_work'} == {
    k: v for k, v in json.loads(reference).items() if k != 'current_work'}
assert sha(ledger) == before
ledger.write_bytes(updated)
files = {str(path.relative_to(ROOT)): sha(path) for path in E.rglob('*') if path.is_file()}
record = {
    'status': decision['status'], 'checkpoint_kind': 'fresh validation and failure reproduction; no runtime admission',
    'previous_closure': str(previous_path.relative_to(ROOT)), 'previous_closure_sha256': sha(previous_path),
    'evidence_files_sha256': files, 'artifact_expectations': PREFIX + 'artifact-expectations-v1.json.gz',
    'artifact_expectations_sha256': sha(E / 'artifact-expectations-v1.json.gz'), 'artifact_expectations_schema': 3,
    'new_physical_artifacts_verified': len(physical), 'inherited_expected_artifact_paths': len(prior),
    'inherited_missing_artifact_paths': len(inherited_missing), 'complete_artifact_audit_passed': False,
    'historical_preservation_complete': False, 'implementation_sha256': {},
    'inherited_implementation_sha256': inherited, 'extra_unchanged_inputs_sha256': extra,
    'pending_native_sha256': pending, 'live_source_dependency_pins_sha256': dependencies,
    'ledger_before_sha256': before, 'ledger_after_sha256': sha(ledger), 'only_current_work_changed': True,
    'python_sources_parse': True, 'runtime_admitted': False, 'full_plan_complete': False}
dump(E / 'checkpoint-v1.json', record)
print(json.dumps({k: record[k] for k in ['new_physical_artifacts_verified', 'only_current_work_changed', 'runtime_admitted']}), flush=True)
