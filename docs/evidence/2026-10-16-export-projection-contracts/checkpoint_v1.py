"""Pin export projection correctness and physical evidence without rewriting history."""
import ast
import concurrent.futures
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
ROOT = E.parents[2]
PARENT = E.parent / '2026-10-16-stl-native-validation'
WORK = ROOT / 'target/properties/export-projection-contracts'
BASE = 'c9410688b0c7854fba6c9a90f34cae5b3382318a'
PREFIX = str(E.relative_to(ROOT)) + '/'
sys.path.insert(0, str(E.parent / '2026-10-16-physical-build-requalification'))
from load_expectations_v1 import load


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def gz(path):
    with gzip.open(path, 'rt') as stream:
        return json.load(stream)


def span(raw):
    start = re.search(rb'"current_work"\s*:\s*', raw).end()
    value, length = json.JSONDecoder().raw_decode(raw[start:].decode())
    return start, start + len(raw[start:].decode()[:length].encode()), value


assert not (E / 'checkpoint-v1.json').exists()
assert not (E / 'artifact-expectations-v1.json.gz').exists()
assert (E / 'progress-v1.md').is_file()
assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == BASE
subprocess.run(['git', 'diff', '--cached', '--quiet'], cwd=ROOT, check=True)
previous_path = PARENT / 'checkpoint-v3.json'
previous = read(previous_path)
assert all(sha(ROOT / path) == digest for path, digest in previous['evidence_files_sha256'].items())
assert sha(ROOT / previous['artifact_expectations']) == previous['artifact_expectations_sha256']
prior, inherited_missing = load(ROOT / previous['artifact_expectations'], allow_unqualified=True)
assert len(prior) == 906112 and len(inherited_missing) == 27093
pending = previous['pending_native_sha256']
assert len(pending) == 35 and all(sha(ROOT / path) == digest for path, digest in pending.items())
inherited = previous['inherited_implementation_sha256'] | previous['implementation_sha256']
extra, dependencies = previous['extra_unchanged_inputs_sha256'], previous['live_source_dependency_pins_sha256']
decision, application = read(E / 'decision-v1.json'), read(E / 'application-v1.json')
implementation = application['implementation_sha256']
assert decision['export_projection_fix_admitted'] and implementation == decision['implementation_sha256']
assert set(implementation) == {'src/io/mod.rs','src/io/obj.rs','src/io/stl.rs','tests/export_projection_contracts.rs','scripts/test-wasm-bindings.sh','tests/wasm/export_projection.cjs'}
assert all(sha(ROOT / path) == digest for pins in [inherited | implementation, extra, dependencies] for path, digest in pins.items())
inputs = gz(E / 'final-input-hashes-v1.json.gz')
for folder, source in [('candidate-validation-v2','candidate-source-v2'),('wasm-validation-v1','wasm-source-v1')]:
    assert all(sha(WORK / source / path) == digest for path, digest in gz(E / folder / 'source-hashes.json.gz').items())
assert all(sha(ROOT / path) == digest for path, digest in inputs.items() if path != 'docs/properties-status.json')
assert application['format_exit_code'] == 0
assert sha(E / 'main-format.stdout') == application['format_stdout_sha256']
assert sha(E / 'main-format.stderr') == application['format_stderr_sha256']
for path in E.rglob('*.py'):
    ast.parse(path.read_text(), filename=str(path))
assert all(sha(E / name) == row['sha256'] == sha(row['original']) for name, row in application['retained_completed_evidence'].items())
qualification, cost = read(E / 'candidate-validation-v2/result.json'), read(E / 'cost-v1/result.json')
assert qualification['complete'] and not qualification['unexpected_exits']
assert cost['complete'] and cost['runtime_processes'] == 90 and cost['sample_rows'] == 180
assert cost['complete_input_and_output_comparisons_equal'] and cost['library_inputs_unchanged']
for name, runner in [('baseline-v1', 'baseline_v1.py'), ('candidate-validation-v1', 'candidate_v1.py'), ('candidate-validation-v2', 'candidate_v2.py'), ('cost-v1', 'cost_v1.py'), ('wasm-validation-v1','wasm_v1.py'), ('wasm-tools-v1','wasm_tools_v1.py'), ('fractional-cost-v1','fractional_cost_v1.py')]:
    protocol = read(E / name / 'protocol.json')
    assert sha(E / runner) == protocol['runner_sha256']
    if 'source_hashes_sha256' in protocol:
        assert sha(E / name / 'source-hashes.json.gz') == protocol['source_hashes_sha256']
    for row in protocol.get('probes', {}).values():
        assert sha(row['path']) == row['sha256']
assert sha(E / 'export_cost_v1.rs') == read(E / 'cost-v1/protocol.json')['probe_sha256']
for row in read(E / 'candidate-validation-v2/commands.json'):
    for suffix in ['stdout', 'stderr', 'resources']:
        assert sha(E / 'candidate-validation-v2' / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
    assert all(sha(record['retained']) == record['sha256'] for record in row['binaries'].values())
assert all(sha(record['retained']) == record['sha256'] for record in read(E / 'candidate-validation-v2/probe-bindings.json').values())
assert all(sha(path) == digest for path, digest in read(E / 'cost-v1/library-hashes.json').items())

fractional = read(E / 'fractional-cost-v1/result.json')
assert fractional['complete'] and fractional['sample_rows'] == 40
assert fractional['library_inputs_unchanged'] and fractional['all_repeated_complete_outputs_equal']
assert sha(E / 'fractional_cost_v1.rs') == read(E / 'fractional-cost-v1/protocol.json')['probe_sha256']
assert all(sha(path) == digest for path,digest in read(E / 'fractional-cost-v1/library-hashes.json').items())
wasm = read(E / 'wasm-validation-v1/result.json')
assert wasm['all_commands_passed'] and wasm['command_count'] == 4
assert all(sha(path) == row['sha256'] for path, row in wasm['artifacts'].items())
assert read(E / 'candidate-validation-v1/result.json')['unexpected_exits'] == ['strict-clippy','stl-clippy']

# Every physical path is read separately. Never infer identity from inode reuse.
paths = []
symlinks = {}
for path in WORK.rglob('*'):
    assert not path.is_mount(), path
    if path.is_symlink():
        target = path.resolve(strict=True)
        assert target.is_file() and target.is_relative_to(WORK), path
        symlinks[str(path)] = {'link_text': os.readlink(path), 'resolved_target': str(target)}
    if path.is_file():
        paths.append(path)


def inspect(path):
    before, link_before = path.stat(), path.lstat()
    digest = sha(path)
    after, link_after = path.stat(), path.lstat()
    assert all(getattr(link_before, key) == getattr(link_after, key)
               for key in ['st_dev','st_ino','st_size','st_mtime_ns','st_ctime_ns']), path
    if str(path) in symlinks:
        assert os.readlink(path) == symlinks[str(path)]['link_text']
        assert str(path.resolve(strict=True)) == symlinks[str(path)]['resolved_target']
    assert all(getattr(before, key) == getattr(after, key)
               for key in ['st_dev', 'st_ino', 'st_size', 'st_mtime_ns', 'st_ctime_ns']), path
    return str(path), digest


print('Hashing completed physical export files:', len(paths), flush=True)
with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
    physical = dict(pool.map(inspect, paths, buffersize=128))
assert prior.keys().isdisjoint(physical)
expectations = {
    'schema_version': 3, 'kind': 'new physical export evidence; inherited historical audit remains failed/incomplete',
    'parent_expectations': previous['artifact_expectations'],
    'parent_expectations_sha256': previous['artifact_expectations_sha256'],
    'inherited_expected_artifact_paths': len(prior), 'inherited_missing_artifact_paths': len(inherited_missing),
    'added_physical_artifacts_sha256': physical, 'new_physical_artifacts_verified': len(physical),
    'local_tool_symlink_targets': symlinks,
    'expected_artifact_paths': len(prior) + len(physical), 'inherited_artifacts_reverified': False,
    'complete_artifact_audit_passed': False, 'historical_preservation_complete': False,
    'scope': 'Completed physical native/WASM export sources, targets, tools, logs and probes are hashed independently. Intentional package-manager links are pinned separately and each linked file is also read and hashed through its own path. Parent expectations, known nested-view failures and inherited missing paths remain unchanged. No whole-history verification or zero-loss claim.'}
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
work['export_projection_contracts'] = {
    'status': decision['status'], 'main_parent_commit': BASE,
    'contract': PREFIX + 'contract-v1.md', 'progress': PREFIX + 'progress-v1.md',
    'decision': PREFIX + 'decision-v1.json', 'checkpoint': PREFIX + 'checkpoint-v1.json',
    'artifact_expectations': PREFIX + 'artifact-expectations-v1.json.gz',
    'implementation_sha256': implementation, 'original_format_diagnostic': decision['original_format_diagnostic'],
    'cost': PREFIX + 'cost-v1/result.json', 'cost_contract': PREFIX + 'cost-contract-v1.md', 'export_projection_fix_admitted': True,
    'native_runtime_admitted': False, 'pending_native_files_unchanged': 35,
    'published_package_pipeline_executed': False, 'full_plan_complete': False, 'next': decision['next']}
updated = raw[:start] + json.dumps(work, indent=2).encode() + raw[end:]
ustart, uend, _ = span(updated)
assert (updated[:ustart], updated[uend:]) == (reference[:rstart], reference[rend:])
assert {key: value for key, value in json.loads(updated).items() if key != 'current_work'} == {
    key: value for key, value in json.loads(reference).items() if key != 'current_work'}
assert sha(ledger) == before
ledger.write_bytes(updated)
# Ignored Python bytecode is neither authored evidence nor a verification input.
files = {str(path.relative_to(ROOT)): sha(path) for path in E.rglob('*')
         if path.is_file() and '__pycache__' not in path.parts}
record = {
    'status': decision['status'], 'checkpoint_kind': 'narrow export source admission; historical artifact limitations retained',
    'previous_closure': str(previous_path.relative_to(ROOT)), 'previous_closure_sha256': sha(previous_path),
    'evidence_files_sha256': files, 'artifact_expectations': PREFIX + 'artifact-expectations-v1.json.gz',
    'artifact_expectations_sha256': sha(E / 'artifact-expectations-v1.json.gz'), 'artifact_expectations_schema': 3,
    'new_physical_artifacts_verified': len(physical), 'inherited_expected_artifact_paths': len(prior),
    'inherited_missing_artifact_paths': len(inherited_missing), 'complete_artifact_audit_passed': False,
    'historical_preservation_complete': False, 'implementation_sha256': implementation,
    'inherited_implementation_sha256': inherited, 'extra_unchanged_inputs_sha256': extra,
    'pending_native_sha256': pending, 'live_source_dependency_pins_sha256': dependencies,
    'ledger_before_sha256': before, 'ledger_after_sha256': sha(ledger), 'only_current_work_changed': True,
    'python_sources_parse': True, 'export_projection_fix_admitted': True, 'native_runtime_admitted': False,
    'full_plan_complete': False}
(E / 'checkpoint-v1.json').write_text(json.dumps(record, indent=2, sort_keys=True) + '\n')
print(json.dumps({key: record[key] for key in ['new_physical_artifacts_verified', 'only_current_work_changed', 'export_projection_fix_admitted']}), flush=True)
