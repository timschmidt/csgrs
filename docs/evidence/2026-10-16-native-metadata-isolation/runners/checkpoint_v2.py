"""Close the trial as an integrity diagnostic without claiming a complete artifact audit."""
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
E = Path(__file__).resolve().parents[1]
ROOT = E.parents[2]
OLD = E.parent / '2026-10-16-native-admission-recheck'
WORK = Path('/dev/shm/csgrs-native-metadata-isolation')
BASE = '73472622dd4034b8b8777cb2a116c4020f6bb328'
PREFIX = str(E.relative_to(ROOT)) + '/'
sys.path.insert(0, str(E.parent / '2026-10-16-polyhedral-construction'))
from load_artifact_audit_v1 import load


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
    end = start + len(raw[start:].decode()[:length].encode())
    return start, end, value


assert not (E / 'checkpoint-v2.json').exists()
assert not (E / 'artifact-expectations-v2.json.gz').exists()
assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == BASE
subprocess.run(['git', 'diff', '--cached', '--quiet'], cwd=ROOT, check=True)
previous_path = OLD / 'checkpoint-v1.json'
previous = read(previous_path)
assert all(sha(ROOT / p) == h for p, h in previous['evidence_files_sha256'].items())
assert sha(ROOT / previous['artifact_audit']) == previous['artifact_audit_sha256']
prior, inherited_missing = load(ROOT / previous['artifact_audit'])
assert len(prior) == 835959 and len(inherited_missing) == 27093
pending = previous['pending_native_sha256']
assert len(pending) == 35 and all(sha(ROOT / p) == h for p, h in pending.items())
inherited = previous['inherited_implementation_sha256'] | previous['implementation_sha256']
extra = previous['extra_unchanged_inputs_sha256']
dependencies = previous['live_source_dependency_pins_sha256']
assert all(sha(ROOT / p) == h for pins in [inherited, extra, dependencies] for p, h in pins.items())
for phase in ['baseline-v1', 'candidate-v1']:
    assert all(sha(ROOT / p) == h for p, h in gz(E / phase / 'source-hashes.json.gz').items())
    result = read(E / phase / 'result.json')
    for row in result['commands']:
        assert row['sources_unchanged']
        for suffix in ['stdout', 'stderr', 'resources']:
            assert sha(E / phase / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
preparation = read(E / 'preparation-v1.json')
source_maps = gz(E / 'source-hashes-v1.json.gz')
assert sha(E / 'source-hashes-v1.json.gz') == preparation['source_hashes_sha256']
assert all(sha(ROOT / p) == h for p, h in source_maps['main_guard'].items())
for variant, source in preparation['sources'].items():
    assert {str(p.relative_to(source)): sha(p) for p in Path(source).rglob('*') if p.is_file()} == source_maps[variant]
cost_protocol = read(E / 'cost-v1/protocol.json')
for variant, entries in gz(E / 'cost-v1/source-hashes.json.gz').items():
    source = Path(cost_protocol['sources'][variant])
    assert {str(p.relative_to(source)): sha(p) for p in source.rglob('*') if p.is_file()} == entries
for path in E.rglob('*.py'):
    ast.parse(path.read_text(), filename=str(path))
references = gz(E / 'closure-diagnostic-v1/references-v1.json.gz')
assert references['count'] == 191 and references['all_referenced_bytes_retained']
assert all(sha(row['verified_content_path']) == row['expected_sha256'] for row in references['references'])
nested = read(E / 'closure-diagnostic-v1/overlay-probe-v1/result.json')
flat = read(E / 'closure-diagnostic-v1/flat-probe-v1/result.json')
assert nested['observations'] == flat['observations'] == 8
assert not nested['all_views_match_physical_sources'] and flat['all_views_match_physical_sources']
assert nested['physical_sources_unchanged'] and flat['physical_sources_unchanged']
diagnostic = read(E / 'closure-diagnostic-v1/digests-v1.json')
assert diagnostic['audit_exit_code'] == 1 and diagnostic['ledger_unchanged']
known_mismatches = [row for row in diagnostic['uncached_reads']
                    if row.get('expected_sha256') and row.get('sha256')
                    and row['expected_sha256'] != row['sha256']]
assert known_mismatches
for row in known_mismatches:
    assert prior[row['path']] == row['expected_sha256']

# Read each physical path independently; do not deduplicate content by virtual inode.
paths = []
for p in WORK.rglob('*'):
    assert not p.is_symlink() and not p.is_mount(), p
    if p.is_file():
        paths.append(p)
print('Hashing physical trial files:', len(paths), flush=True)


def inspect(path):
    before = path.stat()
    digest = sha(path)
    after = path.stat()
    fields = ['st_dev', 'st_ino', 'st_size', 'st_mtime_ns', 'st_ctime_ns']
    assert all(getattr(before, f) == getattr(after, f) for f in fields), path
    return str(path), digest


with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
    physical = dict(pool.map(inspect, paths, buffersize=128))
assert prior.keys().isdisjoint(physical)
audit = {
    'schema_version': 3, 'kind': 'diagnostic expectations; inherited audit failed/incomplete',
    'parent_audit': previous['artifact_audit'], 'parent_audit_sha256': previous['artifact_audit_sha256'],
    'inherited_expected_artifact_paths': len(prior), 'inherited_missing_artifact_paths': len(inherited_missing),
    'inherited_expectations_preserved': True, 'inherited_artifacts_reverified': False,
    'added_physical_artifacts_sha256': physical,
    'new_physical_artifacts_verified': len(physical),
    'expected_artifact_paths': len(prior) + len(physical),
    'referenced_executable_expectations': {r['path']: r['expected_sha256'] for r in references['references']},
    'referenced_executable_physical_locations': {r['path']: r['verified_content_path'] for r in references['references']},
    'known_view_mismatches_observed': known_mismatches,
    'complete_artifact_audit_passed': False, 'compiler_input_identity_qualified': False,
    'historical_preservation_complete': False,
    'scope': 'New physical trial files are byte-hashed individually. All parent expectations and inherited missing paths remain pinned through the parent manifest. The failed whole-view audit is not converted into a pass; no current whole-history verification count or zero-loss claim is supplied.'}
with gzip.open(E / 'artifact-expectations-v2.json.gz', 'wt') as stream:
    json.dump(audit, stream, sort_keys=True)
assert gz(E / 'artifact-expectations-v2.json.gz') == audit
old_decision = read(E / 'decision-v1.json')
decision = {
    'status': 'observations retained; qualification withheld after reproduced storage identity failure',
    'decision': 'Do not admit the metadata-order candidate or the 35 pending native files.',
    'supersedes_admission_interpretation': ['progress-v1.md', 'decision-v1.json', 'qualification-assessment-v1.json', 'cost-assessment-v1.json'],
    'base_commit': BASE, 'owning_layer': 'validation infrastructure', 'properties': ['P19', 'P27', 'P36', 'P37'],
    'source_or_runtime_edits': 0, 'pending_native_files_unchanged': 35,
    'original_native_gates_observed': old_decision['original_native_gates'],
    'failed_full_artifact_audit_exit_code': 1, 'nested_identity_probe_observations': 8,
    'nested_identity_probe_passed': False, 'flat_control_observations': 8, 'flat_control_passed': True,
    'referenced_executable_byte_sequences_retained': 191,
    'complete_artifact_audit_passed': False, 'compiler_input_identity_qualified': False,
    'runtime_admitted': False, 'application_speedup_claimed': False, 'full_plan_complete': False,
    'next': 'Validate a fresh physical target and rebuild affected checks without nested overlay inputs. Preserve original runtime assertions and all prior diagnostic evidence.'}
dump(E / 'decision-v2.json', decision)
ledger = ROOT / 'docs/properties-status.json'
raw = ledger.read_bytes()
before = sha(ledger)
assert before == previous['ledger_after_sha256']
reference = subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT)
start, end, work = span(raw)
rstart, rend, _ = span(reference)
assert (raw[:start], raw[end:]) == (reference[:rstart], reference[rend:])
work['native_metadata_isolation'] = {
    'status': decision['status'], 'baseline_commit': BASE,
    'contract': PREFIX + 'contract-v1.md', 'progress': PREFIX + 'progress-v2.md',
    'decision': PREFIX + 'decision-v2.json', 'checkpoint': PREFIX + 'checkpoint-v2.json',
    'artifact_expectations': PREFIX + 'artifact-expectations-v2.json.gz',
    'observed_qualification': PREFIX + 'qualification-assessment-v1.json',
    'observed_cost': PREFIX + 'cost-assessment-v1.json', 'reports': PREFIX + 'report-replay-v1/result.json',
    'original_native_gates_observed': decision['original_native_gates_observed'],
    'source_or_runtime_edits': 0, 'pending_native_files_unchanged': 35,
    'compiler_input_identity_qualified': False, 'runtime_admitted': False, 'full_plan_complete': False,
    'historical_preservation': 'Parent expectations and 27093 inherited missing paths remain pinned. Full audit failed; nested view aliasing reproduced. No new whole-history completeness or zero-loss claim.',
    'next': decision['next']}
work['validation_storage_integrity'] = {
    'status': 'nested overlay compiler-input identity unqualified; clean rebuild required',
    'report': PREFIX + 'progress-v2.md', 'nested_reproduction': PREFIX + 'closure-diagnostic-v1/overlay-probe-v1/result.json',
    'flat_control': PREFIX + 'closure-diagnostic-v1/flat-probe-v1/result.json',
    'scope': 'Prior checks using affected nested views require requalification before supporting a current acceptance claim. Historical results remain observations; this does not prove production code wrong.',
    'complete_artifact_audit_passed': False, 'full_plan_complete': False}
updated = raw[:start] + json.dumps(work, indent=2).encode() + raw[end:]
ustart, uend, _ = span(updated)
assert (updated[:ustart], updated[uend:]) == (reference[:rstart], reference[rend:])
assert {k: v for k, v in json.loads(updated).items() if k != 'current_work'} == {
    k: v for k, v in json.loads(reference).items() if k != 'current_work'}
assert sha(ledger) == before
ledger.write_bytes(updated)
files = {str(p.relative_to(ROOT)): sha(p) for p in E.rglob('*') if p.is_file()}
record = {
    'status': decision['status'], 'checkpoint_kind': 'diagnostic; not implementation admission',
    'previous_closure': str(previous_path.relative_to(ROOT)), 'previous_closure_sha256': sha(previous_path),
    'evidence_files_sha256': files, 'artifact_expectations': PREFIX + 'artifact-expectations-v2.json.gz',
    'artifact_expectations_sha256': sha(E / 'artifact-expectations-v2.json.gz'), 'artifact_expectations_schema': 3,
    'new_physical_artifacts_verified': len(physical), 'inherited_expected_artifact_paths': len(prior),
    'inherited_missing_artifact_paths': len(inherited_missing),
    'complete_artifact_audit_passed': False, 'compiler_input_identity_qualified': False,
    'historical_preservation_complete': False,
    'implementation_sha256': {}, 'inherited_implementation_sha256': inherited,
    'extra_unchanged_inputs_sha256': extra, 'pending_native_sha256': pending,
    'live_source_dependency_pins_sha256': dependencies,
    'ledger_before_sha256': before, 'ledger_after_sha256': sha(ledger), 'only_current_work_changed': True,
    'python_sources_parse': True, 'runtime_admitted': False, 'full_plan_complete': False}
dump(E / 'checkpoint-v2.json', record)
print(json.dumps({k: record[k] for k in ['new_physical_artifacts_verified', 'complete_artifact_audit_passed', 'only_current_work_changed', 'runtime_admitted']}), flush=True)
