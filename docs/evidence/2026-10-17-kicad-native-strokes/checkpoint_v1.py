"""Close rejected E36 evidence without changing any production source."""
import ast
import concurrent.futures
import gzip
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

sys.dont_write_bytecode = True
W = Path(__file__).resolve().parent
ROOT = W.parents[2]
E = ROOT / 'docs/evidence/2026-10-17-kicad-native-strokes'
PARENT = ROOT / 'docs/evidence/2026-10-17-native-stroke-completion'
PREFIX = str(E.relative_to(ROOT)) + '/'
sys.path.insert(0, str(E.parent / '2026-10-16-physical-build-requalification'))
from load_expectations_v1 import load

def sha(p):
    with Path(p).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def gz(p):
    with gzip.open(p, 'rt') as stream:
        return json.load(stream)

def span(raw):
    start = re.search(rb'"current_work"\s*:\s*', raw).end()
    value, n = json.JSONDecoder().raw_decode(raw[start:].decode())
    return start, start + len(raw[start:].decode()[:n].encode()), value

assert not (E / 'checkpoint-v1.json').exists()
assert not (E / 'artifact-expectations-v1.json.gz').exists()
previous_path = PARENT / 'checkpoint-v1.json'
previous = read(previous_path)
decision = read(E / 'decision-v1.json')
BASE = decision['main_parent_commit']
assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == BASE
subprocess.run(['git', 'diff', '--quiet'], cwd=ROOT, check=True)
subprocess.run(['git', 'diff', '--cached', '--quiet'], cwd=ROOT, check=True)
assert decision['implementation_sha256'] == {} and not decision['source_changes_applied']
assert previous['pending_native_sha256'] == {} and previous['native_disposition_preserved']
assert all(sha(ROOT / p) == h for p, h in previous['evidence_files_sha256'].items())
assert sha(ROOT / previous['artifact_expectations']) == previous['artifact_expectations_sha256']
prior, missing = load(ROOT / previous['artifact_expectations'], allow_unqualified=True)
assert len(prior) == 1135362 and len(missing) == 27093
inherited = previous['inherited_implementation_sha256'] | previous['implementation_sha256']
extra = previous['extra_unchanged_inputs_sha256']
dependencies = previous['live_source_dependency_pins_sha256']
live = previous['restored_native_sha256'] | inherited
assert all(sha(ROOT / p) == h for pins in [live, extra, dependencies] for p, h in pins.items())
assert all(not (ROOT / p).exists() for p in previous['removed_added_paths'] if p not in inherited)
assert all(sha(ROOT / previous['candidate_snapshot'] / p) == h for p, h in previous['rejected_native_sha256'].items())
baseline = gz(E / 'reproduction-v1/source-hashes.json.gz')
assert len(baseline) == 2835
assert all(sha(ROOT / p) == sha(W / 'baseline-source-v1' / p) == h for p, h in baseline.items())
for version in [1, 3, 4]:
    manifest = gz(E / f'candidate-preparation-v{version}/source-hashes.json.gz')
    assert all(sha(W / f'candidate-source-v{version}' / p) == h for p, h in manifest.items())
for version, changed in decision['candidate_source_changes_sha256'].items():
    assert all(sha(E / f'candidate-changes-v{version}' / p) == sha(W / f'candidate-source-v{version}' / p) == h
               for p, h in changed.items())
for name, row in decision['retained_completed_evidence'].items():
    assert sha(E / name) == sha(row['original']) == row['sha256']
for row in decision['physical_artifacts_not_duplicated_in_git']:
    assert sha(row['path']) == row['sha256']
for p, h in read(E / 'candidate-v3-cancellation-v1/result.json')['terminal_validation_files_sha256'].items():
    assert sha(p) == h
assert read(E / 'candidate-validation-v4/result.json')['unexpected_exits'] == ['full-board', 'copper-neck', 'clippy']
assert read(E / 'terminal-observations-v2.json')['candidate_v4']['cargo_target_closed']
shutil.copy2(__file__, E / Path(__file__).name)

with (E / 'main-format.stdout').open('wb') as out, (E / 'main-format.stderr').open('wb') as err:
    result = subprocess.run(['cargo', 'fmt', '--all', '--', '--check'], cwd=ROOT, stdout=out, stderr=err)
assert result.returncode == 0
for p in E.rglob('*.py'):
    ast.parse(p.read_text(), filename=str(p))
ledger = ROOT / 'docs/properties-status.json'
raw = ledger.read_bytes()
before = sha(ledger)
assert before == previous['ledger_after_sha256']
reference = subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT)
start, end, current = span(raw)
rs, re_, _ = span(reference)
assert (raw[:start], raw[end:]) == (reference[:rs], reference[re_:])

physical_roots = [W, Path('/tmp/csgrs-kicad-native-target-v4')]
paths, links = [], {}
for root in physical_roots:
    assert root.is_dir()
    for p in root.rglob('*'):
        assert not p.is_mount(), p
        if p.is_symlink():
            target = p.resolve(strict=True)
            assert target.is_file() and any(target.is_relative_to(r) for r in physical_roots), p
            links[str(p)] = {'link_text': os.readlink(p), 'resolved_target': str(target)}
        if p.is_file():
            paths.append(p)

def inspect(p):
    before_stat, before_link = p.stat(), p.lstat()
    digest = sha(p)
    after_stat, after_link = p.stat(), p.lstat()
    for a, b in [(before_stat, after_stat), (before_link, after_link)]:
        assert all(getattr(a, k) == getattr(b, k) for k in ['st_dev', 'st_ino', 'st_size', 'st_mtime_ns', 'st_ctime_ns']), p
    if str(p) in links:
        assert os.readlink(p) == links[str(p)]['link_text']
        assert str(p.resolve(strict=True)) == links[str(p)]['resolved_target']
    return str(p), digest

print('Hashing every completed physical path:', len(paths), flush=True)
with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
    physical = dict(pool.map(inspect, paths, buffersize=128))
assert len(physical) == len(paths) and prior.keys().isdisjoint(physical)
expectations = {
    'schema_version': 3, 'kind': 'rejected exact KiCad ingress experiment; inherited artifact audit incomplete',
    'parent_expectations': previous['artifact_expectations'], 'parent_expectations_sha256': previous['artifact_expectations_sha256'],
    'inherited_expected_artifact_paths': len(prior), 'inherited_missing_artifact_paths': len(missing),
    'added_physical_artifacts_sha256': physical, 'new_physical_artifacts_verified': len(physical),
    'local_tool_symlink_targets': links, 'physical_roots': [str(p) for p in physical_roots],
    'expected_artifact_paths': len(prior) + len(physical), 'inherited_artifacts_reverified': False,
    'complete_artifact_audit_passed': False, 'historical_preservation_complete': False,
    'scope': 'Each new physical path hashed independently with stable stat/lstat, including rejected snapshots, closed targets, failed attempts and raw observations. No inode deduplication or whole-history/zero-loss claim.'
}
with gzip.open(E / 'artifact-expectations-v1.json.gz', 'wt') as stream:
    json.dump(expectations, stream, sort_keys=True)
current['updated_on'] = '2026-10-17'
current['kicad_native_strokes'] = {
    'status': decision['status'], 'main_parent_commit': BASE,
    'contract': PREFIX + 'contract-v1.md', 'decision': PREFIX + 'decision-v1.json',
    'progress': PREFIX + 'progress-v1.md', 'checkpoint': PREFIX + 'checkpoint-v1.json',
    'artifact_expectations': PREFIX + 'artifact-expectations-v1.json.gz',
    'implementation_sha256': {}, 'source_changes_applied': False,
    'original_archived_regressions': decision['original_archived_regressions'],
    'final_candidate_test_results': decision['final_candidate_test_results'],
    'full_board_resource_gate': decision['full_board_resource_gate'],
    'copper_neck_resource_gate': decision['copper_neck_resource_gate'],
    'report_counts': decision['report_counts'], 'original_workload_gates_unchanged': True,
    'native_disposition_preserved': True, 'pending_native_paths_remaining': 0,
    'full_archived_integration_admitted': False, 'native_runtime_admitted': False,
    'full_plan_complete': False, 'limitations': decision['limitations'], 'next': decision['next']
}
updated = raw[:start] + json.dumps(current, indent=2).encode() + raw[end:]
us, ue, _ = span(updated)
assert (updated[:us], updated[ue:]) == (reference[:rs], reference[re_:])
assert {k: v for k, v in json.loads(updated).items() if k != 'current_work'} == {
    k: v for k, v in json.loads(reference).items() if k != 'current_work'}
assert sha(ledger) == before
ledger.write_bytes(updated)
files = {str(p.relative_to(ROOT)): sha(p) for p in E.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
record = {
    'status': decision['status'], 'checkpoint_kind': 'rejection of exact KiCad source ingress prototype',
    'previous_closure': str(previous_path.relative_to(ROOT)), 'previous_closure_sha256': sha(previous_path),
    'evidence_files_sha256': files, 'artifact_expectations': PREFIX + 'artifact-expectations-v1.json.gz',
    'artifact_expectations_sha256': sha(E / 'artifact-expectations-v1.json.gz'), 'artifact_expectations_schema': 3,
    'new_physical_artifacts_verified': len(physical), 'inherited_expected_artifact_paths': len(prior),
    'inherited_missing_artifact_paths': len(missing), 'complete_artifact_audit_passed': False,
    'historical_preservation_complete': False, 'implementation_sha256': {}, 'inherited_implementation_sha256': inherited,
    'extra_unchanged_inputs_sha256': extra, 'pending_native_sha256': {},
    'rejected_native_sha256': previous['rejected_native_sha256'], 'restored_native_sha256': previous['restored_native_sha256'],
    'removed_added_paths': previous['removed_added_paths'], 'candidate_snapshot': previous['candidate_snapshot'],
    'native_disposition_pin_scope': previous['native_disposition_pin_scope'],
    'live_native_disposition_overrides_sha256': previous['live_native_disposition_overrides_sha256'],
    'independently_reimplemented_archived_paths_sha256': previous['independently_reimplemented_archived_paths_sha256'],
    'independently_admitted_archived_components_sha256': previous['independently_admitted_archived_components_sha256'],
    'live_source_dependency_pins_sha256': dependencies, 'ledger_before_sha256': before,
    'ledger_after_sha256': sha(ledger), 'only_current_work_changed': True, 'python_sources_parse': True,
    'format_exit_code': result.returncode, 'native_disposition_preserved': True,
    'single_line_kernel_admitted': True, 'typed_text_consumer_migration': True,
    'general_exact_kicad_stroke_fidelity_claimed': False, 'source_changes_applied': False,
    'native_candidate_admitted': False, 'native_runtime_admitted': False,
    'full_archived_integration_admitted': False, 'full_plan_complete': False
}
(E / 'checkpoint-v1.json').write_text(json.dumps(record, indent=2, sort_keys=True) + '\n')
print(json.dumps({'new_physical_artifacts_verified': len(physical), 'only_current_work_changed': True,
                  'source_changes_applied': False, 'inherited_missing_artifact_paths': len(missing)}), flush=True)
