"""Retain full-project attribution and advance only the ledger's current_work."""
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
E = ROOT / 'docs/evidence/2026-10-17-native-track-cost-attribution'
PARENT = ROOT / 'docs/evidence/2026-10-17-kicad-native-strokes'
PREFIX = str(E.relative_to(ROOT)) + '/'
sys.path.insert(0, str(E.parent / '2026-10-16-physical-build-requalification'))
from load_expectations_v1 import load

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def dump(p, value):
    Path(p).write_text(json.dumps(value, indent=2, sort_keys=True) + '\n')

def span(raw):
    start = re.search(rb'"current_work"\s*:\s*', raw).end()
    value, n = json.JSONDecoder().raw_decode(raw[start:].decode())
    return start, start + len(raw[start:].decode()[:n].encode()), value

assert not E.exists()
decision = read(W / 'decision-v1.json')
BASE = decision['main_parent_commit']
assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == BASE
subprocess.run(['git', 'diff', '--quiet'], cwd=ROOT, check=True)
subprocess.run(['git', 'diff', '--cached', '--quiet'], cwd=ROOT, check=True)
assert decision['implementation_sha256'] == {} and not decision['source_changes_applied']
previous_path = PARENT / 'checkpoint-v1.json'
previous = read(previous_path)
assert previous['pending_native_sha256'] == {} and previous['native_disposition_preserved']
assert all(sha(ROOT / p) == h for p, h in previous['evidence_files_sha256'].items())
assert sha(ROOT / previous['artifact_expectations']) == previous['artifact_expectations_sha256']
prior, missing = load(ROOT / previous['artifact_expectations'], allow_unqualified=True)
assert len(prior) == 1157081 and len(missing) == 27093
inherited = previous['inherited_implementation_sha256'] | previous['implementation_sha256']
extra = previous['extra_unchanged_inputs_sha256']
dependencies = previous['live_source_dependency_pins_sha256']
live = previous['restored_native_sha256'] | inherited
assert all(sha(ROOT / p) == h for pins in [live, extra, dependencies] for p, h in pins.items())
assert all(not (ROOT / p).exists() for p in previous['removed_added_paths'] if p not in inherited)
assert all(sha(ROOT / previous['candidate_snapshot'] / p) == h for p, h in previous['rejected_native_sha256'].items())
protocol = read(W / 'stage-observations-v1/protocol.json')
result = read(W / 'stage-observations-v1/result.json')
assert result['complete'] and result['command_failures'] == []
assert all(sha(p) == h for p, h in protocol['read_only_sha256'].items())
assert all(sha(p) == h for p, h in result['compiled_binary_sha256'].items())
for source in protocol['sources'].values():
    assert all(sha(Path(source['source']) / p) == h for p, h in source['inputs'].items())
baseline = protocol['sources']['baseline']['inputs']
differences = {p: {'baseline': h, 'main': sha(ROOT / p)} for p, h in baseline.items() if sha(ROOT / p) != h}
allowed = {'docs/properties-status.json'} | {
    p for p in inherited if p.startswith('crates/hypercircuit/tests/fixtures/') and p.endswith('.json')
    and p in differences}
assert set(differences) == allowed and len(differences) == 8
assert all(p == 'docs/properties-status.json' or sha(ROOT / p) == inherited[p] for p in differences)
for commands in W.glob('*/commands.json'):
    for row in read(commands):
        for suffix in ['stdout', 'stderr', 'resources']:
            assert sha(commands.parent / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']

E.mkdir()
retained, physical_only = {}, []
for p in sorted(W.rglob('*')):
    if not p.is_file() or '__pycache__' in p.parts:
        continue
    rel = p.relative_to(W)
    if 'projects' in rel.parts:
        continue
    with p.open('rb') as f:
        magic = f.read(4)
    if magic == b'\x7fELF' or p.name == 'perf.data':
        physical_only.append({'path': str(p), 'sha256': sha(p), 'bytes': p.stat().st_size})
        continue
    assert p.stat().st_size < 50_000_000
    dest = E / rel
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(p, dest)
    assert sha(dest) == sha(p)
    retained[str(rel)] = {'original': str(p), 'sha256': sha(p)}
dump(E / 'source-equivalence-v1.json', {
    'complete': True, 'baseline_source_inputs': len(baseline), 'main_differences': differences,
    'native_implementation_inputs_byte_identical': True,
    'scope': 'The reused E35 compiled baseline differs from current main only in the ledger and seven previously admitted Hypercircuit expected JSON artifacts. None is a HyperDRC implementation input.'})
with (E / 'main-format.stdout').open('wb') as out, (E / 'main-format.stderr').open('wb') as err:
    fmt = subprocess.run(['cargo', 'fmt', '--all', '--', '--check'], cwd=ROOT, stdout=out, stderr=err)
assert fmt.returncode == 0
for p in E.rglob('*.py'):
    ast.parse(p.read_text(), filename=str(p))
dump(E / 'retention-v1.json', {'complete': True, 'retained_files': retained,
    'physical_artifacts_not_duplicated_in_git': physical_only, 'source_changes_applied': False,
    'runner_sha256': sha(__file__), 'format_exit_code': fmt.returncode})

ledger = ROOT / 'docs/properties-status.json'
raw, before = ledger.read_bytes(), sha(ledger)
assert before == previous['ledger_after_sha256']
reference = subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT)
start, end, current = span(raw)
rs, re_, _ = span(reference)
assert (raw[:start], raw[end:]) == (reference[:rs], reference[re_:])
paths, links = [], {}
for p in W.rglob('*'):
    assert not p.is_mount()
    if p.is_symlink():
        target = p.resolve(strict=True)
        assert target.is_file() and target.is_relative_to(W)
        links[str(p)] = {'link_text': os.readlink(p), 'resolved_target': str(target)}
    if p.is_file():
        paths.append(p)

def inspect(p):
    before_stat, before_link = p.stat(), p.lstat()
    digest = sha(p)
    after_stat, after_link = p.stat(), p.lstat()
    for a, b in [(before_stat, after_stat), (before_link, after_link)]:
        assert all(getattr(a, k) == getattr(b, k) for k in ['st_dev', 'st_ino', 'st_size', 'st_mtime_ns', 'st_ctime_ns'])
    if str(p) in links:
        assert os.readlink(p) == links[str(p)]['link_text']
        assert str(p.resolve(strict=True)) == links[str(p)]['resolved_target']
    return str(p), digest

print('Hashing every completed physical path:', len(paths), flush=True)
with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
    physical = dict(pool.map(inspect, paths, buffersize=128))
assert len(physical) == len(paths) and prior.keys().isdisjoint(physical)
expectations = {
    'schema_version': 3, 'kind': 'full-project native track cost attribution; inherited audit incomplete',
    'parent_expectations': previous['artifact_expectations'], 'parent_expectations_sha256': previous['artifact_expectations_sha256'],
    'inherited_expected_artifact_paths': len(prior), 'inherited_missing_artifact_paths': len(missing),
    'added_physical_artifacts_sha256': physical, 'new_physical_artifacts_verified': len(physical),
    'local_tool_symlink_targets': links, 'physical_roots': [str(W)],
    'expected_artifact_paths': len(prior) + len(physical), 'inherited_artifacts_reverified': False,
    'complete_artifact_audit_passed': False, 'historical_preservation_complete': False,
    'scope': 'Every new physical path independently hashed with stable stat/lstat, including full extracted projects, compiled diagnostic executables, failed observers and raw profiles. No inode deduplication or whole-history/zero-loss claim.'
}
with gzip.open(E / 'artifact-expectations-v1.json.gz', 'wt') as stream:
    json.dump(expectations, stream, sort_keys=True)
current['updated_on'] = '2026-10-17'
current['native_track_cost_attribution'] = {
    'status': decision['status'], 'main_parent_commit': BASE,
    'contract': PREFIX + 'contract-v1.md', 'decision': PREFIX + 'decision-v1.json',
    'progress': PREFIX + 'progress-v1.md', 'checkpoint': PREFIX + 'checkpoint-v1.json',
    'artifact_expectations': PREFIX + 'artifact-expectations-v1.json.gz',
    'implementation_sha256': {}, 'source_changes_applied': False,
    'completed_diagnostic_commands': 6, 'full_project_observations': 4,
    'owning_layer_findings': decision['owning_layer_findings'],
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
    'status': decision['status'], 'checkpoint_kind': 'read-only full-project attribution after rejected exact ingress',
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
    'format_exit_code': fmt.returncode, 'native_disposition_preserved': True,
    'single_line_kernel_admitted': True, 'typed_text_consumer_migration': True,
    'general_exact_kicad_stroke_fidelity_claimed': False, 'source_changes_applied': False,
    'native_candidate_admitted': False, 'native_runtime_admitted': False,
    'full_archived_integration_admitted': False, 'full_plan_complete': False
}
dump(E / 'checkpoint-v1.json', record)
print(json.dumps({'new_physical_artifacts_verified': len(physical), 'only_current_work_changed': True,
                  'source_changes_applied': False, 'inherited_missing_artifact_paths': len(missing)}), flush=True)
