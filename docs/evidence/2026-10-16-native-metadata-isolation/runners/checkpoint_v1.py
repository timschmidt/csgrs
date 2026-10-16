"""Preserve the isolated metadata trial and every inherited artifact expectation."""
import concurrent.futures
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import subprocess
import sys
import threading

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parents[1]
ROOT = E.parents[2]
OLD = E.parent / '2026-10-16-native-admission-recheck'
BASE_COMMIT = '73472622dd4034b8b8777cb2a116c4020f6bb328'
PREFIX = str(E.relative_to(ROOT)) + '/'
sys.path.insert(0, str(E.parent / '2026-10-16-polyhedral-construction'))
from load_artifact_audit_v1 import load
memo = {}
lock = threading.Lock()


def sha(path):
    path = Path(path)
    before = path.stat()
    key = (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns)
    with lock:
        value = memo.get(key)
    if value is None:
        with path.open('rb') as stream:
            value = hashlib.file_digest(stream, 'sha256').hexdigest()
        after = path.stat()
        assert key == (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns), path
        with lock:
            memo[key] = value
    return value


def gz(path):
    with gzip.open(path, 'rt') as stream:
        return json.load(stream)


def span(raw):
    start = re.search(r'"current_work"\s*:\s*', raw).end()
    value, length = json.JSONDecoder().raw_decode(raw[start:])
    return start, start + length, value


assert not (E / 'checkpoint-v1.json').exists()
assert not (E / 'artifact-audit-v1.json.gz').exists()
assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == BASE_COMMIT
subprocess.run(['git', 'diff', '--cached', '--quiet'], cwd=ROOT, check=True)
previous_path = OLD / 'checkpoint-v1.json'
previous = json.loads(previous_path.read_text())
assert all(sha(ROOT / p) == h for p, h in previous['evidence_files_sha256'].items())
assert sha(ROOT / previous['artifact_audit']) == previous['artifact_audit_sha256']
prior, inherited_missing = load(ROOT / previous['artifact_audit'])
assert len(prior) == 835959 and len(inherited_missing) == 27093
pending = previous['pending_native_sha256']
assert len(pending) == 35 and all(sha(ROOT / p) == h for p, h in pending.items())
inherited = previous['inherited_implementation_sha256'] | previous['implementation_sha256']
extra_inputs = previous['extra_unchanged_inputs_sha256']
dependency_pins = previous['live_source_dependency_pins_sha256']
assert all(sha(ROOT / p) == h for pins in [inherited, extra_inputs, dependency_pins] for p, h in pins.items())
ledger = ROOT / 'docs/properties-status.json'
raw = ledger.read_text()
before = sha(ledger)
assert before == previous['ledger_after_sha256']
reference = subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT, text=True)
start, end, work = span(raw)
rstart, rend, _ = span(reference)
assert (raw[:start], raw[end:]) == (reference[:rstart], reference[rend:])
summary = json.loads((E / 'decision-v1.json').read_text())
assert summary['production_or_test_edits'] == 0
expected = dict(prior)


def retain(name, digest):
    if name in expected:
        assert expected[name] == digest, name
    expected[name] = digest


base_runner = E.parent / '2026-10-15-factored-product-proofs/validate_v8.py'
assert sha(base_runner) == 'f3ee76657ed7c1e8792452ded7c2380c1ad03a41abb312d80f9adf68b4dc6f05'
wrappers = {sha(E / 'validate_v1.py')}
targets = {Path('/dev/shm/csgrs-native-metadata-isolation')}
protected_sources = {}
for name, phase in summary['phases'].items():
    folder = E / name
    protocol = json.loads((folder / 'protocol.json').read_text())
    assert protocol['runner_sha256'] == sha(base_runner)
    assert protocol['wrapper_sha256'] in wrappers and protocol['base_commit'] == BASE_COMMIT
    specification = json.loads((E / (name + '.json')).read_text())
    assert protocol['commands'] == specification
    for path, digest in protocol['preserved_prior_executables_sha256'].items():
        retain(path, digest)
    snapshot = gz(folder / 'source-hashes.json.gz')
    for path, digest in snapshot.items():
        if path in protected_sources:
            assert protected_sources[path] == digest, path
        protected_sources[path] = digest
    result = json.loads((folder / 'result.json').read_text())
    assert [r['name'] for r in result['commands']] == [r[0] for r in specification]
    assert phase['passed'] == result['passed']
    for row in result['commands']:
        assert row['sources_unchanged']
        for suffix in ['stdout', 'stderr', 'resources']:
            assert sha(folder / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
        for path, digest in row['binaries_sha256'].items():
            retain(path, digest)
    targets.add(ROOT / json.loads((E / (name + '-target.json')).read_text()))
    if (folder / 'overlay.json').exists():
        overlay = json.loads((folder / 'overlay.json').read_text())
        hashes = folder / 'overlay-lower-sha256.json.gz'
        assert sha(hashes) == overlay['lower_hashes_sha256']
        for path, digest in gz(hashes).items():
            retain(path, digest)
        assert Path(overlay['target']).is_mount()
assert all(sha(ROOT / path) == digest for path, digest in protected_sources.items())
spec = importlib.util.spec_from_file_location('source_inventory', base_runner)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
core = module.sources()
assert all(protected_sources.get(p) == h for p, h in core.items())
preparation = json.loads((E / 'preparation-v1.json').read_text())
assert preparation['head'] == BASE_COMMIT
assert sha(E / 'source-hashes-v1.json.gz') == preparation['source_hashes_sha256']
source_maps = gz(E / 'source-hashes-v1.json.gz')
assert core == source_maps['main_guard']
assert preparation['pending_native_sha256'] == pending
for variant, source in preparation['sources'].items():
    actual = {str(path.relative_to(source)): sha(path)
              for path in Path(source).rglob('*') if path.is_file()}
    assert actual == source_maps[variant]

cost = E / 'cost-v1'
cost_protocol = json.loads((cost / 'protocol.json').read_text())
assert sha(E / 'runners/cost_v1.py') == cost_protocol['runner_sha256']
assert sha(E / 'cost_probe.rs') == cost_protocol['probe_sha256']
assert sha(cost / 'source-hashes.json.gz') == cost_protocol['source_hashes_sha256']
cost_result = json.loads((cost / 'result.json').read_text())
assert cost_result['complete'] and cost_result['sources_unchanged']
assert cost_result['fresh_processes'] == 20 and cost_result['observations'] == 360
assert cost_result['all_expected_outcomes_passed']
for row in json.loads((cost / 'commands.json').read_text()):
    assert row['exit_code'] == 0
    for suffix in ['stdout', 'stderr', 'resources']:
        assert sha(cost / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
for variant, entries in gz(cost / 'source-hashes.json.gz').items():
    source = Path(cost_protocol['sources'][variant])
    assert {str(p.relative_to(source)): sha(p) for p in source.rglob('*') if p.is_file()} == entries
    target = Path(cost_protocol['targets'][variant])
    assert target.is_mount()
    targets.add(target)
    folder = E / ('cost-' + variant + '-v1')
    overlay = json.loads((folder / 'overlay.json').read_text())
    hashes = folder / 'overlay-lower-sha256.json.gz'
    assert sha(hashes) == overlay['lower_hashes_sha256']
    for path, digest in gz(hashes).items():
        retain(path, digest)
for binary in json.loads((cost / 'binaries.json').read_text()).values():
    assert sha(binary['path']) == binary['sha256']
    retain(binary['path'], binary['sha256'])

replay = E / 'report-replay-v1'
report_protocol = json.loads((replay / 'protocol.json').read_text())
assert sha(E / 'runners/replay_v1.py') == report_protocol['runner_sha256']
assert sha(replay / 'input-hashes.json.gz') == report_protocol['input_hashes_sha256']
for path, digest in gz(replay / 'input-hashes.json.gz').items():
    assert sha(path) == digest
    retain(path, digest)
for binary in report_protocol['binaries'].values():
    assert sha(binary['path']) == binary['sha256']
    retain(binary['path'], binary['sha256'])
report_result = json.loads((replay / 'result.json').read_text())
assert report_result['complete']
for row in json.loads((replay / 'commands.json').read_text()):
    assert row['sources_unchanged']
    for suffix in ['stderr', 'resources']:
        assert sha(replay / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
    assert sha(row['stdout_path']) == row['stdout_sha256']
    with gzip.open(replay / (row['name'] + '.json.gz'), 'rb') as stream:
        assert stream.read() == Path(row['stdout_path']).read_bytes()
    retain(row['stdout_path'], row['stdout_sha256'])
assert summary['report_replay'] == report_result
print('Discovering new immutable targets and retained diagnostics.', flush=True)
for target in sorted(targets):
    assert target.is_dir(), target
    for path in target.rglob('*'):
        assert not path.is_symlink(), path
        if path.is_file():
            retain(str(path), sha(path))
print('Auditing all inherited and new artifact paths with eight readers.', flush=True)


def verify(item):
    name, digest = item
    path = ROOT / name
    if not path.exists():
        return name, digest
    assert sha(path) == digest, name
    return None


missing = {}
with concurrent.futures.ThreadPoolExecutor(max_workers=8) as executor:
    for index, absent in enumerate(executor.map(verify, expected.items(), buffersize=128), 1):
        if absent:
            missing[absent[0]] = absent[1]
        if index % 100000 == 0:
            print('Verified expectations:', index, flush=True)
assert missing == inherited_missing
added = {name: digest for name, digest in expected.items() if name not in prior}
audit = {'schema_version': 2, 'parent_audit': previous['artifact_audit'],
         'parent_audit_sha256': previous['artifact_audit_sha256'], 'added_artifacts_sha256': added,
         'expected_artifact_paths': len(expected), 'verified_artifact_paths': len(expected) - len(missing),
         'missing_artifact_paths': len(missing), 'hash_mismatches': 0, 'new_missing_artifacts': 0,
         'historical_preservation_complete': False,
         'scope': 'Pinned parent plus disjoint additions. Every available expected path was verified; inherited missing expectations remain. Eight readers change only audit scheduling. All new source copies, FUSE targets, physical overlays, cost samples and report replays remain retained.'}
with gzip.open(E / 'artifact-audit-v1.json.gz', 'wt') as stream:
    json.dump(audit, stream, sort_keys=True)
assert load(E / 'artifact-audit-v1.json.gz') == (expected, missing)
work['native_metadata_isolation'] = {
    'status': summary['status'], 'baseline_commit': BASE_COMMIT,
    'contract': PREFIX + 'contract-v1.md', 'progress': PREFIX + 'progress-v1.md',
    'decision': PREFIX + 'decision-v1.json',
    'validation': PREFIX + 'qualification-assessment-v1.json',
    'cost': PREFIX + 'cost-assessment-v1.json',
    'reports': PREFIX + 'report-replay-v1/result.json',
    'checkpoint': PREFIX + 'checkpoint-v1.json',
    'artifact_audit': PREFIX + 'artifact-audit-v1.json.gz',
    'original_native_gates': summary['original_native_gates'],
    'production_or_test_edits': 0, 'pending_native_files_unchanged': 35,
    'decision_reason': summary['decision_reason'],
    'limitations': summary['limitations'], 'next': summary['next'],
    'historical_preservation': '27093 inherited missing paths retain their expected hashes; no new missing artifact or mismatch.',
    'runtime_admitted': False, 'full_plan_complete': False,
}
updated = raw[:start] + json.dumps(work, indent=2) + raw[end:]
ustart, uend, _ = span(updated)
assert (updated[:ustart], updated[uend:]) == (reference[:rstart], reference[rend:])
assert {k: v for k, v in json.loads(updated).items() if k != 'current_work'} == {
    k: v for k, v in json.loads(reference).items() if k != 'current_work'}
assert sha(ledger) == before
ledger.write_text(updated)
files = {str(path.relative_to(ROOT)): sha(path) for path in E.rglob('*')
         if path.is_file() and path.name != 'checkpoint-v1.json'}
record = {'status': summary['status'],
          'previous_closure': str(previous_path.relative_to(ROOT)), 'previous_closure_sha256': sha(previous_path),
          'evidence_files_sha256': files, 'artifact_audit': PREFIX + 'artifact-audit-v1.json.gz',
          'artifact_audit_sha256': sha(E / 'artifact-audit-v1.json.gz'), 'artifact_audit_schema': 2,
          'expected_artifact_paths': len(expected), 'verified_artifact_paths': len(expected) - len(missing),
          'missing_artifact_paths': len(missing), 'historical_preservation_complete': False,
          'implementation_sha256': {}, 'inherited_implementation_sha256': inherited,
          'extra_unchanged_inputs_sha256': extra_inputs, 'pending_native_sha256': pending,
          'live_source_dependency_pins_sha256': dependency_pins,
          'source_guard_scope': 'Main source inventory and all 35 pending native paths remain unchanged. Both qualification copies and both probe copies retain their complete pinned source maps. Used runners, raw results, post-phase assessments and closure files are separately pinned by this checkpoint.',
          'phases': summary['phases'], 'ledger_before_sha256': before, 'ledger_after_sha256': sha(ledger),
          'only_current_work_changed': True, 'runtime_admitted': False, 'full_plan_complete': False}
(E / 'checkpoint-v1.json').write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps({k: record[k] for k in ['expected_artifact_paths', 'verified_artifact_paths', 'missing_artifact_paths', 'runtime_admitted', 'full_plan_complete']}), flush=True)
