"""Close the rejected small-hit evaluation with a verified compact archive."""
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
import tarfile
from pathlib import Path

sys.dont_write_bytecode = True
W = Path(__file__).resolve().parent
ROOT = W.parents[2]
R = Path('/dev/shm/csgrs-circle-support-small-hit-v1')
M = Path('/run/user/1000/csgrs-circle-support-small-hit-metadata-v1')
E = ROOT / 'docs/evidence/2026-10-17-circle-support-small-hit'
PARENT = ROOT / 'docs/evidence/2026-10-17-circle-support-lazy-index'
PREFIX = str(E.relative_to(ROOT)) + '/'
sys.path.insert(0, str(E.parent / '2026-10-16-physical-build-requalification'))
from load_expectations_v1 import load


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True) + '\n')


def span(raw):
    start = re.search(rb'"current_work"\s*:\s*', raw).end()
    value, count = json.JSONDecoder().raw_decode(raw[start:].decode())
    return start, start + len(raw[start:].decode()[:count].encode()), value


def check_reserve():
    fs = os.statvfs(ROOT)
    assert fs.f_bavail * fs.f_frsize >= 512 * 1024**2


decision = read(W / 'decision-v1.json')
base = decision['main_parent_commit']
assert not E.exists() and decision['implementation_sha256'] == {}
assert decision['retention_decision'] == 'rejected' and not decision['cache_index_admitted']
assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == base
subprocess.run(['git', 'diff', '--quiet'], cwd=ROOT, check=True)
subprocess.run(['git', 'diff', '--cached', '--quiet'], cwd=ROOT, check=True)
check_reserve()
previous_path = PARENT / 'checkpoint-v1.json'
previous = read(previous_path)
assert all(sha(ROOT / path) == digest for path, digest in previous['evidence_files_sha256'].items())
assert sha(ROOT / previous['artifact_expectations']) == previous['artifact_expectations_sha256']
prior, missing = load(ROOT / previous['artifact_expectations'], allow_unqualified=True)
assert len(prior) == 1223480 and len(missing) == 27093
inherited = previous['inherited_implementation_sha256'] | previous['implementation_sha256']
extra = previous['extra_unchanged_inputs_sha256']
dependencies = previous['live_source_dependency_pins_sha256']
live = previous['restored_native_sha256'] | inherited
assert all(sha(ROOT / path) == digest for pins in [live, extra, dependencies] for path, digest in pins.items())
with gzip.open(R / 'build-v1/main-input-hashes.json.gz', 'rt') as stream:
    main = json.load(stream)
assert all(sha(ROOT / path) == digest for path, digest in main.items())
names = ['reproduction-v1', 'build-v1', 'cost-preflight-v1', 'cost-matched-v1', 'proposal-v1', 'collection-v1']
for output in [*(R / name for name in names), M]:
    result = read(output / 'result.json')
    assert result['complete'] and result['output_closed']
    assert not result.get('command_failures', result.get('failures', []))
    if (output / 'commands.json').exists():
        for row in read(output / 'commands.json'):
            assert row['exit_code'] == 0 and row['external_stop_reason'] is None
            for suffix in ['stdout', 'stderr', 'resources']:
                assert sha(output / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
    protocol = read(output / 'protocol.json') if (output / 'protocol.json').exists() else {}
    for record in [result, protocol]:
        for key in ['input_sha256', 'read_only_dependencies_sha256']:
            assert all(sha(path) == digest for path, digest in record.get(key, {}).items())
    for source in protocol.get('sources', {}).values():
        assert all(sha(Path(source['path']) / path) == digest for path, digest in source['sha256'].items())
    if isinstance(protocol.get('source'), dict):
        source = protocol['source']
        assert all(sha(Path(source['path']) / path) == digest for path, digest in source['sha256'].items())
    assert all(sha(ROOT / path) == digest for path, digest in protocol.get('dependency_source_sha256', {}).items())
    for key in ['runtime_source_mirror_sha256', 'source_mirror_sha256']:
        assert all(sha(output / 'runtime-source-mirror' / path) == digest
                   for path, digest in protocol.get(key, {}).items())
assert decision['focused_contract_tests_per_variant'] == 9
assert len(decision['decisive_rejection_cases']) == 1
assert decision['decisive_rejection_cases'][0]['predeclared_rejection_criterion_met']
assert not (R / 'support-cost-preflight-v1').exists()
proposal = read(R / 'proposal-v1/result.json')
assert all(sha(Path(proposal['source']) / path) == digest for path, digest in proposal['proposal_sha256'].items())
assert all(sha(path) == digest for path, digest in decision['input_sha256'].items())
collection = read(R / 'collection-v1/result.json')
assert collection['all_member_bytes_independently_verified'] and collection['original_files_unchanged']
assert sha(collection['archive']) == collection['archive_sha256']
assert sha(collection['archive_members']) == collection['archive_members_sha256']
with gzip.open(collection['archive_members'], 'rt') as stream:
    members = json.load(stream)
verified = set()
with tarfile.open(collection['archive'], 'r:xz') as archive:
    for info in archive:
        assert info.isfile() and info.name in members and info.name not in verified
        item = members[info.name]
        assert info.size == item['bytes']
        with archive.extractfile(info) as stream:
            assert hashlib.file_digest(stream, 'sha256').hexdigest() == item['sha256']
        assert sha(item['original']) == item['sha256']
        verified.add(info.name)
assert verified == set(members) and len(members) == collection['member_count']
for path in W.glob('*.py'):
    ast.parse(path.read_text(), filename=str(path))

E.mkdir()
copied = {}
for original, relative in [
    (Path(collection['archive']), 'raw-evidence-v1.tar.xz'),
    (Path(collection['archive_members']), 'archive-members-v1.json.gz'),
    (R / 'collection-v1/result.json', 'collection-v1.json'),
    *[(W / name, name) for name in ['contract-v1.md', 'metadata-contract-v1.md', 'executable-contract-v1.md',
                                   'library-contract-v1.md', 'shared-aggregate-contract-v1.md',
                                   'progress-v1.md', 'decision-v1.json', 'checkpoint_v1.py']],
    *[(R / 'proposal-v1' / name, 'proposal/' + name) for name in [
        'curve_intersection.rs.patch', 'circle_support_cache_tests.rs.patch', 'README.md.patch']],
]:
    dest = E / relative
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(original, dest)
    assert sha(original) == sha(dest)
    copied[relative] = {'original': str(original), 'sha256': sha(dest)}
(E / 'README.md').write_text('# Direct small-cache-hit evidence\n\nThis proposal is **rejected** by the preserved cold-batch cost gate. Read the\n[decision](decision-v1.json), [evaluation](progress-v1.md) and\n[contract](contract-v1.md). The reviewable patches are under `proposal/`.\nNo production cache change is applied.\n\nThe [lossless archive](raw-evidence-v1.tar.xz) retains helpers and preparation\nunder `work/`, raw executable checks and measurements under `run/`, and the\nseparate metadata checks under `metadata/`. The\n[member index](archive-members-v1.json.gz) records each original path, byte\nlength and SHA-256. Every archived byte stream was verified independently.\nList entries with `tar -tJf raw-evidence-v1.tar.xz`.\n\nPrepared helper files do not establish execution; the decision identifies\nunexecuted checks and the refused support preflight. Original snapshots,\nbinaries and raw artifacts remain physically retained. The\n[checkpoint](checkpoint-v1.json) discloses the inherited missing paths.\n')
ledger = ROOT / 'docs/properties-status.json'
raw, before = ledger.read_bytes(), sha(ledger)
assert before == previous['ledger_after_sha256']
reference = subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT)
start, end, current = span(raw)
rs, re_, _ = span(reference)
assert (raw[:start], raw[end:]) == (reference[:rs], reference[re_:])
old_current = json.loads(json.dumps(current))
current['circle_support_small_hit'] = {
    key: value for key, value in decision.items() if key != 'input_sha256'}
current['circle_support_small_hit'].update({
    'contract': PREFIX + 'contract-v1.md', 'decision': PREFIX + 'decision-v1.json',
    'progress': PREFIX + 'progress-v1.md', 'checkpoint': PREFIX + 'checkpoint-v1.json',
    'artifact_expectations': PREFIX + 'artifact-expectations-v1.json.gz',
    'raw_evidence_archive': PREFIX + 'raw-evidence-v1.tar.xz'})
assert [key for key in old_current.keys() | current.keys() if old_current.get(key) != current.get(key)] == ['circle_support_small_hit']
updated = raw[:start] + json.dumps(current, indent=2).encode() + raw[end:]
us, ue, _ = span(updated)
assert (updated[:us], updated[ue:]) == (reference[:rs], reference[re_:])
assert {key: value for key, value in json.loads(updated).items() if key != 'current_work'} == {
    key: value for key, value in json.loads(reference).items() if key != 'current_work'}
ledger.write_bytes(updated)
with (E / 'main-format.stdout').open('wb') as out, (E / 'main-format.stderr').open('wb') as err:
    fmt = subprocess.run(['cargo', 'fmt', '--all', '--', '--check'], cwd=ROOT, stdout=out, stderr=err)
assert fmt.returncode == 0
assert all(sha(ROOT / path) == digest for path, digest in main.items() if path != 'docs/properties-status.json')
assert all(sha(ROOT / path) == digest for pins in [live, extra, dependencies] for path, digest in pins.items())
dump(E / 'retention-v1.json', {
    'complete': True, 'main_parent_commit': base, 'source_changes_applied': False,
    'cache_index_applied': False, 'retention_decision': 'rejected',
    'main_sources_unchanged': True, 'source_inputs': len(main),
    'copied_evidence_files': copied, 'archive_members_verified': len(members),
    'all_original_physical_files_unchanged': True, 'all_new_output_roots_closed': True,
    'format_exit_code': fmt.returncode, 'python_sources_parse': True,
    'only_current_work_changed': True, 'full_plan_complete': False,
})
assert sum(p.stat().st_size for root in [W, E] for p in root.rglob('*') if p.is_file()) < 4 * 1024**2
roots, paths, directories, links = [W, M, R], [], [], {}
for root in roots:
    assert root.exists() and not root.is_mount()
    for path in [root, *root.rglob('*')]:
        if path.is_dir():
            directories.append(str(path))
        if path.is_symlink():
            target = path.resolve(strict=True)
            assert target.is_file()
            links[str(path)] = {'link_text': os.readlink(path), 'resolved_target': str(target)}
        if path.is_file():
            paths.append(path)
assert len({str(path) for path in paths}) == len(paths)


def inspect(path):
    before_stat, before_link = path.stat(), path.lstat()
    digest = sha(path)
    after_stat, after_link = path.stat(), path.lstat()
    for a, b in [(before_stat, after_stat), (before_link, after_link)]:
        assert all(getattr(a, key) == getattr(b, key)
                   for key in ['st_dev', 'st_ino', 'st_size', 'st_mtime_ns', 'st_ctime_ns'])
    if str(path) in links:
        assert os.readlink(path) == links[str(path)]['link_text']
        assert str(path.resolve(strict=True)) == links[str(path)]['resolved_target']
    return str(path), digest


print('Hashing every completed physical path:', len(paths), flush=True)
with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
    physical = dict(pool.map(inspect, paths, buffersize=128))
assert len(physical) == len(paths) and prior.keys().isdisjoint(physical)
expectations = {
    'schema_version': 3, 'kind': 'direct small-hit circle-support index rejected on cold activation cost',
    'parent_expectations': previous['artifact_expectations'],
    'parent_expectations_sha256': previous['artifact_expectations_sha256'],
    'inherited_expected_artifact_paths': len(prior), 'inherited_missing_artifact_paths': len(missing),
    'added_physical_artifacts_sha256': physical, 'new_physical_artifacts_verified': len(physical),
    'local_tool_symlink_targets': links, 'physical_roots': [str(path) for path in roots],
    'directory_inventory': directories, 'expected_artifact_paths': len(prior) + len(physical),
    'inherited_artifacts_reverified': False, 'complete_artifact_audit_passed': False,
    'historical_preservation_complete': False,
    'scope': 'Every new final physical file path independently hashed with stable stat/lstat and no inode deduplication. All original logs, snapshots, executables and allocation traces remain retained. A separately verified lossless archive supplies compact Git copies of raw evidence; source mirrors and executables are not duplicated in Git.',
}
with gzip.open(E / 'artifact-expectations-v1.json.gz', 'wt') as stream:
    json.dump(expectations, stream, sort_keys=True)
files = {str(path.relative_to(ROOT)): sha(path) for path in E.rglob('*') if path.is_file()}
record = {key: previous[key] for key in [
    'rejected_native_sha256', 'restored_native_sha256', 'removed_added_paths', 'candidate_snapshot',
    'native_disposition_pin_scope', 'live_native_disposition_overrides_sha256',
    'independently_reimplemented_archived_paths_sha256', 'independently_admitted_archived_components_sha256',
]}
record.update({
    'status': decision['status'], 'checkpoint_kind': 'small-hit cost rejection; no production change',
    'previous_closure': str(previous_path.relative_to(ROOT)), 'previous_closure_sha256': sha(previous_path),
    'evidence_files_sha256': files, 'artifact_expectations': PREFIX + 'artifact-expectations-v1.json.gz',
    'artifact_expectations_sha256': sha(E / 'artifact-expectations-v1.json.gz'), 'artifact_expectations_schema': 3,
    'new_physical_artifacts_verified': len(physical), 'inherited_expected_artifact_paths': len(prior),
    'inherited_missing_artifact_paths': len(missing), 'complete_artifact_audit_passed': False,
    'historical_preservation_complete': False, 'implementation_sha256': {},
    'inherited_implementation_sha256': inherited, 'extra_unchanged_inputs_sha256': extra,
    'pending_native_sha256': {}, 'live_source_dependency_pins_sha256': dependencies,
    'ledger_before_sha256': before, 'ledger_after_sha256': sha(ledger), 'only_current_work_changed': True,
    'python_sources_parse': True, 'format_exit_code': fmt.returncode,
    'native_disposition_preserved': True, 'source_changes_applied': False, 'cache_index_admitted': False,
    'cache_retention_decision': 'rejected', 'production_prefixes_unchanged': True,
    'single_line_kernel_admitted': True, 'typed_text_consumer_migration': True,
    'general_exact_kicad_stroke_fidelity_claimed': False, 'native_candidate_admitted': False,
    'native_runtime_admitted': False, 'full_archived_integration_admitted': False, 'full_plan_complete': False,
})
dump(E / 'checkpoint-v1.json', record)
check_reserve()
print(json.dumps({'new_physical_artifacts_verified': len(physical), 'only_current_work_changed': True,
    'cache_index_applied': False, 'cache_retention_decision': 'rejected',
    'inherited_missing_artifact_paths': len(missing), 'evidence_files_pinned': len(files)}), flush=True)
