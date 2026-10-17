"""Close the guarded prototype evidence while keeping production unmodified."""
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
R = Path('/run/user/1000/csgrs-circle-support-guarded-v1')
E = ROOT / 'docs/evidence/2026-10-17-circle-support-guarded'
PARENT = ROOT / 'docs/evidence/2026-10-17-circle-support-consumers'
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
    assert fs.f_bavail * fs.f_frsize >= 768 * 1024**2


decision = read(W / 'decision-v1.json')
base = decision['main_parent_commit']
assert not E.exists() and decision['implementation_sha256'] == {}
assert decision['cache_index_admitted'] is False
assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == base
subprocess.run(['git', 'diff', '--quiet'], cwd=ROOT, check=True)
subprocess.run(['git', 'diff', '--cached', '--quiet'], cwd=ROOT, check=True)
check_reserve()
previous_path = PARENT / 'checkpoint-v1.json'
previous = read(previous_path)
assert all(sha(ROOT / path) == digest for path, digest in previous['evidence_files_sha256'].items())
assert sha(ROOT / previous['artifact_expectations']) == previous['artifact_expectations_sha256']
prior, missing = load(ROOT / previous['artifact_expectations'], allow_unqualified=True)
assert len(prior) == 1212316 and len(missing) == 27093
inherited = previous['inherited_implementation_sha256'] | previous['implementation_sha256']
extra = previous['extra_unchanged_inputs_sha256']
dependencies = previous['live_source_dependency_pins_sha256']
live = previous['restored_native_sha256'] | inherited
assert all(sha(ROOT / path) == digest for pins in [live, extra, dependencies] for path, digest in pins.items())
with gzip.open(R / 'build-v3/main-input-hashes.json.gz', 'rt') as stream:
    main = json.load(stream)
assert all(sha(ROOT / path) == digest for path, digest in main.items())
outputs = ['reproduction-v1', 'build-v1', 'focused-v1', 'build-v2', 'corpus-v1',
           'build-v3', 'corpus-v2', 'all-feature-library-v1', 'cost-preflight-v1',
           'cost-matched-v1', 'heap-v1', 'proposal-v1']
for name in outputs:
    output = R / name
    result = read(output / 'result.json')
    assert result['complete'] and result['output_closed']
    if (output / 'commands.json').exists():
        for row in read(output / 'commands.json'):
            for suffix in ['stdout', 'stderr', 'resources']:
                if suffix + '_sha256' in row:
                    assert sha(output / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
    if (output / 'protocol.json').exists():
        protocol = read(output / 'protocol.json')
        for key in ['input_sha256', 'read_only_dependencies_sha256']:
            assert all(sha(path) == digest for path, digest in protocol.get(key, {}).items())
        for source in protocol.get('sources', {}).values():
            assert all(sha(Path(source['path']) / path) == digest for path, digest in source['sha256'].items())
        if isinstance(protocol.get('source'), dict):
            source = protocol['source']
            assert all(sha(Path(source['path']) / path) == digest for path, digest in source['sha256'].items())
        assert all(sha(ROOT / path) == digest for path, digest in protocol.get('dependency_source_sha256', {}).items())
        for key in ['runtime_source_mirror_sha256', 'source_mirror_sha256']:
            assert all(sha(output / 'runtime-source-mirror' / path) == digest
                       for path, digest in protocol.get(key, {}).items())
assert read(R / 'focused-v1/result.json')['command_failures'] == ['candidate-clippy']
assert read(R / 'corpus-v1/result.json')['exit_code'] == -15
assert read(W / 'corpus-stop-v1.json')['requested_signal'] == 'SIGTERM'
for name in ['build-v3', 'corpus-v2', 'all-feature-library-v1']:
    assert not read(R / name / 'result.json')['command_failures']
for name in ['corpus-v2', 'all-feature-library-v1']:
    assert read(R / name / 'result.json')['source_mirror_unchanged']
    assert not read(R / name / 'result.json')['property_persistence_warnings']
for name in ['cost-preflight-v1', 'cost-matched-v1']:
    assert read(R / name / 'result.json')['all_exact_rows_equal_and_certified']
assert read(R / 'heap-v1/result.json')['all_instrumented_witnesses_identical']
proposal = read(R / 'proposal-v1/result.json')
assert all(sha(Path(proposal['source']) / path) == digest for path, digest in proposal['proposal_sha256'].items())
assert all(sha(path) == digest for path, digest in decision['input_sha256'].items())
for path in W.glob('*.py'):
    ast.parse(path.read_text(), filename=str(path))

E.mkdir()
retained, physical_only = {}, []
for root, prefix in [(W, Path('.')), (R, Path('run'))]:
    for path in sorted(root.rglob('*')):
        if not path.is_file() or '__pycache__' in path.parts:
            continue
        relative = path.relative_to(root)
        if any(part in ['baseline-source', 'candidate-source', 'runtime-source-mirror'] for part in relative.parts):
            continue
        with path.open('rb') as stream:
            magic = stream.read(4)
        if magic == b'\x7fELF' or path.suffix in ['.rmeta', '.zst']:
            physical_only.append({'path': str(path), 'sha256': sha(path), 'bytes': path.stat().st_size})
            continue
        dest = E / prefix / relative
        encoding = 'raw'
        if path.suffix in ['.stdout', '.stderr', '.massif'] and path.stat().st_size >= 65536:
            dest = dest.with_name(dest.name + '.gz')
            encoding = 'gzip'
        dest.parent.mkdir(parents=True, exist_ok=True)
        check_reserve()
        if encoding == 'gzip':
            with path.open('rb') as source, dest.open('wb') as target:
                with gzip.GzipFile(filename='', mode='wb', fileobj=target, mtime=0) as compressed:
                    shutil.copyfileobj(source, compressed)
            with gzip.open(dest, 'rb') as stream:
                assert hashlib.file_digest(stream, 'sha256').hexdigest() == sha(path)
        else:
            shutil.copy2(path, dest)
            assert sha(dest) == sha(path)
        retained[str(dest.relative_to(E))] = {'original': str(path), 'raw_sha256': sha(path),
            'retained_sha256': sha(dest), 'encoding': encoding, 'raw_bytes': path.stat().st_size}

ledger = ROOT / 'docs/properties-status.json'
raw, before = ledger.read_bytes(), sha(ledger)
assert before == previous['ledger_after_sha256']
reference = subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT)
start, end, current = span(raw)
rs, re_, _ = span(reference)
assert (raw[:start], raw[end:]) == (reference[:rs], reference[re_:])
current['updated_on'] = '2026-10-17'
current['circle_support_guarded'] = {
    'status': decision['status'], 'retention_decision': 'unadmitted', 'progress_classification': 'progress',
    'main_parent_commit': base, 'contract': PREFIX + 'contract-v1.md', 'decision': PREFIX + 'decision-v1.json',
    'progress': PREFIX + 'progress-v1.md', 'checkpoint': PREFIX + 'checkpoint-v1.json',
    'artifact_expectations': PREFIX + 'artifact-expectations-v1.json.gz', 'results': decision['results'],
    'proposal_sha256': proposal['proposal_sha256'], 'source_changes_applied': False, 'implementation_sha256': {},
    'cache_index_admitted': False, 'original_unguarded_index_remains_rejected': True,
    'native_disposition_preserved': True, 'pending_native_paths_remaining': 0,
    'full_archived_integration_admitted': False, 'native_runtime_admitted': False, 'full_plan_complete': False,
    'limitations': decision['limits'], 'next': decision['next'],
}
current['circle_support_index']['guarded_prototype'] = PREFIX + 'decision-v1.json'
assert current['circle_support_index']['retention_decision'] == 'rejected'
updated = raw[:start] + json.dumps(current, indent=2).encode() + raw[end:]
us, ue, _ = span(updated)
assert (updated[:us], updated[ue:]) == (reference[:rs], reference[re_:])
assert {key: value for key, value in json.loads(updated).items() if key != 'current_work'} == {
    key: value for key, value in json.loads(reference).items() if key != 'current_work'
}
assert sha(ledger) == before
ledger.write_bytes(updated)
with (E / 'main-format.stdout').open('wb') as out, (E / 'main-format.stderr').open('wb') as err:
    fmt = subprocess.run(['cargo', 'fmt', '--all', '--', '--check'], cwd=ROOT, stdout=out, stderr=err)
assert fmt.returncode == 0
assert all(sha(ROOT / path) == digest for path, digest in main.items() if path != 'docs/properties-status.json')
assert all(sha(ROOT / path) == digest for pins in [live, extra, dependencies] for path, digest in pins.items())
for path in E.rglob('*.py'):
    ast.parse(path.read_text(), filename=str(path))
dump(E / 'retention-v1.json', {
    'complete': True, 'main_parent_commit': base, 'source_changes_applied': False, 'cache_index_applied': False,
    'retention_decision': 'unadmitted', 'main_sources_unchanged': True, 'source_inputs': len(main),
    'retained_evidence_files': retained, 'physical_artifacts_not_duplicated_in_git': physical_only,
    'gzip_copies_verified_against_original_raw_bytes': True, 'original_physical_raw_files_unchanged': True,
    'all_new_output_roots_closed': True, 'format_exit_code': fmt.returncode, 'python_sources_parse': True,
    'only_current_work_changed': True, 'full_plan_complete': False,
})

roots, paths, directories, links = [W, R], [], [], {}
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
    'schema_version': 3, 'kind': 'guarded circle-support prototype; no production admission',
    'parent_expectations': previous['artifact_expectations'],
    'parent_expectations_sha256': previous['artifact_expectations_sha256'],
    'inherited_expected_artifact_paths': len(prior), 'inherited_missing_artifact_paths': len(missing),
    'added_physical_artifacts_sha256': physical, 'new_physical_artifacts_verified': len(physical),
    'local_tool_symlink_targets': links, 'physical_roots': [str(path) for path in roots],
    'directory_inventory': directories, 'expected_artifact_paths': len(prior) + len(physical),
    'inherited_artifacts_reverified': False, 'complete_artifact_audit_passed': False,
    'historical_preservation_complete': False,
    'scope': 'Every new final physical file path independently hashed with stable stat/lstat and no inode deduplication. Includes failed and interrupted attempts, all frozen source snapshots, executables and original raw cost/allocation output. Gzip Git copies supplement untouched physical raw files.',
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
    'status': decision['status'], 'checkpoint_kind': 'guarded circle-support prototype qualification; no production change',
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
    'cache_retention_decision': 'unadmitted', 'production_prefixes_unchanged': True,
    'single_line_kernel_admitted': True, 'typed_text_consumer_migration': True,
    'general_exact_kicad_stroke_fidelity_claimed': False, 'native_candidate_admitted': False,
    'native_runtime_admitted': False, 'full_archived_integration_admitted': False, 'full_plan_complete': False,
})
dump(E / 'checkpoint-v1.json', record)
check_reserve()
print(json.dumps({'new_physical_artifacts_verified': len(physical), 'only_current_work_changed': True,
                  'cache_index_applied': False, 'cache_retention_decision': 'unadmitted',
                  'inherited_missing_artifact_paths': len(missing)}), flush=True)
