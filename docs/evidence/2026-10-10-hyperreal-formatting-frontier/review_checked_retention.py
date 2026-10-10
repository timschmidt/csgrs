"""Audit evidence and staged/runtime source identity before committing the checked API."""
import difflib
import gzip
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
info = lambda path: {'bytes': path.stat().st_size, 'sha256': digest(path)}
names = [
    'checked-scientific-focused.json', 'checked-frontier.json',
    'checked-scalar-validation.json', 'checked-cost.json',
    'class-precheck-validation.json', 'class-precheck-cost.json',
    'rounding-cell-audit.json', 'downstream-checked-scientific.json',
    'checked-native-validation.json',
]
records = {name: read(EVIDENCE / name) for name in names}
verified = {}
for record in records.values():
    for collection in ['artifacts', 'scripts']:
        for name, expected in record.get(collection, {}).items():
            assert info(EVIDENCE / name) == expected, name
            verified[name] = expected
binary_references = {}
for name in ['checked-cost/binaries.json', 'class-precheck-cost/binaries.json']:
    for item in read(EVIDENCE / name).values():
        path = Path(item['path'])
        assert info(path) == {key: item[key] for key in ['bytes', 'sha256']}
        binary_references[str(path)] = info(path)
for name in ['checked-frontier/library.json', 'class-precheck-validation/library.json']:
    item = read(EVIDENCE / name)
    path = Path(item['retained_path'])
    assert info(path) == {key: item[key] for key in ['bytes', 'sha256']}
    binary_references[str(path)] = info(path)
for item in [*records['downstream-checked-scientific.json']['probe_binaries'].values(), records['checked-native-validation.json']['library'], records['checked-native-validation.json']['binary']]:
    path = Path(item['path'])
    assert info(path) == {key: item[key] for key in ['bytes', 'sha256']}
    binary_references[str(path)] = info(path)
assert digest(EVIDENCE / 'audit_rounding_cells.py') == records['rounding-cell-audit.json']['script_sha256']
assert records['checked-frontier.json']['preparation_or_concurrency_receipt_differences_from_fresh'] == 0
assert records['checked-frontier.json']['legacy_scientific_completed_outputs_preserved'] == 36
assert records['checked-frontier.json']['legacy_scientific_cutoffs_now_typed_unknown_zero'] == 12
assert records['rounding-cell-audit.json']['returned_strings_audited'] == 2256
assert records['class-precheck-cost.json']['decision'].startswith('exact-class precheck not retained;')
assert records['downstream-checked-scientific.json']['historical_witnesses'] == 231
assert all(row['exit_code'] == 0 for row in records['downstream-checked-scientific.json']['commands'])
assert all(row['exit_code'] == 0 for row in records['checked-native-validation.json']['steps'])
assert records['checked-native-validation.json']['differences_from_production_checked_candidate'] == 0
production = read(EVIDENCE / 'checked-scientific/sources.json')
native = read(EVIDENCE / 'checked-native/sources.json')
maps = {}
for name, source in [('production', production), ('native', native)]:
    path = Path(source['source_hashes_file'])
    assert digest(path) == source['source_hashes_file_sha256']
    maps[name] = json.loads(gzip.decompress(path.read_bytes()))
    assert all(digest(Path(source['source_directory']) / filename) == expected for filename, expected in maps[name].items())
changed = production['changed_files']
assert changed == native['changed_files']
staged = subprocess.check_output(['git', 'diff', '--cached', '--name-only', '-z'], cwd=ROOT).decode().split('\0')
assert set(filter(None, staged)) == set(changed), staged
index_hashes = {}
documentation_difference = {}
for name in changed:
    raw = subprocess.check_output(['git', 'show', ':' + name], cwd=ROOT)
    index_hashes[name] = hashlib.sha256(raw).hexdigest()
    assert index_hashes[name] == maps['production'][name], name
    if name == 'crates/hyperreal/README.md':
        before = (Path(native['source_directory']) / name).read_text()
        anchor = '| Enclose while retaining exact rational points | `certified_rational_interval` |\n'
        row = '| Reuse an existing enclosure without refinement | `cached_rational_interval` |\n'
        assert before.count(anchor) == 1 and row not in before
        expected = before.replace(anchor, anchor + row)
        assert (ROOT / name).read_text() == expected
        documentation_difference[name] = {'scope': 'Preserve exactly the preexisting uncommitted cached-enclosure API table row. It was absent from the isolated native snapshot; it is not part of the checked-formatter commit.', 'sha256': digest(ROOT / name), 'row': row.rstrip('\n')}
        (EVIDENCE / 'retention-review/preserved-readme.patch').write_text(''.join(difflib.unified_diff(before.splitlines(True), expected.splitlines(True), 'validated-native/' + name, 'main/' + name)))
    else:
        assert digest(ROOT / name) == maps['native'][name], name
original_main = read(Path(native['main_source_record']))['source_hashes']
for name in original_main:
    expected = documentation_difference.get(name, {}).get('sha256', maps['native'][name])
    assert digest(ROOT / name) == expected, name
    if name not in changed:
        assert digest(ROOT / name) == original_main[name], name
native_extra = sorted(set(changed) - set(original_main))
integrated = EVIDENCE / 'integrated-main'
integrated.mkdir(exist_ok=True)
main_hashes = {name: digest(ROOT / name) for name in sorted(set(original_main) | set(changed))}
compressed = integrated / 'source-hashes.json.gz'
compressed.write_bytes(gzip.compress((json.dumps(main_hashes, indent=2) + '\n').encode(), mtime=0))
main_record = {
    'scope': 'Main worktree after checked-formatter integration, including the existing unadmitted native prototype and its preserved README row. This snapshot is a guard for future work, not native admission.',
    'source_directory': str(ROOT), 'source_files': len(main_hashes),
    'source_hashes_file': str(compressed), 'source_hashes_file_sha256': digest(compressed),
    'parent_source_record': native['main_source_record'], 'parent_source_record_sha256': native['main_source_record_sha256'],
    'validated_native_source_record': str(EVIDENCE / 'checked-native/sources.json'),
    'validated_native_source_record_sha256': digest(EVIDENCE / 'checked-native/sources.json'),
    'checked_formatter_files': changed, 'documented_difference_from_native_snapshot': documentation_difference,
}
(integrated / 'sources.json').write_text(json.dumps(main_record, indent=2) + '\n')
summary = {
    'decision': 'retain the first checked scientific-formatting API; reject the exact-class precheck and keep native geometry unadmitted',
    'public_api': ['Real::try_scientific', 'ScientificCase::{Lower,Upper}'],
    'behavior': 'Certified nearest-even scientific output with explicit decimal-place and exponent-case intent; exact rational ties and carries; caller refinement floor; typed UnknownZero, Exhausted and OutOfRange outcomes. Existing formatting traits and callers keep their implementation.',
    'retention_basis': 'This adds a stronger explicit rounding/completion boundary, including typed outcomes for twelve scientific queries that reached the retained external cutoff. Its cost is reported as a distinct API contract, not claimed as a speed optimization. Focused, complete scalar, downstream and current-worktree compatibility checks pass.',
    'resource_tradeoff': records['checked-cost.json']['assessment'],
    'rejected_variant': records['class-precheck-cost.json']['assessment'],
    'corpus': {'rows_each_process': 3816, 'preparation_and_concurrency_differences': 0, 'returned_strings_with_exact_independent_cell_check': 2256, 'legacy_returned_scientific_strings_preserved': 36, 'legacy_scientific_cutoffs_now_typed_unknown_zero': 12},
    'downstream': {'historical_witnesses': 231, 'witness_processes': 2, 'report_processes': 2, 'reports': records['downstream-checked-scientific.json']['reports'], 'original_board_gate_process_cpu_seconds': 54.44, 'matched_application_speedup_claim': False},
    'integration': {'staged_files_match_validated_production_candidate': index_hashes, 'worktree_runtime_and_tests_match_validated_native_candidate': {name: digest(ROOT / name) for name in changed if name.endswith('.rs')}, 'documented_readme_difference': documentation_difference, 'original_main_source_files_checked': len(original_main), 'additional_main_files_checked': native_extra, 'unrelated_native_source_hashes_unchanged': True, 'main_source_record': str(integrated / 'sources.json'), 'main_source_record_sha256': digest(integrated / 'sources.json')},
    'records': {name: info(EVIDENCE / name) for name in names},
    'verified_artifact_references': len(verified),
    'verified_retained_binaries_and_libraries': binary_references,
    'runtime_scope': 'Only the seven checked-formatter implementation/documentation/test files are staged by this review. The indexed source matches the admitted-production candidate; worktree runtime/test source matches the separately validated native combination, with the preexisting README row explicitly preserved. Existing native changes are not included in the API commit.',
    'review_correction': 'The first final-review assertion incorrectly required README identity with the native snapshot, which had omitted the existing cached-enclosure table row. The original reviewer and reproduced failure are retained; the corrected check admits exactly that documented existing row and still requires exact runtime/test source identity. No production, candidate or test source changed to resolve this metadata failure.',
    'review_artifacts': {str(path.relative_to(EVIDENCE)): info(path) for path in sorted((EVIDENCE / 'retention-review').iterdir()) if path.is_file()},
    'open': ['existing Display/LowerExp/UpperExp/alternate traits can still seek a leading bit on unresolved zeros', 'underlying structural analysis and a single approximation have no universal total-work bound from this API', 'callers needing explicit scientific failure must choose the checked boundary', 'native source/consumer completeness and its original performance gate remain open', 'full workspace/pre-push workflow matrix, all properties, all fourteen targets and crate future work remain in scope'],
    'script_sha256': digest(Path(__file__)),
}
(EVIDENCE / 'retention.json').write_text(json.dumps(summary, indent=2) + '\n')
print('Retention review passed:', len(verified), 'artifact references;', len(changed), 'staged files match validated production; native runtime/test source matches independently validated source, with the existing README row preserved.')
