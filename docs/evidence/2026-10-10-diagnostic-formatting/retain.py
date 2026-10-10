"""Verify immutable evidence, record scoped retention, and update the property ledger."""
import gzip
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
PRODUCTION = EVIDENCE / 'candidate-v5'
NATIVE = EVIDENCE / 'native-compatibility'
read = lambda p: json.loads(p.read_text())
digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
info = lambda p: {'bytes': p.stat().st_size, 'sha256': digest(p)}


def verify_summary(folder, filename):
    result = read(folder / filename)
    for name, expected in result['artifacts'].items():
        assert info(folder / name) == expected, name
    for name, expected in result.get('libraries', {}).items():
        assert info(Path(name)) == expected, name
    for name, expected in result.get('binaries', {}).items():
        assert info(Path(name)) == expected, name
    for command in result.get('commands', []):
        assert command['exit_code'] == 0
    return result


focused = verify_summary(PRODUCTION / 'focused', 'validation.json')
full = verify_summary(PRODUCTION / 'full', 'validation.json')
corpus = verify_summary(PRODUCTION / 'corpus', 'summary.json')
reports_cost = verify_summary(PRODUCTION / 'reports-cost-v2', 'summary.json')
native_tests = verify_summary(NATIVE / 'focused', 'validation.json')
native_corpus = verify_summary(NATIVE / 'corpus', 'summary.json')
assert focused['libraries'] == full['libraries']
cost = read(PRODUCTION / 'reports-cost-v2/assessment.json')
old_library = next(value for name, value in read(EVIDENCE / 'baseline.json')['libraries'].items() if name.endswith('/libhyperdrc.rlib'))
new_library = next(iter(full['libraries'].values()))
assert old_library['bytes'] == cost['admitted_parent_library_bytes']
assert new_library['bytes'] == cost['source_production_library_bytes']
assert new_library['bytes'] - old_library['bytes'] == cost['library_size_delta_bytes']
for name, expected in read(PRODUCTION / 'corpus/binary-retention.json')['binaries'].items():
    assert info(Path(name)) == expected
for label, folder in [('production', PRODUCTION), ('native', NATIVE)]:
    record = read(folder / 'sources.json')
    for kind, base in [('source', Path(record['source_directory'])), ('main', ROOT)]:
        path = Path(record[kind + '_hashes_file'])
        assert digest(path) == record[kind + '_hashes_file_sha256']
        values = json.loads(gzip.decompress(path.read_bytes()))
        assert all(digest(base / name) == expected for name, expected in values.items()), (label, kind)
for key in ['adaptive_rows_each_process', 'adaptive_outcomes_each_process', 'adaptive_cache_receipt_changes', 'scientific_receipts_each_process', 'consumer_cases_completed']:
    assert corpus[key] == native_corpus[key], key
for name in ['decimal-forward.stdout', 'decimal-reverse.stdout']:
    assert (PRODUCTION / 'corpus' / name).read_bytes() == (NATIVE / 'corpus' / name).read_bytes()
for board, report in reports_cost['reports'].items():
    assert report['byte_identical_to_admitted'] and report['changed_messages'] == 0
    for trial in [1, 2]:
        data = gzip.decompress((PRODUCTION / 'reports-cost-v2' / f'reports-{trial}-{board}.json.gz').read_bytes())
        assert len(data) == report['bytes'] and hashlib.sha256(data).hexdigest() == report['sha256']
assert read(PRODUCTION / 'main-format/result.json')['exit_code'] == 0
source = read(PRODUCTION / 'sources.json')
assert read(PRODUCTION / 'migration-audit.json')['macro_sites'] == 110
assert read(PRODUCTION / 'corpus/consumer-difference-audit.json')['unchanged_outputs'] == 96
assert corpus['consumer_cases_completed'] == 120 and corpus['baseline_cutoffs_now_complete'] == 12
scope_files = source['changed_files']
result = {
    'decision': 'retain checked adaptive decimal diagnostics and reviewed human-output consumers',
    'previous_goal_turn_classification': {'classification': 'progress', 'commits': ['50188cad5', 'ca8025f97'], 'scope_preserved': 'All 39 properties, all 14 targets, and crate-specific future work.'},
    'checkpoint_commits': ['de7272b14', 'c7b8d7726'],
    'production_source_record': str(PRODUCTION / 'sources.json'), 'production_source_record_sha256': digest(PRODUCTION / 'sources.json'),
    'native_source_record': str(NATIVE / 'sources.json'), 'native_source_record_sha256': digest(NATIVE / 'sources.json'),
    'source_files': {name: info(Path(source['source_directory']) / name) for name in scope_files},
    'main_overlapping_preexisting_changes': source['main_overlapping_preexisting_changes'],
    'main_and_candidate_source_guards_unchanged_through_validation': True,
    'behavior': 'Exact nearest-even adaptive rounding uses the mathematical fixed-notation window [2^-19, 2^60). The diagnostic view renders explicit failure markers. The calculator, alternate lattice aggregates, and 110 DRC message/trace macros use the view. Legacy scalar formatting traits and the three machine-key sites are unchanged.',
    'production_test_executions': {row['name']: {'passed': row['tests_passed'], 'ignored': row['tests_ignored']} for row in full['commands'] if 'tests_passed' in row},
    'production_commands': full['commands'],
    'focused_test_executions': sum(row.get('tests_passed', 0) for row in focused['commands']),
    'adaptive_corpus': {key: corpus[key] for key in ['adaptive_rows_each_process', 'adaptive_processes', 'adaptive_outcomes_each_process', 'adaptive_cache_receipt_changes', 'all_adaptive_ok_strings_match_exact_reference', 'legacy_rational_rows', 'legacy_rational_differences']},
    'scientific_receipts_per_process': 3816, 'scientific_processes_per_variant': 2, 'scientific_receipts_preserved': True,
    'consumers': {'cases': 120, 'all_completed': True, 'previous_cutoffs_now_complete': 12, 'unchanged_output_cases': 96, 'changed_unavailable_value_outputs': 24},
    'reports': reports_cost['reports'], 'report_processes': 2,
    'native_compatibility': {'test_executions_passed': sum(row.get('tests_passed', 0) for row in native_tests['commands']), 'commands': native_tests['commands'], 'adaptive_rows_identical_to_production': True, 'consumer_cases_completed': native_corpus['consumer_cases_completed'], 'scientific_receipts_preserved': True, 'prototype_admitted': False},
    'cost_assessment': str(PRODUCTION / 'reports-cost-v2/assessment.json'),
    'cost_interpretation': 'A certified diagnostic contract, not a drop-in speedup claim. Some rational families improve, while pi and several other computable families require more time/allocations; full reports are unchanged and the existing production board gate passes.',
    'provenance_note': 'candidate-revisions.json verifies all 125 frozen changed-file copies and gives authoritative revision deltas, including the correction for inherited noncumulative v3 delta lists in v4/v5 metadata.',
    'limitations': ['The three artifact-analysis display-derived comparison keys remain open; open-machine-keys.json is a source audit, not a runtime reproduction.', 'Legacy scalar formatting traits can still seek a leading bit for unresolved zero.', 'Finite precision/expansion guards do not bound all underlying evaluation or structural-analysis work.', 'The native geometry prototype remains unadmitted; its earlier full-board performance failure is unchanged in status.', 'The 28 ignored curve tests were already ignored; no exhaustive representation or universal cache-frontier proof is claimed.', 'The full repository pre-push workflow matrix has not been run in this checkpoint; no push was performed.', 'All 39 properties, 14 targets, and crate future-work requirements remain in scope.'],
}
ledger_path = ROOT / 'docs/properties-status.json'
ledger = read(ledger_path)
assert len(ledger['properties']) == 39 and len(ledger['targets']) == 14
note = ' Checked adaptive diagnostics now certify nearest-even decimal output and expose unavailable values explicitly in 110 DRC message/trace sites, lattice alternate displays and calculator answers. The checkpoint retains 12 previous cutoffs, 5,282 oracle rows per process, all 3,816 prior scientific receipts, byte-identical full reports and measured time/allocation costs. Legacy scalar traits, three display-derived artifact keys, total underlying work bounds and property-wide coverage remain open. See docs/evidence/2026-10-10-diagnostic-formatting/retention.json.'
for row in ledger['properties']:
    if row['id'] in ['P6', 'P26', 'P29', 'P35', 'P36', 'P37', 'P39']:
        path = 'docs/evidence/2026-10-10-diagnostic-formatting/retention.json'
        if path not in row['evidence']:
            row['evidence'].append(path)
        if note not in row['remaining']:
            row['remaining'] += note
ledger['next_actions'][0] = 'The diagnostic formatting checkpoint passes the scoped scalar/lattice/curve/DRC suites, strict Clippy/docs, two repeated complete board reports, cost observations, and native compatibility checks. Preserve all raw failures and earlier evidence. The native geometry prototype remains unadmitted. Complete the entire repository pre-push matrix before any push. See docs/evidence/2026-10-10-diagnostic-formatting/retention.json.'
ledger['next_actions'][3] = 'Reproduce and repair the three artifact-analysis comparison keys that still depend on rounded display strings; use exact identity and explicit unavailable handling. Preserve the checked scientific and adaptive contracts, their fresh/prepared/concurrent receipts, and the diagnostic consumer fixes. Continue the remaining scalar formatting traits/callers and full underlying evaluation bounds. The scoped evidence is in docs/evidence/2026-10-10-diagnostic-formatting/retention.json; the broader property, target and future-work scope is unchanged.'
ledger_path.write_text(json.dumps(ledger, indent=2) + '\n')
result['ledger'] = {'path': str(ledger_path), **info(ledger_path), 'properties': 39, 'targets': 14, 'statuses_unchanged': True}
result['artifacts'] = {str(p.relative_to(EVIDENCE)): info(p) for p in sorted(EVIDENCE.rglob('*')) if p.is_file() and p.name != 'retention.json' and p.parent.name not in ['reports-1', 'reports-2']}
(EVIDENCE / 'retention.json').write_text(json.dumps(result, indent=2) + '\n')
print('Retention verified:', len(result['artifacts']), 'artifacts;', result['native_compatibility']['test_executions_passed'], 'native tests; all 39 properties and 14 targets remain in scope.', flush=True)
