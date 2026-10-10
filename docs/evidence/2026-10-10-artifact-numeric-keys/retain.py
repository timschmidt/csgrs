"""Verify completed evidence and update the existing property ledger.

Run only after every build, validation, and measurement has finished.
"""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
info = lambda p: {'bytes': p.stat().st_size, 'sha256': sha(p)}
production = EVIDENCE / 'candidate-v2'
native = EVIDENCE / 'native-compatibility'
records = [read(p / 'sources.json') for p in [production, native]]
for record in records:
    for kind, base in [('source', Path(record['source_directory'])), ('main', ROOT)]:
        path = Path(record[kind + '_hashes_file'])
        assert sha(path) == record[kind + '_hashes_file_sha256']
        assert all(sha(base / name) == expected for name, expected in json.loads(gzip.decompress(path.read_bytes())).items())
    for name in record['changed_files']:
        assert sha(ROOT / name) == sha(Path(record['source_directory']) / name)
full = read(production / 'full/summary.json')
compatibility = read(native / 'native/summary.json')
measured = read(production / 'reports-cost/summary.json')
baseline = read(EVIDENCE / 'baseline/summary.json')
corpus = read(EVIDENCE / 'candidate-v2-corpus/summary.json')
native_corpus = read(EVIDENCE / 'native-corpus/summary.json')
for folder in [EVIDENCE / 'baseline', EVIDENCE / 'candidate-corpus', EVIDENCE / 'candidate-v2-corpus', EVIDENCE / 'native-corpus', EVIDENCE / 'candidate-v1/focused', production / 'full', production / 'reports-cost', native / 'native']:
    summary = read(folder / 'summary.json')
    for name, expected in summary['artifacts'].items():
        assert info(folder / name) == expected, str(folder / name)
    for name, expected in summary['libraries'].items():
        assert info(Path(name)) == expected, name
for result in [full, compatibility, measured, corpus, native_corpus]:
    assert result['guards_unchanged']
    if 'commands' in result:
        assert all(row['exit_code'] == 0 for row in result['commands'])
for result, folder in [(corpus, 'candidate-v2-corpus'), (native_corpus, 'native-corpus')]:
    assert all(not row['differences'] and row['cases'] == 879 for row in result['trials'])
    for row in result['resources']:
        assert row['exit_code'] == 0 and row['completed']
        receipt = read(EVIDENCE / folder / (row['name'] + '.stdout'))
        messages = [v.get('message', '') for v in receipt['violations']]
        unavailable = row['token'] in ['1e1000000000', '1e-1000000000']
        assert any(('unavailable exact' if unavailable else 'has invalid') in text for text in messages)
    for order in ['forward', 'reverse']:
        assert (EVIDENCE / folder / (order + '.stdout')).read_bytes() == (EVIDENCE / 'candidate-corpus' / (order + '.stdout')).read_bytes()
subprocess.run(['python3', str(EVIDENCE / 'audit_receipts.py')], check=True)
comparisons = read(production / 'reports-cost/cost-comparison.json')
cost_rows = read(production / 'reports-cost/cost-rows.json')
assert all(row['byte_identical_to_admitted'] for row in measured['reports'].values())
baseline_library = next(v for k, v in baseline['libraries'].items() if k.endswith('/libhyperdrc.rlib'))
candidate_library = next(iter(full['libraries'].values()))
relative = str(EVIDENCE.relative_to(ROOT) / 'retention.json')
ledger_path = ROOT / 'docs/properties-status.json'
ledger = read(ledger_path)
statuses = [(row['id'], row['status']) for row in ledger['properties']]
note = ('The three artifact-analysis display-derived numeric keys are repaired with exact rational coordinate identity and Euclidean rotation normalization. '
        'The independent 879-case Fraction oracle repeats in both orders; 54 false overlap findings and 174 missed rotation findings are corrected. '
        'Unmaterializable scientific sources report explicit unavailable-number warnings; all 18 failure/resource probes complete. '
        'The original rounded-alias test is retained with a true-coincidence control. Production validation, unchanged full-resolution reports, native compatibility, '
        'and paired timing/allocation/size observations are retained in ' + relative + '. '
        'Legacy scalar formatting traits/callers, total work/output bounds, the unadmitted native geometry prototype, and complete property/target/future-work scope remain open.')
for row in ledger['properties']:
    if row['id'] in ['P1', 'P6', 'P26', 'P29', 'P35', 'P36', 'P37', 'P39']:
        assert relative not in row['evidence']
        row['evidence'].append(relative)
        row['remaining'] += ' ' + note
ledger['next_actions'][0] = ('The artifact numeric-key checkpoint passes the scoped DRC suite, strict Clippy/docs, exact public-API oracle, repeated complete board reports, '
                             'cost observations, and native compatibility checks. Preserve all raw failures and earlier evidence. The native geometry prototype remains unadmitted. '
                             'Complete the entire repository pre-push matrix before any push. See ' + relative + '.')
ledger['next_actions'][3] = ('Preserve the exact artifact numeric keys and explicit unavailable handling. Continue the remaining scalar formatting traits/callers and full underlying '
                             'evaluation/output bounds. Preserve the checked scientific/adaptive contracts, their fresh/prepared/concurrent receipts, and diagnostic consumer fixes. '
                             'See ' + relative + '; all broader property, target and future-work scope is unchanged.')
assert statuses == [(row['id'], row['status']) for row in ledger['properties']]
assert len(ledger['properties']) == 39 and len(ledger['targets']) == 14
ledger_path.write_text(json.dumps(ledger, indent=2) + '\n')
retention = {
    'decision': 'kept: scoped exactness repair; full goal remains active',
    'baseline_commit': 'aec9694c0',
    'production_source_record': str(production / 'sources.json'),
    'production_source_record_sha256': sha(production / 'sources.json'),
    'native_source_record': str(native / 'sources.json'),
    'native_source_record_sha256': sha(native / 'sources.json'),
    'changed_runtime_and_test_files': {name: info(ROOT / name) for name in records[0]['changed_files']},
    'source_guards': {kind: records[0][kind + '_files'] for kind in ['source', 'main']},
    'behavior': {
        'keys': 'Private typed Rational keys with exact total ordering; no decimal display participates in equality, ordering, grouping, or parity. Rotation uses exact Euclidean remainder modulo 360.',
        'unavailable': 'Rational scientific expansion exhaustion is retained as Problem::Exhausted until rendered into explicit readiness warnings and centroid ::number-unavailable parser diagnostics. Invalid syntax and zero-denominator sources remain invalid-number.',
        'display': 'Numeric finding text now uses exact integer/mixed-fraction output, so distinct values remain distinguishable. Five legacy mm/deg parser warnings disappear because parser and readiness now share numeric syntax.',
        'old_test': 'The original distinct R1/R2 placements remain in the fixture; a truly equal R3 was added and the assertion now requires R1/R3 without R2.',
        'clippy_revision': 'Initial full DRC tests passed; Clippy rejected rational interior caches in two key declarations. Candidate v2 adds two scoped expectations documenting invariant mathematical values. No runtime expression changed; full validation repeated.'
    },
    'oracle': {'cases_per_order': 879, 'orders': 2, 'baseline_errors_per_order': 228, 'corrected_false_overlaps': 54, 'corrected_rotation_mismatches': 174, 'production_errors': 0, 'native_errors': 0, 'all_receipts_match_between_candidates_and_native': True, 'numeric_receipt_changes': 348, 'unit_suffix_diagnostic_changes': 5, 'unrelated_receipt_changes': 0},
    'failure_cases': {'cases': 18, 'baseline_timeouts': 3, 'candidate_completed': 18, 'native_completed': 18, 'timeout_seconds': 3, 'address_space_cap_mib': 512},
    'production_commands': full['commands'],
    'native_commands': compatibility['commands'],
    'reports': measured['reports'],
    'report_processes': 2,
    'cost': {'trials_per_variant': 5, 'iterations': 20, 'rows_per_artifact': 256, 'families': 6, 'comparison': comparisons, 'allocation_rows': [row for row in cost_rows if row['kind'] == 'allocation'], 'baseline_library': baseline_library, 'candidate_library': candidate_library, 'library_byte_delta': candidate_library['bytes'] - baseline_library['bytes'], 'interpretation': 'Same original inputs, alternating serial old/new timing processes; separate allocation builds. Near-placement and near-rotation outputs deliberately change to correct decisions. Timing includes complete sidecar readiness, not only key operations. Exactness takes precedence; no universal speedup claim.'},
    'limitations': ['Rational parsing bounds exponent-only storage amplification, not total arithmetic, row count, output size, or wall time.', 'Board report replay preserves the original twelve-check, full-resolution workloads. Sidecar decisions are separately covered by the public-API oracle and artifact tests.', 'Exact extreme-number messages increase the measured peak live allocation from 623451 to 1048716 bytes, although aggregate allocated bytes decrease. Ordinary and coincident timing ranges overlap; lower medians are observations, not universal performance guarantees.', 'This validates the scoped artifact repair and compatibility; it does not admit the existing native geometry prototype or close the earlier full-board native performance gate.', 'The full repository pre-push workflow matrix was not run; no push is performed.', 'All 39 properties, 14 targets and crate future work remain in scope; property statuses are unchanged.'],
    'ledger': info(ledger_path),
}
retention['artifacts'] = {str(p.relative_to(EVIDENCE)): info(p) for p in sorted(EVIDENCE.rglob('*')) if p.is_file() and p.name != 'retention.json' and p.parent.name not in ['reports-1', 'reports-2']}
(EVIDENCE / 'retention.json').write_text(json.dumps(retention, indent=2) + '\n')
print('Retained exact artifact keys with', len(retention['artifacts']), 'verified/hashed evidence artifacts.', flush=True)
