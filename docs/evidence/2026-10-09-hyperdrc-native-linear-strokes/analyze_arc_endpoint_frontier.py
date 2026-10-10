"""Compare complete endpoint receipts and report exercised outcomes precisely."""
from collections import Counter
import gzip
import hashlib
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'arc-endpoint-frontier'
read = lambda path: json.loads(path.read_text())
digest = lambda data: hashlib.sha256(data).hexdigest()
status = read(OUT / 'status.json')
assert len(status) == 18 and all(row['exit_code'] == 0 for row in status)
assert read(OUT / 'corrected-filter-source-verification.json')['corrected_segment_tests_each_variant'] == 2
assert read(OUT / 'source-verification.json')['all_probe_production_and_main_source_hashes_unchanged']
oracle = read(OUT / 'oracle.json')
cases = {row['id']: row for row in oracle['cases']}
expected_counts = {'matrix': 59584, 'unresolved_and_aborted': 36, 'shared_queries': 4032}
prefixes = {'matrix': 'ARC_ENDPOINT_FRONTIER ', 'unresolved_and_aborted': 'ARC_ENDPOINT_SPECIAL ', 'shared_queries': 'ARC_ENDPOINT_CONCURRENT '}


def log_text(path):
    return path.read_text() if path.exists() else gzip.decompress(Path(str(path) + '.gz').read_bytes()).decode()


records = {}
test_counts = {}
for variant in ['baseline', 'candidate']:
    records[variant] = {}
    test_counts[variant] = {}
    for row in status:
        if row['variant'] != variant:
            continue
        path = OUT / (variant + '-' + row['name'] + '.log')
        raw = log_text(path)
        assert digest(raw.encode()) == row['log_sha256']
        tests = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;', raw)
        if tests:
            assert all(int(failed) == 0 for _, failed, _ in tests)
            test_counts[variant][row['name']] = {'binaries': len(tests), 'passed': sum(int(passed) for passed, _, _ in tests), 'ignored': sum(int(ignored) for _, _, ignored in tests)}
    for label, prefix in prefixes.items():
        raw = log_text(OUT / (variant + '-' + label + '.log'))
        rows = []
        for line in raw.splitlines():
            if prefix in line:
                rows.append(json.loads(line.split(prefix, 1)[1]))
        assert len(rows) == expected_counts[label], (variant, label, len(rows))
        keys = [json.dumps({key: value for key, value in row.items() if key != 'queries'}, sort_keys=True) for row in rows]
        assert len(set(keys)) == len(rows)
        assert all(len(row['queries']) == 7 for row in rows)
        records[variant][label] = rows
    assert test_counts[variant]['segment-regressions']['passed'] == 0
    assert test_counts[variant]['segment-regressions-corrected']['passed'] == 2
    assert test_counts[variant]['point-regressions']['passed'] == 14
    assert test_counts[variant]['arc-regressions']['passed'] == 17

assert records['baseline'] == records['candidate'], 'Classifications, certainty or later-query receipts differ; do not advance to resources.'
canonical = (json.dumps(records['baseline'], sort_keys=True, separators=(',', ':')) + '\n').encode()
(OUT / 'canonical-receipts.json.gz').write_bytes(gzip.compress(canonical, mtime=0))
coverage = {}
for label, rows in records['baseline'].items():
    groups = {}
    for row in rows:
        context = row.get('context', row.get('slot', 0) % 2)
        group = groups.setdefault(context, {'rows': 0, 'queries': {}})
        group['rows'] += 1
        for query, value, certainty in row['queries']:
            group['queries'].setdefault(query, Counter())[(value, certainty)] += 1
    coverage[label] = [{'context': context, 'rows': group['rows'], 'queries': {query: [{'value': value, 'certainty': certainty, 'count': count} for (value, certainty), count in sorted(counts.items())] for query, counts in group['queries'].items()}} for context, group in sorted(groups.items())]
(OUT / 'outcome-coverage.json').write_text(json.dumps(coverage, indent=2) + '\n')
oracle_coverage = Counter()
for label in ['matrix', 'shared_queries']:
    for row in records['baseline'][label]:
        context = row.get('context', row.get('slot', 0) % 2)
        for query, value, certainty in row['queries']:
            if query not in ['membership', 'membership_after'] or context in [4, 5] or certainty != 'Certified':
                continue
            category = (row['representation'], query, value)
            oracle_coverage[category] += 1
            if value.startswith('Decided('):
                assert value == 'Decided(' + str(cases[row['case']]['member']).lower() + ')'
(OUT / 'oracle-coverage.json').write_text(json.dumps([{'representation': representation, 'query': query, 'value': value, 'rows': count} for (representation, query, value), count in sorted(oracle_coverage.items())], indent=2) + '\n')
compression = []
for variant in ['baseline', 'candidate']:
    for label in prefixes:
        path = OUT / (variant + '-' + label + '.log')
        raw = log_text(path).encode()
        compressed = Path(str(path) + '.gz')
        if not compressed.exists():
            compressed.write_bytes(gzip.compress(raw, mtime=0))
        assert gzip.decompress(compressed.read_bytes()) == raw
        compression.append({'file': compressed.name, 'raw_bytes': len(raw), 'raw_sha256': digest(raw), 'compressed_sha256': digest(compressed.read_bytes())})
        if path.exists():
            path.unlink()
(OUT / 'lossless-log-compression.json').write_text(json.dumps(compression, indent=2) + '\n')
record = {
    'decision': 'Focused differential endpoint gate passed; production resource and downstream gates remain pending.',
    'source_files_verified': read(OUT / 'source-verification.json'),
    'runtime_change': 'Reuse Some(false) from the first endpoint query only when both approximate terminal and edge-preview classification are disabled; successful circle incidence then enters the unchanged non-endpoint sweep predicate. Unknown and lossy-policy retries remain.',
    'tests': test_counts,
    'strict_curve_clippy_and_format_each_variant': True,
    'validation_correction': 'The initial segment::tests:: filter ran zero tests; both original zero-test logs are preserved and not credited. A corrected segment:: filter excludes only the new probe module and passes both existing production segment regressions on each frozen variant. The correction runner initially expected seven tests by mistakenly borrowing the native-prototype count, then failed its count assertion after two baseline tests passed. That script and cause are preserved. The source-verified successful baseline log is reused and the candidate runs freshly. No Rust changed.',
    'independent_oracle': {'rational_cases': len(cases), 'transported_membership_expectations': oracle['membership_expectations_after_similarity_and_reversal'], 'method': oracle['scope'], 'similarity_argument': oracle['similarity_argument']},
    'identical_receipt_rows_per_variant': expected_counts,
    'identical_query_receipts_per_variant': sum(expected_counts.values()) * 7,
    'canonical_receipts_sha256': digest(canonical),
    'contexts': ['strict', 'bounded exact strict', 'approximate 512', 'bounded exact approximate (terminal disabled)', 'strict edge preview', 'approximate edge preview', 'strict pass on approximate context (terminal disabled)'],
    'matrix_stages': ['fresh fixture handles', 'repeated queries', 'cloned handles', 'fresh fixture with coordinate intervals prepared at -128'],
    'concurrent_scope': 'Four threads share one arc and point, request -16/-128/-32 coordinate intervals, and alternate strict/bounded-exact contexts. Six queries per arc include independently constructed and aliased endpoints, a non-endpoint circle point, center and outside point; all seven representations and eight authored arc cases are covered.',
    'outcome_coverage': 'arc-endpoint-frontier/outcome-coverage.json',
    'oracle_coverage': 'arc-endpoint-frontier/oracle-coverage.json',
    'primary_membership_outcomes': {label: [{'value': value, 'certainty': certainty, 'rows': count} for (value, certainty), count in sorted(Counter(tuple(row['queries'][0][1:]) for row in rows).items())] for label, rows in records['baseline'].items()},
    'main_runtime_unchanged': True,
    'limitations': 'Scoped differential receipts cannot prove all cache-frontier behavior. Fresh fixture graphs may share already warmed global constants. Certainty labels inside edge preview do not certify topology. Approximate-capable contexts are not evidence of actual terminal consumption; consult outcome counts. Transported symbolic cases use proved similarity identities, not independent symbolic solving. The test-only observer is absent from production resource trees. No full curve/DRC suites, board gate, historical witnesses, report replay, explicit shared-work-budget sweep, native prototype admission or performance claim is supplied by this focused gate.',
}
record['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path.read_bytes())} for path in sorted(OUT.iterdir()) if path.is_file()}
for name in ['arc_endpoint_oracle.py', 'arc_endpoint_frontier_probe.rs', 'reproduce_arc_endpoint_frontier.py', 'reproduce_arc_endpoint_segment_regressions.py', 'analyze_arc_endpoint_frontier.py']:
    path = EVIDENCE / name
    record['artifacts'][name] = {'bytes': path.stat().st_size, 'sha256': digest(path.read_bytes())}
(EVIDENCE / 'diagnostics-77.json').write_text(json.dumps(record, indent=2) + '\n')
print('Identical complete endpoint receipts:', sum(expected_counts.values()), 'rows /', record['identical_query_receipts_per_variant'], 'queries per variant.', flush=True)
