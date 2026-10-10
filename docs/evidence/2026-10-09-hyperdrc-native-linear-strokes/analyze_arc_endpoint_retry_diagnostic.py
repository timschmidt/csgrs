"""Audit complete-fixture endpoint retry counts without admitting an optimization."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import tarfile

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'arc-endpoint-retry-diagnostic'
SOURCE = ROOT / 'target/arc-endpoint-retry-diagnostic/source'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
source = read(OUT / 'sources.json')
assert digest(Path(source['source_hashes_file'])) == source['source_hashes_file_sha256']
assert digest(Path(source['native_source_record'])) == source['native_source_record_sha256']
assert digest(Path(source['fallback_source_record'])) == source['fallback_source_record_sha256']
hashes = json.loads(gzip.decompress(Path(source['source_hashes_file']).read_bytes()))
native = read(Path(source['native_source_record']))
assert len(hashes) == source['source_files']
assert all(digest(SOURCE / name) == value for name, value in hashes.items())
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
assert not (SOURCE / source['removed_prior_test_only_probe']).exists()
assert hashes['crates/hyperreal/src/computable/node/algebra.rs'] == native['source_hashes']['crates/hyperreal/src/computable/node/algebra.rs']
with tarfile.open(OUT / 'instrumentation.tar.gz') as archive:
    assert set(archive.getnames()) == set(source['only_instrumented_sources'])
    for name in archive.getnames():
        assert hashlib.sha256(archive.extractfile(name).read()).hexdigest() == hashes[name]
assert read(OUT / 'source-verification.json')['all_instrumented_and_main_source_hashes_unchanged']
status = read(OUT / 'status.json')
assert [row['name'] for row in status] == ['format', 'curve-clippy', 'point-regressions', 'segment-regressions', 'arc-regressions', 'build-board-smoke', 'full-board-smoke']
assert all(row['exit_code'] == 0 for row in status[:-1])
assert status[-1]['exit_code'] in [0, 101]
tests = {}
for name in ['point-regressions', 'segment-regressions', 'arc-regressions']:
    matches = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored', (OUT / (name + '.log')).read_text())
    assert matches and all(int(row[1]) == 0 for row in matches)
    tests[name] = {'binaries': len(matches), 'passed': sum(int(row[0]) for row in matches), 'ignored': sum(int(row[2]) for row in matches)}
binary = read(OUT / 'binary.json')
assert digest(Path(binary['path'])) == binary['sha256']
log = (OUT / 'full-board-smoke.log').read_text()
assert 'skipping complex project fixture test' not in log
assert log.count('hyperdrc: finished emit report') == 2
findings = list(map(int, re.findall(r'finished run \((\d+) active finding\(s\)', log)))
assert findings == [12, 197]
tag = 'ARC_ENDPOINT_RETRY_DIAGNOSTIC '
observations = [json.loads(line.split(tag, 1)[1]) for line in log.splitlines() if tag in line]
assert observations and all(len(row['rows']) == 288 for row in observations)
assert len({row['thread'] for row in observations}) == len(observations)
totals = [sum(row['rows'][index] for row in observations) for index in range(288)]
assert all(isinstance(value, int) and value >= 0 for value in totals)
assert sum(totals) > 0
decoded = []
for index, count in enumerate(totals):
    if not count:
        continue
    quotient, repeated = divmod(index, 4)
    quotient, radius = divmod(quotient, 4)
    quotient, first = divmod(quotient, 3)
    mode, bounded = divmod(quotient, 2)
    assert mode < 3
    if first == 1:
        assert radius == repeated == 3
    else:
        assert radius != 3
        assert (repeated != 3) == (radius == 1)
    decoded.append({'policy': mode, 'bounded': bool(bounded), 'first_endpoint': first, 'radius': radius, 'repeated_endpoint': repeated, 'count': count})
groups = []
for mode in range(3):
    for bounded in [False, True]:
        rows = [row for row in decoded if row['policy'] == mode and row['bounded'] == bounded]
        count = lambda condition: sum(row['count'] for row in rows if condition(row))
        groups.append({
            'policy': source['counter_layout']['policy'][mode], 'bounded_exact_pass': bounded,
            'calls': count(lambda row: True),
            'initial_endpoint_proved': count(lambda row: row['first_endpoint'] == 1),
            'radius_queries': count(lambda row: row['radius'] != 3),
            'repeated_endpoint_queries': count(lambda row: row['repeated_endpoint'] != 3),
            'earlier_non_endpoint_on_circle': count(lambda row: row['first_endpoint'] == 0 and row['radius'] == 1),
            'unknown_then_endpoint_proved': count(lambda row: row['first_endpoint'] == 2 and row['repeated_endpoint'] == 1),
            'negative_then_endpoint_proved': count(lambda row: row['first_endpoint'] == 0 and row['repeated_endpoint'] == 1),
        })
calls = sum(totals)
repeated = sum(row['repeated_endpoint_queries'] for row in groups)
eligible = sum(row['earlier_non_endpoint_on_circle'] for row in groups if row['policy'] == 'exact_only')
contradictions = sum(row['negative_then_endpoint_proved'] for row in groups if row['policy'] == 'exact_only')
summary = {
    'threads': len(observations), 'arc_membership_calls': calls, 'repeated_endpoint_queries': repeated,
    'initial_positive_endpoint_queries': sum(row['count'] for row in decoded if row['first_endpoint'] == 1),
    'initial_unknown_endpoint_queries': sum(row['count'] for row in decoded if row['first_endpoint'] == 2),
    'unknown_radius_queries': sum(row['count'] for row in decoded if row['radius'] == 2),
    'exact_negative_before_successful_radius_query': eligible,
    'eligible_fraction_of_membership_calls': eligible / calls,
    'eligible_fraction_of_repeated_endpoint_queries': eligible / repeated if repeated else 0,
    'exact_negative_then_endpoint_proved': contradictions,
    'unknown_then_endpoint_proved_all_policies': sum(row['unknown_then_endpoint_proved'] for row in groups),
}
observed = {'scope': 'Aggregated counters from the complete two-board diagnostic only; focused-test counters are excluded.', 'layout': source['counter_layout'], 'threads': observations, 'totals': totals, 'nonzero_rows': decoded, 'policy_groups': groups, 'summary': summary}
(OUT / 'observations.json').write_text(json.dumps(observed, indent=2) + '\n')
timing = (OUT / 'full-board-smoke-time.log').read_text()
resource = lambda label: re.search(re.escape(label) + r': ([^\n]+)', timing)[1]
worker = re.search(r'complex project KiCad smoke suite took ([0-9.]+)s of worker time', log)
if status[-1]['exit_code']:
    assert worker and float(worker[1]) >= 120
next_step = (
    'Investigate the observed exact-negative/positive contradiction before any retry optimization.' if contradictions else
    'The complete observed workload offers no exact-negative retry reuse; decline this optimization before implementation.' if not eligible else
    'Evaluate a separate candidate that reuses only an earlier exact negative endpoint certificate on the same arc and point, retaining unknown, approximate and preview retries. Require cold/warm/subsequent-query decision and receipt checks before resource measurement; these counts alone do not establish cache-frontier preservation.'
)
record = {
    'decision': 'diagnostic complete; no arc-membership optimization admitted',
    'scope': source['scope'], 'source_files_verified': {'instrumented': len(hashes), 'main': len(native['source_hashes'])},
    'profile_basis': 'diagnostics-35.json; Point2 equality is 15.975% of recovered inclusive samples, overlapping normal-form and predicate groups. This is motivation, not predicted savings.',
    'focused_tests': tests, 'strict_curve_clippy_and_format_passed': True,
    'observations': summary, 'policy_groups': groups,
    'observed_policy_categories': [{'policy': row['policy'], 'bounded_exact_pass': row['bounded_exact_pass']} for row in groups if row['calls']],
    'board_gate': {
        'passed': status[-1]['exit_code'] == 0, 'exit_code': status[-1]['exit_code'],
        'unchanged_worker_cpu_limit_seconds': 120,
        'worker_cpu_seconds': float(worker[1]) if worker else 'not printed on success',
        'process_cpu_seconds': round(float(resource('User time (seconds)')) + float(resource('System time (seconds)')), 2),
        'wall_elapsed': resource('Elapsed (wall clock) time (h:mm:ss or m:ss)'),
        'peak_rss_kib': int(resource('Maximum resident set size (kbytes)')),
        'findings': {'cparti-fpga': findings[0], 'hvp109a': findings[1]},
        'instrumented_time_is_not_admission_evidence': True,
    },
    'operation_order_argument': 'The same first contains_endpoint call runs before the same distance-squared/radius expression and zero query. A successful radius query executes the same second endpoint call formerly inside the sweep wrapper, then the same true return or non-endpoint sweep method. Integer counters observe completed results and retain no geometry or scalar values. No retry or mathematical query is removed.',
    'main_runtime_unchanged': True, 'next_step': next_step,
    'limitations': 'The observed board calls use only unbounded exact policy and initially proven non-endpoints. They do not exercise initial positive/unknown endpoints, uncertain radii, bounded passes, approximation or preview. Full native suites, complete reports, cross-library measurements and a proof of all cold/warm frontiers are not supplied by this diagnostic. Zero observed contradictory transitions would not prove universal idempotence. Counter frequency is not weighted CPU or memory savings. The wider prototype and all remaining property/target scope stay open.',
}
paths = [path for path in sorted(OUT.iterdir()) if path.is_file()]
paths += [EVIDENCE / name for name in ['arc_endpoint_retry_diagnostic.rs', 'reproduce_arc_endpoint_retry_diagnostic.py', 'analyze_arc_endpoint_retry_diagnostic.py']]
record['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in paths}
(EVIDENCE / 'diagnostics-76.json').write_text(json.dumps(record, indent=2) + '\n')
print('Focused tests:', tests)
print('Endpoint retry observations:', summary)
print('Board gate:', record['board_gate'])
print('Next:', next_step)
