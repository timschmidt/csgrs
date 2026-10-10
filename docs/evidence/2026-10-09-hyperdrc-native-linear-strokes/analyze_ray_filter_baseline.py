"""Bind the exact ray-filter counterexamples to admitted production inputs."""
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
helper = EVIDENCE / 'ray-filter-overflow-baseline'
ordinary = EVIDENCE / 'ray-filter-public-baseline-corrected'
retained = EVIDENCE / 'ray-filter-retained-baseline'
failed = EVIDENCE / 'ray-filter-public-baseline'
h, o, r = [read(path / 'manifest.json') for path in [helper, ordinary, retained]]
assert h['case_count'] == 34 and h['failure_count'] == 16
assert h['commands'][0]['exit_code'] == 0 and h['commands'][1]['exit_code'] == 101
helper_rows = [json.loads(line) for line in (helper / 'run.jsonl').read_text().splitlines()]
assert all(row['all_inputs_finite'] for row in helper_rows)
assert sum(row['actual_miss'] != row['expected_miss'] for row in helper_rows) == 16
assert len(o['rows']) == 6 and all(row['passed'] for row in o['rows'])
assert o['commands'][-1]['exit_code'] == 0
assert len(r['rows']) == 6 and sum(not row['passed'] for row in r['rows']) == 4
assert r['commands'][-1]['exit_code'] == 101
assert all(row['actual'] == ('Inside' if row['origin'] == '0' else 'Outside') for row in r['rows'])
for manifest in [o, r]:
    assert digest(Path(manifest['library']['path'])) == manifest['library']['sha256']
    source = read(Path(manifest['source_record']))
    directory = ROOT / 'target/bbox-initial-pair-production/source'
    assert all(digest(directory / name) == value for name, value in source['source_hashes'].items())
body = subprocess.check_output(['git', 'show', h['head'] + ':' + h['source_file']], cwd=ROOT, text=True)
start = body.index('fn f64_box_certainly_misses_forward_ray(')
body = body[start:body.index('\n}\n', start)+3]
assert hashlib.sha256(body.encode()).hexdigest() == h['source_body_sha256']
assert (helper / 'probe.rs').read_text().startswith(body)
assert body in (ROOT / h['source_file']).read_text()
failure = read(failed / 'failure.json')
assert all(digest(failed / name) == value for name, value in failure['files'].items())
record = {
    'decision': 'confirmed exactness defect; isolated finite-intermediate guard and regression validation are in progress, with no main runtime repair admitted yet',
    'production_source_revision': h['head'],
    'helper_counterexamples': {'rows': 34, 'false_rejections': 16, 'all_inputs_finite': True, 'verbatim_function_sha256': h['source_body_sha256']},
    'public_native_chart_controls': {'rows': 6, 'all_passed': True},
    'public_retained_chart_counterexamples': {'rows': 6, 'false_certified_outside_answers': 4, 'ordinary_scale_controls_passed': 2, 'policies': ['STRICT', 'APPROXIMATE_512'], 'origins': ['1e308', '-1e308'], 'scale': '1e307'},
    'independent_oracle': 'The normalized closed lens has x=2t-1 and y=+/-2t(1-t) on [0,1]. Its center has -1/2 < y=0 < 1/2 at x=0. Positive exact affine scaling and translation preserve strict interior. The retained sources substitute t=u-1 on [1,2]; all public coordinates are parsed as exact decimal Reals. The helper matrix separately proves box hits from ray parameter one or strict origin containment, using exact binary64 rational inputs as degenerate certified enclosures.',
    'cause': 'Finite inputs can overflow during enclosure midpoint construction or projection. NaN makes every inequality false, leaving the initially true side-rejection flags unchanged. The filter can therefore skip an intersecting box and publish a false certified geometric answer.',
    'repair_under_validation': 'When the computed error bound, side projection or forward projection is non-finite, decline this floating-point exclusion and preserve the existing exact predicate fallback. No proof budget, tolerance, policy or public API changes.',
    'harness_failure': {'record': 'ray-filter-public-baseline/failure.json', 'cause': failure['cause'], 'geometry_queries_ran': False},
    'main_runtime_unchanged': True,
    'next_step': 'Complete focused and production regression validation, preserve prior exact report/capability witnesses, measure resource effects and admit only the verified guard and its tests.',
    'limitations': 'The native-chart controls do not exercise the same retained-source path. The helper matrix is not a full floating-point error proof, and this counterexample does not close the wider exactness or resource properties. Native-stroke admission remains separate.',
}
paths = [path for folder in [helper, ordinary, retained, failed] for path in sorted(folder.iterdir()) if path.is_file()]
paths += [EVIDENCE / name for name in ['reproduce_ray_filter_overflow.py', 'ray_filter_public_probe.rs', 'reproduce_ray_filter_public_baseline.py', 'ray_filter_retained_probe.rs', 'reproduce_ray_filter_retained_baseline.py', 'analyze_ray_filter_baseline.py']]
record['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in paths}
(EVIDENCE / 'diagnostics-43.json').write_text(json.dumps(record, indent=2)+'\n')
print('Verified 16 finite-input helper failures, four false public Outside answers and eight passing public controls.')
