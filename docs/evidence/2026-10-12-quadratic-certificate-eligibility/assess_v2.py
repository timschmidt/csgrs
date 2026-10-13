"""Assess the completed eligibility diagnostic without crediting it as timing evidence."""
import collections
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    out = E / 'diagnostic-v2'
    result = json.loads((out / 'result.json').read_text())
    protocol = json.loads((out / 'protocol.json').read_text())
    assert result['complete'] and result['observed_calls'] == 19296432
    assert sha(E / 'diagnose_v2.py') == protocol['runner_sha256']
    for phase in result['phases']:
        for stream in ['stdout', 'stderr']:
            assert sha(out / (phase['name'] + '.' + stream)) == phase[stream + '_sha256']
        assert phase['exit_code'] == (101 if phase['name'] == 'full-board' else 0)
    counters = json.loads((out / 'counters.json').read_text())
    shapes = counters['shapes']
    aggregate = {}
    for field in ['lane', 'operation', 'root']:
        groups = collections.defaultdict(lambda: {'calls': 0, 'successes': 0, 'projection_nanos': 0})
        for row in shapes:
            group = groups[row[field]]
            group['calls'] += row['calls']
            group['successes'] += row['calls'] if row['success'] else 0
            group['projection_nanos'] += row['projection_nanos']
        aggregate[field] = dict(sorted(groups.items()))
    totals = {key: sum(row[key] for row in aggregate['lane'].values())
              for key in ['calls', 'successes', 'projection_nanos']}
    eligible = [row for row in shapes if row['lane'] not in ['decline', 'intrinsic_only']]
    assert all(row['success'] for row in eligible)
    eligible_calls = sum(row['calls'] for row in eligible)
    eligible_nanos = sum(row['projection_nanos'] for row in eligible)
    stderr = (out / 'full-board.stderr').read_text()
    worker_seconds = float(re.search(r'took ([0-9.]+)s of worker time', stderr)[1])
    assert worker_seconds > 120
    assert stderr.count('finished emit report') == 2
    assessment = {
        'status': 'diagnostic-complete-shallow-path-not-advanced',
        'previous_goal_turn': 'progress',
        'scope': 'Root and immediate-child certificate observations on the complete original two-board test; only exiting-thread counters are collected.',
        'evidence_sha256': {str(p.relative_to(E)): sha(p) for p in [
            E / 'diagnose_v1.py', E / 'diagnostic-v1/result.json',
            E / 'diagnostic-v1-correction.json', E / 'diagnose_v2.py',
            out / 'protocol.json', out / 'result.json', out / 'counters.json',
            out / 'instrumentation.patch', out / 'instrumentation.tar.gz']},
        'qualification': 'One diagnostic regression, six inherited cache regressions, formatting and strict release all-feature all-target scalar Clippy pass. These are focused diagnostic checks, not full runtime admission.',
        'failed_attempt': 'The frozen v1 build failed before executing tests because LinearCombination3 coefficients and values have different types. The corrected v2 observes only its Computable coefficients; the original source and compiler logs remain available.',
        'aggregate': aggregate,
        'totals': totals,
        'shallow_eligible': {
            'calls': eligible_calls,
            'all_original_queries_succeeded': True,
            'fraction_of_successes': eligible_calls / totals['successes'],
            'projection_nanos': eligible_nanos,
            'fraction_of_observed_projection_nanos': eligible_nanos / totals['projection_nanos'],
        },
        'linear_combination3_rows_observed': sum(row['calls'] for row in shapes if row['operation'] == 'linear_combination3'),
        'full_board': {'exit_code': 101, 'worker_cpu_seconds': worker_seconds,
                       'unchanged_worker_limit_seconds': 120, 'findings': [12, 197],
                       'report_equality_asserted_here': False},
        'limitations': [
            'Observation, timing, TLS counters and Rational radicand equality perturb timing and cache warmth; this is not matched application performance evidence.',
            'Projection wall timers exclude observation and cover a different boundary from the prior per-pass diagnostic. They are not additive whole-process CPU samples.',
            'Counters come only from threads whose TLS destructors run. No universal call coverage or proof is claimed.',
            'Finding counts do not establish report equality. The preceding separate integer-leaf report replay retains its own complete-byte evidence.',
        ],
        'decision': 'Do not build the shallow consumer as the next board-cost candidate: eligible calls occupy only a small observed projection-time share. Keep both earlier recursive direct-consumer rejections.',
        'next_step': 'Prototype consuming the existing owned Laurent coefficient when merging equal exponents, avoiding its temporary vector clone. Preserve arithmetic order, radical/atom limits, recursive memo ownership at traversal boundaries and original fallback. Use the same three fixed cost probes, focused cache/frontier regressions and original native gates before any admission. Earlier inline-vector experiments remain rejected.',
        'runtime_admitted': False,
        'full_properties_plan_complete': False,
    }
    (E / 'assessment-v2.json').write_text(json.dumps(assessment, indent=2) + '\n')
    print(json.dumps({'totals': totals, 'shallow_eligible': assessment['shallow_eligible'],
                      'worker_cpu_seconds': worker_seconds, 'runtime_admitted': False}))


if __name__ == '__main__':
    main()
