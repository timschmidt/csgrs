"""Retain every cost observation and assess this isolated scalar experiment."""
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    validation = json.loads((E / 'validation-v1/result.json').read_text())
    cost = json.loads((E / 'cost-v1/result.json').read_text())
    assert validation['complete'] and all(p['exit_code'] == 0 for p in validation['phases'])
    assert cost['complete'] and cost['mathematical_witnesses_identical']
    for directory, commands in [
        ('validation-v1', validation['phases']),
        ('cost-v1', json.loads((E / 'cost-v1/commands.json').read_text())),
    ]:
        for command in commands:
            assert command['exit_code'] == 0
            for stream in ['stdout', 'stderr']:
                assert sha(E / directory / (command['name'] + '.' + stream)) == command[stream + '_sha256']
    inventory = {}
    for row in cost['representation_rows']:
        key = row['suite'], row['mode']
        assert row['rows'] == inventory.setdefault(key, row['rows'])
    timing, memory = [], []
    for row in cost['summary']:
        base = row['variants']['baseline']
        candidate = row['variants']['candidate']
        if row['mode'] == 'time':
            left, right = base['nanos']['samples'], candidate['nanos']['samples']
            timing.append({
                'suite': row['suite'], 'name': row['name'],
                'baseline_median_ns': base['nanos']['median'],
                'candidate_median_ns': candidate['nanos']['median'],
                'median_ratio': candidate['nanos']['median'] / base['nanos']['median'],
                'paired_ratios': [y / x for x, y in zip(left, right)],
                'slower_pairs': sum(y > x for x, y in zip(left, right)),
                'baseline_range_ns': [min(left), max(left)],
                'candidate_range_ns': [min(right), max(right)],
            })
        else:
            memory.append({'suite': row['suite'], 'name': row['name'],
                           'medians': {field: [base[field]['median'], candidate[field]['median']] for field in base}})
    binaries = json.loads((E / 'cost-v1/binaries.json').read_text())
    sizes = []
    for suite in ['rational', 'quadratic', 'integer']:
        for mode in ['time', 'memory']:
            name = suite + '-' + mode
            base = binaries['binaries']['baseline-' + name]
            candidate = binaries['binaries']['candidate-' + name]
            assert base['sha256'] != candidate['sha256']
            sizes.append({'name': name, 'baseline_bytes': base['bytes'],
                          'candidate_bytes': candidate['bytes'], 'delta_bytes': candidate['bytes'] - base['bytes']})
    assert all(r['medians']['allocation_calls'][1] <= r['medians']['allocation_calls'][0] for r in memory)
    assert all(r['medians']['allocated_bytes'][1] <= r['medians']['allocated_bytes'][0] for r in memory)
    assert all(r['medians']['retained_bytes_delta'][1] == r['medians']['retained_bytes_delta'][0] for r in memory)
    assessment = {
        'decision': 'advance-isolated-native-diagnostics-with-unresolved-scalar-timing',
        'scope': 'Owned equal-exponent coefficient merging, layered over the frozen integer-leaf and persistent-cache prototype. Runtime remains isolated.',
        'implementation': 'Update the existing temporary coefficient in place instead of cloning its radical and opaque-atom vectors. All callers discard the temporary polynomial if merging fails. No recursive prepass, new dependency, public layout or recognition-limit change.',
        'qualification': {name: re.findall(r'test result: .*', (E / 'validation-v1' / (name + '.stdout')).read_text())
                          for name in ['merge-regressions', 'cache-regressions', 'scalar-library', 'scalar-doctests']},
        'other_gates': 'Workspace formatting, strict release all-feature all-target scalar Clippy, and warning-denied all-feature documentation pass. Focused tests are included in the 866 library total.',
        'sampling': 'Three unchanged probe suites, five fixed alternating pairs per suite/mode, 4096 time inputs and 1000 allocation inputs. Construction/first/warm/cloned rows are all retained. No extra selective sampling.',
        'mathematical_witnesses_identical': True,
        'representation_inventories_identical_within_suite_and_mode': True,
        'timing_rows': timing,
        'memory_rows': memory,
        'binary_sizes': sizes,
        'observations': [
            'Measured allocation-call and allocated-byte medians never increase; several radical, opaque and mixed-field cases allocate less. Retained-byte medians are unchanged.',
            'Query timing medians often improve, including opaque declines (all five first/warm/cloned pairs faster) and the conjugate-then-affine cloned path (all five pairs faster).',
            'Signed-word-boundary construction is slower in all five pairs, with median ratio about 1.213 and overlapping sample ranges. This is an unresolved adverse observation, not dismissed as noise.',
            'Other adverse medians include rational quadratic-reciprocal construction, conjugate-product cloning, canonical-integer first queries and wide-integer cloning. Their pair directions vary; all samples remain in the raw cost result.',
            'Some peak-byte medians grow by 48 or 64 bytes. Probe binaries shrink by 768 or 784 bytes. No statistical-significance or application-speedup claim is made.',
        ],
        'reason_for_next_step': 'Reduced allocations and passing exactness/fallback checks justify measuring the unchanged native gates in isolation. This does not resolve the construction-time regression or admit runtime code.',
        'remaining_before_admission': [
            'Resolve the signed-word construction slowdown and other scalar timing tradeoffs with evidence that preserves this original run.',
            'Original native full-board and copper-neck limits, broader/default/feature consumer qualification, and complete unnormalized report replay.',
            'Public two-pass recognition-budget wording, broader cache frontiers, independent integration of the typed scalar prototype and the full properties plan.',
        ],
        'evidence_sha256': {name: sha(E / name) for name in [
            'prepare_v1.py', 'preparation.json', 'baseline-source-hashes.json.gz',
            'validate_v1.py', 'validation-v1/protocol.json', 'validation-v1/result.json',
            'validation-v1/candidate.patch', 'validation-v1/candidate-delta.tar.gz',
            'measure_v1.py', 'cost-v1/protocol.json', 'cost-v1/commands.json',
            'cost-v1/result.json', 'cost-v1/binaries.json']},
        'runtime_admitted': False, 'plan_complete': False,
    }
    (E / 'scalar-assessment-v1.json').write_text(json.dumps(assessment, indent=2) + '\n')
    print(json.dumps({'decision': assessment['decision'], 'timing_rows': len(timing),
                      'memory_rows': len(memory), 'sizes': sizes}))


if __name__ == '__main__':
    main()
