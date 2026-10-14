"""Validate every collected exact witness and classify the geometric proof gaps."""
from collections import Counter, defaultdict
import hashlib
import importlib.util
import json
from pathlib import Path

E = Path(__file__).resolve().parent
OUT = E / 'sphere-zero-witnesses-v2'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def identify(indices, segments, stacks):
    levels = defaultdict(list)
    poles = []
    for i in indices:
        if i == 0:
            poles.append('north')
        elif i == segments * (stacks - 1) + 1:
            poles.append('south')
        else:
            longitude, latitude = divmod(i - 1, stacks - 1)
            levels[latitude + 1].append(longitude)
    if not poles and len(levels) == 2 and all(len(v) == 2 for v in levels.values()):
        sums = {sum(v) % segments for v in levels.values()}
        if len(sums) == 1:
            return 'parallel_chords_between_two_latitudes'
    if len(poles) == 1 and segments % 4 == 0 and stacks % 2 == 0:
        equator = levels.get(stacks // 2, [])
        pairs = [(latitude, values) for latitude, values in levels.items() if len(values) == 2]
        if len(equator) == 1 and len(pairs) == 1:
            latitude, pair = pairs[0]
            if (latitude * segments) % (2 * stacks) == 0:
                phi = latitude * segments // (2 * stacks)
                for first, second in [pair, pair[::-1]]:
                    delta = (equator[0] - first) % segments
                    if delta in [segments // 4, 3 * segments // 4]:
                        sign = 1 if delta == 3 * segments // 4 else -1
                        expected = first + (segments // 2 + sign * phi if poles[0] == 'north' else -sign * phi)
                        if second == expected % segments:
                            return 'pole_equator_pythagorean_identity'
    return 'other_exact_cyclotomic_zero'


def main():
    destination = E / 'sphere-zero-witness-assessment-v2.json'
    assert not destination.exists()
    result = json.loads((OUT / 'result.json').read_text())
    assert result['complete'] and result['all_sources_and_inputs_unchanged']
    assert all(sha(OUT / p) == h for p, h in result['artifacts_sha256'].items())
    protocol = json.loads((OUT / 'protocol.json').read_text())
    assert all(sha(E / p) == h for p, h in protocol['inputs_sha256'].items())
    commands = json.loads((OUT / 'commands.json').read_text())
    assert len(commands) == result['commands']
    assert all(c['exit_code'] == 0 for c in commands[:2])
    spec = importlib.util.spec_from_file_location('sphere_oracle_for_assessment', E / 'check_sphere_trace_cyclotomic_v1.py')
    oracle = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(oracle)
    dimensions = {name: (segments, stacks) for name, segments, stacks in protocol['cases']}
    assert [case['case'] for case in result['cases']] == list(dimensions)
    cases = []
    for case in result['cases']:
        segments, stacks = dimensions[case['case']]
        field = oracle.Field(segments, stacks)
        points = [field.point(i) for i in range(segments * (stacks - 1) + 2)]
        categories = Counter()
        proofs, added = [], []
        for record in case['records']:
            name = f"{case['case']}-{record['attempt']:03d}"
            group_file = OUT / (name + '.groups')
            assert sha(group_file) == record['groups_sha256']
            groups = [list(map(int, line.split())) for line in group_file.read_text().splitlines()]
            assert groups == added
            if 'indices' in record:
                value = field.orient(*(points[i] for i in record['indices']))
                assert value == record['determinant_remainder']
                if record['new_group_added']:
                    assert not value and set(record['indices']).issubset(record['group'])
                    a, b, c, _ = record['indices']
                    ab = [field.sub(x, y) for x, y in zip(points[b], points[a])]
                    ac = [field.sub(x, y) for x, y in zip(points[c], points[a])]
                    normal = [field.sub(field.mul(ab[i], ac[j]), field.mul(ab[j], ac[i]))
                              for i, j in [(1, 2), (2, 0), (0, 1)]]
                    assert any(normal) and normal == record['anchor_normal_polynomials']
                    expected = [i for i, point in enumerate(points)
                                if not field.orient(points[a], points[b], points[c], point)]
                    assert record['group'] == expected
                    added.append(record['group'])
                    category = identify(record['indices'], segments, stacks)
                    categories[category] += 1
                    proofs.append({'indices': record['indices'], 'category': category})
            if record.get('certified'):
                raw = (OUT / (name + '.stdout')).read_text()
                assert 'certified=true original_positions_preserved=true' in raw
                assert case['outcome'] == 'certified_with_independently_proven_groups'
        assert added == case['added_groups'] and len(added) <= protocol['max_added_groups_per_case']
        cases.append({'case': case['case'], 'outcome': case['outcome'], 'attempts': len(case['records']),
            'added_exact_groups': len(added), 'maximal_group_sizes': [len(g) for g in added],
            'geometric_categories': dict(categories), 'proofs': proofs})
    assessment = {'complete': True, 'result_sha256': sha(OUT / 'result.json'),
        'assessor_sha256': sha(__file__), 'commands': len(commands),
        'nonzero_command_exits': [{'name': c['name'], 'exit_code': c['exit_code']} for c in commands if c['exit_code']],
        'every_added_group_rechecked_exactly': True, 'every_consumed_group_file_matches_prior_proofs': True,
        'cases': cases,
        'scope': 'External exact mathematical facts supplied to the existing hull API on unchanged original points. This does not implement native discovery or a retained construction carrier.',
        'future_work': ['Replace enumeration and repeated complete hull queries with a bounded native certificate design.',
            'The collected groups remain fixed evidence; do not extend this closed trial or claim a budget stop is a mathematical impossibility.',
            'Every admitted maximal plane group was independently checked against a nonzero anchor normal and all original points.',
            'All source binding, transformed/remapped identity retention, public edit invalidation, scalar resource limits and cost gates still apply.'],
        'runtime_admitted': False, 'sphere_certificate_pending': True, 'plan_complete': False}
    destination.write_text(json.dumps(assessment, indent=2) + '\n')
    print(json.dumps({'complete': True, 'commands': len(commands), 'cases': [
        {k: c[k] for k in ['case', 'outcome', 'attempts', 'added_exact_groups', 'geometric_categories']} for c in cases],
        'assessment_sha256': sha(destination)}))


if __name__ == '__main__':
    main()
