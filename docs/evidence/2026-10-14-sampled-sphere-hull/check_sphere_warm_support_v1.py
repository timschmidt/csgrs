"""Exact support checks for every changed warm face; no native library calls."""
from collections import Counter
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import signal
import sys

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
OUT = E / 'sphere-warm-support-v1'
COUNTEREXAMPLE = E / 'sphere-warm-row-counterexample-v1'
PRIOR = E / 'sphere-native-support-v1'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def canonical(face):
    return min(face, face[1:] + face[:1], face[2:] + face[:2])


def parse(text):
    order = json.loads(next(line.split('=', 1)[1] for line in text.splitlines()
                            if line.startswith('position_order=')))
    assert sorted(order) == list(range(482))
    rows = [tuple(order[int(i)] for i in row) for row in re.findall(
        r'Triangle \{ v0: (\d+), v1: (\d+), v2: (\d+) \}', text)]
    assert len(rows) == 960 and all(len(set(row)) == 3 for row in rows)
    faces = {canonical(row) for row in rows}
    assert len(faces) == 960
    assert {i for row in rows for i in row} == set(range(482))
    edges = {}
    for face, (a, b, c) in enumerate(rows):
        for u, v in [(a, b), (b, c), (c, a)]:
            edges.setdefault(tuple(sorted((u, v))), []).append((u, v, face))
    assert len(edges) == 1440 and 482 - len(edges) + len(rows) == 2
    neighbors = [[] for _ in rows]
    for incidence in edges.values():
        assert len(incidence) == 2
        left, right = incidence
        assert left[:2] == right[:2][::-1]
        neighbors[left[2]].append(right[2])
        neighbors[right[2]].append(left[2])
    seen, pending = set(), [0]
    while pending:
        face = pending.pop()
        if face not in seen:
            seen.add(face)
            pending.extend(neighbors[face])
    assert len(seen) == len(rows)
    return faces


def timeout(_signal, _frame):
    raise TimeoutError('Frozen 600-second support-check budget exhausted')


def main():
    assert not OUT.exists()
    scope = json.loads((E / 'sphere-warm-support-scope-v1.json').read_text())
    assert sha(COUNTEREXAMPLE / 'result.json') == scope['counterexample_result_sha256']
    assert sha(PRIOR / 'result.json') == scope['prior_support_result_sha256']
    counterexample = json.loads((COUNTEREXAMPLE / 'result.json').read_text())
    prior = json.loads((PRIOR / 'result.json').read_text())
    assert counterexample['complete'] and not counterexample['trial_complete']
    assert prior['complete']
    inputs = {str(p.relative_to(E)): sha(p) for p in [Path(__file__),
        E / 'sphere-warm-support-scope-v1.json',
        E / 'check_sphere_trace_cyclotomic_v1.py',
        E / 'check_native_sphere_support_v1.py',
        COUNTEREXAMPLE / 'result.json', PRIOR / 'result.json']}
    for directory, result in [(COUNTEREXAMPLE, counterexample), (PRIOR, prior)]:
        for name, digest in result['artifacts_sha256'].items():
            path = directory / name
            assert sha(path) == digest
            inputs[str(path.relative_to(E))] = digest
    oracle = load_module('warm_support_field', E / 'check_sphere_trace_cyclotomic_v1.py')
    bounds = load_module('warm_support_bounds', E / 'check_native_sphere_support_v1.py')
    cold = parse((COUNTEREXAMPLE / 'cold.stdout').read_text())
    warm = parse((COUNTEREXAMPLE / 'warm.stdout').read_text())
    assert (COUNTEREXAMPLE / 'warm.stdout').read_bytes() == (COUNTEREXAMPLE / 'replay.stdout').read_bytes()
    assert parse((COUNTEREXAMPLE / 'replay.stdout').read_text()) == warm
    changed = json.loads((COUNTEREXAMPLE / 'changed-faces.json').read_text())
    added, removed = sorted(warm - cold), sorted(cold - warm)
    assert added == [tuple(row) for row in changed['added_original_index_faces']]
    assert removed == [tuple(row) for row in changed['removed_original_index_faces']]
    assert len(added) == len(removed) == 118 and len(cold & warm) == 842
    prior_faces = {canonical(tuple(row['indices'])): row for row in
                   json.loads((PRIOR / 'original_medium.json').read_text())['faces']}
    assert set(prior_faces) == cold
    assert all(face in prior_faces for face in cold & warm)
    OUT.mkdir()
    pi = bounds.pi_interval()
    dump(OUT / 'protocol.json', {'inputs_sha256': inputs,
        'new_face_point_pairs': 56876, 'reused_oriented_faces': 842,
        'interval_bits': bounds.BITS, 'atan_terms': bounds.ATAN_TERMS,
        'cos_terms': bounds.COS_TERMS, 'wall_stop_seconds': 600,
        'pi_interval': [[value.numerator, value.denominator] for value in pi]})
    field = oracle.Field(32, 16)
    assert field.order == 32
    points = [field.point(i) for i in range(482)]
    cosines = [bounds.cosine_interval(k, field.order, pi)
               for k in range(len(field.modulus) - 1)]
    intervals = [[bounds.evaluate_real(coordinate, cosines) for coordinate in point]
                 for point in points]
    faces = []
    totals = {'strictly_inside': 0, 'coplanar': 0}
    for face_index, (a, b, c) in enumerate(added):
        ab = [field.sub(x, y) for x, y in zip(points[b], points[a])]
        ac = [field.sub(x, y) for x, y in zip(points[c], points[a])]
        normal = [field.sub(field.mul(ab[i], ac[j]), field.mul(ab[j], ac[i]))
                  for i, j in [(1, 2), (2, 0), (0, 1)]]
        assert any(normal), (face_index, 'degenerate')
        normal_intervals = [bounds.evaluate_real(axis, cosines) for axis in normal]
        assert sum(bounds.product(axis, axis)[0] for axis in normal_intervals) > 0
        zero_indices = []
        largest_negative_upper = None
        for index, point in enumerate(points):
            lo = hi = 0
            for axis in range(3):
                lower, upper = bounds.product(normal_intervals[axis],
                    bounds.subtract(intervals[index][axis], intervals[a][axis]))
                lo += lower
                hi += upper
            assert lo <= hi
            if hi < 0:
                totals['strictly_inside'] += 1
                largest_negative_upper = hi if largest_negative_upper is None else max(hi, largest_negative_upper)
            elif lo > 0:
                raise AssertionError((face_index, index, 'outside', lo, hi))
            else:
                remainder = field.orient(points[a], points[b], points[c], point)
                assert not remainder, (face_index, index, 'unresolved_nonzero', remainder)
                totals['coplanar'] += 1
                zero_indices.append(index)
        assert {a, b, c}.issubset(zero_indices) and largest_negative_upper is not None
        faces.append({'indices': [a, b, c], 'anchor_normal': normal,
            'coplanar_original_indices': zero_indices,
            'largest_strict_upper_numerator_over_2_pow_192': largest_negative_upper})
    assert sum(totals.values()) == 56876
    removed_planes = Counter(tuple(prior_faces[face]['coplanar_original_indices']) for face in removed)
    added_planes = Counter(tuple(face['coplanar_original_indices']) for face in faces)
    assert removed_planes == added_planes
    dump(OUT / 'changed-face-support.json', {'cosine_enclosures_over_2_pow_96': cosines,
        'faces': faces, 'totals': totals})
    dump(OUT / 'changed-planes.json', [{'coplanar_original_indices': list(plane),
        'removed_triangles': count, 'added_triangles': added_planes[plane]}
        for plane, count in sorted(removed_planes.items())])
    assert all(sha(E / name) == digest for name, digest in inputs.items())
    dump(OUT / 'result.json', {'complete': True, 'new_faces_checked': 118,
        'unchanged_original_points': 482, 'new_face_point_pairs': 56876, **totals,
        'previously_proved_shared_oriented_faces': 842,
        'all_960_warm_faces_nondegenerate_outward_and_supporting': True,
        'all_original_vertices_used_and_closed_connected_oriented_euler_two': True,
        'changed_supporting_planes': len(removed_planes),
        'changed_plane_triangle_counts_preserved': True,
        'warm_equals_replay': True, 'all_inputs_unchanged': True,
        'artifacts_sha256': {p.name: sha(p) for p in sorted(OUT.iterdir()) if p.is_file()},
        'claim': 'The observed warm/replay output is the same exact convex hull of the original finite point set, with different triangulation and vertex order. All changed supporting planes and their triangle counts agree with the prior cold output.',
        'limits': 'Finite corpus proof using exact Python arithmetic and prior independently checked faces; not formal verification of the native implementation or evidence of the cause of row instability.',
        'resource_trial_complete': False, 'runtime_admitted': False, 'plan_complete': False})
    print(json.dumps({'complete': True, 'new_face_point_pairs': 56876,
        'changed_supporting_planes': len(removed_planes),
        'result_sha256': sha(OUT / 'result.json')}), flush=True)


if __name__ == '__main__':
    signal.signal(signal.SIGALRM, timeout)
    signal.alarm(600)
    main()
