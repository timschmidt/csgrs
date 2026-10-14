"""Independent full support proof for the deterministic-selection finite corpus."""
from collections import Counter
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import signal
import sys

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
ROOT = E.parents[2]
TRIAL = E / 'sphere-stable-selection-v1'
OUT = E / 'sphere-stable-support-v1'
CASES = [('fifteen_degrees', 24, 12), ('original_medium', 32, 16),
         ('fifth_turn', 10, 7), ('seventh_turn', 7, 5)]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def parse(raw, count):
    order = json.loads(next(line.split('=', 1)[1] for line in raw.splitlines()
                            if line.startswith('position_order=')))
    assert sorted(order) == list(range(count))
    rows = [tuple(order[int(i)] for i in row) for row in re.findall(
        r'Triangle \{ v0: (\d+), v1: (\d+), v2: (\d+) \}', raw)]
    assert len(rows) == 2 * count - 4 and all(len(set(row)) == 3 for row in rows)
    assert len({min(row, row[1:] + row[:1], row[2:] + row[:2]) for row in rows}) == len(rows)
    assert {index for row in rows for index in row} == set(range(count))
    edges = {}
    for face, (a, b, c) in enumerate(rows):
        for u, v in [(a, b), (b, c), (c, a)]:
            edges.setdefault(tuple(sorted((u, v))), []).append((u, v, face))
    assert count - len(edges) + len(rows) == 2
    neighbors = [[] for _ in rows]
    for incidence in edges.values():
        assert len(incidence) == 2 and incidence[0][:2] == incidence[1][:2][::-1]
        left, right = incidence[0][2], incidence[1][2]
        neighbors[left].append(right)
        neighbors[right].append(left)
    seen, pending = set(), [0]
    while pending:
        face = pending.pop()
        if face not in seen:
            seen.add(face)
            pending.extend(neighbors[face])
    assert len(seen) == len(rows)
    return rows


def timeout(_signal, _frame):
    raise TimeoutError('Frozen 600-second independent support budget exhausted')


def main():
    assert not OUT.exists()
    result = json.loads((TRIAL / 'result.json').read_text())
    assert result['complete'] and result['commands'] == 18
    assert result['native_regression_passed'] and result['dyadic_cube_parent_full_transcript_preserved']
    inputs = {str(path.relative_to(E)): sha(path) for path in [Path(__file__),
        E / 'sphere-stable-support-scope-v1.json', E / 'check_sphere_trace_cyclotomic_v1.py',
        E / 'check_native_sphere_support_v1.py', TRIAL / 'result.json']}
    for name, digest in result['artifacts_sha256'].items():
        assert sha(TRIAL / name) == digest
        inputs[str((TRIAL / name).relative_to(E))] = digest
    protocol = json.loads((TRIAL / 'protocol.json').read_text())
    inputs.update(protocol['inputs_sha256'])
    commands = json.loads((TRIAL / 'commands.json').read_text())
    assert len(commands) == 18
    assert all(row['exit_code'] == row['expected_exit_code'] for row in commands)
    assert [(row['name'], row['exit_code']) for row in commands if row['exit_code']] == [('negative-regression', 101)]
    assert 'vertex row order changed for 32x16' in (TRIAL / 'negative-regression.stderr').read_text()
    assert 'test result: ok. 1 passed; 0 failed;' in (TRIAL / 'candidate-regression.stdout').read_text()
    libraries = json.loads((TRIAL / 'libraries.json').read_text())
    binaries = json.loads((TRIAL / 'binaries.json').read_text())
    source_mapping = json.loads(gzip.decompress((TRIAL / 'source-hashes.json.gz').read_bytes()))
    root_mapping = json.loads(gzip.decompress((TRIAL / 'root-source-hashes.json.gz').read_bytes()))
    parent_mapping = json.loads(gzip.decompress((E / 'sphere-cyclotomic-image-v1/source-hashes.json.gz').read_bytes()))

    def guard():
        assert all(sha(E / name) == digest for name, digest in inputs.items())
        for records in [libraries, binaries, protocol['parent_libraries']]:
            assert all(sha(row['path']) == row['sha256'] for row in records.values())
        for base, mapping in [(Path(protocol['source']), source_mapping),
                              (ROOT, root_mapping), (Path(protocol['parent_source']), parent_mapping)]:
            assert all(sha(base / name) == digest for name, digest in mapping.items())

    guard()
    oracle = load_module('stable_support_field', E / 'check_sphere_trace_cyclotomic_v1.py')
    bounds = load_module('stable_support_bounds', E / 'check_native_sphere_support_v1.py')
    OUT.mkdir()
    pi = bounds.pi_interval()
    dump(OUT / 'protocol.json', {'inputs_sha256': inputs, 'interval_bits': bounds.BITS,
        'atan_terms': bounds.ATAN_TERMS, 'cos_terms': bounds.COS_TERMS,
        'pi_interval': [[value.numerator, value.denominator] for value in pi],
        'complete_face_point_pairs': 612288, 'wall_stop_seconds': 600})
    cases = []
    for case, segments, stacks in CASES:
        raw = (TRIAL / (case + '-cold.witness')).read_text()
        assert all(raw == (TRIAL / (case + '-' + phase + '.witness')).read_text() for phase in ['warm', 'replay'])
        count = segments * (stacks - 1) + 2
        rows = parse(raw, count)
        field = oracle.Field(segments, stacks)
        points = [field.point(i) for i in range(count)]
        assert len({tuple(tuple(axis) for axis in point) for point in points}) == count
        cosines = [bounds.cosine_interval(k, field.order, pi) for k in range(len(field.modulus) - 1)]
        intervals = [[bounds.evaluate_real(axis, cosines) for axis in point] for point in points]
        faces = []
        totals = {'strictly_inside': 0, 'coplanar': 0}
        for face_index, (a, b, c) in enumerate(rows):
            ab = [field.sub(x, y) for x, y in zip(points[b], points[a])]
            ac = [field.sub(x, y) for x, y in zip(points[c], points[a])]
            normal = [field.sub(field.mul(ab[i], ac[j]), field.mul(ab[j], ac[i]))
                      for i, j in [(1, 2), (2, 0), (0, 1)]]
            assert any(normal), (case, face_index, 'degenerate')
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
                    raise AssertionError((case, face_index, index, 'outside', lo, hi))
                else:
                    remainder = field.orient(points[a], points[b], points[c], point)
                    assert not remainder, (case, face_index, index, 'unresolved_nonzero', remainder)
                    totals['coplanar'] += 1
                    zero_indices.append(index)
            assert {a, b, c}.issubset(zero_indices) and largest_negative_upper is not None
            faces.append({'indices': [a, b, c], 'anchor_normal': normal,
                'coplanar_original_indices': zero_indices,
                'largest_strict_upper_numerator_over_2_pow_192': largest_negative_upper})
        assert sum(totals.values()) == len(rows) * count
        planes = Counter(tuple(face['coplanar_original_indices']) for face in faces)
        assert all(len(plane) in [3, 4] and triangles == len(plane) - 2 for plane, triangles in planes.items())
        dump(OUT / (case + '.json'), {'root_of_unity_order': field.order,
            'cosine_enclosures_over_2_pow_96': cosines, 'faces': faces, 'totals': totals,
            'supporting_plane_triangle_counts': [{'original_indices': list(plane), 'triangles': triangles}
                for plane, triangles in sorted(planes.items())]})
        row = {'case': case, 'points': count, 'triangles': len(rows),
            'face_point_pairs': count * len(rows), **totals,
            'all_faces_nondegenerate_outward_and_supporting': True,
            'closed_connected_oriented_topology': True, 'supporting_planes': len(planes),
            'all_supporting_planes_completely_triangulated': True,
            'cold_warm_replay_transcripts_identical': True}
        cases.append(row)
        print(json.dumps(row), flush=True)
    assert sum(row['face_point_pairs'] for row in cases) == 612288
    guard()
    dump(OUT / 'result.json', {'complete': True, 'native_trial_driver_terminal_exit_code': 0,
        'native_commands_expected_zero': 17, 'expected_negative_control_failures': 1,
        'cases': cases, 'complete_face_point_pairs': 612288,
        'all_frozen_sources_inputs_libraries_and_binaries_rechecked': True,
        'artifacts_sha256': {path.name: sha(path) for path in sorted(OUT.iterdir()) if path.is_file()},
        'claim': 'All twelve replay outputs describe the exact convex hulls of the four unchanged original finite sampled point sets. Each case has one identical cold/warm/replay transcript. Changed rows have been checked independently, not inferred from counts.',
        'limits': 'Finite corpus validation with exact Python arithmetic, not formal verification of the native implementation or a performance comparison.',
        'runtime_admitted': False, 'plan_complete': False})
    print(json.dumps({'complete': True, 'result_sha256': sha(OUT / 'result.json')}), flush=True)


if __name__ == '__main__':
    signal.signal(signal.SIGALRM, timeout)
    signal.alarm(600)
    main()
