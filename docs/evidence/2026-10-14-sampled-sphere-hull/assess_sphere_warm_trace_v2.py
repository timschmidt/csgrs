"""Compare frozen scheduling traces and reconstruct every selected float score."""
import ast
import gzip
import hashlib
import json
import math
from pathlib import Path
import re
import struct

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
TRIAL = E / 'sphere-warm-trace-v2'
OUT = E / 'sphere-warm-trace-analysis-v2'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def bits(value):
    return struct.unpack('>Q', struct.pack('>d', value))[0]


def from_bits(value):
    return struct.unpack('>d', struct.pack('>Q', value))[0]


def distance(points, face, point):
    a, b, c = [points[index] for index in face]
    p = points[point]
    ab = [b[i] - a[i] for i in range(3)]
    ac = [c[i] - a[i] for i in range(3)]
    ap = [p[i] - a[i] for i in range(3)]
    determinant = ((ab[1] * ac[2] - ab[2] * ac[1]) * ap[0]
        + (ab[2] * ac[0] - ab[0] * ac[2]) * ap[1]
        + (ab[0] * ac[1] - ab[1] * ac[0]) * ap[2])
    return abs(determinant) if math.isfinite(determinant) else 0.0


def select(points, face, outside):
    scores = [distance(points, face, point) for point in outside]
    assert scores and all(math.isfinite(score) and score >= 0 for score in scores)
    # Rust Iterator::max_by returns the last equal maximum. All finite absolute
    # scores here are nonnegative, so numeric order equals total_cmp order.
    position = max(range(len(outside)), key=lambda i: (scores[i], i))
    score = scores[position]
    return {'eye': outside[position], 'score_bits': bits(score),
        'tied_maximum_points_in_iteration_order': [point for point, value in zip(outside, scores) if value == score]}


def parse(raw):
    phases = []
    current = None
    for line in raw.splitlines():
        prefix = 'hull-warm-trace '
        assert line.startswith(prefix)
        line = line[len(prefix):]
        if line.startswith('begin '):
            match = re.fullmatch(r'begin points=(\d+) seed=(\[.*\])', line)
            assert match and int(match[1]) == 482
            current = {'seed': ast.literal_eval(match[2]), 'approximate_bits': [],
                'bvh': [], 'faces': [], 'fallback_faces': [], 'selections': []}
            phases.append(current)
        elif line.startswith('approx '):
            match = re.fullmatch(r'approx point=(\d+) bits=(\[.*\])', line)
            assert match and int(match[1]) == len(current['approximate_bits'])
            row = ast.literal_eval(match[2])
            assert len(row) == 3 and all(0 <= value < 1 << 64 for value in row)
            current['approximate_bits'].append(row)
        elif line.startswith('bvh '):
            assert line in ['bvh available=true', 'bvh available=false']
            current['bvh'].append(line.endswith('true'))
        elif line.startswith('face '):
            match = re.fullmatch(r'face vertices=(\[.*?\]) outside=(\[.*\])', line)
            assert match
            current['faces'].append({'face': ast.literal_eval(match[1]), 'outside': ast.literal_eval(match[2])})
        elif line.startswith('fallback '):
            match = re.fullmatch(r'fallback face=(\[.*\])', line)
            assert match
            current['fallback_faces'].append(ast.literal_eval(match[1]))
        else:
            match = re.fullmatch(r'select face=(\[.*?\]) outside=(\[.*?\]) eye=(\d+) score_bits=(\d+)', line)
            assert match
            current['selections'].append({'face': ast.literal_eval(match[1]),
                'outside': ast.literal_eval(match[2]), 'eye': int(match[3]), 'score_bits': int(match[4])})
    assert len(phases) == 3
    for phase in phases:
        assert len(phase['approximate_bits']) == 482 and len(phase['bvh']) == 1
        phase['points'] = [[from_bits(value) for value in row] for row in phase['approximate_bits']]
        for selection in phase['selections']:
            assert len(set(selection['outside'])) == len(selection['outside'])
            computed = select(phase['points'], selection['face'], selection['outside'])
            assert computed['eye'] == selection['eye'] and computed['score_bits'] == selection['score_bits']
    return phases


def main():
    assert not OUT.exists()
    result = json.loads((TRIAL / 'result.json').read_text())
    assert result['complete'] and result['commands_all_zero'] == 6
    assert result['all_three_stdout_transcripts_identical_to_uninstrumented_replay']
    assert all(sha(TRIAL / name) == digest for name, digest in result['artifacts_sha256'].items())
    protocol = json.loads((TRIAL / 'protocol.json').read_text())
    assert all(sha(E / name) == digest for name, digest in protocol['inputs_sha256'].items())
    for name, base in [('source-hashes.json.gz', Path(protocol['source'])),
                       ('root-source-hashes.json.gz', ROOT)]:
        mapping = json.loads(gzip.decompress((TRIAL / name).read_bytes()))
        assert all(sha(base / path) == digest for path, digest in mapping.items())
    parent_mapping = json.loads(gzip.decompress((E / 'sphere-cyclotomic-image-v1/source-hashes.json.gz').read_bytes()))
    assert all(sha(Path(protocol['parent_source']) / path) == digest for path, digest in parent_mapping.items())
    libraries = json.loads((TRIAL / 'libraries.json').read_text())
    assert all(sha(row['path']) == row['sha256'] for row in libraries.values())
    binary = json.loads((TRIAL / 'binary.json').read_text())
    assert sha(binary['path']) == binary['sha256']
    commands = json.loads((TRIAL / 'commands.json').read_text())
    assert len(commands) == 6 and all(row['exit_code'] == 0 for row in commands)
    cold, warm, replay = parse((TRIAL / 'original_medium.stderr').read_text())
    phases = [cold, warm, replay]
    first = next((i for i, (a, b) in enumerate(zip(cold['selections'], warm['selections']))
                  if a['face'] != b['face'] or a['eye'] != b['eye']), None)
    assert first is not None
    coordinate_differences = [{'point': point, 'axis': axis, 'cold_bits': a[axis],
        'warm_bits': b[axis], 'replay_bits': c[axis]}
        for point, (a, b, c) in enumerate(zip(*(phase['approximate_bits'] for phase in phases)))
        for axis in range(3) if a[axis] != b[axis] or b[axis] != c[axis]]
    selected_cold, selected_warm = cold['selections'][first], warm['selections'][first]
    same_face = selected_cold['face'] == selected_warm['face']
    same_set = set(selected_cold['outside']) == set(selected_warm['outside'])
    cross = []
    if same_face:
        for coordinate_name, coordinates in [('cold', cold['points']), ('warm', warm['points'])]:
            for order_name, selection in [('cold', selected_cold), ('warm', selected_warm)]:
                cross.append({'coordinates': coordinate_name, 'outside_order': order_name,
                    **select(coordinates, selected_cold['face'], selection['outside'])})
    OUT.mkdir()
    dump(OUT / 'coordinate-differences.json', coordinate_differences)
    dump(OUT / 'first-selection-divergence.json', {'selection_index': first,
        'cold': selected_cold, 'warm': selected_warm, 'same_face': same_face,
        'same_outside_set': same_set, 'crossed_coordinate_and_outside_order_choices': cross})
    dump(OUT / 'result.json', {'complete': True, 'trace_driver_terminal_exit_code': 0,
        'commands_all_zero': 6, 'trace_result_sha256': sha(TRIAL / 'result.json'),
        'assessor_sha256': sha(__file__), 'all_frozen_sources_inputs_and_binaries_rechecked': True,
        'full_stdout_matches_uninstrumented_original_medium': True,
        'coordinate_components_with_changed_bits': len(coordinate_differences),
        'warm_and_replay_coordinate_bits_equal': warm['approximate_bits'] == replay['approximate_bits'],
        'seeds_equal': cold['seed'] == warm['seed'] == replay['seed'],
        'phase_bvh_available': [phase['bvh'][0] for phase in phases],
        'phase_face_counts': [len(phase['faces']) for phase in phases],
        'phase_bvh_to_linear_fallback_counts': [len(phase['fallback_faces']) for phase in phases],
        'phase_selection_counts': [len(phase['selections']) for phase in phases],
        'every_native_eye_and_selected_score_reconstructed_from_logged_data': True,
        'first_different_eye_or_face_index': first,
        'first_different_selection_same_face': same_face,
        'first_different_selection_same_outside_set': same_set,
        'warm_and_replay_selection_traces_equal': warm['selections'] == replay['selections'],
        'artifacts_sha256': {path.name: sha(path) for path in sorted(OUT.iterdir()) if path.is_file()},
        'limits': 'Reconstruction concerns floating scheduling data and point order only; it is not an exact-geometry predicate or a repair. All native outputs match the independently proved reference hulls.',
        'not_a_cost_trial': True, 'runtime_admitted': False, 'plan_complete': False})
    print(json.dumps({'complete': True, 'coordinate_components_with_changed_bits': len(coordinate_differences),
        'first_different_eye_or_face_index': first, 'result_sha256': sha(OUT / 'result.json')}))


if __name__ == '__main__':
    main()
