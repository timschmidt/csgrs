"""Preserve the closed resource trial's first exact-row mismatch."""
import gzip
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
TRIAL = E / 'sphere-cyclotomic-resources-v1'
OUT = E / 'sphere-warm-row-counterexample-v1'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def parse(text):
    lines = text.splitlines()
    order = json.loads(next(line.split('=', 1)[1] for line in lines if line.startswith('position_order=')))
    assert sorted(order) == list(range(482))
    triangles = [tuple(map(int, row)) for row in re.findall(r'Triangle \{ v0: (\d+), v1: (\d+), v2: (\d+) \}', text)]
    assert len(triangles) == 960 and all(len(set(t)) == 3 for t in triangles)
    mapped = [tuple(order[index] for index in triangle) for triangle in triangles]
    canonical = lambda triangle: min(triangle, triangle[1:] + triangle[:1], triangle[2:] + triangle[:2])
    faces = set(map(canonical, mapped))
    assert len(faces) == 960
    edges = {}
    for a, b, c in mapped:
        for u, v in [(a, b), (b, c), (c, a)]:
            edges.setdefault(tuple(sorted((u, v))), []).append((u, v))
    assert len(edges) == 1440
    assert all(len(rows) == 2 and rows[0] == rows[1][::-1] for rows in edges.values())
    assert 482 - len(edges) + 960 == 2
    return order, mapped, faces


def main():
    assert not OUT.exists() and not (TRIAL / 'result.json').exists()
    protocol = json.loads((TRIAL / 'protocol.json').read_text())
    assert all(sha(E / name) == digest for name, digest in protocol['inputs_sha256'].items())
    assert all(sha(row['path']) == row['sha256'] for libs in protocol['libraries'].values() for row in libs.values())
    binaries = json.loads((TRIAL / 'binaries.json').read_text())
    assert all(sha(row['path']) == row['sha256'] for row in binaries.values())
    root_mapping = json.loads(gzip.decompress((TRIAL / 'root-source-hashes.json.gz').read_bytes()))
    assert all(sha(ROOT / name) == digest for name, digest in root_mapping.items())
    for group, preparation in [('baseline', 'mesh-identity-preparation-v1'),
                               ('late', 'sphere-cyclotomic-v2'), ('image', 'sphere-cyclotomic-image-v1')]:
        path = E / preparation / 'source-hashes.json.gz'
        assert sha(path) == protocol['source_mappings_sha256'][group]
        mapping = json.loads(gzip.decompress(path.read_bytes()))
        assert all(sha(Path(protocol['sources'][group]) / name) == digest for name, digest in mapping.items())
    commands = json.loads((TRIAL / 'commands.json').read_text())
    assert len(commands) == 10 and all(row['exit_code'] == 0 for row in commands)
    assert commands[-1]['name'] == 'sample-1-late-original_medium'
    for row in commands:
        for stream in ['stdout', 'stderr']:
            assert sha(TRIAL / (row['name'] + '.' + stream)) == row[stream + '_sha256']
    samples = json.loads((TRIAL / 'samples.json').read_text())
    assert [s['name'] for s in samples] == ['sample-1-baseline-fifteen_degrees', 'sample-1-late-fifteen_degrees']
    raw = (TRIAL / 'sample-1-late-original_medium.stdout').read_text()
    driver_error = (E / 'sphere-cyclotomic-resources-driver-v1.stderr').read_text()
    assert 'parse_output' in driver_error and driver_error.rstrip().endswith('AssertionError')
    parts = raw.split('witness_phase=')[1:]
    assert len(parts) == 3
    OUT.mkdir()
    observations = {}
    for part, phase in zip(parts, ['cold', 'warm', 'replay']):
        lines = part.splitlines()
        assert lines[0] == phase
        witness = '\n'.join(line for line in lines[1:] if not line.startswith('{')) + '\n'
        assert witness.splitlines()[-1] == 'result case=original_medium certified=true original_positions_preserved=true positions=482 triangles=960'
        (OUT / (phase + '.stdout')).write_text(witness)
        order, mapped, faces = parse(witness)
        observations[phase] = {'witness': witness, 'order': order, 'mapped': mapped, 'faces': faces}
    cold = observations['cold']
    warm = observations['warm']
    assert cold['witness'] == (E / 'sphere-cyclotomic-v2/candidate-original_medium.stdout').read_text()
    assert warm['witness'] == observations['replay']['witness'] != cold['witness']
    removed = sorted(cold['faces'] - warm['faces'])
    added = sorted(warm['faces'] - cold['faces'])
    assert len(removed) == len(added) == 118
    dump(OUT / 'changed-faces.json', {'removed_original_index_faces': removed, 'added_original_index_faces': added,
        'shared_oriented_faces': len(cold['faces'] & warm['faces']),
        'first_vertex_order_difference': next(i for i, (a, b) in enumerate(zip(cold['order'], warm['order'])) if a != b)})
    dump(OUT / 'result.json', {
        'complete': True, 'trial_complete': False, 'trial_driver_terminal_exit_code': 1,
        'native_commands_all_zero': 10, 'completed_sample_processes': 3,
        'samples_with_all_transcripts_accepted': 2, 'planned_sample_processes': 27,
        'failure': 'Full-resolution 32x16 late-candidate warm/replay hulls differ from the first query and the frozen cold reference.',
        'cold_output_matches_prior_independent_support_proof': True,
        'warm_equals_replay': True, 'changed_oriented_faces': 118,
        'all_three_have_all_482_original_vertices_and_closed_oriented_960_face_euler_two_topology': True,
        'geometry_support_of_changed_faces_proved': False,
        'finite_image_candidate_sampled': False,
        'frozen_sources_inputs_and_libraries_rechecked': True,
        'trial_artifacts_sha256': {p.name: sha(p) for p in sorted(TRIAL.iterdir()) if p.is_file()},
        'driver_artifacts_sha256': {name: sha(E / name) for name in [
            'sphere-cyclotomic-resources-driver-v1.stdout', 'sphere-cyclotomic-resources-driver-v1.stderr']},
        'assessor_sha256': sha(__file__),
        'artifacts_sha256': {p.name: sha(p) for p in sorted(OUT.iterdir()) if p.is_file()},
        'interpretation': 'Deterministic row preservation failed. This does not yet prove a wrong geometric set or contradictory scalar facts. The image candidate and allocator-free replay require separate diagnostics. No partial allocation comparison is admitted.',
        'trial_may_be_resumed_extended_or_pooled': False,
        'runtime_admitted': False, 'plan_complete': False,
    })
    print(json.dumps({'complete': True, 'trial_complete': False, 'changed_faces': 118,
                      'result_sha256': sha(OUT / 'result.json')}))


if __name__ == '__main__':
    main()
