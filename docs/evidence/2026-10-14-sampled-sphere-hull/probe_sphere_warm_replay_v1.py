"""Fixed default-allocator replay diagnostic; no timing or resource comparison."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'sphere-warm-replay-v1'
WORK = ROOT / 'target/sphere-warm-replay-v1'
CASES = {'fifteen_degrees': (24, 12), 'original_medium': (32, 16),
         'fifth_turn': (10, 7), 'seventh_turn': (7, 5)}
GROUPS = {'baseline': list(CASES)[:1], 'late': list(CASES), 'image': list(CASES)}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def parse(raw, case):
    segments, stacks = CASES[case]
    count = segments * (stacks - 1) + 2
    expected_last = f'result case={case} certified=true original_positions_preserved=true positions={count} triangles={2 * count - 4}'
    assert raw.splitlines()[-1] == expected_last
    order = json.loads(next(line.split('=', 1)[1] for line in raw.splitlines()
                            if line.startswith('position_order=')))
    assert sorted(order) == list(range(count))
    rows = [tuple(order[int(i)] for i in row) for row in re.findall(
        r'Triangle \{ v0: (\d+), v1: (\d+), v2: (\d+) \}', raw)]
    assert len(rows) == 2 * count - 4 and all(len(set(row)) == 3 for row in rows)
    faces = {min(row, row[1:] + row[:1], row[2:] + row[:2]) for row in rows}
    assert len(faces) == len(rows)
    assert {i for row in rows for i in row} == set(range(count))
    edges = {}
    for a, b, c in rows:
        for u, v in [(a, b), (b, c), (c, a)]:
            edges.setdefault(tuple(sorted((u, v))), []).append((u, v))
    assert count - len(edges) + len(rows) == 2
    assert all(len(pair) == 2 and pair[0] == pair[1][::-1] for pair in edges.values())
    return order, faces


def main():
    assert not OUT.exists() and not WORK.exists()
    scope = json.loads((E / 'sphere-warm-replay-scope-v1.json').read_text())
    assert scope['groups'] == GROUPS
    assert sha(E / 'sphere-warm-row-counterexample-v1/result.json') == scope['parent_failure_result_sha256']
    assert sha(E / 'sphere-warm-support-v1/result.json') == scope['independent_changed_face_support_result_sha256']
    names = ['probe_sphere_warm_replay_v1.py', 'sphere_warm_replay_probe_v1.rs',
        'sphere-warm-replay-scope-v1.json', 'sphere-resource-failure-closure-v1.json',
        'sphere-warm-row-counterexample-v1/result.json', 'sphere-warm-support-v1/result.json',
        'sphere-native-support-v1/result.json',
        *[f'sphere-cyclotomic-v2/candidate-{case}.stdout' for case in CASES]]
    inputs = {name: sha(E / name) for name in names}
    for name in ['sphere-warm-row-counterexample-v1', 'sphere-warm-support-v1', 'sphere-native-support-v1']:
        result = json.loads((E / name / 'result.json').read_text())
        assert result['complete']
        for artifact, digest in result['artifacts_sha256'].items():
            assert sha(E / name / artifact) == digest
            inputs[name + '/' + artifact] = digest
    specifications = {'baseline': ('mesh-identity-preparation-v1', 'mesh-identity-qualification-v1'),
        'late': ('sphere-cyclotomic-v2', 'sphere-cyclotomic-v2'),
        'image': ('sphere-cyclotomic-image-v1', 'sphere-cyclotomic-image-v1')}
    sources, mappings, libraries = {}, {}, {}
    for group, (preparation, build) in specifications.items():
        source_protocol = json.loads((E / preparation / 'protocol.json').read_text())
        sources[group] = Path(source_protocol['source'])
        mappings[group] = json.loads(gzip.decompress((E / preparation / 'source-hashes.json.gz').read_bytes()))
        inputs[preparation + '/source-hashes.json.gz'] = sha(E / preparation / 'source-hashes.json.gz')
        libraries[group] = json.loads((E / build / 'libraries.json').read_text())
        result = json.loads((E / build / 'result.json').read_text())
        assert result['complete']
        inputs[build + '/result.json'] = sha(E / build / 'result.json')
        for name, digest in result['artifacts_sha256'].items():
            assert sha(E / build / name) == digest
            inputs[build + '/' + name] = digest
    features = {group: {name: row['features'] for name, row in libs.items()}
                for group, libs in libraries.items()}
    assert features['baseline'] == features['late'] == features['image']
    root_mapping = {name: sha(ROOT / name) for name in mappings['baseline']}
    assert all(root_mapping[name] == digest for name, digest in mappings['baseline'].items()
               if name != 'docs/properties-status.json')
    expected = {case: (E / f'sphere-cyclotomic-v2/candidate-{case}.stdout').read_text() for case in CASES}
    known_warm = (E / 'sphere-warm-row-counterexample-v1/warm.stdout').read_text()
    binaries = {}

    def guard(full=False):
        assert all(sha(E / name) == digest for name, digest in inputs.items())
        assert all(sha(row['path']) == row['sha256'] for libs in libraries.values() for row in libs.values())
        assert all(sha(row['path']) == row['sha256'] for row in binaries.values())
        if full:
            assert all(sha(ROOT / name) == digest for name, digest in root_mapping.items())
            for group, mapping in mappings.items():
                assert all(sha(sources[group] / name) == digest for name, digest in mapping.items())

    guard(full=True)
    OUT.mkdir()
    WORK.mkdir()
    cpu = min(os.sched_getaffinity(0))
    (OUT / 'root-source-hashes.json.gz').write_bytes(gzip.compress(
        (json.dumps(root_mapping, sort_keys=True) + '\n').encode(), mtime=0))
    dump(OUT / 'protocol.json', {'inputs_sha256': inputs, 'libraries': libraries,
        'sources': {group: str(path) for group, path in sources.items()},
        'source_files': {group: len(mapping) for group, mapping in mappings.items()},
        'root_source_files': len(root_mapping), 'case_sets': GROUPS,
        'fixed_processes': 9, 'queries_per_process': 3, 'pinned_cpu': cpu,
        'compiler': subprocess.check_output(['rustc', '-Vv'], text=True),
        'host': list(os.uname()), 'not_a_cost_trial': True,
        'interpretation': 'Output-only diagnostic with default allocator; process wall records identify executions, not performance. No allocation, timing or speedup comparison.',
        'runtime_admitted': False, 'plan_complete': False})
    commands, observations = [], []

    def run(name, command, group):
        guard()
        print(json.dumps({'start': name}), flush=True)
        started = time.monotonic()
        with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', '600', *command],
                cwd=sources[group], stdout=stdout, stderr=stderr).returncode
        row = {'name': name, 'command': command, 'group': group, 'exit_code': code,
            'diagnostic_process_wall_seconds': time.monotonic() - started,
            'stdout_sha256': sha(OUT / (name + '.stdout')), 'stderr_sha256': sha(OUT / (name + '.stderr'))}
        commands.append(row)
        dump(OUT / 'commands.json', commands)
        print(json.dumps({'finished': name, 'exit_code': code}), flush=True)
        guard()
        if code:
            dump(OUT / 'failure.json', {'complete': False, 'failed_command': row,
                'runtime_admitted': False, 'plan_complete': False})
            raise SystemExit(code)
        return (OUT / (name + '.stdout')).read_text()

    run('format', ['rustfmt', '--edition', '2024', '--check', str(E / 'sphere_warm_replay_probe_v1.rs')], 'baseline')
    for group, libs in libraries.items():
        binary = WORK / group
        run('link-' + group, ['rustc', '--edition=2024', '-O', '-D', 'warnings',
            str(E / 'sphere_warm_replay_probe_v1.rs'), '--extern', 'csgrs=' + libs['csgrs']['path'],
            '-L', 'dependency=' + str(Path(libs['hypermesh']['path']).parent), '-o', str(binary)], group)
        binaries[group] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
    dump(OUT / 'binaries.json', binaries)
    for group, cases in GROUPS.items():
        for case in cases:
            name = group + '-' + case
            raw = run(name, ['taskset', '-c', str(cpu), binaries[group]['path'], case], group)
            parts = raw.split('witness_phase=')
            assert parts[0] == f'replay_case={case} repetitions=3 grammar_policy=default allocator=default\n'
            assert len(parts) == 4
            witnesses, orders, faces = {}, {}, {}
            for phase, part in zip(['cold', 'warm', 'replay'], parts[1:]):
                actual_phase, witness = part.split('\n', 1)
                assert actual_phase == phase
                witnesses[phase] = witness
                orders[phase], faces[phase] = parse(witness, case)
                (OUT / (name + '-' + phase + '.witness')).write_text(witness)
            assert witnesses['cold'] == expected[case]
            observation = {'group': group, 'case': case,
                'all_three_certified_with_complete_vertices_and_closed_oriented_topology': True,
                'cold_matches_prior_exact_output': True,
                'cold_equals_warm': witnesses['cold'] == witnesses['warm'],
                'warm_equals_replay': witnesses['warm'] == witnesses['replay'],
                'oriented_faces_removed_from_cold': len(faces['cold'] - faces['warm']),
                'oriented_faces_added_to_warm': len(faces['warm'] - faces['cold']),
                'first_vertex_order_difference': next((i for i, (a, b) in enumerate(zip(orders['cold'], orders['warm'])) if a != b), None),
                'phase_exact_support_proof_match': {phase: ('prior-cold' if witness == expected[case]
                    else 'prior-32-warm' if case == 'original_medium' and witness == known_warm else None)
                    for phase, witness in witnesses.items()},
                'witnesses_sha256': {phase: hashlib.sha256(witness.encode()).hexdigest() for phase, witness in witnesses.items()}}
            observations.append(observation)
            dump(OUT / 'observations.json', observations)
            print(json.dumps(observation), flush=True)
    assert len(commands) == 13 and len(observations) == 9
    guard(full=True)
    dump(OUT / 'result.json', {'complete': True, 'commands_all_zero': 13,
        'processes': 9, 'hull_queries': 27,
        'all_frozen_sources_inputs_and_libraries_unchanged': True,
        'all_phases_match_prior_exact_support_proofs': all(all(row['phase_exact_support_proof_match'].values()) for row in observations),
        'row_instability_observed': any(not row['cold_equals_warm'] or not row['warm_equals_replay'] for row in observations),
        'artifacts_sha256': {p.name: sha(p) for p in sorted(OUT.iterdir()) if p.is_file()},
        'not_a_cost_trial': True, 'runtime_admitted': False, 'plan_complete': False})
    print(json.dumps({'complete': True, 'result_sha256': sha(OUT / 'result.json')}), flush=True)


if __name__ == '__main__':
    main()
