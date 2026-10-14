"""Fixed feasibility trial for deterministic hull eye selection; no admission."""
import gzip
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'sphere-stable-selection-v1'
WORK = ROOT / 'target/sphere-stable-selection-v1'
SOURCE = WORK / 'source'
TARGET = WORK / 'target'
ENV = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
       'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never', 'CARGO_TARGET_DIR': str(TARGET)}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def main():
    assert not OUT.exists() and not WORK.exists()
    scope = json.loads((E / 'sphere-stable-selection-scope-v1.json').read_text())
    assert sha(E / 'sphere-warm-trace-interpretation-v2.json') == scope['trace_interpretation_sha256']
    inputs = {name: sha(E / name) for name in [
        'probe_sphere_stable_selection_v1.py', 'sphere-stable-selection-scope-v1.json',
        'sphere-stable-selection-v1.patch', 'sphere_warm_replay_probe_v1.rs',
        'dyadic_hull_rows_probe_v1.rs', 'probe_sphere_warm_replay_v1.py',
        'sphere-warm-trace-interpretation-v2.json', 'sphere-warm-trace-analysis-v2/result.json',
        'sphere-cyclotomic-image-v1/result.json', 'sphere-cyclotomic-image-v1/source-hashes.json.gz',
        'mesh-identity-preparation-v1/source-hashes.json.gz',
        *[f'sphere-cyclotomic-v2/candidate-{case}.stdout' for case in scope['cases']]]}
    parent_result = json.loads((E / 'sphere-cyclotomic-image-v1/result.json').read_text())
    assert parent_result['complete']
    for name, digest in parent_result['artifacts_sha256'].items():
        assert sha(E / 'sphere-cyclotomic-image-v1' / name) == digest
        inputs['sphere-cyclotomic-image-v1/' + name] = digest
    parent_libraries = json.loads((E / 'sphere-cyclotomic-image-v1/libraries.json').read_text())
    parent = ROOT / scope['parent_source']
    mapping = json.loads(gzip.decompress((E / 'sphere-cyclotomic-image-v1/source-hashes.json.gz').read_bytes()))
    baseline = json.loads(gzip.decompress((E / 'mesh-identity-preparation-v1/source-hashes.json.gz').read_bytes()))
    root_mapping = {name: sha(ROOT / name) for name in baseline}
    assert all(root_mapping[name] == digest for name, digest in baseline.items()
               if name != 'docs/properties-status.json')
    expected = mapping | scope['expected_delta_sha256']
    libraries, binaries = {}, {}
    spec = importlib.util.spec_from_file_location('stable_selection_parser', E / 'probe_sphere_warm_replay_v1.py')
    parser = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(parser)

    def guard(full=False, prepared=True):
        assert all(sha(E / name) == digest for name, digest in inputs.items())
        assert all(sha(row['path']) == row['sha256'] for row in parent_libraries.values())
        assert all(sha(row['path']) == row['sha256'] for row in libraries.values())
        assert all(sha(row['path']) == row['sha256'] for row in binaries.values())
        if prepared:
            selected = expected if full else scope['expected_delta_sha256']
            assert all(sha(SOURCE / name) == digest for name, digest in selected.items())
        if full:
            assert all(sha(ROOT / name) == digest for name, digest in root_mapping.items())
            assert all(sha(parent / name) == digest for name, digest in mapping.items())

    guard(full=True, prepared=False)
    OUT.mkdir()
    WORK.mkdir()
    subprocess.run(['cp', '-a', '--reflink=auto', str(parent), str(SOURCE)], check=True)
    cpu = min(os.sched_getaffinity(0))
    commands = []

    def run(name, command, seconds=900, expected_code=0, prepared=True):
        guard(prepared=prepared)
        print(json.dumps({'start': name}), flush=True)
        start = time.monotonic()
        with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', str(seconds), *command],
                cwd=SOURCE, env=os.environ | ENV, stdout=stdout, stderr=stderr).returncode
        row = {'name': name, 'command': command, 'exit_code': code, 'expected_exit_code': expected_code,
            'diagnostic_process_wall_seconds': time.monotonic() - start,
            'stdout_sha256': sha(OUT / (name + '.stdout')), 'stderr_sha256': sha(OUT / (name + '.stderr'))}
        commands.append(row)
        dump(OUT / 'commands.json', commands)
        print(json.dumps({'finished': name, 'exit_code': code, 'expected_exit_code': expected_code}), flush=True)
        if code != expected_code:
            dump(OUT / 'failure.json', {'complete': False, 'failed_command': row,
                'runtime_admitted': False, 'plan_complete': False})
            raise SystemExit(code or 1)
        guard()
        return (OUT / (name + '.stdout')).read_text()

    run('apply', ['patch', '--batch', '--forward', '-p1', '-i', str(E / 'sphere-stable-selection-v1.patch')], prepared=False)
    actual = {str(path.relative_to(SOURCE)): sha(path) for path in SOURCE.rglob('*') if path.is_file()}
    assert actual == expected and len(actual) == 10136
    for name, value in [('source-hashes.json.gz', actual), ('root-source-hashes.json.gz', root_mapping)]:
        (OUT / name).write_bytes(gzip.compress((json.dumps(value, sort_keys=True) + '\n').encode(), mtime=0))
    dump(OUT / 'protocol.json', {'inputs_sha256': inputs, 'source': str(SOURCE),
        'parent_source': str(parent), 'target': str(TARGET), 'source_files': len(actual),
        'delta_sha256': scope['expected_delta_sha256'], 'environment': ENV,
        'parent_libraries': parent_libraries, 'pinned_cpu': cpu,
        'compiler': subprocess.check_output(['rustc', '-Vv'], text=True), 'scope': scope,
        'not_a_cost_trial': True, 'runtime_admitted': False, 'plan_complete': False})
    run('format', ['cargo', 'fmt', '--all', '--', '--check'])
    run('probe-format', ['rustfmt', '--edition', '2024', '--check', str(E / 'dyadic_hull_rows_probe_v1.rs')])

    def link(name, source, libs, test=False):
        binary = WORK / name
        command = ['rustc', '--edition=2024', '-O', '-D', 'warnings']
        if test:
            command.append('--test')
        command += [str(source), '--extern', 'csgrs=' + libs['csgrs']['path'],
            '-L', 'dependency=' + str(Path(libs['hypermesh']['path']).parent), '-o', str(binary)]
        run('link-' + name, command)
        binaries[name] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
        dump(OUT / 'binaries.json', binaries)
        return binary

    negative = link('negative-regression', SOURCE / 'tests/native_hull_determinism.rs', parent_libraries, test=True)
    negative_stdout = run('negative-regression', ['taskset', '-c', str(cpu), str(negative),
        '--test-threads=1', '--nocapture'], seconds=600, expected_code=101)
    negative_stderr = (OUT / 'negative-regression.stderr').read_text()
    assert 'vertex row order changed for 32x16' in negative_stderr
    assert 'test result: FAILED. 0 passed; 1 failed;' in negative_stdout
    run('mesh-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hypermesh', '--lib', '--', '-D', 'warnings'])
    run('candidate-regression', ['cargo', 'test', '--locked', '--release', '-p', 'csgrs', '--no-default-features',
        '--test', 'native_hull_determinism', '--', '--test-threads=1', '--nocapture'], seconds=600)
    assert 'test result: ok. 1 passed; 0 failed;' in (OUT / 'candidate-regression.stdout').read_text()
    run('regression-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'csgrs', '--no-default-features',
        '--test', 'native_hull_determinism', '--', '-D', 'warnings'])
    guard(full=True)
    run('build', ['cargo', 'build', '--locked', '--release', '-p', 'csgrs', '--no-default-features',
        '--lib', '--message-format=json'])
    events = [json.loads(line) for line in (OUT / 'build.stdout').read_text().splitlines() if line.startswith('{')]
    for name in ['csgrs', 'hypermesh']:
        rows = [row for row in events if row.get('reason') == 'compiler-artifact' and row['target']['name'] == name
                and any(path.endswith('.rlib') for path in row['filenames'])]
        assert len(rows) == 1
        path = next(path for path in rows[0]['filenames'] if path.endswith('.rlib'))
        libraries[name] = {'path': path, 'sha256': sha(path), 'features': rows[0]['features']}
    assert {name: row['features'] for name, row in libraries.items()} == {name: row['features'] for name, row in parent_libraries.items()}
    dump(OUT / 'libraries.json', libraries)
    probe = link('sphere-probe', E / 'sphere_warm_replay_probe_v1.rs', libraries)
    cube_parent = link('cube-parent', E / 'dyadic_hull_rows_probe_v1.rs', parent_libraries)
    cube_candidate = link('cube-candidate', E / 'dyadic_hull_rows_probe_v1.rs', libraries)
    parent_cube = run('cube-parent', [str(cube_parent)], seconds=600)
    assert run('cube-candidate', [str(cube_candidate)], seconds=600) == parent_cube
    observations = []
    for case in scope['cases']:
        raw = run(case, ['taskset', '-c', str(cpu), str(probe), case], seconds=600)
        parts = raw.split('witness_phase=')
        assert len(parts) == 4 and parts[0] == f'replay_case={case} repetitions=3 grammar_policy=default allocator=default\n'
        witnesses, faces = {}, {}
        for phase, part in zip(['cold', 'warm', 'replay'], parts[1:]):
            actual_phase, witness = part.split('\n', 1)
            assert actual_phase == phase
            witnesses[phase] = witness
            _, faces[phase] = parser.parse(witness, case)
            (OUT / (case + '-' + phase + '.witness')).write_text(witness)
        assert witnesses['cold'] == witnesses['warm'] == witnesses['replay']
        prior = (E / f'sphere-cyclotomic-v2/candidate-{case}.stdout').read_text()
        _, prior_faces = parser.parse(prior, case)
        observations.append({'case': case, 'all_three_certified_closed_and_all_original_vertices_used': True,
            'all_three_full_transcripts_identical': True,
            'full_transcript_matches_prior_cold': witnesses['cold'] == prior,
            'oriented_faces_removed_from_prior': len(prior_faces - faces['cold']),
            'oriented_faces_added_to_candidate': len(faces['cold'] - prior_faces),
            'new_faces_independently_support_checked': False,
            'witness_sha256': sha(OUT / (case + '-cold.witness'))})
        dump(OUT / 'observations.json', observations)
        print(json.dumps(observations[-1]), flush=True)
    guard(full=True)
    assert len(commands) == 18 and sum(row['exit_code'] == 0 for row in commands) == 17
    dump(OUT / 'result.json', {'complete': True, 'commands': 18,
        'commands_expected_zero': 17, 'expected_negative_control_failures': 1,
        'native_regression_passed': True, 'all_four_full_original_spheres_replay_identically': True,
        'dyadic_cube_parent_full_transcript_preserved': True,
        'all_sources_inputs_libraries_and_binaries_unchanged': True,
        'new_geometry_independent_support_proof_complete': False,
        'artifacts_sha256': {path.name: sha(path) for path in sorted(OUT.iterdir()) if path.is_file()},
        'not_a_cost_trial': True, 'runtime_admitted': False, 'plan_complete': False})
    print(json.dumps({'complete': True, 'result_sha256': sha(OUT / 'result.json')}), flush=True)


if __name__ == '__main__':
    main()
