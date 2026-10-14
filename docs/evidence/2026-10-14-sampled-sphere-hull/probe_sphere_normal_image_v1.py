"""Frozen normal-form finite-image feasibility trial; no performance admission."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'sphere-normal-image-v1'
WORK = ROOT / 'target/sphere-normal-image-v1'
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
    scope = json.loads((E / 'sphere-normal-image-scope-v1.json').read_text())
    assert sha(E / scope['scalar_cost_evidence']) == scope['scalar_cost_evidence_sha256']
    cost = json.loads((E / scope['scalar_cost_evidence']).read_text())
    assert cost['complete'] and cost['commands_all_zero'] == 52
    qualification = json.loads((E / scope['parent_qualification']).read_text())
    assert qualification['complete'] and qualification['commands_all_zero'] == 10
    inputs = {name: sha(E / name) for name in [
        'probe_sphere_normal_image_v1.py', 'sphere-normal-image-scope-v1.json',
        'sphere-normal-image-v1.patch', 'sphere_warm_replay_probe_v1.rs',
        scope['scalar_cost_evidence'], scope['parent_qualification'], scope['parent_source_mapping'],
        'sphere-cyclotomic-scalar-costs-closure-v2.json',
        'sphere-stable-selection-closure-v1.json', 'sphere-stable-support-v1/result.json',
        'mesh-identity-preparation-v1/source-hashes.json.gz']}
    prior = E / 'sphere-stable-selection-v1'
    parent_result = json.loads((prior / 'result.json').read_text())
    assert parent_result['complete']
    inputs['sphere-stable-selection-v1/result.json'] = sha(prior / 'result.json')
    for name, digest in parent_result['artifacts_sha256'].items():
        assert sha(prior / name) == digest
        inputs['sphere-stable-selection-v1/' + name] = digest
    parent = ROOT / scope['parent_source']
    mapping = json.loads(gzip.decompress((E / scope['parent_source_mapping']).read_bytes()))
    expected = mapping | scope['expected_delta_sha256']
    baseline = json.loads(gzip.decompress((E / 'mesh-identity-preparation-v1/source-hashes.json.gz').read_bytes()))
    root_mapping = {name: sha(ROOT / name) for name in baseline}
    assert all(root_mapping[name] == digest for name, digest in baseline.items() if name != 'docs/properties-status.json')
    parent_libraries = json.loads((prior / 'libraries.json').read_text())
    libraries, binaries = {}, {}

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

    def run(name, command, seconds=900, prepared=True):
        guard(prepared=prepared)
        print(json.dumps({'start': name}), flush=True)
        started = time.monotonic()
        with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', str(seconds), *command],
                cwd=SOURCE, env=os.environ | ENV, stdout=stdout, stderr=stderr).returncode
        row = {'name': name, 'command': command, 'exit_code': code,
            'diagnostic_process_wall_seconds': time.monotonic() - started,
            'stdout_sha256': sha(OUT / (name + '.stdout')), 'stderr_sha256': sha(OUT / (name + '.stderr'))}
        commands.append(row)
        dump(OUT / 'commands.json', commands)
        print(json.dumps({'finished': name, 'exit_code': code}), flush=True)
        if code:
            dump(OUT / 'failure.json', {'complete': False, 'failed_command': row,
                'preserve_executed_source': True, 'runtime_admitted': False, 'plan_complete': False})
            raise SystemExit(code)
        guard()
        return (OUT / (name + '.stdout')).read_text()

    run('apply', ['patch', '--batch', '--forward', '-p1', '-i', str(E / 'sphere-normal-image-v1.patch')], prepared=False)
    actual = {str(path.relative_to(SOURCE)): sha(path) for path in SOURCE.rglob('*') if path.is_file()}
    assert actual == expected and len(actual) == scope['source_files']
    for name, value in [('source-hashes.json.gz', actual), ('root-source-hashes.json.gz', root_mapping)]:
        (OUT / name).write_bytes(gzip.compress((json.dumps(value, sort_keys=True) + '\n').encode(), mtime=0))
    dump(OUT / 'protocol.json', {'inputs_sha256': inputs, 'source': str(SOURCE),
        'parent_source': str(parent), 'target': str(TARGET), 'source_files': len(actual),
        'delta_sha256': scope['expected_delta_sha256'], 'environment': ENV,
        'parent_libraries': parent_libraries, 'pinned_cpu': cpu,
        'compiler': subprocess.check_output(['rustc', '-Vv'], text=True), 'scope': scope,
        'not_a_cost_trial': True, 'runtime_admitted': False, 'plan_complete': False})
    run('format', ['cargo', 'fmt', '--all', '--', '--check'])
    raw = run('targeted-scalar-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal',
        '--lib', 'cyclotomic', '--', '--test-threads=1', '--nocapture'])
    assert f"test result: ok. {scope['targeted_scalar_test_count']} passed; 0 failed;" in raw
    run('scalar-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal',
        '--lib', '--tests', '--', '-D', 'warnings'])
    guard(full=True)
    run('build-native-consumer', ['cargo', 'build', '--locked', '--release', '-p', 'csgrs',
        '--no-default-features', '--lib', '--message-format=json'])
    events = [json.loads(line) for line in (OUT / 'build-native-consumer.stdout').read_text().splitlines() if line.startswith('{')]
    for name in ['csgrs', 'hypermesh']:
        rows = [row for row in events if row.get('reason') == 'compiler-artifact' and row['target']['name'] == name
                and any(path.endswith('.rlib') for path in row['filenames'])]
        assert len(rows) == 1
        path = next(path for path in rows[0]['filenames'] if path.endswith('.rlib'))
        libraries[name] = {'path': path, 'sha256': sha(path), 'features': rows[0]['features']}
    assert {name: row['features'] for name, row in libraries.items()} == {name: row['features'] for name, row in parent_libraries.items()}
    dump(OUT / 'libraries.json', libraries)
    binary = WORK / 'sphere-probe'
    run('link-sphere-probe', ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(E / 'sphere_warm_replay_probe_v1.rs'),
        '--extern', 'csgrs=' + libraries['csgrs']['path'],
        '-L', 'dependency=' + str(Path(libraries['hypermesh']['path']).parent), '-o', str(binary)])
    binaries['sphere-probe'] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
    dump(OUT / 'binaries.json', binaries)
    observations = []
    for case in scope['cases']:
        raw = run(case, ['taskset', '-c', str(cpu), str(binary), case], seconds=600)
        reference = prior / (case + '.stdout')
        assert raw == reference.read_text()
        assert (OUT / (case + '.stdout')).read_bytes() == reference.read_bytes()
        assert raw.count('certified=true original_positions_preserved=true') == 3
        observations.append({'case': case, 'all_three_full_transcripts_match_parent': True,
            'all_three_certified': True, 'independent_support_reused_by_exact_witness_identity': True,
            'full_stdout_sha256': sha(reference), 'full_stdout_bytes': reference.stat().st_size})
        dump(OUT / 'observations.json', observations)
    guard(full=True)
    assert len(commands) == scope['commands'] == 10
    dump(OUT / 'result.json', {'complete': True, 'commands_all_zero': 10,
        'targeted_scalar_tests_passed': 14, 'all_twelve_full_original_native_outputs_preserved': True,
        'all_sources_inputs_libraries_and_binaries_unchanged': True,
        'artifacts_sha256': {path.name: sha(path) for path in sorted(OUT.iterdir()) if path.is_file()},
        'not_a_cost_trial': True, 'runtime_admitted': False, 'plan_complete': False})
    print(json.dumps({'complete': True, 'result_sha256': sha(OUT / 'result.json')}), flush=True)


if __name__ == '__main__':
    main()
