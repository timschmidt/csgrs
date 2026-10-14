"""Isolated stderr-only scheduling trace for the original warm sphere failure."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'sphere-warm-trace-v1'
WORK = ROOT / 'target/sphere-warm-trace-v1'
SOURCE = WORK / 'source'
TARGET = WORK / 'target'
ENV = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
       'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never', 'CARGO_TARGET_DIR': str(TARGET)}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def main():
    assert not WORK.exists() and not OUT.exists()
    scope = json.loads((E / 'sphere-warm-trace-scope-v1.json').read_text())
    assert sha(E / 'sphere-warm-replay-v1/result.json') == scope['parent_replay_result_sha256']
    assert sha(E / 'sphere-warm-replay-assessment-v1.json') == scope['parent_replay_assessment_sha256']
    replay = json.loads((E / 'sphere-warm-replay-v1/result.json').read_text())
    assert replay['complete'] and replay['all_phases_match_prior_exact_support_proofs']
    inputs = {name: sha(E / name) for name in [
        'trace_sphere_warm_v1.py', 'sphere-warm-trace-scope-v1.json',
        'sphere-warm-trace-v1.patch', 'sphere_warm_replay_probe_v1.rs',
        'sphere-warm-replay-v1/result.json', 'sphere-warm-replay-assessment-v1.json',
        'sphere-cyclotomic-image-v1/source-hashes.json.gz',
        'mesh-identity-preparation-v1/source-hashes.json.gz']}
    for name, digest in replay['artifacts_sha256'].items():
        assert sha(E / 'sphere-warm-replay-v1' / name) == digest
        inputs['sphere-warm-replay-v1/' + name] = digest
    parent = ROOT / scope['parent_source']
    mapping = json.loads(gzip.decompress((E / 'sphere-cyclotomic-image-v1/source-hashes.json.gz').read_bytes()))
    baseline_mapping = json.loads(gzip.decompress((E / 'mesh-identity-preparation-v1/source-hashes.json.gz').read_bytes()))
    root_mapping = {name: sha(ROOT / name) for name in baseline_mapping}
    assert all(root_mapping[name] == digest for name, digest in baseline_mapping.items()
               if name != 'docs/properties-status.json')
    expected = mapping | scope['expected_delta_sha256']
    libraries, binary_record = {}, {}

    def guard(full=False, prepared=True):
        assert all(sha(E / name) == digest for name, digest in inputs.items())
        if prepared:
            selected = expected if full else scope['expected_delta_sha256']
            assert all(sha(SOURCE / name) == digest for name, digest in selected.items())
        assert all(sha(row['path']) == row['sha256'] for row in libraries.values())
        if binary_record:
            assert sha(binary_record['path']) == binary_record['sha256']
        if full:
            assert all(sha(ROOT / name) == digest for name, digest in root_mapping.items())
            assert all(sha(parent / name) == digest for name, digest in mapping.items())

    guard(full=True, prepared=False)
    OUT.mkdir()
    WORK.mkdir()
    subprocess.run(['cp', '-a', '--reflink=auto', str(parent), str(SOURCE)], check=True)
    commands = []

    def run(name, command, seconds=900, prepared=True):
        guard(prepared=prepared)
        print(json.dumps({'start': name}), flush=True)
        start = time.monotonic()
        with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', str(seconds), *command],
                cwd=SOURCE, env=os.environ | ENV, stdout=stdout, stderr=stderr).returncode
        row = {'name': name, 'command': command, 'exit_code': code,
            'diagnostic_process_wall_seconds': time.monotonic() - start,
            'stdout_sha256': sha(OUT / (name + '.stdout')), 'stderr_sha256': sha(OUT / (name + '.stderr'))}
        commands.append(row)
        dump(OUT / 'commands.json', commands)
        print(json.dumps({'finished': name, 'exit_code': code}), flush=True)
        if code:
            dump(OUT / 'failure.json', {'complete': False, 'failed_command': row,
                'runtime_admitted': False, 'plan_complete': False})
            raise SystemExit(code)
        guard()

    run('apply', ['patch', '--batch', '--forward', '-p1', '-i', str(E / 'sphere-warm-trace-v1.patch')], prepared=False)
    actual = {str(path.relative_to(SOURCE)): sha(path) for path in SOURCE.rglob('*') if path.is_file()}
    assert actual == expected and len(actual) == 10135
    for name, value in [('source-hashes.json.gz', actual), ('root-source-hashes.json.gz', root_mapping)]:
        (OUT / name).write_bytes(gzip.compress((json.dumps(value, sort_keys=True) + '\n').encode(), mtime=0))
    cpu = min(os.sched_getaffinity(0))
    dump(OUT / 'protocol.json', {'inputs_sha256': inputs, 'source': str(SOURCE),
        'parent_source': str(parent), 'target': str(TARGET), 'source_files': len(actual),
        'delta_sha256': scope['expected_delta_sha256'], 'environment': ENV,
        'pinned_cpu': cpu, 'compiler': subprocess.check_output(['rustc', '-Vv'], text=True),
        'scope': scope, 'not_a_cost_trial': True, 'runtime_admitted': False, 'plan_complete': False})
    run('format', ['cargo', 'fmt', '--all', '--', '--check'])
    run('clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hypermesh', '--lib', '--', '-D', 'warnings'])
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
    parent_libraries = json.loads((E / 'sphere-cyclotomic-image-v1/libraries.json').read_text())
    assert {name: row['features'] for name, row in libraries.items()} == {name: row['features'] for name, row in parent_libraries.items()}
    dump(OUT / 'libraries.json', libraries)
    binary = WORK / 'probe'
    run('link', ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(E / 'sphere_warm_replay_probe_v1.rs'),
        '--extern', 'csgrs=' + libraries['csgrs']['path'],
        '-L', 'dependency=' + str(TARGET / 'release/deps'), '-o', str(binary)])
    binary_record = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
    dump(OUT / 'binary.json', binary_record)
    run('original_medium', ['taskset', '-c', str(cpu), str(binary), 'original_medium'], seconds=600)
    assert (OUT / 'original_medium.stdout').read_bytes() == (E / 'sphere-warm-replay-v1/image-original_medium.stdout').read_bytes()
    trace = (OUT / 'original_medium.stderr').read_text().splitlines()
    assert all(line.startswith('hull-warm-trace ') for line in trace)
    assert sum(line.startswith('hull-warm-trace begin ') for line in trace) == 3
    guard(full=True)
    assert len(commands) == 6
    dump(OUT / 'result.json', {'complete': True, 'commands_all_zero': 6,
        'native_processes': 1, 'hull_queries': 3, 'trace_lines': len(trace),
        'all_three_stdout_transcripts_identical_to_uninstrumented_replay': True,
        'all_sources_inputs_libraries_and_binary_unchanged': True,
        'artifacts_sha256': {path.name: sha(path) for path in sorted(OUT.iterdir()) if path.is_file()},
        'not_a_cost_trial': True, 'runtime_admitted': False, 'plan_complete': False})
    print(json.dumps({'complete': True, 'result_sha256': sha(OUT / 'result.json')}), flush=True)


if __name__ == '__main__':
    main()
