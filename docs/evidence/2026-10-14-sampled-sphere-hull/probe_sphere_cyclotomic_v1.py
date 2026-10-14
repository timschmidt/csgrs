"""Frozen native feasibility trial; no runtime admission or performance claim."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
WORK = ROOT / 'target/sphere-cyclotomic-v1'
SOURCE = WORK / 'source'
TARGET = WORK / 'target'
OUT = E / 'sphere-cyclotomic-v1'
INPUTS = ['probe_sphere_cyclotomic_v1.py', 'sphere-cyclotomic-scope-v1.json',
          'sphere-cyclotomic-candidate-v1.patch', 'reproduce_rows_v1.rs',
          'sphere-cyclotomic-oracle-audit-v1.json']
ENV = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
       'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never', 'CARGO_TARGET_DIR': str(TARGET)}
CASES = ['fifteen_degrees', 'original_medium', 'fifth_turn', 'seventh_turn']


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, data):
    path.write_text(json.dumps(data, indent=2) + '\n')


def main():
    assert not WORK.exists() and not OUT.exists()
    original = json.loads(gzip.decompress((E / 'mesh-identity-preparation-v1/source-hashes.json.gz').read_bytes()))
    baseline = ROOT / 'target/mesh-fact-identity-v1/source'
    assert all(sha(baseline / p) == h for p, h in original.items())
    main_sources = {p: sha(ROOT / p) for p in original}
    assert all(main_sources[p] == h for p, h in original.items() if p != 'docs/properties-status.json')
    scope = json.loads((E / 'sphere-cyclotomic-scope-v1.json').read_text())
    inputs = {p: sha(E / p) for p in INPUTS}
    prior = E / 'sphere-zero-witness-assessment-v2.json'
    assert sha(prior) == scope['closed_diagnostic_assessment_sha256']
    WORK.mkdir()
    OUT.mkdir()
    subprocess.run(['cp', '-a', '--reflink=auto', str(baseline), str(SOURCE)], check=True)
    commands = []

    def run(name, command, timeout=900):
        assert all(sha(E / p) == h for p, h in inputs.items())
        started = time.monotonic()
        with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', str(timeout), *command],
                                  cwd=SOURCE, env=os.environ | ENV, stdout=stdout, stderr=stderr).returncode
        row = {'name': name, 'command': command, 'exit_code': code, 'timeout_seconds': timeout,
               'process_wall_seconds': time.monotonic() - started,
               'stdout_sha256': sha(OUT / (name + '.stdout')), 'stderr_sha256': sha(OUT / (name + '.stderr'))}
        commands.append(row)
        dump(OUT / 'commands.json', commands)
        print(json.dumps(row), flush=True)
        assert all(sha(E / p) == h for p, h in inputs.items())
        if code:
            dump(OUT / 'failure.json', {'complete': False, 'failed_command': row, 'runtime_admitted': False,
                                      'preserve_executed_source': True, 'plan_complete': False})
            raise SystemExit(code)

    run('apply', ['patch', '--batch', '--forward', '-p1', '-i', str(E / 'sphere-cyclotomic-candidate-v1.patch')])
    mapping = {str(p.relative_to(SOURCE)): sha(p) for p in SOURCE.rglob('*') if p.is_file()}
    delta = {p: h for p, h in mapping.items() if original.get(p) != h}
    assert delta == scope['expected_source_delta_sha256']
    assert set(mapping) == set(original) | set(delta)
    (OUT / 'source-hashes.json.gz').write_bytes(gzip.compress((json.dumps(mapping, sort_keys=True) + '\n').encode(), mtime=0))
    dump(OUT / 'protocol.json', {'source': str(SOURCE), 'target': str(TARGET), 'environment': ENV,
        'inputs_sha256': inputs, 'source_files': len(mapping), 'source_delta_sha256': delta,
        'root_sources_at_start_sha256': main_sources,
        'cases': CASES, 'native_query_wall_limits_seconds': {'original_medium': 600, 'other_cases': 180},
        'scope': 'Six targeted scalar tests, strict library/test lint, and unchanged ordinary sphere APIs. Later broad tests, formal obligations, native cost and allocation/size qualification remain mandatory before admission.',
        'runtime_admitted': False, 'plan_complete': False})

    def guard():
        assert all(sha(SOURCE / p) == h for p, h in mapping.items())
        assert all(sha(ROOT / p) == h for p, h in main_sources.items())
        assert all(sha(baseline / p) == h for p, h in original.items())

    run('source-format', ['cargo', 'fmt', '--all', '--', '--check'])
    guard()
    run('targeted-scalar-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--lib',
        'cyclotomic', '--', '--test-threads=1', '--nocapture'])
    guard()
    run('scalar-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal', '--lib', '--tests', '--', '-D', 'warnings'])
    guard()
    run('build-native-consumer', ['cargo', 'build', '--locked', '--release', '-p', 'csgrs', '--no-default-features',
        '--lib', '--message-format=json'])
    cargo = [json.loads(line) for line in (OUT / 'build-native-consumer.stdout').read_text().splitlines() if line.startswith('{')]
    libraries = {}
    for name in ['csgrs', 'hypermesh']:
        rows = [r for r in cargo if r.get('reason') == 'compiler-artifact' and r['target']['name'] == name
                and any(p.endswith('.rlib') for p in r['filenames'])]
        assert len(rows) == 1
        path = next(p for p in rows[0]['filenames'] if p.endswith('.rlib'))
        libraries[name] = {'path': path, 'sha256': sha(path), 'features': rows[0]['features']}
    dump(OUT / 'libraries.json', libraries)
    qualified_libraries = json.loads((E / 'mesh-identity-qualification-v1/libraries.json').read_text())
    binaries = {}
    for name, libs in [('baseline', qualified_libraries), ('candidate', libraries)]:
        binary = WORK / ('sphere-' + name)
        assert all(sha(r['path']) == r['sha256'] for r in libs.values())
        run('link-' + name, ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(E / 'reproduce_rows_v1.rs'),
            '--extern', 'csgrs=' + libs['csgrs']['path'], '--extern', 'hypermesh=' + libs['hypermesh']['path'],
            '-L', 'dependency=' + str(Path(libs['hypermesh']['path']).parent), '-o', str(binary)])
        binaries[name] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
    dump(OUT / 'binaries.json', binaries)
    run('baseline-fifteen_degrees', [binaries['baseline']['path'], 'fifteen_degrees'], 180)
    outcomes = {}
    for case in CASES:
        guard()
        run('candidate-' + case, [binaries['candidate']['path'], case], 600 if case == 'original_medium' else 180)
        stdout = (OUT / ('candidate-' + case + '.stdout')).read_text()
        outcomes[case] = {'certified': 'certified=true original_positions_preserved=true' in stdout}
        if case == 'fifteen_degrees':
            assert stdout == (OUT / 'baseline-fifteen_degrees.stdout').read_text()
            outcomes[case]['prior_position_order_triangle_rows_and_certainty_unchanged'] = True
    guard()
    assert all(sha(r['path']) == r['sha256'] for r in binaries.values())
    dump(OUT / 'result.json', {'complete': True, 'commands': len(commands), 'outcomes': outcomes,
        'all_sources_and_inputs_unchanged': True,
        'artifacts_sha256': {p.name: sha(p) for p in sorted(OUT.iterdir()) if p.is_file()},
        'runtime_admitted': False, 'plan_complete': False})
    print(json.dumps({'complete': True, 'commands': len(commands), 'outcomes': outcomes}), flush=True)


if __name__ == '__main__':
    main()
