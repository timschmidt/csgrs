"""Prepare the reviewed mesh fact repair and run focused correctness qualification."""
import difflib
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
BASE = E / 'baseline-v1'
WORK = ROOT / 'target/mesh-fact-identity-v1'
SOURCE = WORK / 'source'
TARGET = WORK / 'target'
PREP = E / 'mesh-identity-preparation-v1'
OUT = E / 'mesh-identity-qualification-v1'
INPUTS = ['qualify_mesh_identity_v1.py', 'mesh-fact-identity-scope-v1.json',
          'mesh-fact-identity-runtime-v1.patch', 'mesh_fact_identity_tests_v1.rs',
          'mesh_fact_identity_probe_v1.rs', 'mesh_view_identity_probe_v1.rs',
          'reproduce_v1.rs', 'mesh-fact-identity-audit-v1.json']
ENV = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
       'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never', 'CARGO_TARGET_DIR': str(TARGET)}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, data):
    path.write_text(json.dumps(data, indent=2) + '\n')


def main():
    assert not WORK.exists() and not PREP.exists() and not OUT.exists()
    scope = json.loads((E / 'mesh-fact-identity-scope-v1.json').read_text())
    baseline_result = json.loads((BASE / 'result.json').read_text())
    assert baseline_result['complete'] and baseline_result['commands_passed'] == 7
    assert all(sha(BASE / p) == h for p, h in baseline_result['artifacts_sha256'].items())
    observed_path = E / 'mesh-fact-identity-v1/result.json'
    observed = json.loads(observed_path.read_text())
    assert observed['complete'] and observed['commands_passed'] == 3
    assert len(observed['controls']) == 9 and observed['controls_satisfying_contract'] == 0
    assert all(sha(observed_path.parent / p) == h for p, h in observed['artifacts_sha256'].items())
    assert sha(observed_path) == scope['counterexample_result_sha256']
    view_path = E / 'mesh-view-identity-v1/result.json'
    views = json.loads(view_path.read_text())
    assert views['complete'] and views['commands_passed'] == 3
    assert len(views['controls']) == 4 and views['controls_satisfying_contract'] == 0
    assert all(sha(view_path.parent / p) == h for p, h in views['artifacts_sha256'].items())
    assert sha(view_path) == scope['borrowed_view_result_sha256']
    base_protocol = json.loads((BASE / 'protocol.json').read_text())
    baseline = Path(base_protocol['source'])
    original = json.loads(gzip.decompress((BASE / 'source-hashes.json.gz').read_bytes()))
    assert all(sha(baseline / p) == h for p, h in original.items())
    main_sources = {p: sha(ROOT / p) for p in original}
    assert all(main_sources[p] == h for p, h in original.items() if p != 'docs/properties-status.json')
    inputs = {p: sha(E / p) for p in INPUTS}
    WORK.mkdir()
    PREP.mkdir()
    OUT.mkdir()
    subprocess.run(['cp', '-a', '--reflink=auto', str(baseline), str(SOURCE)], check=True)
    patch_command = ['patch', '--batch', '--forward', '-p1', '-i', str(E / 'mesh-fact-identity-runtime-v1.patch')]
    with (PREP / 'apply.stdout').open('w') as stdout, (PREP / 'apply.stderr').open('w') as stderr:
        code = subprocess.run(patch_command, cwd=SOURCE, stdout=stdout, stderr=stderr).returncode
    dump(PREP / 'apply.json', {'command': patch_command, 'exit_code': code,
         'stdout_sha256': sha(PREP / 'apply.stdout'), 'stderr_sha256': sha(PREP / 'apply.stderr')})
    assert code == 0
    tests = SOURCE / 'crates/hypermesh/src/mesh/fact_identity_tests.rs'
    tests.parent.mkdir(exist_ok=True)
    shutil.copyfile(E / 'mesh_fact_identity_tests_v1.rs', tests)
    mapping = {str(p.relative_to(SOURCE)): sha(p) for p in SOURCE.rglob('*') if p.is_file()}
    delta = {p: h for p, h in mapping.items() if original.get(p) != h}
    assert delta == scope['expected_source_delta_sha256']
    assert set(mapping) == set(original) | {'crates/hypermesh/src/mesh/fact_identity_tests.rs'}
    (PREP / 'source-hashes.json.gz').write_bytes(gzip.compress(
        (json.dumps(mapping, sort_keys=True) + '\n').encode(), mtime=0))
    with (PREP / 'source.patch').open('w') as patch:
        for p in sorted(delta):
            before = (baseline / p).read_text() if p in original else ''
            patch.writelines(difflib.unified_diff(before.splitlines(True), (SOURCE / p).read_text().splitlines(True),
                                                fromfile='a/' + p, tofile='b/' + p))
    dump(PREP / 'protocol.json', {'head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
         'source': str(SOURCE), 'target': str(TARGET), 'source_files': len(mapping),
         'baseline_result_sha256': sha(BASE / 'result.json'), 'counterexample_result_sha256': sha(observed_path),
         'source_hashes_sha256': sha(PREP / 'source-hashes.json.gz'), 'delta_sha256': delta,
         'inputs_sha256': inputs, 'main_source_sha256': main_sources, 'environment': ENV,
         'runtime_admitted': False, 'plan_complete': False})
    commands = []

    def guard(full=False):
        selected = mapping if full else {p: mapping[p] for p in delta}
        assert all(sha(SOURCE / p) == h for p, h in selected.items())
        assert all(sha(E / p) == h for p, h in inputs.items())
        if full:
            assert all(sha(ROOT / p) == h for p, h in main_sources.items())
            assert all(sha(baseline / p) == h for p, h in original.items())
            assert {str(p.relative_to(SOURCE)) for p in SOURCE.rglob('*') if p.is_file()} == set(mapping)

    def run(name, command, extra=None):
        guard()
        started = time.monotonic()
        with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(command, cwd=SOURCE, env=os.environ | ENV | (extra or {}),
                                  stdout=stdout, stderr=stderr).returncode
        elapsed = time.monotonic() - started
        guard()
        row = {'name': name, 'command': command, 'exit_code': code, 'process_wall_seconds': elapsed,
               'stdout_sha256': sha(OUT / (name + '.stdout')), 'stderr_sha256': sha(OUT / (name + '.stderr'))}
        commands.append(row)
        dump(OUT / 'commands.json', commands)
        print(json.dumps(row), flush=True)
        if code:
            raise SystemExit(code)

    run('source-format', ['cargo', 'fmt', '--all', '--', '--check'])
    for selection, features in [('default', []), ('all-features', ['--all-features'])]:
        run(selection + '-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hypermesh',
                                   '--all-targets', *features, '--', '-D', 'warnings'])
        run(selection + '-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hypermesh',
                                  '--lib', '--tests', *features, '--', '--test-threads=2', '--show-output'])
    run('doctests', ['cargo', 'test', '--locked', '--release', '-p', 'hypermesh', '--all-features', '--doc'])
    run('docs', ['cargo', 'doc', '--locked', '--release', '-p', 'hypermesh', '--all-features', '--no-deps'],
        {'RUSTDOCFLAGS': '-D warnings'})
    run('grammar-consumers', ['cargo', 'test', '--locked', '--release', '-p', 'csgrs', '--no-default-features',
        '--test', 'native_queries', '--test', 'geometry_context', '--test', 'solid_cache',
        '--', '--test-threads=2', '--show-output'])
    run('build-consumer-library', ['cargo', 'build', '--locked', '--release', '-p', 'csgrs',
                                  '--no-default-features', '--lib', '--message-format=json'])
    cargo = [json.loads(line) for line in (OUT / 'build-consumer-library.stdout').read_text().splitlines()
             if line.startswith('{')]
    libraries = {}
    for name in ['csgrs', 'hypermesh']:
        rows = [r for r in cargo if r.get('reason') == 'compiler-artifact' and r['target']['name'] == name
                and any(p.endswith('.rlib') for p in r['filenames'])]
        assert len(rows) == 1
        path = next(p for p in rows[0]['filenames'] if p.endswith('.rlib'))
        libraries[name] = {'path': path, 'sha256': sha(path), 'features': rows[0]['features']}
    dump(OUT / 'libraries.json', libraries)
    binaries = {}
    for name, file in [('mesh-identity', 'mesh_fact_identity_probe_v1.rs'),
                       ('view-identity', 'mesh_view_identity_probe_v1.rs'), ('sphere-hull', 'reproduce_v1.rs')]:
        binary = WORK / name
        run('link-' + name, ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(E / file),
            '--extern', 'csgrs=' + libraries['csgrs']['path'], '--extern', 'hypermesh=' + libraries['hypermesh']['path'],
            '-L', 'dependency=' + str(TARGET / 'release/deps'), '-o', str(binary)])
        binaries[name] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
    dump(OUT / 'binaries.json', binaries)
    run('mesh-identity', [binaries['mesh-identity']['path']])
    observations = [dict(word.split('=', 1) for word in line.split())
                    for line in (OUT / 'mesh-identity.stdout').read_text().splitlines()]
    assert [r['control'] for r in observations] == [r['control'] for r in observed['controls']]
    assert len(observations) == 9 and all(r['correct'] == 'true' for r in observations)
    run('view-identity', [binaries['view-identity']['path']])
    view_observations = [dict(word.split('=', 1) for word in line.split())
                         for line in (OUT / 'view-identity.stdout').read_text().splitlines()]
    assert [r['control'] for r in view_observations] == [r['control'] for r in views['controls']]
    assert len(view_observations) == 4 and all(r['correct'] == 'true' for r in view_observations)
    for case in base_protocol['cases']:
        run(case, ['timeout', '--signal=TERM', '--kill-after=10s', '180s',
                   '/usr/bin/time', '-v', binaries['sphere-hull']['path'], case])
        assert (OUT / (case + '.stdout')).read_bytes() == (BASE / (case + '.stdout')).read_bytes()
    guard(full=True)
    assert all(sha(row['path']) == row['sha256'] for row in libraries.values())
    assert all(sha(row['path']) == row['sha256'] for row in binaries.values())
    assert len(commands) == 18
    dump(OUT / 'result.json', {'complete': True, 'commands_passed': len(commands),
         'controls': observations, 'all_nine_public_mutation_controls_correct': True,
         'borrowed_view_controls': view_observations, 'all_four_borrowed_view_controls_correct': True,
         'all_original_sphere_outcomes_preserved': True, 'source_files': len(mapping),
         'preparation_sha256': sha(PREP / 'protocol.json'), 'all_sources_and_inputs_unchanged': True,
         'artifacts_sha256': {p.name: sha(p) for p in sorted(OUT.iterdir()) if p.is_file()},
         'costs_pending': True, 'sphere_certification_pending': True,
         'runtime_admitted': False, 'plan_complete': False})
    print(json.dumps({'complete': True, 'commands_passed': 18, 'thirteen_mutation_controls_fixed': True,
                      'runtime_admitted': False}), flush=True)


if __name__ == '__main__':
    main()
