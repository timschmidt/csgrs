"""Fixed construction-identity coverage diagnostic on all four original spheres."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'sphere-grid-facts-v1'
WORK = ROOT / 'target/sphere-grid-facts-v1'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def main():
    assert not OUT.exists() and not WORK.exists()
    qdir = E / 'mesh-identity-qualification-v1'
    qualified = json.loads((qdir / 'result.json').read_text())
    assert qualified['complete'] and qualified['commands_passed'] == 18
    assert all(sha(qdir / p) == h for p, h in qualified['artifacts_sha256'].items())
    prep = json.loads((E / 'mesh-identity-preparation-v1/protocol.json').read_text())
    mapping = json.loads(gzip.decompress((E / 'mesh-identity-preparation-v1/source-hashes.json.gz').read_bytes()))
    source = Path(prep['source'])
    root_mapping = {p: sha(ROOT / p) for p in mapping}
    assert all(root_mapping[p] == h for p, h in mapping.items() if p != 'docs/properties-status.json')
    libraries = json.loads((qdir / 'libraries.json').read_text())
    inputs = {p: sha(E / p) for p in ['probe_sphere_grid_facts_v1.py', 'sphere_grid_facts_probe_v1.rs', 'sphere-grid-facts-scope-v1.json']}

    def guard(full=False):
        assert all(sha(E / p) == h for p, h in inputs.items())
        assert all(sha(r['path']) == r['sha256'] for r in libraries.values())
        if full:
            assert all(sha(source / p) == h for p, h in mapping.items())
            assert all(sha(ROOT / p) == h for p, h in root_mapping.items())

    guard(full=True)
    OUT.mkdir()
    WORK.mkdir()
    cases = ['fifteen_degrees', 'original_medium', 'fifth_turn', 'seventh_turn']
    modes = ['adjacent', 'all_rectangles', 'coordinate_symmetries']
    binary = WORK / 'probe'
    dump(OUT / 'protocol.json', {'head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        'inputs_sha256': inputs, 'source': str(source), 'source_files': len(mapping),
        'libraries': libraries, 'cases': cases, 'modes': modes, 'runs': 12,
        'timeout_seconds': 180, 'timing_scope': 'Diagnostic stop only; not matched performance evidence.',
        'qualified_runtime_unchanged': True, 'sphere_runtime_candidate_prepared': False, 'plan_complete': False})
    commands = []

    def run(name, command):
        guard()
        print(json.dumps({'start': name}), flush=True)
        start = time.monotonic()
        with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(command, cwd=source, stdout=stdout, stderr=stderr).returncode
        row = {'name': name, 'command': command, 'exit_code': code,
            'process_wall_seconds': time.monotonic() - start,
            'stdout_sha256': sha(OUT / (name + '.stdout')), 'stderr_sha256': sha(OUT / (name + '.stderr'))}
        commands.append(row)
        dump(OUT / 'commands.json', commands)
        print(json.dumps(row), flush=True)
        if code:
            raise SystemExit(code)
        guard()

    run('format', ['rustfmt', '--edition', '2024', '--check', str(E / 'sphere_grid_facts_probe_v1.rs')])
    run('link', ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(E / 'sphere_grid_facts_probe_v1.rs'),
        '--extern', 'csgrs=' + libraries['csgrs']['path'], '--extern', 'hypermesh=' + libraries['hypermesh']['path'],
        '-L', 'dependency=' + str(Path(prep['target']) / 'release/deps'), '-o', str(binary)])
    binary_hash = sha(binary)
    observations = []
    for case in cases:
        for mode in modes:
            assert sha(binary) == binary_hash
            name = case + '-' + mode
            run(name, ['timeout', '--signal=TERM', '--kill-after=10s', '180s', str(binary), case, mode])
            lines = (OUT / (name + '.stdout')).read_text().splitlines()
            assert len(lines) == 2 and lines[0].startswith('input ') and lines[1].startswith('result ')
            observations.append({'case': case, 'mode': mode, 'input': lines[0], 'result': lines[1],
                'certified': 'certified=true' in lines[1]})
    guard(full=True)
    assert len(commands) == 14
    dump(OUT / 'result.json', {'complete': True, 'commands_passed': 14, 'observations': observations,
        'binary': {'path': str(binary), 'sha256': binary_hash, 'bytes': binary.stat().st_size},
        'all_sources_and_inputs_unchanged': True,
        'artifacts_sha256': {p.name: sha(p) for p in sorted(OUT.iterdir()) if p.is_file()},
        'sphere_runtime_candidate_prepared': False, 'sphere_runtime_admitted': False, 'plan_complete': False})
    print(json.dumps({'complete': True, 'commands_passed': 14, 'observations': observations}), flush=True)


if __name__ == '__main__':
    main()
