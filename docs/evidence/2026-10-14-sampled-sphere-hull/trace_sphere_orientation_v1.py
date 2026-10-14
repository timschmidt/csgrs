"""Isolated stderr-only tracing of the unresolved original sphere orientations."""
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
OUT = E / 'sphere-orientation-trace-v1'
WORK = ROOT / 'target/sphere-orientation-trace-v1'
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
    scope = json.loads((E / 'sphere-orientation-trace-scope-v1.json').read_text())
    grid = E / 'sphere-grid-facts-v1'
    previous = json.loads((grid / 'result.json').read_text())
    assert previous['complete'] and previous['commands_passed'] == 14
    assert all(sha(grid / p) == h for p, h in previous['artifacts_sha256'].items())
    prep = json.loads((E / 'mesh-identity-preparation-v1/protocol.json').read_text())
    baseline = Path(prep['source'])
    mapping = json.loads(gzip.decompress((E / 'mesh-identity-preparation-v1/source-hashes.json.gz').read_bytes()))
    root_mapping = {p: sha(ROOT / p) for p in mapping}
    assert all(root_mapping[p] == h for p, h in mapping.items() if p != 'docs/properties-status.json')
    assert all(sha(baseline / p) == h for p, h in mapping.items())
    inputs = {p: sha(E / p) for p in ['trace_sphere_orientation_v1.py', 'sphere-orientation-trace-v1.patch',
        'sphere-orientation-trace-scope-v1.json', 'sphere_grid_facts_probe_v1.rs']}
    OUT.mkdir()
    WORK.mkdir()
    subprocess.run(['cp', '-a', '--reflink=auto', str(baseline), str(SOURCE)], check=True)
    expected = mapping | scope['expected_delta_sha256']
    commands = []

    def guard(full=False, prepared=True):
        assert all(sha(E / p) == h for p, h in inputs.items())
        if prepared:
            chosen = expected if full else scope['expected_delta_sha256']
            assert all(sha(SOURCE / p) == h for p, h in chosen.items())
        if full:
            assert all(sha(ROOT / p) == h for p, h in root_mapping.items())
            assert all(sha(baseline / p) == h for p, h in mapping.items())

    def run(name, command, prepared=True):
        guard(prepared=prepared)
        print(json.dumps({'start': name}), flush=True)
        start = time.monotonic()
        with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(command, cwd=SOURCE, env=os.environ | ENV, stdout=stdout, stderr=stderr).returncode
        row = {'name': name, 'command': command, 'exit_code': code,
            'process_wall_seconds': time.monotonic() - start,
            'stdout_sha256': sha(OUT / (name + '.stdout')), 'stderr_sha256': sha(OUT / (name + '.stderr'))}
        commands.append(row)
        dump(OUT / 'commands.json', commands)
        print(json.dumps(row), flush=True)
        if code:
            raise SystemExit(code)
        guard()

    run('apply', ['patch', '--batch', '--forward', '-p1', '-i', str(E / 'sphere-orientation-trace-v1.patch')], prepared=False)
    actual = {str(p.relative_to(SOURCE)): sha(p) for p in SOURCE.rglob('*') if p.is_file()}
    assert actual == expected
    (OUT / 'source-hashes.json.gz').write_bytes(gzip.compress((json.dumps(actual, sort_keys=True) + '\n').encode(), mtime=0))
    dump(OUT / 'protocol.json', {'inputs_sha256': inputs, 'source': str(SOURCE), 'target': str(TARGET),
        'source_files': len(actual), 'delta_sha256': scope['expected_delta_sha256'],
        'source_hashes_sha256': sha(OUT / 'source-hashes.json.gz'), 'environment': ENV,
        'baseline_grid_result_sha256': sha(grid / 'result.json'), 'scope': scope,
        'runtime_admitted': False, 'plan_complete': False})
    run('format', ['cargo', 'fmt', '--all', '--', '--check'])
    run('build', ['cargo', 'build', '--locked', '--release', '-p', 'csgrs', '--no-default-features', '--lib', '--message-format=json'])
    events = [json.loads(line) for line in (OUT / 'build.stdout').read_text().splitlines() if line.startswith('{')]
    libraries = {}
    for name in ['csgrs', 'hypermesh']:
        rows = [r for r in events if r.get('reason') == 'compiler-artifact' and r['target']['name'] == name
                and any(p.endswith('.rlib') for p in r['filenames'])]
        assert len(rows) == 1
        path = next(p for p in rows[0]['filenames'] if p.endswith('.rlib'))
        libraries[name] = {'path': path, 'sha256': sha(path), 'features': rows[0]['features']}
    dump(OUT / 'libraries.json', libraries)
    binary = WORK / 'probe'
    run('link', ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(E / 'sphere_grid_facts_probe_v1.rs'),
        '--extern', 'csgrs=' + libraries['csgrs']['path'], '--extern', 'hypermesh=' + libraries['hypermesh']['path'],
        '-L', 'dependency=' + str(TARGET / 'release/deps'), '-o', str(binary)])
    binary_hash = sha(binary)
    observations = []
    for case in scope['cases']:
        run(case, ['timeout', '--signal=TERM', '--kill-after=10s', '180s', str(binary), case, scope['mode']])
        assert (OUT / (case + '.stdout')).read_bytes() == (grid / (case + '-' + scope['mode'] + '.stdout')).read_bytes()
        traces = re.findall(r'sphere-hull-undecided face=\[(\d+), (\d+), (\d+)\] point=(\d+) point_count=(\d+)',
                            (OUT / (case + '.stderr')).read_text())
        assert traces
        observations.append({'case': case, 'traces': [list(map(int, row)) for row in traces], 'stdout_unchanged': True})
    guard(full=True)
    assert sha(binary) == binary_hash and all(sha(r['path']) == r['sha256'] for r in libraries.values())
    assert len(commands) == 7
    dump(OUT / 'result.json', {'complete': True, 'commands_passed': 7, 'observations': observations,
        'binary': {'path': str(binary), 'sha256': binary_hash, 'bytes': binary.stat().st_size},
        'all_sources_and_inputs_unchanged': True,
        'artifacts_sha256': {p.name: sha(p) for p in sorted(OUT.iterdir()) if p.is_file()},
        'runtime_admitted': False, 'sphere_certificate_pending': True, 'plan_complete': False})
    print(json.dumps({'complete': True, 'commands_passed': 7, 'observations': observations}), flush=True)


if __name__ == '__main__':
    main()
