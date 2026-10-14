"""Run fixed exact-rational cache-identity controls on the frozen main libraries."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
BASE = E / 'baseline-v1'
OUT = E / 'mesh-view-identity-v1'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, data):
    path.write_text(json.dumps(data, indent=2) + '\n')


def main():
    assert not OUT.exists()
    result = json.loads((BASE / 'result.json').read_text())
    assert result['complete'] and result['commands_passed'] == 7
    assert all(sha(BASE / p) == h for p, h in result['artifacts_sha256'].items())
    protocol = json.loads((BASE / 'protocol.json').read_text())
    source = Path(protocol['source'])
    mapping = json.loads(gzip.decompress((BASE / 'source-hashes.json.gz').read_bytes()))
    assert sha(BASE / 'source-hashes.json.gz') == protocol['source_hashes_sha256']
    assert all(sha(source / p) == h for p, h in mapping.items())
    assert all(sha(ROOT / p) == h for p, h in mapping.items() if p != 'docs/properties-status.json')
    cargo = [json.loads(line) for line in (BASE / 'build-libraries.stdout').read_text().splitlines()
             if line.startswith('{')]
    libraries = {}
    for name in ['csgrs', 'hypermesh']:
        rows = [r for r in cargo if r.get('reason') == 'compiler-artifact' and r['target']['name'] == name
                and any(p.endswith('.rlib') for p in r['filenames'])]
        assert len(rows) == 1
        path = next(p for p in rows[0]['filenames'] if p.endswith('.rlib'))
        libraries[name] = {'path': path, 'sha256': sha(path), 'features': rows[0]['features']}
    inputs = {p: sha(E / p) for p in ['probe_mesh_view_identity_v1.py', 'mesh_view_identity_probe_v1.rs']}
    OUT.mkdir()
    binary = Path(protocol['target']).parent / 'mesh-view-identity-v1'
    commands = []
    schedule = [
        ('format', ['rustfmt', '--edition', '2024', '--check', str(E / 'mesh_view_identity_probe_v1.rs')]),
        ('link', ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(E / 'mesh_view_identity_probe_v1.rs'),
                  '--extern', 'csgrs=' + libraries['csgrs']['path'],
                  '--extern', 'hypermesh=' + libraries['hypermesh']['path'],
                  '-L', 'dependency=' + str(Path(protocol['target']) / 'release/deps'), '-o', str(binary)]),
        ('observe', [str(binary)]),
    ]
    dump(OUT / 'protocol.json', {'inputs_sha256': inputs, 'baseline_result_sha256': sha(BASE / 'result.json'),
         'source_hashes_sha256': protocol['source_hashes_sha256'], 'libraries': libraries,
         'commands': schedule, 'scope': 'Four exact-rational public borrowed-view mutation controls; observations, not timing evidence.'})
    for name, command in schedule:
        assert all(sha(E / p) == h for p, h in inputs.items())
        assert all(sha(row['path']) == row['sha256'] for row in libraries.values())
        started = time.monotonic()
        with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(command, cwd=source, env=os.environ,
                                  stdout=stdout, stderr=stderr).returncode
        row = {'name': name, 'command': command, 'exit_code': code,
               'process_wall_seconds': time.monotonic() - started,
               'stdout_sha256': sha(OUT / (name + '.stdout')), 'stderr_sha256': sha(OUT / (name + '.stderr'))}
        commands.append(row)
        dump(OUT / 'commands.json', commands)
        print(json.dumps(row), flush=True)
        if code:
            raise SystemExit(code)
    observations = []
    for line in (OUT / 'observe.stdout').read_text().splitlines():
        fields = dict(part.split('=', 1) for part in line.split())
        assert fields['correct'] in ['true', 'false']
        observations.append(fields)
    assert len(observations) == len({r['control'] for r in observations}) == 4
    assert all(sha(source / p) == h for p, h in mapping.items())
    assert all(sha(ROOT / p) == h for p, h in mapping.items() if p != 'docs/properties-status.json')
    dump(OUT / 'result.json', {'complete': True, 'commands_passed': 3, 'controls': observations,
         'controls_satisfying_contract': sum(r['correct'] == 'true' for r in observations),
         'baseline_source_unchanged': True, 'main_runtime_unchanged': True,
         'binary': {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size},
         'artifacts_sha256': {p.name: sha(p) for p in sorted(OUT.iterdir()) if p.is_file()},
         'runtime_admitted': False, 'plan_complete': False})
    print(json.dumps({'complete': True, 'observations': observations}), flush=True)


if __name__ == '__main__':
    main()
