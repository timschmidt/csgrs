"""Fixed signed-zero display-warming diagnostic on two frozen implementations."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'dyadic-zero-replay-v1'
WORK = ROOT / 'target/dyadic-zero-replay-v1'
GROUPS = {'image': E / 'sphere-cyclotomic-image-v1', 'stable': E / 'sphere-stable-selection-v1'}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def main():
    assert not OUT.exists() and not WORK.exists()
    inputs = {name: sha(E / name) for name in [
        'probe_dyadic_zero_replay_v1.py', 'dyadic_zero_replay_probe_v1.rs',
        'dyadic-zero-replay-scope-v1.json', 'sphere-stable-selection-qualification-v1/result.json',
        'sphere-stable-selection-qualification-closure-v1.json',
        'mesh-identity-preparation-v1/source-hashes.json.gz']}
    qualified = json.loads((E / 'sphere-stable-selection-qualification-v1/result.json').read_text())
    assert qualified['complete'] and qualified['commands_all_zero'] == 10
    protocols, mappings, libraries = {}, {}, {}
    for group, prior in GROUPS.items():
        result = json.loads((prior / 'result.json').read_text())
        assert result['complete']
        inputs[str((prior / 'result.json').relative_to(E))] = sha(prior / 'result.json')
        for name, digest in result['artifacts_sha256'].items():
            assert sha(prior / name) == digest
            inputs[str((prior / name).relative_to(E))] = digest
        protocols[group] = json.loads((prior / 'protocol.json').read_text())
        mappings[group] = json.loads(gzip.decompress((prior / 'source-hashes.json.gz').read_bytes()))
        libraries[group] = json.loads((prior / 'libraries.json').read_text())
    baseline = json.loads(gzip.decompress((E / 'mesh-identity-preparation-v1/source-hashes.json.gz').read_bytes()))
    root_mapping = {name: sha(ROOT / name) for name in baseline}
    assert all(root_mapping[name] == digest for name, digest in baseline.items() if name != 'docs/properties-status.json')
    binaries = {}

    def guard(full=False):
        assert all(sha(E / name) == digest for name, digest in inputs.items())
        assert all(sha(row['path']) == row['sha256'] for records in libraries.values() for row in records.values())
        assert all(sha(row['path']) == row['sha256'] for row in binaries.values())
        if full:
            assert all(sha(ROOT / name) == digest for name, digest in root_mapping.items())
            for group, mapping in mappings.items():
                assert all(sha(Path(protocols[group]['source']) / name) == digest for name, digest in mapping.items())

    guard(full=True)
    OUT.mkdir()
    WORK.mkdir()
    (OUT / 'root-source-hashes.json.gz').write_bytes(gzip.compress(
        (json.dumps(root_mapping, sort_keys=True) + '\n').encode(), mtime=0))
    dump(OUT / 'protocol.json', {'inputs_sha256': inputs, 'libraries': libraries,
        'sources': {group: protocol['source'] for group, protocol in protocols.items()},
        'source_files': {group: len(mapping) for group, mapping in mappings.items()},
        'root_source_files': len(root_mapping), 'scope': json.loads((E / 'dyadic-zero-replay-scope-v1.json').read_text()),
        'compiler': subprocess.check_output(['rustc', '-Vv'], text=True),
        'not_a_cost_trial': True, 'runtime_admitted': False, 'plan_complete': False})
    commands, observations = [], []

    def run(name, command, group):
        guard()
        print(json.dumps({'start': name}), flush=True)
        start = time.monotonic()
        with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', '600', *command],
                cwd=protocols[group]['source'], stdout=stdout, stderr=stderr).returncode
        row = {'name': name, 'command': command, 'exit_code': code,
            'diagnostic_process_wall_seconds': time.monotonic() - start,
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

    run('format', ['rustfmt', '--edition', '2024', '--check', str(E / 'dyadic_zero_replay_probe_v1.rs')], 'image')
    for group, libs in libraries.items():
        binary = WORK / group
        run('link-' + group, ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(E / 'dyadic_zero_replay_probe_v1.rs'),
            '--extern', 'csgrs=' + libs['csgrs']['path'], '--extern', 'hypermesh=' + libs['hypermesh']['path'],
            '-L', 'dependency=' + str(Path(libs['hypermesh']['path']).parent), '-o', str(binary)], group)
        binaries[group] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
    dump(OUT / 'binaries.json', binaries)
    for group in GROUPS:
        raw = run(group, [binaries[group]['path']], group)
        matches = re.findall(r'^counterexample mask=(\d+) masks_checked=(\d+) negative_zero_before=(\d+) display_f32_negative_zeros=(\d+) negative_zero_after=(\d+)$', raw, re.MULTILINE)
        assert len(matches) <= 1
        if matches:
            mask, checked, before, display, after = map(int, matches[0])
            assert checked == mask + 1 and 0 <= mask < 4096 and before == mask.bit_count()
            assert raw.count('position_order=') == raw.count('triangle_rows=') == 3
            observation = {'group': group, 'counterexample': True, 'mask': mask,
                'masks_checked': checked, 'negative_zero_before': before,
                'display_f32_negative_zeros': display, 'negative_zero_after': after,
                'geometry_independently_validated': False}
        else:
            assert raw.splitlines()[-1] == 'no_counterexample masks_checked=4096'
            observation = {'group': group, 'counterexample': False, 'masks_checked': 4096}
        observations.append(observation)
        dump(OUT / 'observations.json', observations)
        print(json.dumps(observation), flush=True)
    guard(full=True)
    assert len(commands) == 5
    dump(OUT / 'result.json', {'complete': True, 'commands_all_zero': 5,
        'observations': observations, 'all_frozen_sources_inputs_libraries_and_binaries_unchanged': True,
        'artifacts_sha256': {path.name: sha(path) for path in sorted(OUT.iterdir()) if path.is_file()},
        'not_a_cost_trial': True, 'runtime_admitted': False, 'plan_complete': False})
    print(json.dumps({'complete': True, 'result_sha256': sha(OUT / 'result.json')}), flush=True)


if __name__ == '__main__':
    main()
