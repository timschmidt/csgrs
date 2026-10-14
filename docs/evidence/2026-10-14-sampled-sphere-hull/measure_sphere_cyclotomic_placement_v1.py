"""Fixed paired native process timing screen for early versus late zero proofs."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import resource
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'sphere-cyclotomic-placement-costs-v1'
GROUPS = {'late': E / 'sphere-cyclotomic-v2', 'early': E / 'sphere-cyclotomic-early-v1'}
CASES = ['fifteen_degrees', 'original_medium', 'fifth_turn', 'seventh_turn']
ORDERS = [['late', 'early'], ['early', 'late']] * 3


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def main():
    assert not OUT.exists()
    inputs = {name: sha(E / name) for name in [
        'measure_sphere_cyclotomic_placement_v1.py', 'assess_sphere_cyclotomic_placement_v1.py',
        'sphere-cyclotomic-placement-cost-scope-v1.json',
        'sphere-cyclotomic-early-assessment-v1.json', 'sphere-cyclotomic-assessment-v2.json',
        'sphere-cyclotomic-qualification-assessment-v1.json', 'sphere-native-support-v1/result.json']}
    binaries = {}
    protocols = {}
    mappings = {}
    for group, prior in GROUPS.items():
        result = json.loads((prior / 'result.json').read_text())
        assert result['complete'] and result['commands'] == 12
        assert all(sha(prior / name) == digest for name, digest in result['artifacts_sha256'].items())
        protocols[group] = json.loads((prior / 'protocol.json').read_text())
        mappings[group] = json.loads(gzip.decompress((prior / 'source-hashes.json.gz').read_bytes()))
        binaries[group] = json.loads((prior / 'binaries.json').read_text())['candidate']
    assert protocols['late']['inputs_sha256']['reproduce_rows_v1.rs'] == protocols['early']['inputs_sha256']['reproduce_rows_v1.rs']
    assert {key: value for key, value in protocols['late']['environment'].items() if key != 'CARGO_TARGET_DIR'} == {
        key: value for key, value in protocols['early']['environment'].items() if key != 'CARGO_TARGET_DIR'}
    features = {group: {name: row['features'] for name, row in json.loads((prior / 'libraries.json').read_text()).items()}
                for group, prior in GROUPS.items()}
    assert features['late'] == features['early']
    baseline = json.loads(gzip.decompress((E / 'mesh-identity-preparation-v1/source-hashes.json.gz').read_bytes()))
    root_mapping = {name: sha(ROOT / name) for name in baseline}
    assert all(root_mapping[name] == digest for name, digest in baseline.items() if name != 'docs/properties-status.json')
    expected = {case: (GROUPS['late'] / ('candidate-' + case + '.stdout')).read_bytes() for case in CASES}
    assert all(expected[case] == (GROUPS['early'] / ('candidate-' + case + '.stdout')).read_bytes() for case in CASES)

    def guard(full=False):
        assert all(sha(E / name) == digest for name, digest in inputs.items())
        assert all(sha(row['path']) == row['sha256'] for row in binaries.values())
        if full:
            assert all(sha(ROOT / name) == digest for name, digest in root_mapping.items())
            for group, protocol in protocols.items():
                assert all(sha(Path(protocol['source']) / name) == digest for name, digest in mappings[group].items())
                assert all(sha(E / name) == digest for name, digest in protocol['inputs_sha256'].items())

    guard(full=True)
    OUT.mkdir()
    (OUT / 'root-source-hashes.json.gz').write_bytes(gzip.compress(
        (json.dumps(root_mapping, sort_keys=True) + '\n').encode(), mtime=0))
    cpu = min(os.sched_getaffinity(0))
    dump(OUT / 'protocol.json', {
        'inputs_sha256': inputs, 'binaries': binaries, 'library_features': features,
        'source_protocols_sha256': {group: sha(path / 'protocol.json') for group, path in GROUPS.items()},
        'root_source_files': len(root_mapping), 'source_files': {group: len(mapping) for group, mapping in mappings.items()},
        'compiler': subprocess.check_output(['rustc', '-Vv'], text=True),
        'host': list(os.uname()), 'cpuinfo': Path('/proc/cpuinfo').read_text().split('\n\n')[0],
        'pinned_cpu': cpu, 'load_average_before': list(os.getloadavg()),
        'cases': CASES, 'orders': ORDERS, 'preflight_processes': 8, 'sample_processes': 48,
        'samples_may_be_extended_filtered_or_pooled': False,
        'measurement': 'Full original native reproducer process wall time and child user/system CPU, including sphere construction, hull query, exact checks and row serialization. Each process starts with fresh process-local caches; preflights warm operating-system code pages, not retained scalar values across processes.',
        'limits': 'Focused placement screen only. No allocation, retained-byte, RSS, check-body timing, full workspace qualification or runtime admission claim.',
        'runtime_admitted': False, 'plan_complete': False,
    })
    commands = []
    samples = []

    def run(name, command, group, timeout):
        guard()
        before = resource.getrusage(resource.RUSAGE_CHILDREN)
        with (OUT / (name + '.stdout')).open('wb') as stdout, (OUT / (name + '.stderr')).open('wb') as stderr:
            started = time.perf_counter()
            code = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', str(timeout), *command],
                                  cwd=protocols[group]['source'], stdout=stdout, stderr=stderr).returncode
            elapsed = time.perf_counter() - started
        after = resource.getrusage(resource.RUSAGE_CHILDREN)
        row = {'name': name, 'command': command, 'group': group, 'exit_code': code,
               'process_wall_seconds': elapsed, 'child_user_cpu_seconds': after.ru_utime - before.ru_utime,
               'child_system_cpu_seconds': after.ru_stime - before.ru_stime,
               'stdout_sha256': sha(OUT / (name + '.stdout')), 'stderr_sha256': sha(OUT / (name + '.stderr'))}
        commands.append(row)
        dump(OUT / 'commands.json', commands)
        print(json.dumps(row), flush=True)
        guard()
        if code:
            dump(OUT / 'failure.json', {'complete': False, 'failed_command': row,
                'runtime_admitted': False, 'plan_complete': False})
            raise SystemExit(code)
        return row

    for group in GROUPS:
        run('size-' + group, ['/usr/bin/size', '-A', binaries[group]['path']], group, 180)
    for round_number, order in [(0, ['late', 'early']), *enumerate(ORDERS, 1)]:
        for position, group in enumerate(order, 1):
            for case in CASES:
                name = ('preflight' if round_number == 0 else 'sample') + f'-{round_number}-{group}-{case}'
                row = run(name, ['taskset', '-c', str(cpu), binaries[group]['path'], case],
                          group, 600 if case == 'original_medium' else 180)
                assert (OUT / (name + '.stdout')).read_bytes() == expected[case]
                samples.append(row | {'case': case, 'round': round_number, 'position': position,
                                      'preflight': round_number == 0, 'full_output_identical': True})
                dump(OUT / 'samples.json', samples)
    assert len(commands) == 58 and len(samples) == 56
    guard(full=True)
    dump(OUT / 'result.json', {'complete': True, 'commands_all_zero': 58,
        'preflight_processes': 8, 'sample_processes': 48, 'all_exact_native_outputs_identical': True,
        'all_frozen_inputs_sources_and_binaries_unchanged': True, 'load_average_after': list(os.getloadavg()),
        'artifacts_sha256': {path.name: sha(path) for path in sorted(OUT.iterdir()) if path.is_file()},
        'runtime_admitted': False, 'plan_complete': False})
    print(json.dumps({'complete': True, 'samples': 48, 'result_sha256': sha(OUT / 'result.json')}), flush=True)


if __name__ == '__main__':
    main()
