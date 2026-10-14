"""Fixed paired native process costs for a row-preserving normal-form optimization."""
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
OUT = E / 'normal-image-native-costs-v1'
GROUPS = {'parent': E / 'sphere-stable-selection-v1', 'candidate': E / 'sphere-normal-image-v1'}
CASES = ['fifteen_degrees', 'original_medium', 'fifth_turn', 'seventh_turn']
ORDERS = [['parent', 'candidate'], ['candidate', 'parent']] * 3


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def main():
    assert not OUT.exists()
    qualification = json.loads((E / 'sphere-normal-image-qualification-v1/result.json').read_text())
    assert qualification['complete'] and qualification['commands_all_zero'] == 12
    scope = json.loads((E / 'normal-image-native-cost-scope-v1.json').read_text())
    assert scope['orders'] == ORDERS and scope['cases'] == CASES
    inputs = {name: sha(E / name) for name in [
        'measure_normal_image_native_cost_v1.py', 'assess_normal_image_native_cost_v1.py',
        'normal-image-native-cost-scope-v1.json', 'sphere_warm_replay_probe_v1.rs',
        'sphere-normal-image-qualification-v1/result.json',
        'sphere-normal-image-feasibility-closure-v1.json', 'sphere-stable-support-v1/result.json',
        'normal-image-scalar-cost-assessment-v1.json', 'normal-image-scalar-cost-closure-v1.json']}
    for directory in ['sphere-normal-image-qualification-v1', 'sphere-stable-support-v1']:
        result = json.loads((E / directory / 'result.json').read_text())
        assert result['complete']
        for name, digest in result['artifacts_sha256'].items():
            assert sha(E / directory / name) == digest
            inputs[directory + '/' + name] = digest
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
    assert protocols['parent']['inputs_sha256']['sphere_warm_replay_probe_v1.rs'] == protocols['candidate']['inputs_sha256']['sphere_warm_replay_probe_v1.rs'] == sha(E / 'sphere_warm_replay_probe_v1.rs')
    assert protocols['parent']['compiler'] == protocols['candidate']['compiler'] == subprocess.check_output(['rustc', '-Vv'], text=True)
    assert {key: value for key, value in protocols['parent']['environment'].items() if key != 'CARGO_TARGET_DIR'} == {
        key: value for key, value in protocols['candidate']['environment'].items() if key != 'CARGO_TARGET_DIR'}
    features = {group: {name: row['features'] for name, row in records.items()} for group, records in libraries.items()}
    assert features['parent'] == features['candidate']
    binaries = {group: json.loads((prior / 'binaries.json').read_text())['sphere-probe'] for group, prior in GROUPS.items()}
    references = {group: {case: str((prior / (case + '.stdout')).relative_to(E)) for case in CASES} for group, prior in GROUPS.items()}
    expected = {group: {case: (E / name).read_bytes() for case, name in cases.items()} for group, cases in references.items()}
    assert expected['parent'] == expected['candidate']
    baseline = json.loads(gzip.decompress((E / 'mesh-identity-preparation-v1/source-hashes.json.gz').read_bytes()))
    root_mapping = {name: sha(ROOT / name) for name in baseline}
    assert all(root_mapping[name] == digest for name, digest in baseline.items() if name != 'docs/properties-status.json')

    def guard(full=False):
        assert all(sha(E / name) == digest for name, digest in inputs.items())
        assert all(sha(row['path']) == row['sha256'] for row in binaries.values())
        assert all(sha(row['path']) == row['sha256'] for records in libraries.values() for row in records.values())
        if full:
            assert all(sha(ROOT / name) == digest for name, digest in root_mapping.items())
            for group, protocol in protocols.items():
                assert all(sha(Path(protocol['source']) / name) == digest for name, digest in mappings[group].items())
                assert all(sha(E / name) == digest for name, digest in protocol['inputs_sha256'].items())

    guard(full=True)
    OUT.mkdir()
    cpu = min(os.sched_getaffinity(0))
    (OUT / 'root-source-hashes.json.gz').write_bytes(gzip.compress(
        (json.dumps(root_mapping, sort_keys=True) + '\n').encode(), mtime=0))
    dump(OUT / 'protocol.json', {'inputs_sha256': inputs, 'binaries': binaries,
        'libraries': libraries, 'library_features': features,
        'source_protocols_sha256': {group: sha(prior / 'protocol.json') for group, prior in GROUPS.items()},
        'reference_transcripts': references,
        'reference_output_bytes': {group: {case: len(raw) for case, raw in cases.items()} for group, cases in expected.items()},
        'reference_output_sha256': {group: {case: hashlib.sha256(raw).hexdigest() for case, raw in cases.items()} for group, cases in expected.items()},
        'root_source_files': len(root_mapping), 'source_files': {group: len(mapping) for group, mapping in mappings.items()},
        'compiler': subprocess.check_output(['rustc', '-Vv'], text=True),
        'host': list(os.uname()), 'cpuinfo': Path('/proc/cpuinfo').read_text().split('\n\n')[0],
        'pinned_cpu': cpu, 'load_average_before': list(os.getloadavg()),
        'cases': CASES, 'orders': ORDERS, 'preflight_processes': 8, 'sample_processes': 48,
        'hull_queries_per_process': 3, 'samples_may_be_extended_filtered_or_pooled': False,
        'measurement': scope['measurement'], 'output_contract': scope['output_contract'],
        'limits': scope['limitations'], 'runtime_admitted': False, 'plan_complete': False})
    commands, samples = [], []

    def run(name, command, group, seconds):
        guard()
        print(json.dumps({'start': name}), flush=True)
        before = resource.getrusage(resource.RUSAGE_CHILDREN)
        with (OUT / (name + '.stdout')).open('wb') as stdout, (OUT / (name + '.stderr')).open('wb') as stderr:
            started = time.perf_counter()
            code = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', str(seconds), *command],
                cwd=protocols[group]['source'], stdout=stdout, stderr=stderr).returncode
            elapsed = time.perf_counter() - started
        after = resource.getrusage(resource.RUSAGE_CHILDREN)
        row = {'name': name, 'command': command, 'group': group, 'exit_code': code,
            'process_wall_seconds': elapsed, 'child_user_cpu_seconds': after.ru_utime - before.ru_utime,
            'child_system_cpu_seconds': after.ru_stime - before.ru_stime,
            'stdout_sha256': sha(OUT / (name + '.stdout')), 'stderr_sha256': sha(OUT / (name + '.stderr'))}
        commands.append(row)
        dump(OUT / 'commands.json', commands)
        print(json.dumps({'finished': name, 'exit_code': code}), flush=True)
        guard()
        if code:
            dump(OUT / 'failure.json', {'complete': False, 'failed_command': row,
                'runtime_admitted': False, 'plan_complete': False})
            raise SystemExit(code)
        return row

    for group in GROUPS:
        run('size-' + group, ['/usr/bin/size', '-A', binaries[group]['path']], group, 180)
    for round_number, order in [(0, ['parent', 'candidate']), *enumerate(ORDERS, 1)]:
        for position, group in enumerate(order, 1):
            for case in CASES:
                name = ('preflight' if round_number == 0 else 'sample') + f'-{round_number}-{group}-{case}'
                row = run(name, ['taskset', '-c', str(cpu), binaries[group]['path'], case], group, 600)
                assert (OUT / (name + '.stdout')).read_bytes() == expected[group][case]
                samples.append(row | {'case': case, 'round': round_number, 'position': position,
                    'preflight': round_number == 0, 'full_output_matches_group_reference': True})
                dump(OUT / 'samples.json', samples)
    assert len(commands) == 58 and len(samples) == 56
    guard(full=True)
    dump(OUT / 'result.json', {'complete': True, 'commands_all_zero': 58,
        'preflight_processes': 8, 'sample_processes': 48,
        'all_outputs_match_group_specific_exactly_validated_reference': True,
        'all_outputs_identical_across_groups': True,
        'all_frozen_inputs_sources_libraries_and_binaries_unchanged': True,
        'load_average_after': list(os.getloadavg()),
        'artifacts_sha256': {path.name: sha(path) for path in sorted(OUT.iterdir()) if path.is_file()},
        'runtime_admitted': False, 'plan_complete': False})
    print(json.dumps({'complete': True, 'result_sha256': sha(OUT / 'result.json')}), flush=True)


if __name__ == '__main__':
    main()
