"""Fixed scalar-query timing and allocation comparison; no pooled samples."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'normal-image-scalar-costs-v1'
WORK = ROOT / 'target/normal-image-scalar-costs-v1'
ORDERS = [['parent', 'candidate'], ['candidate', 'parent']] * 3
REPETITIONS = {
    'normal_rational': 262144, 'normal_pi_plus_e': 16384, 'normal_sqrt5': 16384,
    'normal_trig_nonzero': 32768, 'normal_unsupported_order': 65536,
    'normal_pythagorean_zero': 1024, 'normal_double_angle_zero': 1024,
    'affine_dyadic': 262144, 'affine_nondyadic_rational': 262144,
    'affine_unsupported_pi': 65536, 'affine_unsupported_sqrt5': 65536,
    'affine_trig_nonzero': 4096, 'affine_unsupported_order': 65536, 'affine_original_zero': 1024,
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def main():
    assert not OUT.exists() and not WORK.exists()
    qualification = json.loads((E / 'sphere-normal-image-v1/result.json').read_text())
    assert qualification['complete'] and qualification['commands_all_zero'] == 10
    assert qualification['targeted_scalar_tests_passed'] == 14
    previous_cost = json.loads((E / 'sphere-cyclotomic-scalar-costs-assessment-v2.json').read_text())
    assert previous_cost['complete'] and previous_cost['commands_all_zero'] == 52
    scope = json.loads((E / 'normal-image-scalar-cost-scope-v1.json').read_text())
    assert scope['orders'] == ORDERS and scope['time_repetitions'] == REPETITIONS
    specs = {
        'parent': ('sphere-stable-selection-v1', 'sphere-stable-selection-v1/build.stdout'),
        'candidate': ('sphere-normal-image-v1', 'sphere-normal-image-v1/build-native-consumer.stdout'),
    }
    inputs = {name: sha(E / name) for name in [
        'measure_normal_image_scalar_cost_v1.py', 'assess_normal_image_scalar_cost_v1.py',
        'normal_image_scalar_probe_v1.rs', 'normal-image-scalar-cost-scope-v1.json',
        'sphere-normal-image-v1/result.json', 'sphere-normal-image-feasibility-closure-v1.json',
        'sphere-cyclotomic-scalar-costs-assessment-v2.json',
        'sphere-cyclotomic-scalar-costs-closure-v2.json',
        'mesh-identity-preparation-v1/source-hashes.json.gz',
        *[build for _, build in specs.values()],
        *[preparation + '/source-hashes.json.gz' for preparation, _ in specs.values()],
        *[preparation + '/protocol.json' for preparation, _ in specs.values()]]}
    sources, mappings, libraries = {}, {}, {}
    for group, (preparation, build) in specs.items():
        sources[group] = Path(json.loads((E / preparation / 'protocol.json').read_text())['source'])
        mappings[group] = json.loads(gzip.decompress((E / preparation / 'source-hashes.json.gz').read_bytes()))
        records = [json.loads(line) for line in (E / build).read_text().splitlines() if line.startswith('{')]
        matches = [r for r in records if r.get('reason') == 'compiler-artifact'
                   and r['target']['name'] == 'hyperreal' and any(f.endswith('.rlib') for f in r['filenames'])]
        assert len(matches) == 1
        path = next(f for f in matches[0]['filenames'] if f.endswith('.rlib'))
        libraries[group] = {'path': path, 'sha256': sha(path), 'features': matches[0]['features']}
    assert libraries['parent']['features'] == libraries['candidate']['features']
    for preparation, _ in specs.values():
        assert json.loads((E / preparation / 'protocol.json').read_text())['compiler'] == subprocess.check_output(['rustc', '-Vv'], text=True)
    baseline = json.loads(gzip.decompress((E / 'mesh-identity-preparation-v1/source-hashes.json.gz').read_bytes()))
    root_mapping = {name: sha(ROOT / name) for name in baseline}
    assert all(root_mapping[name] == digest for name, digest in baseline.items()
               if name != 'docs/properties-status.json')
    binaries = {}

    def guard(full=False):
        assert all(sha(E / name) == digest for name, digest in inputs.items())
        assert all(sha(row['path']) == row['sha256'] for row in libraries.values())
        assert all(sha(row['path']) == row['sha256'] for row in binaries.values())
        if full:
            assert all(sha(ROOT / name) == digest for name, digest in root_mapping.items())
            for group, mapping in mappings.items():
                assert all(sha(sources[group] / name) == digest for name, digest in mapping.items())

    guard(full=True)
    OUT.mkdir()
    WORK.mkdir()
    cpu = min(os.sched_getaffinity(0))
    (OUT / 'root-source-hashes.json.gz').write_bytes(gzip.compress(
        (json.dumps(root_mapping, sort_keys=True) + '\n').encode(), mtime=0))
    dump(OUT / 'protocol.json', {
        'inputs_sha256': inputs, 'libraries': libraries,
        'sources': {g: str(p) for g, p in sources.items()},
        'source_mapping_files': {g: prep + '/source-hashes.json.gz' for g, (prep, _) in specs.items()},
        'source_files': {g: len(m) for g, m in mappings.items()}, 'root_source_files': len(root_mapping),
        'orders': ORDERS, 'repetitions': REPETITIONS, 'memory_repetitions': 128,
        'preflight_processes': 4, 'fixed_sample_processes': 24,
        'compiler': subprocess.check_output(['rustc', '-Vv'], text=True),
        'inherited_rustflags': os.environ.get('RUSTFLAGS'), 'host': list(os.uname()),
        'cpuinfo': Path('/proc/cpuinfo').read_text().split('\n\n')[0], 'pinned_cpu': cpu,
        'cache_state': 'Fresh process prepares the same fourteen recipes, then queries each once and repeatedly on the same value. Constructors and shared constants may already have caches; first means first explicit query in this harness, not an assertion that every internal cache is empty.',
        'measurement': 'Query-body wall time includes black-box barriers, exact-result assertions and monotone-fact checks. Allocation mode separately records requested allocator traffic and live-byte deltas, not RSS or stack bytes.',
        'compatibility': 'Every definite rational/sign result has an explicit exact expected value. Previously available facts may not disappear during replay. Both groups must preserve the same existing zero capabilities; the image probe argument requires every existing zero proof.',
        'screen_rule': 'For repeated-query phases with identical per-iteration outcome counts, flag a candidate/parent overhead only when every one of six paired ratios exceeds 1.10, their median exceeds 1.15, and both median phase times are at least 2000000 ns. Otherwise inconclusive. First-query single-call times and memory-mode times are not used for this flag.',
        'samples_may_be_extended_filtered_or_pooled': False,
        'runtime_admitted': False, 'plan_complete': False,
    })
    commands, samples = [], []

    def run(name, command, group):
        guard()
        print(json.dumps({'start': name}), flush=True)
        started = time.monotonic()
        with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', '300s', *command],
                cwd=sources[group], stdout=stdout, stderr=stderr).returncode
        row = {'name': name, 'command': command, 'group': group, 'exit_code': code,
               'process_wall_seconds': time.monotonic() - started,
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

    run('format', ['rustfmt', '--edition', '2024', '--check', str(E / 'normal_image_scalar_probe_v1.rs')], 'parent')
    for group, library in libraries.items():
        for mode in ['time', 'memory']:
            key = group + '-' + mode
            binary = WORK / key
            command = ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(E / 'normal_image_scalar_probe_v1.rs'),
                '--extern', 'hyperreal=' + library['path'], '-L', 'dependency=' + str(Path(library['path']).parent)]
            if mode == 'memory':
                command += ['--cfg', 'feature="count-alloc"']
            run('link-' + key, command + ['-o', str(binary)], group)
            binaries[key] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
    dump(OUT / 'binaries.json', binaries)
    for group in libraries:
        run('size-' + group, ['/usr/bin/size', '-A', binaries[group + '-time']['path']], group)
    metadata = {}
    for mode in ['time', 'memory']:
        for number, order in [(0, ['parent', 'candidate']), *enumerate(ORDERS, 1)]:
            for position, group in enumerate(order, 1):
                name = ('preflight' if number == 0 else 'sample') + f'-{mode}-{number}-{group}'
                raw = run(name, ['taskset', '-c', str(cpu), binaries[group + '-' + mode]['path'], 'image'], group)
                lines = raw.splitlines()
                assert lines[0] == 'scalar_group=image cases=14' and len(lines) == 43
                rows = [json.loads(line) for line in lines if line.startswith('{')]
                kinds = [line for line in lines if line.startswith('kind ')]
                assert len(kinds) == 14 and kinds == metadata.setdefault('kinds', kinds)
                assert [(r['case'], r['phase']) for r in rows] == [(c, p) for c in REPETITIONS for p in ['first', 'repeated']]
                for row in rows:
                    count = 1 if row['phase'] == 'first' else 128 if mode == 'memory' else REPETITIONS[row['case']]
                    assert row['iterations'] == count and row['allocation_accounting'] == (mode == 'memory')
                    assert 0 <= row['certified'] <= count
                    if row['case'] in ['normal_rational', 'affine_dyadic']:
                        assert row['certified'] == count
                    elif row['case'] in ['normal_pythagorean_zero', 'normal_double_angle_zero']:
                        assert row['certified'] == count
                    elif row['case'] == 'affine_original_zero':
                        assert row['certified'] == count
                    else:
                        assert row['certified'] == 0
                samples.append({'name': name, 'mode': mode, 'round': number, 'group': group,
                    'position': position, 'preflight': number == 0, 'rows': rows, 'kinds': kinds})
                dump(OUT / 'samples.json', samples)
    assert len(commands) == 35 and len(samples) == 28
    guard(full=True)
    dump(OUT / 'result.json', {
        'complete': True, 'commands_all_zero': 35, 'fixed_sample_processes': 24, 'preflight_processes': 4,
        'all_frozen_sources_inputs_and_libraries_unchanged': True,
        'artifacts_sha256': {p.name: sha(p) for p in sorted(OUT.iterdir()) if p.is_file()},
        'runtime_admitted': False, 'plan_complete': False,
    })
    print(json.dumps({'complete': True, 'result_sha256': sha(OUT / 'result.json')}), flush=True)


if __name__ == '__main__':
    main()
