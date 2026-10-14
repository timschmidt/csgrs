"""Fixed allocation/retention trial on original native sphere consumers."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'sphere-stable-selection-resources-v1'
WORK = ROOT / 'target/sphere-stable-selection-resources-v1'
CASES = ['fifteen_degrees', 'original_medium', 'fifth_turn', 'seventh_turn']
ORDERS = [['baseline', 'image', 'stable'], ['image', 'stable', 'baseline'], ['stable', 'baseline', 'image']]
PHASES = ['construct', 'cold_hull', 'warm_hull', 'replay_hull',
          'drop_outputs_and_input', 'clear_grammar_cache']


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def parse_reference(raw, case):
    lines = raw.splitlines()
    assert lines.pop(0) == f'replay_case={case} repetitions=3 grammar_policy=default allocator=default'
    witnesses = {}
    for index, phase in enumerate(['cold', 'warm', 'replay']):
        offset = index * 5
        assert lines[offset] == 'witness_phase=' + phase
        block = lines[offset + 1:offset + 5]
        assert len(block) == 4 and block[0].startswith('input case=' + case + ' ')
        assert block[1].startswith('position_order=') and block[2].startswith('triangle_rows=')
        assert block[3].startswith('result case=' + case + ' certified=true original_positions_preserved=true ')
        witnesses[phase] = '\n'.join(block) + '\n'
    assert len(lines) == 15
    return witnesses


def parse_output(raw, case, expected):
    rows = []
    witnesses = {}
    phase = None
    lifecycle = []
    for line in raw.splitlines():
        if line.startswith('{'):
            row = json.loads(line)
            if 'phase' in row:
                rows.append(row)
            else:
                assert list(row) == ['lifecycle_retained_requested_bytes']
                lifecycle.append(row)
        elif line.startswith('resource_case='):
            assert line == f'resource_case={case} repetitions=3 grammar_policy=default'
        elif line.startswith('witness_phase='):
            phase = line.split('=', 1)[1]
            assert phase not in witnesses
            witnesses[phase] = []
        else:
            assert phase in witnesses
            witnesses[phase].append(line)
    assert [row['phase'] for row in rows] == PHASES
    assert list(witnesses) == ['cold', 'warm', 'replay'] and len(lifecycle) == 1
    for phase, lines in witnesses.items():
        assert '\n'.join(lines) + '\n' == expected[phase]
    return rows, lifecycle[0]


def main():
    assert not OUT.exists() and not WORK.exists()
    qualification = E / 'sphere-cyclotomic-image-qualification-assessment-v1.json'
    qualified = json.loads(qualification.read_text())
    assert qualified['complete'] and qualified['commands_all_zero'] == 11
    stable_qualified = json.loads((E / 'sphere-stable-selection-qualification-v1/result.json').read_text())
    assert stable_qualified['complete'] and stable_qualified['commands_all_zero'] == 10
    cost = json.loads((E / 'sphere-stable-selection-cost-assessment-v1.json').read_text())
    assert cost['complete'] and cost['commands_all_zero'] == 58
    scope = json.loads((E / 'sphere-stable-selection-resources-scope-v1.json').read_text())
    assert scope['orders'] == ORDERS and scope['fixed_sample_processes'] == 27
    inputs = {name: sha(E / name) for name in [
        'measure_sphere_stable_resources_v1.py', 'assess_sphere_stable_resources_v1.py',
        'sphere_resource_probe_v1.rs', 'sphere-stable-selection-resources-scope-v1.json',
        'sphere-cyclotomic-image-qualification-assessment-v1.json',
        'sphere-cyclotomic-image-qualification-v1/result.json',
        'sphere-cyclotomic-image-cost-assessment-v1.json',
        'sphere-native-support-v1/result.json',
        'mesh-identity-qualification-v1/result.json',
        'sphere-cyclotomic-v2/result.json', 'sphere-cyclotomic-image-v1/result.json',
        'mesh-identity-preparation-v1/source-hashes.json.gz',
        'sphere-cyclotomic-v2/source-hashes.json.gz',
        'sphere-cyclotomic-image-v1/source-hashes.json.gz',
        'sphere-stable-selection-v1/result.json',
        'sphere-stable-selection-v1/source-hashes.json.gz',
        'sphere-stable-selection-qualification-v1/result.json',
        'sphere-stable-selection-qualification-closure-v1.json',
        'sphere-stable-selection-cost-assessment-v1.json',
        'sphere-stable-selection-cost-closure-v1.json',
        'sphere-stable-selection-closure-v1.json',
        'sphere-warm-replay-assessment-v1.json', 'sphere-resource-failure-closure-v1.json',
        'sphere-warm-replay-v1/baseline-fifteen_degrees.stdout',
        *[f'sphere-warm-replay-v1/image-{case}.stdout' for case in CASES],
        *[f'sphere-stable-selection-v1/{case}.stdout' for case in CASES]]}
    for directory in ['sphere-stable-selection-qualification-v1', 'sphere-stable-support-v1',
                      'sphere-warm-replay-v1', 'sphere-warm-support-v1', 'sphere-native-support-v1']:
        result = json.loads((E / directory / 'result.json').read_text())
        assert result['complete']
        inputs[directory + '/result.json'] = sha(E / directory / 'result.json')
        for name, digest in result['artifacts_sha256'].items():
            assert sha(E / directory / name) == digest
            inputs[directory + '/' + name] = digest
    specifications = {
        'baseline': ('mesh-identity-preparation-v1', 'mesh-identity-qualification-v1'),
        'image': ('sphere-cyclotomic-image-v1', 'sphere-cyclotomic-image-v1'),
        'stable': ('sphere-stable-selection-v1', 'sphere-stable-selection-v1'),
    }
    sources = {}
    mappings = {}
    libraries = {}
    for group, (preparation, build) in specifications.items():
        source_protocol = json.loads((E / preparation / 'protocol.json').read_text())
        sources[group] = Path(source_protocol['source'])
        mappings[group] = json.loads(gzip.decompress((E / preparation / 'source-hashes.json.gz').read_bytes()))
        libraries[group] = json.loads((E / build / 'libraries.json').read_text())
        result = json.loads((E / build / 'result.json').read_text())
        assert result['complete']
        assert all(sha(E / build / name) == digest for name, digest in result['artifacts_sha256'].items())
    features = {group: {name: row['features'] for name, row in libs.items()} for group, libs in libraries.items()}
    assert features['baseline'] == features['stable'] == features['image']
    root_mapping = {name: sha(ROOT / name) for name in mappings['baseline']}
    assert all(root_mapping[name] == digest for name, digest in mappings['baseline'].items()
               if name != 'docs/properties-status.json')
    references = {
        'baseline': {'fifteen_degrees': 'sphere-warm-replay-v1/baseline-fifteen_degrees.stdout'},
        'image': {case: f'sphere-warm-replay-v1/image-{case}.stdout' for case in CASES},
        'stable': {case: f'sphere-stable-selection-v1/{case}.stdout' for case in CASES},
    }
    expected = {group: {case: parse_reference((E / name).read_text(), case)
                       for case, name in cases.items()} for group, cases in references.items()}
    binaries = {}

    def guard(full=False):
        assert all(sha(E / name) == digest for name, digest in inputs.items())
        assert all(sha(row['path']) == row['sha256'] for libs in libraries.values() for row in libs.values())
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
        'inputs_sha256': inputs, 'libraries': libraries, 'library_features': features,
        'sources': {group: str(path) for group, path in sources.items()},
        'source_mappings_sha256': {group: sha(E / preparation / 'source-hashes.json.gz')
                                  for group, (preparation, _) in specifications.items()},
        'source_files': {group: len(mapping) for group, mapping in mappings.items()},
        'root_source_files': len(root_mapping), 'phases': PHASES, 'orders': ORDERS,
        'case_sets': {'baseline': CASES[:1], 'image': CASES, 'stable': CASES},
        'reference_transcripts': references,
        'reference_output_sha256': {g: {c: sha(E / n) for c, n in cases.items()} for g, cases in references.items()},
        'phase_witness_sha256': {g: {c: {p: hashlib.sha256(w.encode()).hexdigest() for p, w in phases.items()}
                                   for c, phases in cases.items()} for g, cases in expected.items()},
        'phase_witness_bytes': {g: {c: {p: len(w.encode()) for p, w in phases.items()}
                                  for c, phases in cases.items()} for g, cases in expected.items()},
        'fixed_sample_processes': 27, 'hull_queries_per_process': 3,
        'preflight_processes': 0, 'samples_may_be_extended_filtered_or_pooled': False,
        'compiler': subprocess.check_output(['rustc', '-Vv'], text=True),
        'inherited_rustflags': os.environ.get('RUSTFLAGS'), 'host': list(os.uname()),
        'cpuinfo': Path('/proc/cpuinfo').read_text().split('\n\n')[0], 'pinned_cpu': cpu,
        'measurement': 'Requested global allocator allocation/reallocation traffic, retained live-byte delta, and peak increase above each phase entry. Not usable allocator sizes, stack bytes, RSS or an uninstrumented timing benchmark.',
        'cache_state': 'Each process clears grammar caches, constructs one original sphere, then runs three STRICT hull queries on the same retained input. All three outputs stay alive until validation and disposal. No validation or serialization occurs between the three queries. Process startup/runtime I/O is initialized before lifecycle accounting; global shared scalar constants may survive cleanup.',
        'baseline_scope': 'The admitted runtime already certifies only 24x12. Other original cases are measured for the finite-image parent and deterministic-selection candidate, which both certify them. Failed baseline calls are not substituted for successful work; parent row variation and candidate row changes are explicitly retained.',
        'witness_scope': scope['output_contract'],
        'old_failed_trial_preserved_without_resumption': True,
        'interpretation': 'Publish all three resource observations per group/case/phase, including differences. Any repeat variation is reported, not filtered. No speedup or runtime admission follows from these measurements alone.',
        'runtime_admitted': False, 'plan_complete': False,
    })
    commands = []
    samples = []

    def run(name, command, group, seconds=600):
        guard()
        print(json.dumps({'start': name}), flush=True)
        started = time.monotonic()
        with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', str(seconds), *command],
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

    run('format', ['rustfmt', '--edition', '2024', '--check', str(E / 'sphere_resource_probe_v1.rs')], 'baseline')
    for group, libs in libraries.items():
        binary = WORK / group
        run('link-' + group, ['rustc', '--edition=2024', '-O', '-D', 'warnings',
            str(E / 'sphere_resource_probe_v1.rs'), '--extern', 'csgrs=' + libs['csgrs']['path'],
            '-L', 'dependency=' + str(Path(libs['hypermesh']['path']).parent), '-o', str(binary)], group)
        binaries[group] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
    dump(OUT / 'binaries.json', binaries)
    for group in libraries:
        run('size-' + group, ['/usr/bin/size', '-A', binaries[group]['path']], group)
    for round_number, order in enumerate(ORDERS, 1):
        for position, group in enumerate(order, 1):
            for case in (CASES[:1] if group == 'baseline' else CASES):
                name = f'sample-{round_number}-{group}-{case}'
                raw = run(name, ['taskset', '-c', str(cpu), binaries[group]['path'], case], group)
                rows, lifecycle = parse_output(raw, case, expected[group][case])
                samples.append({'name': name, 'group': group, 'case': case, 'round': round_number,
                    'position': position, 'rows': rows, 'lifecycle': lifecycle, 'all_three_exact_outputs_match_group_phase_reference': True,
                    'all_three_exact_outputs_identical': len(set(expected[group][case].values())) == 1})
                dump(OUT / 'samples.json', samples)
    assert len(commands) == 34 and len(samples) == 27
    guard(full=True)
    dump(OUT / 'result.json', {
        'complete': True, 'commands_all_zero': 34, 'fixed_sample_processes': 27,
        'exact_output_transcripts_rechecked': 81, 'all_frozen_sources_inputs_and_libraries_unchanged': True,
        'all_outputs_match_group_phase_reference': True, 'old_trial_samples_pooled': False,
        'artifacts_sha256': {path.name: sha(path) for path in sorted(OUT.iterdir()) if path.is_file()},
        'runtime_admitted': False, 'plan_complete': False,
    })
    print(json.dumps({'complete': True, 'result_sha256': sha(OUT / 'result.json')}), flush=True)


if __name__ == '__main__':
    main()
