"""Closed six-round paired mesh cost trial, after all correctness qualification."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'mesh-identity-costs-v1'
WORK = ROOT / 'target/mesh-identity-costs-v1'
PHASES = ['retain_empty', 'retain_cube', 'cold_bounds', 'warm_bounds', 'cold_box',
          'warm_box', 'cold_soup', 'warm_soup', 'reversed', 'subdivided', 'union',
          'replace_positions', 'drop_replaced', 'make_mut_positions', 'drop_make_mut']
ORDERS = [['baseline', 'candidate'], ['candidate', 'baseline']] * 3


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def main():
    assert not OUT.exists() and not WORK.exists()
    qualified = E / 'mesh-identity-qualification-v1'
    q = json.loads((qualified / 'result.json').read_text())
    assert q['complete'] and q['commands_passed'] == 18
    assert q['all_nine_public_mutation_controls_correct'] and q['all_four_borrowed_view_controls_correct']
    assert all(sha(qualified / p) == h for p, h in q['artifacts_sha256'].items())
    prep = json.loads((E / 'mesh-identity-preparation-v1/protocol.json').read_text())
    base = json.loads((E / 'baseline-v1/protocol.json').read_text())
    mappings = {group: json.loads(gzip.decompress((E / file).read_bytes())) for group, file in [
        ('baseline', 'baseline-v1/source-hashes.json.gz'),
        ('candidate', 'mesh-identity-preparation-v1/source-hashes.json.gz')]}
    sources = {'baseline': Path(base['source']), 'candidate': Path(prep['source'])}
    targets = {'baseline': Path(base['target']), 'candidate': Path(prep['target'])}
    libraries = {
        'baseline': json.loads((E / 'mesh-fact-identity-v1/protocol.json').read_text())['libraries']['hypermesh'],
        'candidate': json.loads((qualified / 'libraries.json').read_text())['hypermesh']}
    assert libraries['baseline']['features'] == libraries['candidate']['features']
    inputs = {name: sha(E / name) for name in ['measure_mesh_identity_v1.py', 'mesh_identity_cost_probe_v1.rs']}
    root_sources = {p: sha(ROOT / p) for p in mappings['baseline']}
    assert all(root_sources[p] == h for p, h in mappings['baseline'].items() if p != 'docs/properties-status.json')
    binaries = {}

    def guard(full=False):
        assert all(sha(E / p) == h for p, h in inputs.items())
        assert all(sha(r['path']) == r['sha256'] for r in libraries.values())
        assert all(sha(r['path']) == r['sha256'] for r in binaries.values())
        if full:
            assert all(sha(ROOT / p) == h for p, h in root_sources.items())
            for group in mappings:
                assert all(sha(sources[group] / p) == h for p, h in mappings[group].items())
            assert all(sha(E / p) == h for p, h in prep['inputs_sha256'].items())

    guard(full=True)
    cpu = min(os.sched_getaffinity(0))
    OUT.mkdir()
    WORK.mkdir()
    protocol = {'qualification_sha256': sha(qualified / 'result.json'), 'inputs_sha256': inputs,
        'sources': {g: str(p) for g, p in sources.items()}, 'libraries': libraries,
        'compiler': subprocess.check_output(['rustc', '-Vv'], text=True),
        'inherited_rustflags': os.environ.get('RUSTFLAGS'), 'host': list(os.uname()),
        'cpuinfo': Path('/proc/cpuinfo').read_text().split('\n\n')[0], 'pinned_cpu': cpu,
        'phases': PHASES, 'orders': ORDERS, 'time_count': 1024, 'memory_count': 128,
        'preflight_count': 16, 'fixed_sample_processes': 24, 'preflight_processes': 4,
        'all_links_before_preflights_and_samples': True, 'samples_may_be_extended_or_filtered': False,
        'allocation_scope': 'Requested global allocator bytes, including cached data and retained weak backing allocations; not allocator usable sizes or RSS.',
        'witness_scope': 'Full final exact geometry, bounds, planes, certainty and Boolean provenance per phase; repeated operations consume the same fixed geometry.',
        'compatibility_change': 'Arc::get_mut succeeds for independent baseline cube buffers and fails while candidate weak identities exist.',
        'interpretation': 'Descriptive paired ratios only. Correctness outranks these costs; no performance improvement is presumed.',
        'runtime_admitted': False, 'plan_complete': False}
    dump(OUT / 'protocol.json', protocol)
    commands = []

    def run(name, command, group):
        guard()
        print(json.dumps({'start': name}), flush=True)
        start = time.monotonic()
        with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', '300s', *command],
                                  cwd=sources[group], stdout=stdout, stderr=stderr).returncode
        row = {'name': name, 'group': group, 'command': command, 'exit_code': code,
            'process_wall_seconds': time.monotonic() - start,
            'stdout_sha256': sha(OUT / (name + '.stdout')), 'stderr_sha256': sha(OUT / (name + '.stderr'))}
        commands.append(row)
        dump(OUT / 'commands.json', commands)
        print(json.dumps({'finished': name, 'exit_code': code}), flush=True)
        assert code == 0, row
        guard()
        return (OUT / (name + '.stdout')).read_text()

    run('format', ['rustfmt', '--edition', '2024', '--check', str(E / 'mesh_identity_cost_probe_v1.rs')], 'baseline')
    for group in ['baseline', 'candidate']:
        for mode in ['time', 'memory']:
            key = group + '-' + mode
            binary = WORK / key
            command = ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(E / 'mesh_identity_cost_probe_v1.rs'),
                '--extern', 'hypermesh=' + libraries[group]['path'], '-L', 'dependency=' + str(targets[group] / 'release/deps')]
            if mode == 'memory':
                command += ['--cfg', 'feature="count-alloc"']
            run('link-' + key, command + ['-o', str(binary)], group)
            binaries[key] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
    dump(OUT / 'binaries.json', binaries)
    for group in ['baseline', 'candidate']:
        run('size-' + group, ['/usr/bin/size', '-A', binaries[group + '-time']['path']], group)
    trials = []
    witnesses = {}

    def sample(group, mode, count, round_number, position, preflight):
        name = ('preflight' if preflight else 'sample') + f'-{mode}-{round_number}-{group}'
        raw = run(name, ['taskset', '-c', str(cpu), binaries[group + '-' + mode]['path'], str(count)], group)
        data = [json.loads(line) for line in raw.splitlines()]
        rows = [r for r in data if 'name' in r]
        identity = [r for r in data if 'witness' in r]
        compatibility = [r for r in data if 'compatibility' in r]
        assert len(data) == 31 and len(compatibility) == 1
        assert [r['name'] for r in rows] == [r['witness'] for r in identity] == PHASES
        assert all(r['allocation_accounting'] == (mode == 'memory') for r in rows)
        assert all(r['iterations'] == (count * 16 if r['name'] in ['warm_bounds', 'warm_box'] else
                   max(1, count // 16) if r['name'] == 'union' else count) for r in rows)
        assert identity == witnesses.setdefault(count, identity)
        assert compatibility[0]['positions_get_mut'] == compatibility[0]['triangles_get_mut'] == (group == 'baseline')
        trials.append({'name': name, 'group': group, 'mode': mode, 'count': count, 'round': round_number,
            'position': position, 'preflight': preflight, 'rows': rows, 'witnesses': identity, 'compatibility': compatibility[0]})
        dump(OUT / 'trials.json', trials)

    for mode in ['time', 'memory']:
        for position, group in enumerate(['baseline', 'candidate'], 1):
            sample(group, mode, 16, 0, position, True)
    for mode, count in [('time', 1024), ('memory', 128)]:
        for round_number, order in enumerate(ORDERS, 1):
            for position, group in enumerate(order, 1):
                sample(group, mode, count, round_number, position, False)
    assert len(commands) == 35 and len(trials) == 28
    guard(full=True)
    dump(OUT / 'result.json', {'complete': True, 'commands_passed': 35, 'fixed_sample_processes': 24,
        'preflight_processes': 4, 'phases': 15, 'all_exact_witnesses_match': True,
        'all_sources_and_inputs_unchanged': True,
        'artifacts_sha256': {p.name: sha(p) for p in sorted(OUT.iterdir()) if p.is_file()},
        'assessment_pending': True, 'runtime_admitted': False, 'plan_complete': False})
    print(json.dumps({'complete': True, 'commands_passed': 35, 'all_exact_witnesses_match': True}), flush=True)


if __name__ == '__main__':
    main()
