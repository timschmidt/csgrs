"""Closed six-order scalar cost trial; all builds finish before all samples."""
import gzip
import json
import os
from pathlib import Path
import re
import subprocess
import time

from pi_memo_cost_common_v1 import E, ROOT, COST_PREP, COST_WORK, ENV, GROUPS, SUITES, ORDERS, sha, dump, compressed, verify_costs
from pi_memo_cost_rows_v1 import parse


def frequency(cpu):
    base = Path('/sys/devices/system/cpu') / ('cpu' + str(cpu)) / 'cpufreq'
    return {key: (base / key).read_text().strip() if (base / key).exists() else None
            for key in ['scaling_driver', 'scaling_governor', 'scaling_cur_freq', 'scaling_min_freq', 'scaling_max_freq']}


def main():
    preparation = json.loads((COST_PREP / 'protocol.json').read_text())
    verify_costs(preparation, full=True)
    review = json.loads((E / 'scalar-cost-source-review-v1.json').read_text())
    assert review['preparation_sha256'] == sha(COST_PREP / 'protocol.json')
    assert review['all_probes_unchanged'] and review['qualified_source_unchanged']
    assert preparation['compiler'] == subprocess.check_output(['rustc', '-vV'], text=True)
    assert preparation['inherited_rustflags'] == os.environ.get('RUSTFLAGS')
    cpu = min(os.sched_getaffinity(0))
    assert cpu == 0
    out = E / 'scalar-costs-v1'
    out.mkdir(exist_ok=False)
    bins = COST_WORK / 'binaries'
    bins.mkdir(exist_ok=False)
    schedule = [{'suite': suite, 'mode': mode, 'count': count, 'round': repetition, 'order': order}
        for suite in SUITES for mode, count in [('time', 4096), ('memory', 1000)]
        for repetition, order in enumerate(ORDERS, 1)]
    protocol = {'scope': __doc__, 'runner_sha256': sha(__file__),
        'assessor_sha256': sha(E / 'assess_scalar_costs_v1.py'), 'parser_sha256': sha(E / 'pi_memo_cost_rows_v1.py'),
        'source_review_sha256': sha(E / 'scalar-cost-source-review-v1.json'),
        'preparation_sha256': sha(COST_PREP / 'protocol.json'), 'schedule': schedule,
        'groups': GROUPS, 'orders': ORDERS, 'fixed_cost_processes': 108, 'preflight_processes': 18,
        'preflight_inputs': 42, 'pinned_cpu': cpu, 'environment': ENV, 'timeout_seconds': 1800,
        'compiler': preparation['compiler'], 'inherited_rustflags': os.environ.get('RUSTFLAGS'),
        'host': tuple(os.uname()), 'cpuinfo': Path('/proc/cpuinfo').read_text().split('\n\n')[0],
        'frequency_before_builds': frequency(cpu), 'qualified_scalar_tests_reused': 882,
        'all_builds_before_all_samples': True, 'old_binaries_reused_not_old_samples': True,
        'sample_extension_or_filtering': False, 'native_gates_run': False,
        'runtime_admitted': False, 'plan_complete': False}
    dump(out / 'protocol.json', protocol)
    commands, trials = [], []
    libraries, binaries = dict(preparation['reused_libraries']), dict(preparation['reused_binaries'])

    def guard(full=False):
        verify_costs(preparation, full=full)
        assert sha(E / 'scalar-cost-source-review-v1.json') == protocol['source_review_sha256']
        assert all(sha(row['path']) == row['sha256'] for row in binaries.values())
        assert all(sha(row['path']) == row['sha256'] for group in libraries.values() for row in group.values())

    def run(name, command, group):
        guard()
        variant = preparation['variants'][group]
        env = ENV | {'CARGO_TARGET_DIR': variant['target']}
        print(json.dumps({'start': name}), flush=True)
        before = frequency(cpu)
        started = time.monotonic()
        with (out / (name + '.stdout')).open('wb') as stdout, (out / (name + '.stderr')).open('wb') as stderr:
            code = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', '1800', *command],
                cwd=variant['source'], env=os.environ | env, stdout=stdout, stderr=stderr).returncode
        seconds = time.monotonic() - started
        after = frequency(cpu)
        raw, error = ((out / (name + '.' + stream)).read_text() for stream in ['stdout', 'stderr'])
        for stream in ['stdout', 'stderr']:
            path = out / (name + '.' + stream)
            path.with_suffix(path.suffix + '.gz').write_bytes(gzip.compress(path.read_bytes(), mtime=0))
            path.unlink()
        commands.append({'name': name, 'command': command, 'group': group, 'cwd': variant['source'],
            'environment': env, 'exit_code': code, 'process_wall_seconds': seconds,
            'cpu0_before': before, 'cpu0_after': after,
            'stdout_sha256': sha(out / (name + '.stdout.gz')), 'stderr_sha256': sha(out / (name + '.stderr.gz'))})
        dump(out / 'commands.json', commands)
        print(json.dumps({'finished': name, 'exit_code': code, 'seconds': seconds}), flush=True)
        assert code == 0, (name, code)
        guard()
        return raw, error

    raw, _ = run('build-candidate-production-libraries', ['cargo', 'build', '--locked', '--release',
        '-p', 'hyperreal', '-p', 'hypersolve', '-p', 'hypercurve', '-p', 'hyperdrc',
        '--all-features', '--lib', '--message-format=json'], 'candidate')
    events = [json.loads(line) for line in raw.splitlines() if line.startswith('{')]
    libraries['candidate'] = {}
    variant = preparation['variants']['candidate']
    for name in ['hyperreal', 'hyperlimit', 'hypercurve', 'hyperdrc', 'serde_json']:
        artifacts = [r for r in events if r.get('reason') == 'compiler-artifact' and r['target']['name'] == name
                     and any(p.endswith('.rlib') for p in r['filenames'])]
        assert len(artifacts) == 1, name
        artifact = artifacts[0]
        path = Path(next(p for p in artifact['filenames'] if p.endswith('.rlib')))
        assert path.is_relative_to(variant['target'])
        if name != 'serde_json':
            assert Path(artifact['manifest_path']) == Path(variant['source']) / 'crates' / name / 'Cargo.toml'
        libraries['candidate'][name] = {'path': str(path), 'sha256': sha(path), 'bytes': path.stat().st_size,
            'features': artifact['features'], 'manifest_path': artifact['manifest_path']}
    feature_groups = [{n: r['features'] for n, r in libraries[g].items()} for g in GROUPS]
    assert all(features == feature_groups[0] for features in feature_groups)
    qualified_library = json.loads((E / 'qualification-v2/library.json').read_text())
    assert libraries['candidate']['hyperreal']['features'] == qualified_library['features']
    dump(out / 'libraries.json', libraries)
    for suite in SUITES:
        for mode in ['time', 'memory']:
            key = 'candidate-' + suite + '-' + mode
            binary = bins / key
            command = ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(ROOT / preparation['probes'][suite]['path']),
                '-L', 'dependency=' + str(Path(variant['target']) / 'release/deps'),
                '--extern', 'hyperreal=' + libraries['candidate']['hyperreal']['path']]
            if mode == 'memory':
                command += ['--cfg', 'feature="count-alloc"']
            run('link-' + key, command + ['-o', str(binary)], 'candidate')
            binaries[key] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
            dump(out / 'binaries.json', binaries)
    assert len(binaries) == 18 and len(commands) == 7
    identities = {}

    def sample(suite, mode, count, repetition, position, group, preflight):
        key = group + '-' + suite + '-' + mode
        name = ('preflight-' if preflight else 'sample-') + f'{suite}-{mode}-{repetition}-{group}'
        command = ['taskset', '-c', str(cpu), '/usr/bin/time', '-v', binaries[key]['path'], str(count)]
        raw, error = run(name, command, group)
        rows, identity = parse(suite, raw, count)
        assert identity == identities.setdefault((suite, count), identity)
        assert all(r['allocation_accounting'] == (mode == 'memory') for r in rows if 'name' in r)
        rss = re.search(r'Maximum resident set size \(kbytes\): (\d+)', error)
        assert rss
        trials.append({'command_name': name, 'suite': suite, 'mode': mode, 'count': count,
            'round': repetition, 'position': position, 'group': group, 'preflight': preflight,
            'rows': rows, 'identity_matches_reference': True, 'max_rss_kib': int(rss[1])})
        compressed(out / 'trials.json.gz', trials)

    for suite in SUITES:
        for mode in ['time', 'memory']:
            for position, group in enumerate(GROUPS, 1):
                sample(suite, mode, 42, 0, position, group, True)
    for row in schedule:
        for position, group in enumerate(row['order'], 1):
            sample(row['suite'], row['mode'], row['count'], row['round'], position, group, False)
    assert len(commands) == 133 and len(trials) == 126
    guard(full=True)
    result = {'complete': True, 'commands': len(commands), 'fixed_cost_processes': 108,
        'preflight_processes': 18, 'qualified_scalar_tests_reused': 882,
        'fresh_scalar_library_byte_identical_to_qualification': libraries['candidate']['hyperreal']['sha256'] == qualified_library['sha256'],
        'same_qualified_scalar_source_and_features': True, 'all_builds_precede_samples': True,
        'all_original_witnesses_and_representations_match': True,
        'all_sources_and_prior_artifacts_unchanged': True,
        'artifact_sha256': {p.name: sha(p) for p in sorted(out.iterdir()) if p.is_file()},
        'broader_library_costs_pending': True, 'native_gates_run': False,
        'runtime_admitted': False, 'plan_complete': False}
    dump(out / 'result.json', result)
    print(json.dumps({k: v for k, v in result.items() if k != 'artifact_sha256'}), flush=True)


if __name__ == '__main__':
    main()
