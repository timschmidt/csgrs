"""Fixed six-round library costs for main, full-cache and borrowed-integer builds.

The two original groups are reused unchanged. All 198 cost observations are new.
Public timing probes use System; memory probes count requested allocations.
The private metadata harness keeps its original TLS wrapper in both modes.
"""
import gzip
import json
import os
from pathlib import Path
import re
import subprocess
import time

from cost_common_v1 import E, ROOT, PARENT, COST_PREP, COST_WORK, ENV, ORDERS, sha, dump, compressed, verify_costs
from cost_rows_v1 import SPECS, parse

GROUPS = ['main', 'full-cache', 'candidate']


def frequency_snapshot(cpu):
    path = Path('/sys/devices/system/cpu') / ('cpu' + str(cpu)) / 'cpufreq'
    return {name: (path / name).read_text().strip() if (path / name).exists() else None
            for name in ['scaling_driver', 'scaling_governor', 'scaling_cur_freq',
                         'scaling_min_freq', 'scaling_max_freq']}


def main():
    preparation = json.loads((COST_PREP / 'protocol.json').read_text())
    verify_costs(preparation, full=True)
    review = json.loads((E / 'cost-source-review-v1.json').read_text())
    assert review['all_new_cost_source_changes_reviewed']
    assert review['preparation_sha256'] == sha(COST_PREP / 'protocol.json')
    assert preparation['compiler'] == subprocess.check_output(['rustc', '-vV'], text=True)
    assert preparation['inherited_rustflags'] == os.environ.get('RUSTFLAGS')
    out = E / 'library-costs-v1'
    out.mkdir(exist_ok=False)
    bins = COST_WORK / 'binaries'
    bins.mkdir(exist_ok=False)
    probes = PARENT / 'probes-v1'
    cpu = min(os.sched_getaffinity(0))
    assert cpu == 0
    schedule = [{'suite': suite, 'mode': mode, 'count': count, 'round': round_index, 'order': order}
                for suite in ['rational', 'quadratic', 'integer', 'arc', 'metadata', 'broad']
                for mode, count in ([('time', None)] if suite == 'broad' else
                                    [('time', 8192 if suite == 'arc' else 1024 if suite == 'metadata' else 4096),
                                     ('memory', 128 if suite == 'metadata' else 1000)])
                for round_index, order in enumerate(ORDERS, 1)]
    protocol = {'scope': __doc__, 'runner_sha256': sha(__file__),
                'assessor_sha256': sha(E / 'assess_costs_v1.py'), 'parser_sha256': sha(E / 'cost_rows_v1.py'),
                'source_review_sha256': sha(E / 'cost-source-review-v1.json'),
                'preparation_sha256': sha(COST_PREP / 'protocol.json'), 'schedule': schedule,
                'groups': GROUPS, 'orders': ORDERS, 'fixed_cost_processes': 198,
                'preflight_processes': 30, 'abort_control_processes': 9,
                'pinned_cpu': cpu, 'preflight_inputs': 42, 'metadata_preflight_inputs': 2,
                'abort_orders': ORDERS[:3], 'environment': ENV, 'timeout_seconds': 1800,
                'compiler': preparation['compiler'], 'inherited_rustflags': os.environ.get('RUSTFLAGS'),
                'host': tuple(os.uname()), 'cpuinfo': Path('/proc/cpuinfo').read_text().split('\n\n')[0],
                'frequency_snapshot_before_builds': frequency_snapshot(cpu),
                'qualified_scalar_tests_reused': 876, 'no_qualification_artifacts_overwritten': True,
                'all_builds_finish_before_measurements': True, 'sample_extension_or_filtering': False,
                'runtime_admitted': False, 'plan_complete': False}
    dump(out / 'protocol.json', protocol)
    commands, trials, focused = [], [], []
    libraries = dict(preparation['reused_libraries'])
    binaries = dict(preparation['reused_binaries'])

    def guarded(full=False):
        verify_costs(preparation, full=full)
        assert sha(__file__) == protocol['runner_sha256']
        assert sha(E / 'cost-source-review-v1.json') == protocol['source_review_sha256']
        assert all(sha(row['path']) == row['sha256'] for row in binaries.values())
        for rows in libraries.values():
            assert all(sha(row['path']) == row['sha256'] for row in rows.values())

    def run(name, command, label, extra=None):
        guarded()
        variant = preparation['variants'][label]
        env = os.environ | ENV | {'CARGO_TARGET_DIR': variant['target']} | (extra or {})
        print(json.dumps({'start': name}), flush=True)
        before = frequency_snapshot(cpu)
        started = time.monotonic()
        with (out / (name + '.stdout')).open('wb') as stdout, (out / (name + '.stderr')).open('wb') as stderr:
            code = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', '1800', *command],
                                  cwd=variant['source'], env=env, stdout=stdout, stderr=stderr).returncode
        seconds = time.monotonic() - started
        after = frequency_snapshot(cpu)
        raw = (out / (name + '.stdout')).read_text()
        for stream in ['stdout', 'stderr']:
            path = out / (name + '.' + stream)
            path.with_suffix(path.suffix + '.gz').write_bytes(gzip.compress(path.read_bytes(), mtime=0))
            path.unlink()
        row = {'name': name, 'command': command, 'source_label': label, 'cwd': variant['source'],
               'target': variant['target'], 'extra_environment': extra or {}, 'exit_code': code,
               'process_wall_seconds': seconds, 'cpu0_snapshot_before': before, 'cpu0_snapshot_after': after,
               'stdout_sha256': sha(out / (name + '.stdout.gz')), 'stderr_sha256': sha(out / (name + '.stderr.gz'))}
        commands.append(row)
        dump(out / 'commands.json', commands)
        print(json.dumps({'finished': name, 'exit_code': code, 'seconds': seconds}), flush=True)
        assert code == 0, (name, code)
        guarded()
        return raw

    directory = preparation['variants']['candidate']
    raw = run('build-libraries-candidate',
              ['cargo', 'build', '--locked', '--release', '-p', 'hyperreal', '-p', 'hypersolve',
               '-p', 'hypercurve', '-p', 'hyperdrc', '--all-features', '--lib', '--message-format=json'], 'candidate')
    events = [json.loads(line) for line in raw.splitlines() if line.startswith('{')]
    libraries['candidate'] = {}
    for name in ['hyperreal', 'hyperlimit', 'hypercurve', 'hyperdrc', 'serde_json']:
        artifacts = [r for r in events if r.get('reason') == 'compiler-artifact' and r['target']['name'] == name
                     and any(p.endswith('.rlib') for p in r['filenames'])]
        assert len(artifacts) == 1, name
        row = artifacts[0]
        library = Path(next(p for p in row['filenames'] if p.endswith('.rlib')))
        assert library.is_relative_to(directory['target'])
        if name != 'serde_json':
            assert Path(row['manifest_path']) == Path(directory['source']) / 'crates' / name / 'Cargo.toml'
        libraries['candidate'][name] = {'path': str(library), 'sha256': sha(library), 'bytes': library.stat().st_size,
                                        'features': row['features'], 'manifest_path': row['manifest_path']}
    for suite, filename, crates, modes in [
        *[(s, s + '_probe.rs', ['hyperreal'], ['time', 'memory']) for s in ['rational', 'quadratic', 'integer']],
        ('arc', 'arc_cost.rs', ['hypercurve', 'hyperreal'], ['time', 'memory']),
        ('broad', 'operation_probe.rs', ['hyperdrc', 'hyperlimit', 'hypercurve', 'serde_json'], ['time']),
        ('abort', 'final_abort.rs', ['hypercurve'], ['control']),
    ]:
        for mode in modes:
            name = 'candidate-' + suite + '-' + mode
            binary = bins / name
            command = ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(probes / filename),
                       '-L', 'dependency=' + str(Path(directory['target']) / 'release/deps')]
            for crate, library in ((crate, libraries['candidate'][crate]) for crate in crates):
                command += ['--extern', crate + '=' + library['path']]
            if mode == 'memory':
                command += ['--cfg', 'feature="count-alloc"']
            run('link-' + name, command + ['-o', str(binary)], 'candidate')
            binaries[name] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
    label = 'candidate-metadata'
    run('clippy-' + label, ['cargo', 'clippy', '--locked', '--release', '-p', 'hypercurve',
                          '--all-features', '--lib', '--tests', '--', '-D', 'warnings'], label)
    raw = run('build-' + label, ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve',
                                '--all-features', '--lib', '--no-run', '--message-format=json'], label)
    events = [json.loads(line) for line in raw.splitlines() if line.startswith('{')]
    artifacts = [r for r in events if r.get('reason') == 'compiler-artifact' and r.get('executable')
                 and r['target']['name'] == 'hypercurve' and r['target']['kind'] == ['lib'] and r['profile']['test']]
    assert len(artifacts) == 1
    artifact = artifacts[0]
    directory = preparation['variants'][label]
    binary = Path(artifact['executable'])
    assert binary.is_relative_to(directory['target'])
    assert Path(artifact['manifest_path']) == Path(directory['source']) / 'crates/hypercurve/Cargo.toml'
    binaries[label] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size,
                       'features': artifact['features'], 'manifest_path': artifact['manifest_path']}
    feature_groups = [{n: r['features'] for n, r in libraries[group].items()} for group in GROUPS]
    assert all(features == feature_groups[0] for features in feature_groups)
    assert all(binaries[group + '-metadata']['features'] == binaries['main-metadata']['features'] for group in GROUPS)
    qualified_library = json.loads((E / 'qualification-v1/library.json').read_text())
    assert libraries['candidate']['hyperreal']['features'] == qualified_library['features']
    dump(out / 'scalar-build-identity.json', {'qualified_library': qualified_library,
        'cost_library': libraries['candidate']['hyperreal'], 'features_identical': True,
        'bytes_identical': qualified_library['sha256'] == libraries['candidate']['hyperreal']['sha256'],
        'qualified_source_reused_unchanged': True, 'scalar_tests_reused': 876})
    suffixes = [k.removeprefix('main-') for k in binaries if k.startswith('main-')]
    assert len(suffixes) == 11 and len(binaries) == 33
    for suffix in suffixes:
        assert len({binaries[group + '-' + suffix]['sha256'] for group in GROUPS}) == 3, suffix
    dump(out / 'libraries.json', libraries)
    dump(out / 'binaries.json', binaries)
    guarded(full=True)
    for group in GROUPS:
        raw = run('metadata-regressions-' + group,
                  [binaries[group + '-metadata']['path'], 'bezier_region::combined_admission_metadata::', '--test-threads=1'], group + '-metadata')
        assert re.search(r'test result: ok\. 3 passed; 0 failed; 1 ignored;', raw)
        focused.append({'variant': group, 'suite': 'metadata', 'passed': 3, 'failed': 0, 'ignored': 1})
    raw = run('candidate-endpoint-regressions', [binaries['candidate-metadata']['path'], 'endpoint_proof_tests',
                                               '--test-threads=1'], 'candidate-metadata')
    assert re.search(r'test result: ok\. 5 passed; 0 failed; 0 ignored;', raw)
    focused.append({'variant': 'candidate', 'suite': 'endpoint', 'passed': 5, 'failed': 0, 'ignored': 0})
    dump(out / 'focused-tests.json', focused)
    identities = {}

    def sample(suite, mode, count, round_index, position, group, preflight=False):
        label = group + '-metadata' if suite == 'metadata' else group
        key = group + '-metadata' if suite == 'metadata' else group + '-' + suite + '-' + mode
        name = ('preflight-' if preflight else 'sample-') + f'{suite}-{mode}-{round_index}-{group}'
        extra = {}
        command = ['taskset', '-c', str(cpu), '/usr/bin/time', '-v', binaries[key]['path']]
        if suite == 'metadata':
            command += ['bezier_region::combined_admission_metadata::cost::probe', '--exact', '--ignored', '--test-threads=1', '--nocapture']
            extra = {'REGION_PROBE_MODE': mode, 'REGION_PROBE_ITERATIONS': str(count)}
        elif count is not None:
            command.append(str(count))
        raw = run(name, command, label, extra)
        rows, identity = parse(suite, raw, count)
        reference = identities.setdefault((suite, mode, count, preflight), identity)
        same = identity == reference
        stderr = gzip.decompress((out / (name + '.stderr.gz')).read_bytes()).decode()
        rss = re.search(r'Maximum resident set size \(kbytes\): (\d+)', stderr)
        assert rss
        trials.append({'command_name': name, 'suite': suite, 'mode': mode, 'count': count,
                       'round': round_index, 'position': position, 'variant': group, 'preflight': preflight,
                       'rows': rows, 'max_rss_kib': int(rss[1]), 'identity_matches_reference': same})
        compressed(out / 'trials.json.gz', trials)
        assert same, name

    for suite in [*SPECS, 'metadata']:
        for mode in ['time', 'memory']:
            for position, group in enumerate(GROUPS, 1):
                sample(suite, mode, 2 if suite == 'metadata' else 42, 0, position, group, True)
    for round_index, order in enumerate(ORDERS[:3], 1):
        for position, group in enumerate(order, 1):
            sample('abort', 'control', None, round_index, position, group)
    for row in schedule:
        for position, group in enumerate(row['order'], 1):
            sample(row['suite'], row['mode'], row['count'], row['round'], position, group)
    assert len(trials) == 237 and len(commands) == 254
    guarded(full=True)
    result = {'complete': True, 'commands': len(commands), 'fixed_cost_processes': 198,
              'abort_control_processes': 9, 'preflight_processes': 30,
              'all_identities_match': True, 'all_builds_precede_costs': True,
              'sample_extension_or_filtering': False, 'sources_main_and_old_inputs_unchanged': True,
              'focused_tests': focused, 'prior_groups_reused_not_samples': True,
              'artifact_sha256': {p.name: sha(p) for p in sorted(out.iterdir()) if p.is_file()},
              'runtime_admitted': False, 'plan_complete': False}
    dump(out / 'result.json', result)
    print(json.dumps({k: v for k, v in result.items() if k != 'artifact_sha256'}), flush=True)


if __name__ == '__main__':
    main()
