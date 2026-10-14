"""Fixed current-main versus combined costs at independent library boundaries.

All builds finish before controls or costs. Public time probes use System
without counting; memory probes count requested Rust allocations over System.
The private metadata harness has the same TLS wrapper in both modes. No CLI
allocator is imposed by any linked reusable library. Old samples stay closed.
"""
import gzip
import json
import os
from pathlib import Path
import re
import subprocess
import time

from common_v1 import E, ROOT, WORK, PREP, ENV, sha, dump, compressed, verify, order

SPECS = {'rational': (48, 24, 18, 6), 'quadratic': (64, 32, 24, 8),
         'integer': (24, 12, 9, 3), 'arc': (96, 48, 36, 12)}


def parse(suite, raw, count):
    if suite == 'broad':
        rows = []
        for line in raw.splitlines():
            if line.startswith('operation_'):
                name, iterations, nanos, witness = line.split('\t')
                rows.append({'name': name, 'iterations': int(iterations), 'nanos': int(nanos), 'witness': witness})
        assert len(rows) == 231 and len({r['name'] for r in rows}) == 231
        return rows, [{k: v for k, v in r.items() if k != 'nanos'} for r in rows]
    if suite == 'metadata':
        rows = []
        for line in raw.splitlines():
            if 'REGION_COST ' in line:
                case, phase, iterations, nanos, calls, allocated, live, peak = line.split('REGION_COST ', 1)[1].split()
                rows.append({'name': case + '/' + phase, 'iterations': int(iterations), 'nanos': int(nanos),
                             'allocation_calls': int(calls), 'allocated_bytes': int(allocated),
                             'retained_bytes_delta': int(live), 'peak_bytes_delta': int(peak)})
        assert len(rows) == 18 and all(r['iterations'] == count for r in rows)
        return rows, [{k: r[k] for k in ['name', 'iterations']} for r in rows]
    rows = [json.loads(line) for line in raw.splitlines()]
    if suite == 'abort':
        assert len(rows) == 210
        return rows, rows
    total, named, witnesses, fixtures = SPECS[suite]
    assert len(rows) == total and sum('name' in r for r in rows) == named
    assert sum('witness' in r for r in rows) == witnesses and sum('fixture' in r for r in rows) == fixtures
    assert all(r['iterations'] == count for r in rows if 'name' in r)
    return rows, [r for r in rows if 'witness' in r or 'fixture' in r]


def main():
    preparation = json.loads((PREP / 'protocol.json').read_text())
    verify(preparation, full=True)
    out = E / 'library-costs-v1'
    out.mkdir(exist_ok=False)
    bins = WORK / 'library-binaries-v1'
    bins.mkdir(exist_ok=False)
    probes = E / 'probes-v1'
    cpu = min(os.sched_getaffinity(0))
    schedule = [{'suite': suite, 'mode': mode, 'count': count, 'pair': pair, 'order': order(pair)}
                for suite in ['rational', 'quadratic', 'integer', 'arc', 'metadata', 'broad']
                for mode, count in ([('time', None)] if suite == 'broad' else
                                    [('time', 8192 if suite == 'arc' else 1024 if suite == 'metadata' else 4096),
                                     ('memory', 128 if suite == 'metadata' else 1000)])
                for pair in range(1, 6)]
    protocol = {'scope': __doc__, 'runner_sha256': sha(__file__),
                'assessor_sha256': sha(E / 'assess_libraries_v1.py'),
                'preparation_sha256': sha(PREP / 'protocol.json'), 'schedule': schedule,
                'pinned_cpu': cpu, 'preflight_inputs': 42, 'metadata_preflight_inputs': 2,
                'final_abort_pairs': 3, 'environment': ENV, 'timeout_seconds': 1800,
                'compiler': subprocess.check_output(['rustc', '-vV'], text=True),
                'inherited_rustflags': os.environ.get('RUSTFLAGS'), 'host': tuple(os.uname()),
                'cpuinfo': Path('/proc/cpuinfo').read_text().split('\n\n')[0],
                'all_builds_finish_before_measurements': True,
                'sample_extension_or_filtering': False, 'runtime_admitted': False}
    dump(out / 'protocol.json', protocol)
    commands, libraries, binaries, trials = [], {}, {}, []

    def guarded():
        verify(preparation)
        assert sha(__file__) == protocol['runner_sha256']
        assert all(sha(row['path']) == row['sha256'] for row in binaries.values())
        for rows in libraries.values():
            assert all(sha(row['path']) == row['sha256'] for row in rows.values())

    def run(name, command, label, extra=None):
        guarded()
        variant = preparation['variants'][label]
        env = os.environ | ENV | {'CARGO_TARGET_DIR': variant['target']} | (extra or {})
        print(json.dumps({'start': name}), flush=True)
        started = time.monotonic()
        with (out / (name + '.stdout')).open('wb') as stdout, (out / (name + '.stderr')).open('wb') as stderr:
            code = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', '1800', *command],
                                  cwd=variant['source'], env=env, stdout=stdout, stderr=stderr).returncode
        seconds = time.monotonic() - started
        raw = (out / (name + '.stdout')).read_text()
        for stream in ['stdout', 'stderr']:
            path = out / (name + '.' + stream)
            path.with_suffix(path.suffix + '.gz').write_bytes(gzip.compress(path.read_bytes(), mtime=0))
            path.unlink()
        row = {'name': name, 'command': command, 'source_label': label, 'cwd': variant['source'],
               'target': variant['target'], 'extra_environment': extra or {}, 'exit_code': code,
               'process_wall_seconds': seconds,
               'stdout_sha256': sha(out / (name + '.stdout.gz')), 'stderr_sha256': sha(out / (name + '.stderr.gz'))}
        commands.append(row)
        dump(out / 'commands.json', commands)
        print(json.dumps({'finished': name, 'exit_code': code, 'seconds': seconds}), flush=True)
        guarded()
        assert code == 0, (name, code)
        return raw

    for variant in ['baseline', 'candidate']:
        directory = preparation['variants'][variant]
        raw = run('build-libraries-' + variant,
                  ['cargo', 'build', '--locked', '--release', '-p', 'hyperreal', '-p', 'hypersolve',
                   '-p', 'hypercurve', '-p', 'hyperdrc', '--all-features', '--lib', '--message-format=json'], variant)
        events = [json.loads(line) for line in raw.splitlines() if line.startswith('{')]
        libraries[variant] = {}
        for name in ['hyperreal', 'hyperlimit', 'hypercurve', 'hyperdrc', 'serde_json']:
            artifacts = [r for r in events if r.get('reason') == 'compiler-artifact' and r['target']['name'] == name
                         and any(p.endswith('.rlib') for p in r['filenames'])]
            assert len(artifacts) == 1, (variant, name)
            row = artifacts[0]
            library = Path(next(p for p in row['filenames'] if p.endswith('.rlib')))
            assert library.is_relative_to(directory['target'])
            if name != 'serde_json':
                assert Path(row['manifest_path']) == Path(directory['source']) / 'crates' / name / 'Cargo.toml'
            libraries[variant][name] = {'path': str(library), 'sha256': sha(library), 'bytes': library.stat().st_size,
                                        'features': row['features'], 'manifest_path': row['manifest_path']}
        for suite, filename, crates, modes in [
            *[(s, s + '_probe.rs', ['hyperreal'], ['time', 'memory']) for s in ['rational', 'quadratic', 'integer']],
            ('arc', 'arc_cost.rs', ['hypercurve', 'hyperreal'], ['time', 'memory']),
            ('broad', 'operation_probe.rs', ['hyperdrc', 'hyperlimit', 'hypercurve', 'serde_json'], ['time']),
            ('abort', 'final_abort.rs', ['hypercurve'], ['control']),
        ]:
            for mode in modes:
                label = variant + '-' + suite + '-' + mode
                binary = bins / label
                command = ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(probes / filename),
                           '-L', 'dependency=' + str(Path(directory['target']) / 'release/deps')]
                for name in crates:
                    command += ['--extern', name + '=' + libraries[variant][name]['path']]
                if mode == 'memory':
                    command += ['--cfg', 'feature="count-alloc"']
                run('link-' + label, command + ['-o', str(binary)], variant)
                binaries[label] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
        label = variant + '-metadata'
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
    assert {n: r['features'] for n, r in libraries['baseline'].items()} == {n: r['features'] for n, r in libraries['candidate'].items()}
    assert binaries['baseline-metadata']['features'] == binaries['candidate-metadata']['features']
    for suffix in [k.removeprefix('baseline-') for k in binaries if k.startswith('baseline-')]:
        assert binaries['baseline-' + suffix]['sha256'] != binaries['candidate-' + suffix]['sha256'], suffix
    dump(out / 'libraries.json', libraries)
    dump(out / 'binaries.json', binaries)
    verify(preparation, full=True)
    for variant in ['baseline', 'candidate']:
        raw = run('metadata-regressions-' + variant,
                  [binaries[variant + '-metadata']['path'], 'bezier_region::combined_admission_metadata::', '--test-threads=1'], variant + '-metadata')
        assert re.search(r'test result: ok\. 3 passed; 0 failed; 1 ignored;', raw)

    identities = {}

    def sample(suite, mode, count, pair, variant, preflight=False):
        label = variant + '-metadata' if suite == 'metadata' else variant
        key = variant + '-metadata' if suite == 'metadata' else variant + '-' + suite + '-' + mode
        name = ('preflight-' if preflight else 'sample-') + f'{suite}-{mode}-{pair}-{variant}'
        extra = {}
        command = ['taskset', '-c', str(cpu), '/usr/bin/time', '-v', binaries[key]['path']]
        if suite == 'metadata':
            command += ['bezier_region::combined_admission_metadata::cost::probe', '--exact', '--ignored', '--test-threads=1', '--nocapture']
            extra = {'REGION_PROBE_MODE': mode, 'REGION_PROBE_ITERATIONS': str(count)}
        elif count is not None:
            command.append(str(count))
        raw = run(name, command, label, extra)
        rows, identity = parse(suite, raw, count)
        identity_key = (suite, mode, count, preflight)
        reference = identities.setdefault(identity_key, identity)
        same = identity == reference
        stderr = gzip.decompress((out / (name + '.stderr.gz')).read_bytes()).decode()
        rss = re.search(r'Maximum resident set size \(kbytes\): (\d+)', stderr)
        assert rss
        trials.append({'suite': suite, 'mode': mode, 'count': count, 'pair': pair, 'variant': variant,
                       'preflight': preflight, 'rows': rows, 'max_rss_kib': int(rss[1]),
                       'identity_matches_reference': same})
        compressed(out / 'trials.json.gz', trials)
        assert same, name

    for suite in [*SPECS, 'metadata']:
        for mode in ['time', 'memory']:
            for variant in ['baseline', 'candidate']:
                sample(suite, mode, 2 if suite == 'metadata' else 42, 0, variant, True)
    for pair in range(1, 4):
        for variant in order(pair):
            sample('abort', 'control', None, pair, variant)
    for row in schedule:
        for variant in row['order']:
            sample(row['suite'], row['mode'], row['count'], row['pair'], variant)
    assert len(trials) == 136
    assert len([r for r in trials if not r['preflight']]) == 116
    verify(preparation, full=True)
    guarded()
    result = {'complete': True, 'fixed_cost_processes': 110, 'abort_control_processes': 6,
              'preflight_processes': 20, 'all_identities_match': True, 'all_builds_precede_costs': True,
              'sample_extension_or_filtering': False, 'sources_main_and_old_inputs_unchanged': True,
              'artifact_sha256': {p.name: sha(p) for p in sorted(out.iterdir()) if p.is_file()},
              'runtime_admitted': False, 'plan_complete': False}
    dump(out / 'result.json', result)
    print(json.dumps({k: v for k, v in result.items() if k != 'artifact_sha256'}), flush=True)


if __name__ == '__main__':
    main()
