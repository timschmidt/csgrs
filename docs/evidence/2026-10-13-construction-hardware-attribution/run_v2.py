"""Fixed phase hardware observations; no library rebuilds or native gate reruns."""
import gzip
import json
import os
from pathlib import Path
import platform
import shlex
import subprocess

from common_v2 import E, ROOT, OLD, COVERAGE, WORK, ENV, sha, dump, verify_prior, verify

ORDERS = [
    ['main', 'full-cache', 'no-persistent'],
    ['full-cache', 'no-persistent', 'main'],
    ['no-persistent', 'main', 'full-cache'],
    ['main', 'no-persistent', 'full-cache'],
    ['no-persistent', 'full-cache', 'main'],
    ['full-cache', 'main', 'no-persistent'],
]


def records(raw):
    return [json.loads(line) for line in raw.splitlines() if line.startswith(b'{')]


def frequency_snapshot():
    prefix = Path('/sys/devices/system/cpu/cpu0/cpufreq')
    return {name: (prefix / name).read_text().strip() if (prefix / name).exists() else None
            for name in ['scaling_driver', 'scaling_governor', 'scaling_cur_freq',
                         'scaling_min_freq', 'scaling_max_freq']}


def main():
    assert platform.system() == 'Linux' and platform.machine() == 'x86_64'
    verify_prior(full=True)
    old_protocol = json.loads((OLD / 'construction-trace-v1/protocol.json').read_text())
    assert old_protocol['compiler'] == subprocess.check_output(['rustc', '-vV'], text=True)
    assert old_protocol['inherited_rustflags'] == os.environ.get('RUSTFLAGS')
    out = E / 'run-v2'
    out.mkdir(exist_ok=False)
    WORK.mkdir(exist_ok=False)
    old_probe = (OLD / 'construction-trace-v1/probe.rs').read_text()
    suffix = old_probe[old_probe.index('fn inputs(index: usize)'):]
    probe = out / 'probe.rs'
    probe.write_text((E / 'wrapper_v1.rs').read_text() + '\n' + suffix)
    assert probe.read_text()[probe.read_text().index('fn inputs(index: usize)'):] == suffix
    files = ['common_v1.py', 'run_v1.py', 'assess_v1.py', 'counters_v1.c', 'counters_v1.rs',
             'controls_v1.rs', 'wrapper_v1.rs', 'scope-v1.json', 'source-review-v1.json',
             'failure-v1.json', 'common_v2.py', 'run_v2.py', 'assess_v2.py',
             'scope-v2.json', 'source-review-v2.json']
    prior_files = [COVERAGE / 'decision-v1.json', COVERAGE / 'assessment-v1.json',
                   COVERAGE / 'performance-counter-capabilities-v1.json', COVERAGE / 'next-diagnostic-v1.json',
                   OLD / 'construction-trace-v1/protocol.json', OLD / 'construction-trace-v1/result.json',
                   OLD / 'construction-trace-v1/probe.rs', OLD / 'construction-trace-v1/trace-1-full-cache.stdout.gz']
    protocol = {'scope': __doc__, 'inputs_sha256': {str(p): sha(p) for p in [*[E / f for f in files], probe, *prior_files]},
                'system_reference_sha256': {p: sha(p) for p in ['/usr/include/linux/perf_event.h',
                    '/usr/include/asm/unistd_64.h', '/usr/include/asm-generic/ioctl.h',
                    '/usr/share/man/man2/perf_event_open.2.gz']},
                'compiler': old_protocol['compiler'], 'inherited_rustflags': os.environ.get('RUSTFLAGS'),
                'cc_version': subprocess.check_output(['cc', '--version'], text=True),
                'clippy_version': subprocess.check_output(['clippy-driver', '--version'], text=True),
                'kernel': subprocess.check_output(['uname', '-srmo'], text=True),
                'variants': old_protocol['variants'], 'libraries': old_protocol['libraries'],
                'orders': ORDERS, 'inputs_each_process': 4096, 'processes': 18,
                'empty_windows_each_process': 32, 'work_windows_each_process': 32,
                'environment': ENV, 'frequency_snapshot_before_build': frequency_snapshot(),
                'all_library_sources_and_libraries_reused_unchanged': True,
                'fixture_expected_and_witness_suffix_byte_identical': True,
                'diagnostic_only': True, 'runtime_admitted': False, 'plan_complete': False}
    dump(out / 'protocol.json', protocol)
    commands, binaries, samples, headers = [], {}, [], {}
    obj = WORK / 'counters.o'
    object_hash = None

    def guard(full=False):
        verify(protocol, full=full)
        assert all(sha(r['path']) == r['sha256'] for r in binaries.values())
        assert all(sha(p) == digest for p, digest in headers.items())
        if object_hash is not None:
            assert sha(obj) == object_hash

    def run(name, command, cwd=ROOT):
        guard()
        print(json.dumps({'start': name}), flush=True)
        before = frequency_snapshot()
        completed = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', '1800', *command],
                                   cwd=cwd, env=os.environ | ENV, capture_output=True)
        after = frequency_snapshot()
        row = {'name': name, 'command': command, 'cwd': str(cwd), 'exit_code': completed.returncode,
               'cpu0_snapshot_before': before, 'cpu0_snapshot_after': after}
        for stream in ['stdout', 'stderr']:
            path = out / (name + '.' + stream + '.gz')
            path.write_bytes(gzip.compress(getattr(completed, stream), mtime=0))
            row[stream + '_sha256'] = sha(path)
        commands.append(row)
        dump(out / 'commands.json', commands)
        print(json.dumps({'finished': name, 'exit_code': completed.returncode}), flush=True)
        assert completed.returncode == 0, name
        guard()
        return completed.stdout

    run('format', ['rustfmt', '--edition', '2024', '--config', 'skip_children=true', '--check',
                   *[str(E / f) for f in ['counters_v1.rs', 'controls_v1.rs', 'wrapper_v1.rs']]])
    run('compile-c', ['cc', '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror', '-MD', '-MF',
                      str(out / 'counters.d'), '-c', str(E / 'counters_v1.c'), '-o', str(obj)])
    object_hash = sha(obj)
    dependencies = shlex.split((out / 'counters.d').read_text().split(':', 1)[1].replace('\\\n', ' '))
    headers = {str(Path(p).resolve()): sha(p) for p in dependencies}
    dump(out / 'c-dependencies.json', headers)
    dump(out / 'c-object.json', {'path': str(obj), 'sha256': object_hash, 'bytes': obj.stat().st_size})
    base = protocol['variants']['main']
    lib = protocol['libraries']['main']
    flags = ['--edition=2024', '-O', '-D', 'warnings', '-L', 'dependency=' + str(Path(base['target']) / 'release/deps'),
             '--extern', 'serde_json=' + lib['serde_json']['path']]
    run('clippy-controls', ['clippy-driver', *flags, '--cfg', 'feature="pmu-controls"', '--emit=metadata',
                           str(E / 'controls_v1.rs'), '-o', str(WORK / 'controls.rmeta')])
    run('clippy-probe', ['clippy-driver', *flags, '--extern', 'hyperreal=' + lib['hyperreal']['path'],
                        '--emit=metadata', str(probe), '-o', str(WORK / 'probe.rmeta')])
    control_binary = WORK / 'controls'
    run('build-controls', ['rustc', *flags, '--cfg', 'feature="pmu-controls"', str(E / 'controls_v1.rs'),
                           '-C', 'link-arg=' + str(obj), '-o', str(control_binary)])
    binaries['controls'] = {'path': str(control_binary), 'sha256': sha(control_binary), 'bytes': control_binary.stat().st_size}
    for label, variant in protocol['variants'].items():
        binary = WORK / label
        command = ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(probe),
                   '-L', 'dependency=' + str(Path(variant['target']) / 'release/deps')]
        for name, row in protocol['libraries'][label].items():
            command += ['--extern', name + '=' + row['path']]
        run('link-' + label, command + ['-C', 'link-arg=' + str(obj), '-o', str(binary)])
        binaries[label] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
        dump(out / 'binaries.json', binaries)
    assert len({binaries[label]['sha256'] for label in protocol['variants']}) == 3
    control_raw = run('counter-controls', ['taskset', '-c', '0', str(control_binary)])
    controls = records(control_raw)
    assert len(controls) == 1 and controls[0]['passed'] and controls[0]['both_descriptors_closed']
    assert controls[0]['normal_and_unwind_disable_verified']
    dump(out / 'controls.json', controls[0])
    reference = records(gzip.decompress((OLD / 'construction-trace-v1/trace-1-full-cache.stdout.gz').read_bytes()))
    identity = [r for r in reference if 'witness' in r or 'fixture' in r]
    phases = [r['trace_phase'] for r in reference if 'trace_phase' in r]
    assert len(identity) == len(phases) == 32
    for round_index, order in enumerate(ORDERS, 1):
        for position, label in enumerate(order, 1):
            raw = run('sample-' + str(round_index) + '-' + label,
                      ['taskset', '-c', '0', binaries[label]['path'], '4096'],
                      Path(protocol['variants'][label]['source']))
            rows = records(raw)
            assert [r for r in rows if 'witness' in r or 'fixture' in r] == identity
            hardware = [r for r in rows if 'hardware_phase' in r]
            assert [r['hardware_phase'] for r in hardware] == phases
            assert all(r['iterations'] == 4096 for r in hardware)
            samples.append({'round': round_index, 'position': position, 'variant': label, 'rows': hardware})
            dump(out / 'samples.json', samples)
    guard(full=True)
    windows = [r[key] for sample in samples for r in sample['rows'] for key in ['empty_window', 'work_window']]
    result = {'complete': True, 'commands': len(commands), 'fixed_processes': len(samples),
              'work_windows': 576, 'empty_windows': 576, 'usable_windows': sum(w['usable'] for w in windows),
              'unusable_windows': sum(not w['usable'] for w in windows),
              'all_witnesses_and_representation_inventories_identical': True,
              'all_builds_and_controls_precede_samples': True, 'sample_replacement_or_extension': False,
              'all_sources_old_artifacts_libraries_and_headers_unchanged': True,
              'artifact_sha256': {p.name: sha(p) for p in sorted(out.iterdir()) if p.is_file()},
              'runtime_candidate': False, 'runtime_admitted': False, 'plan_complete': False}
    dump(out / 'result.json', result)
    print(json.dumps({k: v for k, v in result.items() if k != 'artifact_sha256'}), flush=True)


if __name__ == '__main__':
    main()
