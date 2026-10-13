"""Fixed paired scalar costs for owned Laurent coefficient merging.

All three public probes are unchanged from the integer-leaf trial. Each suite
uses five fixed alternating pairs per mode, independent exact witnesses and
source-specific build directories. This trial does not admit runtime changes.
"""
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import statistics
import subprocess
import sys
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
WORK = ROOT / 'target/laurent-coefficient-merge'
BASE_WORK = ROOT / 'target/small-integer-certificates'
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources

sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    validation = json.loads((E / 'validation-v1/result.json').read_text())
    assert validation['complete'] and all(p['exit_code'] == 0 for p in validation['phases'])
    preparation = json.loads((E / 'preparation.json').read_text())
    original = json.loads(gzip.decompress((E / 'baseline-source-hashes.json.gz').read_bytes()))
    delta = json.loads((E / 'validation-v1/protocol.json').read_text())['source_changes_sha256']
    maps = {'baseline': original, 'candidate': original | delta}
    dirs = {'baseline': BASE_WORK / 'source', 'candidate': WORK / 'source'}
    targets = {'baseline': BASE_WORK / 'target', 'candidate': WORK / 'target'}
    for variant, entries in maps.items():
        assert all(sha(dirs[variant] / p) == expected for p, expected in entries.items())
    out = E / 'cost-v1'
    out.mkdir(exist_ok=False)
    bins = WORK / 'cost-v1-binaries'
    bins.mkdir(exist_ok=False)
    probes = {
        'rational': ROOT / 'docs/evidence/2026-10-09-hyperdrc-native-linear-strokes/normal_form_ownership_probe.rs',
        'quadratic': ROOT / 'docs/evidence/2026-10-12-quadratic-certificate-consumer/quadratic_probe.rs',
        'integer': ROOT / 'docs/evidence/2026-10-12-small-integer-certificates/integer_probe.rs',
    }
    for suite, probe in probes.items():
        shutil.copyfile(probe, out / (suite + '_probe.rs'))
    cpu = min(os.sched_getaffinity(0))
    env = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
           'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never'}
    protocol = {'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
                'probes_sha256': {suite: sha(probe) for suite, probe in probes.items()},
                'predecessor_validation_sha256': sha(E / 'validation-v1/result.json'),
                'preparation_sha256': sha(E / 'preparation.json'), 'source_changes_sha256': delta,
                'sources': {key: str(value) for key, value in dirs.items()},
                'targets': {key: str(value) for key, value in targets.items()},
                'environment': env, 'inherited_rustflags': os.environ.get('RUSTFLAGS'),
                'compiler': subprocess.check_output(['rustc', '-vV'], text=True),
                'pinned_cpu': cpu, 'time_iterations': 4096, 'memory_iterations': 1000,
                'pairs_each_suite_and_mode': 5, 'preflight_iterations': 42,
                'all_builds_finish_before_measurements': True, 'runtime_admitted': False}
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    commands = []

    def verify():
        assert sources() == preparation['main_source_sha256']
        for variant, entries in maps.items():
            for name in delta:
                p = dirs[variant] / name
                if name in entries:
                    assert sha(p) == entries[name]
                else:
                    assert not p.exists()
        assert all(sha(probe) == protocol['probes_sha256'][suite] for suite, probe in probes.items())

    def run(name, command, cwd=ROOT, extra_env=None):
        verify()
        started = time.monotonic()
        with (out / (name + '.stdout')).open('w') as stdout, (out / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(command, cwd=cwd, env=os.environ | env | (extra_env or {}),
                                  stdout=stdout, stderr=stderr).returncode
        verify()
        record = {'name': name, 'command': command, 'cwd': str(cwd),
                  'extra_environment': extra_env or {}, 'exit_code': code,
                  'seconds': time.monotonic() - started,
                  'stdout_sha256': sha(out / (name + '.stdout')),
                  'stderr_sha256': sha(out / (name + '.stderr'))}
        commands.append(record)
        (out / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
        print(json.dumps({'name': name, 'exit_code': code}), flush=True)
        assert code == 0, name
        return (out / (name + '.stdout')).read_text()

    binaries = {}
    libraries = {}
    for variant in ['baseline', 'candidate']:
        log = run('build-' + variant,
                  ['cargo', 'build', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--lib', '--message-format=json'],
                  dirs[variant], {'CARGO_TARGET_DIR': str(targets[variant])})
        events = [json.loads(line) for line in log.splitlines() if line.startswith('{')]
        artifact = [event for event in events if event.get('reason') == 'compiler-artifact' and event['target']['name'] == 'hyperreal']
        assert len(artifact) == 1
        assert Path(artifact[0]['manifest_path']) == dirs[variant] / 'crates/hyperreal/Cargo.toml'
        library = Path(next(p for p in artifact[0]['filenames'] if p.endswith('.rlib')))
        assert library.is_relative_to(targets[variant])
        libraries[variant] = {'sha256': sha(library), 'path': str(library),
                              'fresh': artifact[0]['fresh'], 'features': artifact[0]['features'],
                              'package_id': artifact[0]['package_id'], 'manifest_path': artifact[0]['manifest_path']}
        if variant == 'baseline':
            prior = json.loads((ROOT / 'docs/evidence/2026-10-12-small-integer-certificates/cost-v1/binaries.json').read_text())
            assert libraries[variant]['sha256'] == prior['libraries']['candidate']['sha256']
        for suite in probes:
            for mode in ['time', 'memory']:
                label = variant + '-' + suite + '-' + mode
                binary = bins / label
                command = ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(out / (suite + '_probe.rs')),
                           '-L', 'dependency=' + str(targets[variant] / 'release/deps'),
                           '--extern', 'hyperreal=' + str(library), '-o', str(binary)]
                if mode == 'memory':
                    command += ['--cfg', 'feature="count-alloc"']
                run('link-' + label, command)
                binaries[label] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
    assert libraries['baseline']['features'] == libraries['candidate']['features']
    assert libraries['baseline']['sha256'] != libraries['candidate']['sha256']
    for suite in probes:
        for mode in ['time', 'memory']:
            assert binaries['baseline-' + suite + '-' + mode]['sha256'] != binaries['candidate-' + suite + '-' + mode]['sha256']
    (out / 'binaries.json').write_text(json.dumps({'libraries': libraries, 'binaries': binaries}, indent=2) + '\n')
    for label, binary in binaries.items():
        run('preflight-' + label, ['taskset', '-c', str(cpu), binary['path'], '42'])
    trials = []
    for suite in probes:
        for mode, count in [('time', 4096), ('memory', 1000)]:
            for pair in range(1, 6):
                order = ['baseline', 'candidate'] if pair % 2 else ['candidate', 'baseline']
                for variant in order:
                    binary = binaries[variant + '-' + suite + '-' + mode]
                    assert sha(Path(binary['path'])) == binary['sha256']
                    log = run(f'{suite}-{mode}-{pair}-{variant}',
                              ['taskset', '-c', str(cpu), '/usr/bin/time', '-v', binary['path'], str(count)])
                    rows = [json.loads(line) for line in log.splitlines()]
                    assert len(rows) == {'rational': 48, 'quadratic': 64, 'integer': 24}[suite]
                    trials.append({'suite': suite, 'mode': mode, 'pair': pair, 'variant': variant, 'rows': rows})
    summary = []
    for suite in probes:
        for mode in ['time', 'memory']:
            selected = [t for t in trials if t['suite'] == suite and t['mode'] == mode]
            witnesses = [r for r in selected[0]['rows'] if 'witness' in r]
            assert len(witnesses) == {'rational': 18, 'quadratic': 24, 'integer': 9}[suite]
            assert all([r for r in t['rows'] if 'witness' in r] == witnesses for t in selected)
            for name in [r['name'] for r in selected[0]['rows'] if 'name' in r]:
                fields = ['nanos'] if mode == 'time' else ['allocation_calls', 'allocated_bytes', 'retained_bytes_delta', 'peak_bytes_delta']
                values = {variant: [next(r for r in t['rows'] if r.get('name') == name)
                                    for t in selected if t['variant'] == variant] for variant in ['baseline', 'candidate']}
                stats = {variant: {field: {'samples': [r[field] for r in rows],
                                          'median': statistics.median(r[field] for r in rows)}
                                   for field in fields} for variant, rows in values.items()}
                summary.append({'suite': suite, 'mode': mode, 'name': name, 'variants': stats})
    verify()
    for variant, entries in maps.items():
        assert all(sha(dirs[variant] / p) == expected for p, expected in entries.items())
    result = {'complete': True, 'source_unchanged': True, 'main_runtime_unchanged': True,
              'mathematical_witnesses_identical': True, 'fixed_sampling_complete': True,
              'summary': summary, 'representation_rows': [
                  {'suite': t['suite'], 'mode': t['mode'], 'pair': t['pair'], 'variant': t['variant'],
                   'rows': [r for r in t['rows'] if 'fixture' in r]} for t in trials], 'runtime_admitted': False}
    (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'complete': True, 'matched_scalar_cost_rows': len(summary), 'runtime_admitted': False}), flush=True)


if __name__ == '__main__':
    main()
