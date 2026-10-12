"""Measure unchanged public normal-form workloads in fresh paired processes.

Five fixed alternating pairs in each of time and allocation modes. Both
libraries use the same release features/compiler and the exact same probe.
Only mathematical witnesses must agree; representation inventory is retained
separately, so construction changes cannot be hidden by reporting query cost.
This early scalar gate does not admit a native or scalar runtime change.
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
WORK = ROOT / 'target/persistent-laurent-cache'
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources

sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    validation = json.loads((E / 'validation-v1/result.json').read_text())
    assert validation['complete'] and all(p['exit_code'] == 0 for p in validation['phases'])
    preparation = json.loads((E / 'preparation.json').read_text())
    original = json.loads(gzip.decompress((E / 'initial-source-hashes.json.gz').read_bytes()))
    delta = json.loads((E / 'validation-v1/protocol.json').read_text())['source_changes_sha256']
    maps = {'baseline': original, 'candidate': original | delta}
    for variant, entries in maps.items():
        assert all(sha(WORK / variant / 'source' / p) == expected for p, expected in entries.items())
    out = E / 'cost-v1'
    out.mkdir(exist_ok=False)
    bins = WORK / 'cost-v1-binaries'
    bins.mkdir(exist_ok=False)
    probe = ROOT / 'docs/evidence/2026-10-09-hyperdrc-native-linear-strokes/normal_form_ownership_probe.rs'
    shutil.copyfile(probe, out / 'probe.rs')
    cpu = min(os.sched_getaffinity(0))
    env = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
           'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never',
           'CARGO_TARGET_DIR': str(ROOT / 'target')}
    protocol = {'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
                'probe_sha256': sha(probe), 'predecessor_validation_sha256': sha(E / 'validation-v1/result.json'),
                'preparation_sha256': sha(E / 'preparation.json'), 'source_changes_sha256': delta,
                'environment': env, 'inherited_rustflags': os.environ.get('RUSTFLAGS'),
                'compiler': subprocess.check_output(['rustc', '-vV'], text=True),
                'pinned_cpu': cpu, 'time_iterations': 4096, 'memory_iterations': 1000,
                'pairs_each_mode': 5, 'all_builds_finish_before_measurements': True,
                'native_runtime_admitted': False}
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    commands = []

    def verify():
        assert sources() == preparation['main_source_sha256']
        for variant, entries in maps.items():
            for name in delta:
                p = WORK / variant / 'source' / name
                if name in entries:
                    assert sha(p) == entries[name]
                else:
                    assert not p.exists()

    def run(name, command, cwd=ROOT):
        verify()
        started = time.monotonic()
        with (out / (name + '.stdout')).open('w') as stdout, (out / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(command, cwd=cwd, env=os.environ | env, stdout=stdout, stderr=stderr).returncode
        verify()
        record = {'name': name, 'command': command, 'cwd': str(cwd), 'exit_code': code,
                  'seconds': time.monotonic() - started, 'stdout_sha256': sha(out / (name + '.stdout')),
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
                  WORK / variant / 'source')
        events = [json.loads(line) for line in log.splitlines() if line.startswith('{')]
        artifact = [event for event in events if event.get('reason') == 'compiler-artifact' and event['target']['name'] == 'hyperreal']
        assert len(artifact) == 1
        library = Path(next(p for p in artifact[0]['filenames'] if p.endswith('.rlib')))
        libraries[variant] = {'sha256': sha(library), 'path': str(library), 'features': artifact[0]['features']}
        for mode in ['time', 'memory']:
            binary = bins / (variant + '-' + mode)
            command = ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(out / 'probe.rs'),
                       '-L', 'dependency=' + str(ROOT / 'target/release/deps'),
                       '--extern', 'hyperreal=' + str(library), '-o', str(binary)]
            if mode == 'memory':
                command += ['--cfg', 'feature="count-alloc"']
            run('link-' + variant + '-' + mode, command)
            binaries[variant + '-' + mode] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
    (out / 'binaries.json').write_text(json.dumps({'libraries': libraries, 'binaries': binaries}, indent=2) + '\n')
    for variant in ['baseline', 'candidate']:
        for mode in ['time', 'memory']:
            run('preflight-' + variant + '-' + mode, ['taskset', '-c', str(cpu), binaries[variant + '-' + mode]['path'], '42'])
    trials = []
    for mode, count in [('time', 4096), ('memory', 1000)]:
        for pair in range(1, 6):
            order = ['baseline', 'candidate'] if pair % 2 else ['candidate', 'baseline']
            for variant in order:
                binary = binaries[variant + '-' + mode]
                assert sha(Path(binary['path'])) == binary['sha256']
                log = run(f'{mode}-{pair}-{variant}', ['taskset', '-c', str(cpu), '/usr/bin/time', '-v', binary['path'], str(count)])
                rows = [json.loads(line) for line in log.splitlines()]
                assert len(rows) == 48
                trials.append({'mode': mode, 'pair': pair, 'variant': variant, 'rows': rows})
    for mode in ['time', 'memory']:
        selected = [t for t in trials if t['mode'] == mode]
        witnesses = [r for r in selected[0]['rows'] if 'witness' in r]
        assert len(witnesses) == 18
        assert all([r for r in t['rows'] if 'witness' in r] == witnesses for t in selected)
    summary = []
    for mode in ['time', 'memory']:
        selected = [t for t in trials if t['mode'] == mode]
        for name in [r['name'] for r in selected[0]['rows'] if 'name' in r]:
            fields = ['nanos'] if mode == 'time' else ['allocation_calls', 'allocated_bytes', 'retained_bytes_delta', 'peak_bytes_delta']
            values = {variant: [next(r for r in t['rows'] if r.get('name') == name)
                                for t in selected if t['variant'] == variant] for variant in ['baseline', 'candidate']}
            stats = {variant: {field: {'samples': [r[field] for r in rows],
                                      'median': statistics.median(r[field] for r in rows)}
                               for field in fields} for variant, rows in values.items()}
            summary.append({'mode': mode, 'name': name, 'variants': stats})
    verify()
    for variant, entries in maps.items():
        assert all(sha(WORK / variant / 'source' / p) == expected for p, expected in entries.items())
    result = {'complete': True, 'source_unchanged': True, 'main_runtime_unchanged': True,
              'mathematical_witnesses_identical': True, 'fixed_sampling_complete': True,
              'summary': summary, 'representation_rows': [
                  {'mode': t['mode'], 'pair': t['pair'], 'variant': t['variant'], 'rows': [r for r in t['rows'] if 'fixture' in r]}
                  for t in trials], 'runtime_admitted': False}
    (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'complete': True, 'matched_scalar_cost_rows': len(summary), 'runtime_admitted': False}), flush=True)


if __name__ == '__main__':
    main()
