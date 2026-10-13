"""Fixed three-variant scalar timing, allocation and size controls.

Three historical probes are copied unchanged. Each suite/mode has five fixed
blocks containing baseline, right-scaling and bidirectional-scaling processes.
All builds precede every probe; each source has a dedicated immutable cost
target. Timing has no allocation wrapper. No sample extension is planned.
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
PREP = E / 'preparation-v1'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    validation = json.loads((E / 'validation-v1/result.json').read_text())
    assert validation['complete'] and all(row['exit_code'] == 0 for row in validation['phases'])
    preparation = json.loads((PREP / 'protocol.json').read_text())
    original = json.loads(gzip.decompress((PREP / 'parent-source-hashes.json.gz').read_bytes()))
    variants = preparation['variants']
    out = E / 'cost-v1'
    out.mkdir(exist_ok=False)
    bins = ROOT / 'target/coefficient-scaling/cost-v1-binaries'
    bins.mkdir(exist_ok=False)
    probes = out / 'probes'
    probes.mkdir()
    inputs = {
        'rational': ROOT / 'docs/evidence/2026-10-09-hyperdrc-native-linear-strokes/normal_form_ownership_probe.rs',
        'quadratic': ROOT / 'docs/evidence/2026-10-12-quadratic-certificate-consumer/quadratic_probe.rs',
        'integer': ROOT / 'docs/evidence/2026-10-12-small-integer-certificates/integer_probe.rs',
    }
    for name, path in inputs.items():
        shutil.copyfile(path, probes / (name + '.rs'))
    blocks = [['baseline', 'right', 'both'], ['both', 'right', 'baseline'],
              ['right', 'baseline', 'both'], ['both', 'baseline', 'right'], ['baseline', 'both', 'right']]
    cpu = min(os.sched_getaffinity(0))
    env = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
           'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never'}
    protocol = {
        'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
        'preparation_sha256': sha(PREP / 'protocol.json'),
        'validation_result_sha256': sha(E / 'validation-v1/result.json'),
        'historical_probe_inputs': {name: {'path': str(path.relative_to(ROOT)), 'sha256': sha(path)} for name, path in inputs.items()},
        'variants': {name: {'source': row['source'], 'target': row['cost_target']} for name, row in variants.items()},
        'fixed_block_order': blocks, 'time_inputs': 4096, 'memory_inputs': 1000, 'preflight_inputs': 42,
        'suites': list(inputs), 'modes': ['time', 'memory'], 'measured_processes': 90, 'preflight_processes': 18,
        'cpu': cpu, 'environment': env, 'inherited_rustflags': os.environ.get('RUSTFLAGS'),
        'compiler': subprocess.check_output(['rustc', '-vV'], text=True),
        'host': tuple(os.uname()), 'cpuinfo': Path('/proc/cpuinfo').read_text().split('\n\n')[0],
        'command_timeout_seconds': 1800, 'runtime_admitted': False,
    }
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    commands, libraries, binaries, trials = [], {}, {}, []

    def verify():
        assert sha(Path(__file__)) == protocol['runner_sha256']
        assert sha(PREP / 'protocol.json') == protocol['preparation_sha256']
        assert sha(E / 'validation-v1/result.json') == protocol['validation_result_sha256']
        assert sources() == preparation['main_source_sha256']
        assert all(sha(ROOT / name) == digest for name, digest in preparation['pending_native_sha256'].items())
        assert all(sha(Path(preparation['parent_source']) / name) == digest for name, digest in original.items())
        for row in variants.values():
            assert all(sha(Path(row['source']) / name) == digest
                       for name, digest in (original | row['source_changes_sha256']).items())
        for name, path in inputs.items():
            assert sha(path) == sha(probes / (name + '.rs')) == protocol['historical_probe_inputs'][name]['sha256']
        assert all(sha(Path(row['path'])) == row['sha256'] for row in libraries.values())
        assert all(sha(Path(row['path'])) == row['sha256'] for row in binaries.values())

    def run(name, command, cwd=ROOT, extra_env=None):
        verify()
        print(json.dumps({'start': name}), flush=True)
        started = time.monotonic()
        command = ['timeout', '--signal=TERM', '--kill-after=10s', '1800', *command]
        with (out / (name + '.stdout')).open('w') as stdout, (out / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(command, cwd=cwd, env=os.environ | env | (extra_env or {}),
                                  stdout=stdout, stderr=stderr).returncode
        row = {'name': name, 'command': command, 'cwd': str(cwd), 'extra_environment': extra_env or {},
               'exit_code': code, 'seconds': time.monotonic() - started,
               'stdout_sha256': sha(out / (name + '.stdout')), 'stderr_sha256': sha(out / (name + '.stderr'))}
        commands.append(row)
        (out / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
        print(json.dumps({'name': name, 'exit_code': code, 'seconds': row['seconds']}), flush=True)
        verify()
        assert code == 0, name
        return (out / (name + '.stdout')).read_text()

    for variant, item in variants.items():
        directory, target = Path(item['source']), Path(item['cost_target'])
        raw = run('build-' + variant, ['cargo', 'build', '--locked', '--release', '-p', 'hyperreal',
                                      '--all-features', '--lib', '--message-format=json'], directory,
                  {'CARGO_TARGET_DIR': str(target)})
        events = [json.loads(line) for line in raw.splitlines() if line.startswith('{')]
        matches = [row for row in events if row.get('reason') == 'compiler-artifact'
                   and row['target']['name'] == 'hyperreal' and any(path.endswith('.rlib') for path in row['filenames'])]
        assert len(matches) == 1
        artifact = matches[0]
        library = Path(next(path for path in artifact['filenames'] if path.endswith('.rlib')))
        assert library.is_relative_to(target)
        assert Path(artifact['manifest_path']) == directory / 'crates/hyperreal/Cargo.toml'
        libraries[variant] = {'path': str(library), 'sha256': sha(library), 'bytes': library.stat().st_size,
                              'manifest_path': artifact['manifest_path'], 'features': artifact['features'],
                              'package_id': artifact['package_id']}
        (out / 'libraries.json').write_text(json.dumps(libraries, indent=2) + '\n')
    assert all(row['features'] == libraries['baseline']['features'] for row in libraries.values())
    assert len({row['sha256'] for row in libraries.values()}) == 3
    for variant, item in variants.items():
        for suite in inputs:
            for mode in ['time', 'memory']:
                label = variant + '-' + suite + '-' + mode
                binary = bins / label
                command = ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(probes / (suite + '.rs')),
                           '-L', 'dependency=' + str(Path(item['cost_target']) / 'release/deps'),
                           '--extern', 'hyperreal=' + libraries[variant]['path'], '-o', str(binary)]
                if mode == 'memory':
                    command += ['--cfg', 'feature="count-alloc"']
                run('link-' + label, command)
                binaries[label] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
                (out / 'binaries.json').write_text(json.dumps(binaries, indent=2) + '\n')
    for suite in inputs:
        for mode in ['time', 'memory']:
            assert len({binaries[f'{variant}-{suite}-{mode}']['sha256'] for variant in variants}) == 3
    # Every library and executable build is terminal before any preflight or measurement.
    for label, binary in binaries.items():
        run('preflight-' + label, ['taskset', '-c', str(cpu), binary['path'], '42'])
    for suite in inputs:
        for mode, count in [('time', 4096), ('memory', 1000)]:
            canonical = None
            for index, order in enumerate(blocks, 1):
                for variant in order:
                    name = f'{suite}-{mode}-{index}-{variant}'
                    binary = binaries[f'{variant}-{suite}-{mode}']
                    raw = run(name, ['taskset', '-c', str(cpu), '/usr/bin/time', '-v', binary['path'], str(count)])
                    rows = [json.loads(line) for line in raw.splitlines()]
                    assert len(rows) == {'rational': 48, 'quadratic': 64, 'integer': 24}[suite]
                    assert sum('name' in row for row in rows) == {'rational': 24, 'quadratic': 32, 'integer': 12}[suite]
                    assert sum('fixture' in row for row in rows) == {'rational': 6, 'quadratic': 8, 'integer': 3}[suite]
                    identity = [row for row in rows if 'witness' in row or 'fixture' in row]
                    assert sum('witness' in row for row in rows) == {'rational': 18, 'quadratic': 24, 'integer': 9}[suite]
                    if canonical is None:
                        canonical = identity
                    assert identity == canonical, name
                    trials.append({'suite': suite, 'mode': mode, 'block': index, 'variant': variant, 'rows': rows})
                    (out / 'trials.json').write_text(json.dumps(trials, indent=2) + '\n')
    summary = []
    for suite in inputs:
        for mode in ['time', 'memory']:
            selected = [trial for trial in trials if trial['suite'] == suite and trial['mode'] == mode]
            for name in [row['name'] for row in selected[0]['rows'] if 'name' in row]:
                fields = ['nanos'] if mode == 'time' else ['allocation_calls', 'allocated_bytes', 'retained_bytes_delta', 'peak_bytes_delta']
                values = {variant: [next(row for row in trial['rows'] if row.get('name') == name)
                                    for trial in selected if trial['variant'] == variant] for variant in variants}
                assert all(len(rows) == 5 for rows in values.values())
                stats = {variant: {field: {'samples': [row[field] for row in rows],
                                          'median': statistics.median(row[field] for row in rows)} for field in fields}
                         for variant, rows in values.items()}
                summary.append({'suite': suite, 'mode': mode, 'name': name, 'variants': stats})
    assert len(trials) == 90
    verify()
    result = {'complete': True, 'fixed_sample_blocks_complete': True, 'mathematical_witnesses_and_inventories_identical': True,
              'source_parent_main_libraries_and_binaries_unchanged': True, 'measured_processes': 90,
              'summary': summary, 'runtime_admitted': False}
    (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'complete': True, 'cost_rows': len(summary), 'measured_processes': 90, 'runtime_admitted': False}), flush=True)


if __name__ == '__main__':
    main()
