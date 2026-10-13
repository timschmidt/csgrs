"""Qualify and run frozen integer-only allocation diagnostics, sequentially.

One fresh process per variant and probe observes the original 1000-input arc
memory workload and 231-operation broad workload. These instrumented durations
are not additional samples for the closed five-pair performance comparison.
"""
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
PREP = E / 'preparation-v1'
TRIAL = ROOT / 'docs/evidence/2026-10-12-endpoint-coordinate-reuse'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    out = E / 'observation-v1'
    out.mkdir(exist_ok=False)
    work = ROOT / 'target/endpoint-cost-attribution/observation-v1'
    work.mkdir(exist_ok=False)
    preparation = json.loads((PREP / 'protocol.json').read_text())
    variants = preparation['variants']
    probes = PREP / 'probes'
    env = {'CARGO_HOME': '/tmp/csgrs-properties-cargo',
           'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
           'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never'}
    protocol = {
        'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
        'preparation_sha256': sha(PREP / 'protocol.json'),
        'prior_trials_sha256': sha(TRIAL / 'controls-v1/trials.json'),
        'sources': {key: row['source'] for key, row in variants.items()},
        'targets': {key: row['target'] for key, row in variants.items()},
        'probe_sha256': preparation['probes_sha256'], 'environment': env,
        'processes_per_variant_probe': 1, 'arc_inputs_per_family': 1000,
        'arc_families': 12, 'arc_measure_rows': 48, 'arc_witness_rows': 36,
        'arc_fixture_rows': 12, 'broad_witness_rows': 231,
        'cpu': 0, 'command_timeout_seconds': 1800,
        'compiler': subprocess.check_output(['rustc', '-vV'], text=True),
        'inherited_rustflags': os.environ.get('RUSTFLAGS'),
        'host': tuple(os.uname()),
        'limits': preparation['limits'], 'runtime_admitted': False,
    }
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    commands, libraries, binaries, tests, observations = [], {}, {}, [], []

    def verify():
        assert sources() == preparation['main_source_sha256']
        assert sha(Path(__file__)) == protocol['runner_sha256']
        assert sha(PREP / 'protocol.json') == protocol['preparation_sha256']
        assert all(sha(probes / name) == digest for name, digest in protocol['probe_sha256'].items())
        for variant, row in variants.items():
            assert all(sha(Path(row['parent_source']) / name) == digest
                       for name, digest in row['parent_hashes'].items()), variant
            expected = row['parent_hashes'] | row['source_changes_sha256']
            assert all(sha(Path(row['source']) / name) == digest
                       for name, digest in expected.items()), variant
        for rows in libraries.values():
            assert all(sha(Path(row['path'])) == row['sha256'] for row in rows.values())
        assert all(sha(Path(row['path'])) == row['sha256'] for row in binaries.values())

    def run(name, command, cwd=ROOT, extra_env=None, passed=None):
        verify()
        print(json.dumps({'start': name}), flush=True)
        started = time.monotonic()
        with (out / (name + '.stdout')).open('w') as stdout, (out / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(
                ['timeout', '--signal=TERM', '--kill-after=10s', '1800', *command],
                cwd=cwd, env=os.environ | env | (extra_env or {}),
                stdout=stdout, stderr=stderr).returncode
        row = {'name': name, 'command': command, 'cwd': str(cwd),
               'extra_environment': extra_env or {}, 'exit_code': code,
               'seconds': time.monotonic() - started,
               'stdout_sha256': sha(out / (name + '.stdout')),
               'stderr_sha256': sha(out / (name + '.stderr'))}
        commands.append(row)
        (out / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
        print(json.dumps(row), flush=True)
        verify()
        assert code == 0, name
        raw = (out / (name + '.stdout')).read_text()
        if passed is not None:
            counts = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;', raw)
            assert counts == [(str(passed), '0', '0')], (name, counts)
            tests.append({'name': name, 'passed': passed, 'failed': 0, 'ignored': 0})
            (out / 'tests.json').write_text(json.dumps(tests, indent=2) + '\n')
        return raw

    for variant, item in variants.items():
        directory = Path(item['source'])
        extra = {'CARGO_TARGET_DIR': item['target']}
        run('format-' + variant, ['cargo', 'fmt', '--all', '--', '--check'], directory, extra)
        run('calibration-' + variant,
            ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features',
             '--lib', 'endpoint_cost_diagnostic::tests', '--', '--test-threads=1'],
            directory, extra, passed=2)
        run('point-equality-' + variant,
            ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--all-features',
             '--lib', 'point::equality_tests', '--', '--test-threads=1'],
            directory, extra, passed=5)
        if variant == 'candidate':
            run('endpoint-regressions-' + variant,
                ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--all-features',
                 '--lib', 'endpoint_proof_tests', '--', '--test-threads=1'],
                directory, extra, passed=5)
        run('strict-clippy-' + variant,
            ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal', '-p', 'hypercurve',
             '-p', 'hyperdrc', '--all-features', '--all-targets', '--', '-D', 'warnings'],
            directory, extra)

    for variant, item in variants.items():
        target = Path(item['target'])
        raw = run('build-libraries-' + variant,
                  ['cargo', 'build', '--locked', '--release', '-p', 'hypercurve', '-p', 'hyperdrc',
                   '--all-features', '--lib', '--message-format=json'],
                  Path(item['source']), {'CARGO_TARGET_DIR': str(target)})
        events = [json.loads(line) for line in raw.splitlines() if line.startswith('{')]
        libraries[variant] = {}
        for name in ['hypercurve', 'hyperreal', 'hyperdrc', 'hyperlimit', 'serde_json']:
            matches = [event for event in events if event.get('reason') == 'compiler-artifact'
                       and event['target']['name'] == name
                       and any(path.endswith('.rlib') for path in event['filenames'])]
            assert len(matches) == 1, (variant, name)
            artifact = matches[0]
            library = Path(next(path for path in artifact['filenames'] if path.endswith('.rlib')))
            assert library.is_relative_to(target)
            if name != 'serde_json':
                assert Path(artifact['manifest_path']) == Path(item['source']) / 'crates' / name / 'Cargo.toml'
            libraries[variant][name] = {
                'path': str(library), 'sha256': sha(library), 'bytes': library.stat().st_size,
                'manifest_path': artifact['manifest_path'], 'features': artifact['features']}
        (out / 'libraries.json').write_text(json.dumps(libraries, indent=2) + '\n')
    assert {name: row['features'] for name, row in libraries['baseline'].items()} == {
        name: row['features'] for name, row in libraries['candidate'].items()}

    for variant, item in variants.items():
        for mode, filename, crates in [
            ('arc', 'arc_cost.rs', ['hypercurve', 'hyperreal']),
            ('broad', 'operation_probe.rs', ['hyperdrc', 'hyperlimit', 'hypercurve', 'hyperreal', 'serde_json']),
        ]:
            label = variant + '-' + mode
            binary = work / label
            command = ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(probes / filename),
                       '-L', 'dependency=' + str(Path(item['target']) / 'release/deps')]
            for name in crates:
                command += ['--extern', name + '=' + libraries[variant][name]['path']]
            if mode == 'arc':
                command += ['--cfg', 'feature="count-alloc"']
            run('build-' + label, command + ['-o', str(binary)])
            binaries[label] = {'path': str(binary), 'sha256': sha(binary),
                               'bytes': binary.stat().st_size}
            (out / 'binaries.json').write_text(json.dumps(binaries, indent=2) + '\n')

    # Every library, test, lint and executable build is terminal before observations.
    prior = json.loads((TRIAL / 'controls-v1/trials.json').read_text())
    for mode in ['arc', 'broad']:
        for variant in ['baseline', 'candidate']:
            label = variant + '-' + mode
            name = 'observe-' + label
            command = ['taskset', '-c', '0', binaries[label]['path']]
            if mode == 'arc':
                command.append('1000')
            raw = run(name, command)
            if mode == 'arc':
                rows = [json.loads(line) for line in raw.splitlines()]
                assert len(rows) == 96
                assert sum('name' in row for row in rows) == 48
                assert sum('witness' in row for row in rows) == 36
                assert sum('fixture' in row for row in rows) == 12
                identity = [row for row in rows if 'witness' in row or 'fixture' in row]
                original = next(row['rows'] for row in prior
                                if row['mode'] == 'memory' and row['index'] == 1 and row['variant'] == variant)
                assert identity == [row for row in original if 'witness' in row or 'fixture' in row]
                measured = [{k: v for k, v in row.items() if k != 'nanos'} for row in rows if 'name' in row]
                old = [{k: v for k, v in row.items() if k != 'nanos'} for row in original if 'name' in row]
                comparison = [{'original': a, 'diagnostic': b} for a, b in zip(old, measured) if a != b]
            else:
                rows = []
                for line in raw.splitlines():
                    assert line.startswith('operation_'), line
                    label, iterations, nanos, witness = line.split('\t')
                    rows.append({'name': label, 'iterations': int(iterations),
                                 'elapsed_ns': int(nanos), 'witness': witness})
                assert len(rows) == 231
                identity = [{k: v for k, v in row.items() if k != 'elapsed_ns'} for row in rows]
                original = next(row['rows'] for row in prior
                                if row['mode'] == 'broad' and row['index'] == 1 and row['variant'] == variant)
                assert identity == [{k: v for k, v in row.items() if k != 'elapsed_ns'} for row in original]
                comparison = None
            observed = {'mode': mode, 'variant': variant, 'rows': rows,
                        'matches_original_identity': True,
                        'arc_original_allocation_differences': comparison}
            observations.append(observed)
            (out / 'observations.json').write_text(json.dumps(observations, indent=2) + '\n')
    verify()
    result = {'complete': True, 'tests': tests, 'processes': len(observations),
              'all_original_witnesses_match': True,
              'original_arc_allocation_rows_match': all(not row['arc_original_allocation_differences']
                                                        for row in observations if row['mode'] == 'arc'),
              'parents_diagnostic_sources_probes_libraries_binaries_main_unchanged': True,
              'instrumented_durations_not_performance_samples': True, 'runtime_admitted': False}
    (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
