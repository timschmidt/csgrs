"""Fixed native costs and receipts for baseline and zero-radical candidate.

Both frozen source trees use new, separate native-control Cargo targets.
All builds finish before probes execute. Historical probes remain byte-for-byte
unchanged. Time has no allocation wrapper; memory counts requested Rust bytes.
Five baseline/candidate blocks are fixed in advance; no extension is planned. Native
gate failures remain recorded and this run cannot authorize runtime admission.
"""
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
WORK = ROOT / 'target/coefficient-zero-radicals'
HISTORY = ROOT / 'docs/evidence/2026-10-09-hyperdrc-native-linear-strokes'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    out = E / 'native-controls-v1'
    out.mkdir(exist_ok=False)
    work = WORK / 'native-controls-v1'
    work.mkdir(exist_ok=False)
    probes = out / 'probes'
    probes.mkdir()
    preparation = json.loads((E / 'preparation-v1/protocol.json').read_text())
    validation = json.loads((E / 'native-validation-v1/result.json').read_text())
    assert validation['complete']
    assert all(row['exit_code'] == 0 for row in validation['phases'] if row['phase'] not in ['full-board', 'copper-neck'])
    parent = json.loads(gzip.decompress((E / 'preparation-v1/parent-source-hashes.json.gz').read_bytes()))
    maps = {name: parent | row['source_changes_sha256'] for name, row in preparation['variants'].items()}
    dirs = {name: Path(row['source']) for name, row in preparation['variants'].items()}
    targets = {name: WORK / name / 'native-controls-target' for name in dirs}
    variants = ['baseline', 'candidate']
    block_order = [['baseline', 'candidate'], ['candidate', 'baseline'], ['candidate', 'baseline'],
                   ['baseline', 'candidate'], ['baseline', 'candidate']]
    decision = json.loads((E / 'scalar-decision-v1.json').read_text())
    assert block_order == decision['native_controls_fixed_order']
    assert list(dirs) == variants
    assert all(not path.exists() for path in targets.values())
    cpu = min(os.sched_getaffinity(0))
    inputs = {
        'arc_cost.rs': HISTORY / 'arc_endpoint_cost_probe.rs',
        'final_abort.rs': HISTORY / 'arc-endpoint-final-abort/probe.rs',
    }
    inputs |= {path.name: path for path in sorted((HISTORY / 'arc-endpoint-production-replay/probe_sources').glob('*.rs')) if path.name != 'full_report_replay.rs'}
    for name, path in inputs.items():
        shutil.copyfile(path, probes / name)
    probe_hashes = {name: sha(probes / name) for name in inputs}
    env = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache', 'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never'}
    protocol = {
        'scope': __doc__, 'runner_sha256': sha(Path(__file__)), 'preparation_sha256': sha(E / 'preparation-v1/protocol.json'),
        'validation_sha256': sha(E / 'native-validation-v1/result.json'),
        'native_assessment_sha256': sha(E / 'native-assessment-v1.json'),
        'scalar_decision_sha256': sha(E / 'scalar-decision-v1.json'),
        'assessor_sha256': sha(E / 'assess_native_controls_v1.py'),
        'sources': {key: str(value) for key, value in dirs.items()}, 'targets': {key: str(value) for key, value in targets.items()},
        'historical_probe_inputs': {name: {'path': str(path.relative_to(ROOT)), 'sha256': sha(path)} for name, path in inputs.items()},
        'probe_sha256': probe_hashes, 'environment': env,
        'fixed_block_order': block_order, 'fixed_blocks': 5, 'time_inputs_per_family': 8192, 'memory_inputs_per_family': 1000, 'preflight_inputs': 42,
        'arc_families': 12, 'arc_query_phases': ['first', 'warm', 'cloned'], 'also_measure_construction': True,
        'broad_witness_rows_per_process': 231, 'final_abort_rows_per_process': 210, 'final_abort_blocks': block_order[:3],
        'cpu': cpu, 'process_timeout_seconds': 1800,
        'historical_abort_limit': 'The cost probe attaches abort to an intermediate expression. Its uncertain-result control is retained as-is; effective final-coordinate abortion is checked separately by final_abort.rs.',
        'compiler': subprocess.check_output(['rustc', '-vV'], text=True), 'inherited_rustflags': os.environ.get('RUSTFLAGS'),
        'host': tuple(os.uname()), 'cpuinfo': Path('/proc/cpuinfo').read_text().split('\n\n')[0],
        'runtime_admitted': False,
    }
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    commands, libraries, binaries = [], {}, {}

    def verify():
        assert sources() == preparation['main_source_sha256']
        assert all(sha(ROOT / name) == digest for name, digest in preparation['pending_native_sha256'].items())
        assert all(sha(Path(preparation['parent_source']) / name) == digest for name, digest in parent.items())
        assert sha(E / 'preparation-v1/protocol.json') == protocol['preparation_sha256']
        assert sha(E / 'native-validation-v1/result.json') == protocol['validation_sha256']
        assert sha(Path(__file__)) == protocol['runner_sha256']
        assert sha(E / 'assess_native_controls_v1.py') == protocol['assessor_sha256']
        assert sha(E / 'native-assessment-v1.json') == protocol['native_assessment_sha256']
        assert sha(E / 'scalar-decision-v1.json') == protocol['scalar_decision_sha256']
        assert all(sha(path) == probe_hashes[name] for name, path in inputs.items())
        assert all(sha(probes / name) == digest for name, digest in probe_hashes.items())
        for variant, expected in maps.items():
            assert all(sha(dirs[variant] / name) == digest for name, digest in expected.items())
        for rows in libraries.values():
            assert all(sha(Path(row['path'])) == row['sha256'] for row in rows.values())
        assert all(sha(Path(row['path'])) == row['sha256'] for row in binaries.values())

    def run(name, command, cwd=ROOT, extra_env=None):
        verify()
        started = time.monotonic()
        print(json.dumps({'start': name}), flush=True)
        with (out / (name + '.stdout')).open('w') as stdout, (out / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', '1800', *command], cwd=cwd,
                                  env=os.environ | env | (extra_env or {}), stdout=stdout, stderr=stderr).returncode
        verify()
        row = {'name': name, 'command': command, 'cwd': str(cwd), 'extra_environment': extra_env or {},
               'exit_code': code, 'seconds': time.monotonic() - started,
               'stdout_sha256': sha(out / (name + '.stdout')), 'stderr_sha256': sha(out / (name + '.stderr'))}
        commands.append(row)
        (out / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
        print(json.dumps(row), flush=True)
        assert code == 0, name
        return (out / (name + '.stdout')).read_text()

    names = ['hypercurve', 'hyperreal', 'hyperdrc', 'hyperlimit', 'serde_json']
    for variant in variants:
        raw = run('build-libraries-' + variant,
                  ['cargo', 'build', '--locked', '--release', '-p', 'hypercurve', '-p', 'hyperdrc', '--all-features', '--lib', '--message-format=json'],
                  dirs[variant], {'CARGO_TARGET_DIR': str(targets[variant])})
        events = [json.loads(line) for line in raw.splitlines() if line.startswith('{')]
        libraries[variant] = {}
        for name in names:
            artifacts = [event for event in events if event.get('reason') == 'compiler-artifact' and event['target']['name'] == name
                         and any(path.endswith('.rlib') for path in event['filenames'])]
            assert len(artifacts) == 1, name
            artifact = artifacts[0]
            library = Path(next(path for path in artifact['filenames'] if path.endswith('.rlib')))
            assert library.is_relative_to(targets[variant])
            if name != 'serde_json':
                assert Path(artifact['manifest_path']) == dirs[variant] / 'crates' / name / 'Cargo.toml'
            libraries[variant][name] = {'path': str(library), 'sha256': sha(library), 'bytes': library.stat().st_size,
                                         'manifest_path': artifact['manifest_path'], 'features': artifact['features']}
        (out / 'libraries.json').write_text(json.dumps(libraries, indent=2) + '\n')
    assert all({name: row['features'] for name, row in libraries['baseline'].items()} ==
               {name: row['features'] for name, row in libraries[variant].items()} for variant in variants)
    assert len({libraries[variant]['hyperreal']['sha256'] for variant in variants}) == 2
    (out / 'libraries.json').write_text(json.dumps(libraries, indent=2) + '\n')
    for variant in variants:
        for mode, filename, crates in [
            ('time', 'arc_cost.rs', ['hypercurve', 'hyperreal']),
            ('memory', 'arc_cost.rs', ['hypercurve', 'hyperreal']),
            ('abort', 'final_abort.rs', ['hypercurve']),
            ('broad', 'operation_probe.rs', ['hyperdrc', 'hyperlimit', 'hypercurve', 'serde_json']),
        ]:
            label = variant + '-' + mode
            binary = work / label
            command = ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(probes / filename), '-L', 'dependency=' + str(targets[variant] / 'release/deps')]
            for name in crates:
                command += ['--extern', name + '=' + libraries[variant][name]['path']]
            if mode == 'memory':
                command += ['--cfg', 'feature="count-alloc"']
            run('build-' + label, command + ['-o', str(binary)])
            binaries[label] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
            (out / 'binaries.json').write_text(json.dumps(binaries, indent=2) + '\n')
    (out / 'binaries.json').write_text(json.dumps(binaries, indent=2) + '\n')
    assert all(len({binaries[variant + '-' + mode]['sha256'] for variant in variants}) == 2
               for mode in ['time', 'memory', 'abort', 'broad'])
    # All builds are terminal before any preflight, receipt or timed query.
    preflight_identity = {}
    for variant in variants:
        for mode in ['time', 'memory']:
            raw = run('preflight-' + variant + '-' + mode, ['taskset', '-c', str(cpu), binaries[variant + '-' + mode]['path'], '42'])
            rows = [json.loads(line) for line in raw.splitlines()]
            assert len(rows) == 96
            identity = [row for row in rows if 'witness' in row or 'fixture' in row]
            assert len(identity) == 48
            assert preflight_identity.setdefault(mode, identity) == identity
    (out / 'preflight-identities.json').write_text(json.dumps(preflight_identity, indent=2) + '\n')
    trials = []
    abort_canonical = None
    for index in range(1, 4):
        for variant in block_order[index - 1]:
            name = f'abort-{index}-{variant}'
            raw = run(name, ['taskset', '-c', str(cpu), binaries[variant + '-abort']['path']])
            rows = [json.loads(line) for line in raw.splitlines()]
            assert len(rows) == 210
            if abort_canonical is None:
                abort_canonical = rows
            assert rows == abort_canonical, name
            trials.append({'mode': 'abort', 'index': index, 'variant': variant, 'rows': rows})
            (out / 'trials.json').write_text(json.dumps(trials, indent=2) + '\n')
    for mode, count in [('time', 8192), ('memory', 1000), ('broad', None)]:
        canonical = None
        for index in range(1, 6):
            for variant in block_order[index - 1]:
                name = f'{mode}-{index}-{variant}'
                command = ['taskset', '-c', str(cpu), '/usr/bin/time', '-v', binaries[variant + '-' + mode]['path']]
                raw = run(name, command + ([] if count is None else [str(count)]))
                if mode == 'broad':
                    rows = []
                    for line in raw.splitlines():
                        if line.startswith('operation_'):
                            label, iterations, nanos, witness = line.split('\t')
                            rows.append({'name': label, 'iterations': int(iterations), 'elapsed_ns': int(nanos), 'witness': witness})
                    assert len(rows) == 231
                    identity = [{key: value for key, value in row.items() if key != 'elapsed_ns'} for row in rows]
                else:
                    rows = [json.loads(line) for line in raw.splitlines()]
                    assert len(rows) == 96 and sum('name' in row for row in rows) == 48
                    assert sum('witness' in row for row in rows) == 36 and sum('fixture' in row for row in rows) == 12
                    identity = [row for row in rows if 'witness' in row or 'fixture' in row]
                if canonical is None:
                    canonical = identity
                rss = int(re.search(r'Maximum resident set size \(kbytes\): (\d+)', (out / (name + '.stderr')).read_text())[1])
                trials.append({'mode': mode, 'index': index, 'variant': variant, 'rows': rows, 'maximum_rss_kib': rss,
                               'matches_first_process_identity': identity == canonical})
                (out / 'trials.json').write_text(json.dumps(trials, indent=2) + '\n')
                assert identity == canonical, name
        if mode == 'broad':
            historical = json.loads((ROOT / 'docs/evidence/2026-10-09-hyperdrc-kicad-pad-sources/cost/rows.json').read_text())
            comparison = {'all_231_historical_witnesses_identical': canonical == historical,
                          'historical_sha256': sha(ROOT / 'docs/evidence/2026-10-09-hyperdrc-kicad-pad-sources/cost/rows.json'),
                          'differences': [{'prior': a, 'current': b} for a, b in zip(historical, canonical) if a != b]}
            (out / 'historical-comparison.json').write_text(json.dumps(comparison, indent=2) + '\n')
            assert comparison['all_231_historical_witnesses_identical']
    verify()
    result = {'complete': True, 'arc_trials': 20, 'broad_trials': 10, 'final_abort_trials': 6,
              'all_variant_identities_match': True, 'source_and_main_unchanged': True,
              'timing_is_fixed_five_baseline_candidate_blocks': True, 'runtime_admitted': False}
    (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
