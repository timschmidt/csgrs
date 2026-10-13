"""Five fixed paired blocks of direct compaction timing and allocation costs.

Two sources, separate fresh Cargo targets, two separately linked probe modes.
All builds end before four preflights and twenty measurement processes. Exact
value witnesses must agree; sign availability may only strengthen. Sampling
never extends to discard adverse rows. Requested precisions and original native
gates are unchanged; this is a scalar microbenchmark, not an application gate.
"""
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'cost-v1'
WORK = ROOT / 'target/quadratic-tower-sign-retention/cost-v1'
BLOCKS = [['baseline', 'candidate'], ['candidate', 'baseline'], ['candidate', 'baseline'],
          ['baseline', 'candidate'], ['baseline', 'candidate']]
FAMILIES = ['quadratic', 'denested_tower', 'nested_tower', 'negative_normalized_tower',
            'denested_tiny_linear', 'denested_tiny_square', 'nested_tiny_linear', 'nested_tiny_square',
            'biquadratic_tiny_linear', 'biquadratic_tiny_square', 'rational_cancellation', 'zero_cancellation',
            'opaque_decline', 'outside_tower_decline', 'aborted_known_tower']
PHASES = ['construct_and_prove_source', 'first_compaction', 'warm_compaction', 'cloned_compaction']
IDENTITY_FIELDS = ['witness', 'inputs', 'covered', 'value_equality_fnv1a64']
FACT_FIELDS = ['immediate_known', 'precision_zero_known']
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def source_state():
    preparation = json.loads((E / 'preparation-v4/protocol.json').read_text())
    baseline = json.loads((E / 'baseline-v1/protocol.json').read_text())
    entries = [(ROOT, 'baseline-v1/original-source-hashes.json.gz'),
               (Path(baseline['source']), 'baseline-v1/baseline-source-hashes.json.gz'),
               (Path(preparation['control_source']), 'preparation-v4/control-source-hashes.json.gz'),
               (Path(preparation['source']), 'preparation-v4/candidate-source-hashes.json.gz')]
    maps = [(directory, json.loads(gzip.decompress((E / manifest).read_bytes()))) for directory, manifest in entries]
    return preparation, baseline, entries, maps


def verify_sources(state):
    _, baseline, _, maps = state
    assert sources() == baseline['main_source_sha256']
    assert all(sha(ROOT / name) == digest for name, digest in baseline['pending_native_sha256'].items())
    assert all(sha(directory / name) == digest for directory, entries in maps for name, digest in entries.items())


def read_rows(raw, count, memory):
    rows = [json.loads(line) for line in raw.splitlines()]
    assert len(rows) == 105
    timed = [row for row in rows if 'name' in row]
    witnesses = [row for row in rows if 'witness' in row]
    assert [row['name'] for row in timed] == [family + '_' + phase for family in FAMILIES for phase in PHASES]
    assert [row['witness'] for row in witnesses] == [family + '_' + phase for family in FAMILIES for phase in PHASES[1:]]
    assert all(row['iterations'] == count and row['allocation_accounting'] == memory for row in timed)
    for family in FAMILIES:
        for row in [row for row in witnesses if row['witness'].startswith(family + '_')]:
            assert row['inputs'] == count
            assert row['covered'] == (0 if family in ['opaque_decline', 'outside_tower_decline'] else count)
            assert all(0 <= row[field] <= row['covered'] for field in FACT_FIELDS)
    return rows


def main():
    assert json.loads((E / 'qualification-v5/result.json').read_text())['complete']
    assert json.loads((E / 'correctness-v1.json').read_text())['decision'] == 'isolated-tower-sign-retention-passes-scoped-correctness-await-fixed-costs'
    state = source_state()
    preparation, _, entries, _ = state
    verify_sources(state)
    OUT.mkdir(exist_ok=False)
    WORK.mkdir(exist_ok=False)
    (WORK / 'bins').mkdir()
    variants = {'baseline': {'source': preparation['control_source'], 'target': str(WORK / 'baseline-target')},
                'candidate': {'source': preparation['source'], 'target': str(WORK / 'candidate-target')}}
    env = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
           'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never'}
    cpu = min(os.sched_getaffinity(0))
    protocol = {'scope': __doc__, 'runner_sha256': sha(Path(__file__)), 'assessor_sha256': sha(E / 'assess_cost_v1.py'),
                'probe_sha256': sha(E / 'compaction_cost_probe_v1.rs'),
                'correctness_sha256': sha(E / 'correctness-v1.json'), 'qualification_result_sha256': sha(E / 'qualification-v5/result.json'),
                'preparation_sha256': sha(E / 'preparation-v4/protocol.json'), 'baseline_protocol_sha256': sha(E / 'baseline-v1/protocol.json'),
                'source_manifests_sha256': {manifest: sha(E / manifest) for _, manifest in entries},
                'variants': variants, 'families': FAMILIES, 'phases': PHASES, 'fixed_block_order': BLOCKS,
                'time_inputs': 4096, 'memory_inputs': 1000, 'preflight_inputs': 42,
                'preflight_processes': 4, 'measured_processes': 20, 'rows_per_mode': 60, 'witnesses_per_process': 45,
                'cpu': cpu, 'environment': env, 'inherited_rustflags': os.environ.get('RUSTFLAGS'),
                'compiler': subprocess.check_output(['rustc', '-vV'], text=True), 'host': tuple(os.uname()),
                'cpuinfo': Path('/proc/cpuinfo').read_text().split('\n\n')[0], 'timeout_seconds': 1800,
                'limits': 'Sources are constructed and proved before first compaction; first does not mean a process-wide cold cache. Equality checks and fact observations occur outside timed/allocation phases and before subsequent warm/cloned phases. Counts describe requested Rust allocations. Probe size includes linked artifact/path effects and is not production executable size.',
                'runtime_admitted': False}
    (OUT / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    commands, libraries, binaries, artifacts, trials, preflights = [], {}, {}, {}, [], []

    def verify():
        assert sha(Path(__file__)) == protocol['runner_sha256']
        assert sha(E / 'assess_cost_v1.py') == protocol['assessor_sha256']
        assert sha(E / 'compaction_cost_probe_v1.rs') == protocol['probe_sha256']
        for name, key in [('correctness-v1.json', 'correctness_sha256'), ('qualification-v5/result.json', 'qualification_result_sha256'),
                          ('preparation-v4/protocol.json', 'preparation_sha256'), ('baseline-v1/protocol.json', 'baseline_protocol_sha256')]:
            assert sha(E / name) == protocol[key]
        assert all(sha(E / name) == digest for name, digest in protocol['source_manifests_sha256'].items())
        verify_sources(state)
        assert all(sha(Path(row['path'])) == row['sha256'] for row in list(libraries.values()) + list(binaries.values()))
        assert all(sha(Path(name)) == digest for entries in artifacts.values() for name, digest in entries.items())

    def run(name, command, cwd=ROOT, extra_env=None):
        verify()
        wrapped = ['timeout', '--signal=TERM', '--kill-after=10s', '1800', *command]
        print(json.dumps({'start': name}), flush=True)
        started = time.monotonic()
        with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(wrapped, cwd=cwd, env=os.environ | env | (extra_env or {}), stdout=stdout, stderr=stderr).returncode
        row = {'name': name, 'command': wrapped, 'cwd': str(cwd), 'extra_environment': extra_env or {}, 'exit_code': code,
               'seconds': time.monotonic() - started, 'stdout_sha256': sha(OUT / (name + '.stdout')),
               'stderr_sha256': sha(OUT / (name + '.stderr'))}
        commands.append(row)
        (OUT / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
        print(json.dumps({key: row[key] for key in ['name', 'exit_code', 'seconds']}), flush=True)
        verify()
        assert code == 0, name
        return (OUT / (name + '.stdout')).read_text()

    for variant, item in variants.items():
        source, target = Path(item['source']), Path(item['target'])
        raw = run('build-' + variant, ['cargo', 'build', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--lib', '--message-format=json'],
                  source, {'CARGO_TARGET_DIR': str(target)})
        events = [event for event in map(json.loads, raw.splitlines()) if event.get('reason') == 'compiler-artifact']
        matches = [event for event in events if event['target']['name'] == 'hyperreal' and event['target']['kind'] == ['lib']
                   and any(path.endswith('.rlib') for path in event['filenames'])]
        assert len(matches) == 1 and not matches[0]['profile']['test']
        event = matches[0]
        library = Path(next(path for path in event['filenames'] if path.endswith('.rlib')))
        assert library.is_relative_to(target) and Path(event['manifest_path']) == source / 'crates/hyperreal/Cargo.toml'
        libraries[variant] = {'path': str(library), 'sha256': sha(library), 'bytes': library.stat().st_size,
                              'manifest_path': event['manifest_path'], 'features': event['features'], 'profile': event['profile']}
        artifacts[variant] = {name: sha(Path(name)) for artifact in events for name in artifact['filenames'] if name.endswith(('.rlib', '.so'))}
        (OUT / 'libraries.json').write_text(json.dumps(libraries, indent=2) + '\n')
        (OUT / 'build-artifacts.json').write_text(json.dumps(artifacts, indent=2) + '\n')
    assert libraries['baseline']['features'] == libraries['candidate']['features']
    for variant, item in variants.items():
        for mode in ['time', 'memory']:
            label = variant + '-' + mode
            binary = WORK / 'bins' / label
            command = ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(E / 'compaction_cost_probe_v1.rs'),
                       '-L', 'dependency=' + str(Path(item['target']) / 'release/deps'), '--extern', 'hyperreal=' + libraries[variant]['path'], '-o', str(binary)]
            if mode == 'memory':
                command += ['--cfg', 'feature="count-alloc"']
            run('link-' + label, command)
            binaries[label] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
            (OUT / 'binaries.json').write_text(json.dumps(binaries, indent=2) + '\n')
    for mode in ['time', 'memory']:
        assert binaries['baseline-' + mode]['sha256'] != binaries['candidate-' + mode]['sha256']
    for label, binary in binaries.items():
        raw = run('preflight-' + label, ['taskset', '-c', str(cpu), binary['path'], '42'])
        preflights.append({'label': label, 'rows': read_rows(raw, 42, label.endswith('memory'))})
        (OUT / 'preflights.json').write_text(json.dumps(preflights, indent=2) + '\n')
    for mode, count in [('time', 4096), ('memory', 1000)]:
        identity = None
        for block, order in enumerate(BLOCKS, 1):
            for variant in order:
                name = f'{mode}-{block}-{variant}'
                raw = run(name, ['taskset', '-c', str(cpu), '/usr/bin/time', '-v', binaries[variant + '-' + mode]['path'], str(count)])
                rows = read_rows(raw, count, mode == 'memory')
                current = [{field: row[field] for field in IDENTITY_FIELDS} for row in rows if 'witness' in row]
                if identity is None:
                    identity = current
                assert current == identity
                trials.append({'name': name, 'mode': mode, 'block': block, 'variant': variant, 'rows': rows})
                (OUT / 'trials.json').write_text(json.dumps(trials, indent=2) + '\n')
    assert len(trials) == 20 and len(preflights) == 4
    verify()
    result = {'complete': True, 'fixed_samples_closed': True, 'preflight_processes': 4, 'measured_processes': 20,
              'mathematical_witnesses_match_within_each_mode': True, 'source_main_inputs_libraries_and_binaries_unchanged': True,
              'runtime_admitted': False, 'next_step': 'Audit every timing/allocation row and all fact strengthening before a scoped retention decision.'}
    (OUT / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
