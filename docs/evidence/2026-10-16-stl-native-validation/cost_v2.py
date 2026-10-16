"""Fixed paired, instrumented STL import costs after candidate validation."""
import hashlib
import json
import os
from pathlib import Path
import resource
import statistics
import subprocess
import time

ROOT = Path('/home/tim/Documents/GitHub/workspace/csgrs')
E = ROOT / 'docs/evidence/2026-10-16-stl-native-validation'
WORK = ROOT / 'target/properties/stl-native-validation'
OUT = WORK / 'cost-v2'
VALIDATION = WORK / 'candidate-validation-v2'
OLD = Path('/dev/shm/csgrs-obj-index-contracts')


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


result = json.loads((VALIDATION / 'result.json').read_text())
assert result['complete'] and not result['unexpected_exits']
OUT.mkdir(exist_ok=False)
bindings = {
    'baseline': json.loads((WORK / 'baseline-v1/protocol.json').read_text())['bindings'],
    'candidate': json.loads((VALIDATION / 'probe-bindings.json').read_text()),
}
deps = {'baseline': OLD / 'candidate-target-v1/release/deps',
        'candidate': WORK / 'candidate-target-v2/release/deps'}
libraries = {}
for variant, records in bindings.items():
    for record in records.values():
        assert sha(record['retained']) == record['sha256']
        libraries[record['retained']] = record['sha256']
    for path in sorted(deps[variant].iterdir()):
        if path.is_file() and path.suffix in {'.rlib', '.rmeta', '.so'}:
            libraries[str(path)] = sha(path)
dump(OUT / 'library-hashes.json', libraries)
protocol = {'runner_sha256': sha(__file__), 'probe_sha256': sha(E / 'import_cost_v1.rs'),
            'contract_sha256': sha(E / 'cost-contract-v1.md'), 'bindings': bindings,
            'dependencies': {k: str(v) for k, v in deps.items()},
            'library_hashes_sha256': sha(OUT / 'library-hashes.json'),
            'validation_result_sha256': sha(VALIDATION / 'result.json'),
            'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
            'hardware': subprocess.check_output(['lscpu'], text=True),
            'uname': list(os.uname()), 'pairs': 5,
            'elapsed_limit_seconds_per_process': 120, 'address_space_limit_bytes_per_process': 8 * 1024**3,
            'completed_targets_modified': False, 'optimization_claimed': False}
dump(OUT / 'protocol.json', protocol)
rows = []
environment = os.environ | {'TMPDIR': str(OUT), 'RUST_BACKTRACE': '0'}


def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


def run(name, command, directory):
    start = time.monotonic()
    with (directory / 'stdout').open('wb') as stdout, (directory / 'stderr').open('wb') as stderr:
        process = subprocess.run(['/usr/bin/time', '-v', '-o', str(directory / 'resources'),
                                  'timeout', '--signal=TERM', '--kill-after=10s', '120s', *command],
                                 cwd=ROOT, env=environment, stdout=stdout, stderr=stderr,
                                 preexec_fn=limits, start_new_session=True)
    row = {'name': name, 'command': command, 'exit_code': process.returncode,
           'elapsed_seconds': time.monotonic() - start, 'directory': str(directory),
           **{suffix + '_sha256': sha(directory / suffix) for suffix in ['stdout', 'stderr', 'resources']}}
    rows.append(row)
    dump(OUT / 'commands.json', rows)
    assert process.returncode == 0, name


binaries = {}
for variant, records in bindings.items():
    directory = OUT / ('compile-' + variant)
    directory.mkdir()
    binary = directory / 'import-cost'
    command = ['rustc', '--edition=2024', '-C', 'opt-level=3', '-C', 'debug-assertions=off',
               '-L', 'dependency=' + str(deps[variant])]
    for crate, record in records.items():
        command += ['--extern', crate + '=' + record['retained']]
    run('compile-' + variant, [*command, str(E / 'import_cost_v1.rs'), '-o', str(binary)], directory)
    binaries[variant] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
dump(OUT / 'binaries.json', binaries)
samples = []
for pair in range(5):
    for encoding in ['binary', 'ascii']:
        for cubes in [0, 1, 256]:
            paths = {}
            for variant in (['baseline', 'candidate'] if pair % 2 == 0 else ['candidate', 'baseline']):
                name = f'pair-{pair}-{encoding}-{cubes}-{variant}'
                directory = OUT / name
                directory.mkdir()
                binary = binaries[variant]
                assert sha(binary['path']) == binary['sha256']
                run(name, [binary['path'], str(directory), str(cubes), encoding], directory)
                phases = [json.loads(line) for line in (directory / 'stdout').read_text().splitlines()]
                assert len(phases) == 2 and [r['phase'] for r in phases] == ['first', 'warm']
                assert all(r['completion'] == 'certified' for r in phases)
                assert [r['iterations'] for r in phases] == [1, 4 if cubes == 256 else 64]
                for row in phases:
                    samples.append(row | {'pair': pair, 'encoding': encoding, 'variant': variant,
                                          'input_sha256': sha(directory / 'input.stl'),
                                          'output_sha256': sha(directory / 'output.rows')})
                paths[variant] = directory
            for filename in ['input.stl', 'output.rows']:
                assert (paths['baseline'] / filename).read_bytes() == (paths['candidate'] / filename).read_bytes()
    dump(OUT / 'samples.json', samples)
    print(f'Completed pair {pair + 1}/5: all six complete input/output comparisons equal.', flush=True)
summaries = []
for encoding in ['binary', 'ascii']:
    for cubes in [0, 1, 256]:
        for phase in ['first', 'warm']:
            by_variant = {variant: [r for r in samples if r['variant'] == variant and r['encoding'] == encoding and r['cubes'] == cubes and r['phase'] == phase] for variant in binaries}
            baseline, candidate = by_variant['baseline'], by_variant['candidate']
            assert len(baseline) == len(candidate) == 5
            metrics = {}
            for metric in ['elapsed_ns', 'import_ns', 'allocation_calls', 'allocated_bytes', 'peak_additional_live_bytes', 'retained_delta_bytes']:
                ratios = [b[metric] / a[metric] if a[metric] else None for a, b in zip(baseline, candidate)]
                metrics[metric] = {'baseline_median_per_iteration': statistics.median(r[metric] / r['iterations'] for r in baseline),
                                   'candidate_median_per_iteration': statistics.median(r[metric] / r['iterations'] for r in candidate),
                                   'median_paired_ratio': statistics.median(r for r in ratios if r is not None) if any(r is not None for r in ratios) else None,
                                   'paired_ratios': ratios, 'candidate_greater_count': sum(b[metric] > a[metric] for a, b in zip(baseline, candidate))}
            summaries.append({'encoding': encoding, 'cubes': cubes, 'phase': phase, 'metrics': metrics})
assert all(sha(path) == digest for path, digest in libraries.items())
assert sha(__file__) == protocol['runner_sha256']
assert sha(E / 'import_cost_v1.rs') == protocol['probe_sha256']
assert sha(E / 'cost-contract-v1.md') == protocol['contract_sha256']
dump(OUT / 'result.json', {'complete': True, 'pairs': 5, 'runtime_processes': 60, 'sample_rows': len(samples),
                         'complete_input_and_output_comparisons_equal': True, 'library_inputs_unchanged': True,
                         'binaries': binaries, 'summaries': summaries, 'optimization_claimed': False,
                         'native_runtime_admitted': False, 'full_plan_complete': False})
