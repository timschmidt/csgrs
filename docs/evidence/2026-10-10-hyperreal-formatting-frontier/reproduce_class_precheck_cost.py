"""Compare equal checked-formatting contracts with a fixed five-sample plan."""
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'class-precheck-cost'
WORK = ROOT / 'target/hyperreal-class-precheck-cost'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
assert read(EVIDENCE / 'class-precheck-validation.json')['differences_from_first_candidate'] == 0
sources = {'first': read(EVIDENCE / 'checked-scientific/sources.json'), 'class_precheck': read(EVIDENCE / 'class-precheck/sources.json')}
maps = {}
for variant, record in sources.items():
    path = Path(record['source_hashes_file'])
    assert digest(path) == record['source_hashes_file_sha256']
    maps[variant] = json.loads(gzip.decompress(path.read_bytes()))
native = read(Path(sources['first']['main_source_record']))['source_hashes']
libraries = {'first': read(EVIDENCE / 'checked-frontier/library.json'), 'class_precheck': read(EVIDENCE / 'class-precheck-validation/library.json')}
OUT.mkdir()
WORK.mkdir()
shutil.copyfile(EVIDENCE / 'checked_cost_probe.rs', OUT / 'probe.rs')
shutil.copyfile(__file__, OUT / 'reproduce.py')
probe_hash = digest(OUT / 'probe.rs')
for variant, record in libraries.items():
    original = Path(record['retained_path'])
    assert digest(original) == record['sha256']
    copied = WORK / ('libhyperreal_' + variant + '.rlib')
    shutil.copyfile(original, copied)
    record['linked_path'] = str(copied)
(OUT / 'libraries.json').write_text(json.dumps(libraries, indent=2) + '\n')
plan = {
    'scope': 'Matched comparison of the first checked formatter and a separate exact-class zero precheck, with identical public rounding, failure and refinement contracts.',
    'rows': 'All 28 original checked rows: eleven successful families and three typed-failure families, with first and repeated queries. Every result is checked in every iteration.',
    'samples_each_variant_and_mode': 5, 'order': 'alternating variants; all builds complete before any sample',
    'timing_iterations': 8192, 'allocation_iterations': 1000, 'cpu_affinity': [0],
    'allocation_scope': 'Requested Rust allocations/reallocations, retained and peak requested bytes. Allocator overhead and native heaps are excluded.',
    'source_and_probe_frozen': True, 'timing_probe_has_no_allocation_counter': True,
    'sampling_stop': 'One fixed five-pair set for all rows. Retain every sample and adverse result; do not extend sampling in response to outcome.',
    'limitations': 'Input construction is outside measured rows; constants can already be warm. Equality/checksum overhead is included equally. This does not measure downstream callers or establish a total-work bound.',
}
(OUT / 'plan.json').write_text(json.dumps(plan, indent=2) + '\n')
commands, trials, binaries = [], [], {}


def verify():
    for variant, record in sources.items():
        assert all(digest(Path(record['source_directory']) / name) == value for name, value in maps[variant].items())
        assert digest(Path(libraries[variant]['linked_path'])) == libraries[variant]['sha256']
    assert all(digest(ROOT / name) == value for name, value in native.items())
    assert digest(OUT / 'probe.rs') == probe_hash


def run(name, command):
    verify()
    print('start', name, flush=True)
    start = time.monotonic()
    with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
        result = subprocess.run(['timeout', '--signal=TERM', '--kill-after=5s', '300', *command], cwd=ROOT, stdout=stdout, stderr=stderr, check=False)
    commands.append({'name': name, 'command': command, 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start})
    (OUT / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
    verify()
    print('finish', name, result.returncode, flush=True)
    assert result.returncode == 0, name
    return (OUT / (name + '.stdout')).read_text()


run('format', ['rustfmt', '--edition', '2024', '--check', str(OUT / 'probe.rs')])
for variant, source in sources.items():
    for mode in ['time', 'memory']:
        binary = WORK / (variant + '-' + mode)
        command = ['rustc', '--edition=2024', '-O', '-D', 'warnings', '--check-cfg', 'cfg(feature, values("checked-api", "count-alloc"))', '--cfg', 'feature="checked-api"', str(OUT / 'probe.rs'), '-L', 'dependency=' + str(Path(source['target_directory']) / 'release/deps'), '--extern', 'hyperreal=' + libraries[variant]['linked_path'], '-o', str(binary)]
        if mode == 'memory':
            command += ['--cfg', 'feature="count-alloc"']
        run('build-' + variant + '-' + mode, command)
        binaries[variant + '-' + mode] = {'path': str(binary), 'sha256': digest(binary), 'bytes': binary.stat().st_size}
(OUT / 'binaries.json').write_text(json.dumps(binaries, indent=2) + '\n')
for mode, iterations in [('time', 8192), ('memory', 1000)]:
    for index in range(1, 6):
        for variant in (['first', 'class_precheck'] if index % 2 else ['class_precheck', 'first']):
            binary = binaries[variant + '-' + mode]
            assert digest(Path(binary['path'])) == binary['sha256']
            raw = run(f'{variant}-{mode}-{index}', ['taskset', '-c', '0', binary['path'], str(iterations)])
            rows = [json.loads(line) for line in raw.splitlines()]
            assert len(rows) == 28
            assert all(row['iterations'] == iterations and row['allocation_accounting'] == (mode == 'memory') for row in rows)
            trials.append({'variant': variant, 'mode': mode, 'index': index, 'rows': rows})
            (OUT / 'trials.json').write_text(json.dumps(trials, indent=2) + '\n')
verify()
assert all(digest(Path(binary['path'])) == binary['sha256'] for binary in binaries.values())
(OUT / 'source-verification.json').write_text(json.dumps({'all_source_library_probe_and_binary_hashes_unchanged': True, 'production_inputs': {variant: len(values) for variant, values in maps.items()}, 'main_files': len(native), 'samples': len(trials), 'no_builds_or_tests_during_measurement': True, 'sampling_complete': True}, indent=2) + '\n')
print('Exact-class precheck samples complete; every planned result retained.', flush=True)
