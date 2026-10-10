"""Early matched resource gate using the frozen production libraries only."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import statistics
import subprocess
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
PRODUCTION = EVIDENCE / 'arc-endpoint-certificate'
OUT = EVIDENCE / 'arc-endpoint-cost'
WORK = ROOT / 'target/arc-endpoint-cost'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
assert read(EVIDENCE / 'diagnostics-77.json')['identical_query_receipts_per_variant'] == 445564
assert read(EVIDENCE / 'diagnostics-77.json')['tests']['candidate']['segment-regressions-corrected']['passed'] == 2
records = {variant: read(PRODUCTION / (variant + '-sources.json')) for variant in ['baseline', 'candidate']}
hashes = {}
for variant, record in records.items():
    path = Path(record['source_hashes_file'])
    assert digest(path) == record['source_hashes_file_sha256']
    hashes[variant] = json.loads(gzip.decompress(path.read_bytes()))
native = read(EVIDENCE / 'native-exactness-gate/sources.json')['source_hashes']
OUT.mkdir()
WORK.mkdir()
(WORK / 'measured-libraries').mkdir()
shutil.copyfile(__file__, OUT / 'reproduce.py')
shutil.copyfile(EVIDENCE / 'arc_endpoint_cost_probe.rs', OUT / 'probe.rs')
probe_hash = digest(OUT / 'probe.rs')
commands, trials, binaries, libraries, features = [], [], {}, {}, {}


def verify():
    for variant, record in records.items():
        source = Path(record['source_directory'])
        assert all(digest(source / name) == value for name, value in hashes[variant].items())
    assert all(digest(ROOT / name) == value for name, value in native.items())
    assert digest(OUT / 'probe.rs') == probe_hash
    for crates in libraries.values():
        assert all(digest(Path(row['path'])) == row['sha256'] for row in crates.values())


def run(name, command, source=ROOT, env=None):
    verify()
    print('start', name, flush=True)
    start = time.monotonic()
    with (OUT / (name + '.jsonl')).open('w') as stdout, (OUT / (name + '.log')).open('w') as stderr:
        result = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', '1800', *command], cwd=source, env=env, stdout=stdout, stderr=stderr, check=False)
    commands.append({'name': name, 'command': command, 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start})
    (OUT / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
    verify()
    print('finish', name, result.returncode, flush=True)
    assert result.returncode == 0, name
    return (OUT / (name + '.jsonl')).read_text()


for variant, record in records.items():
    env = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache', 'CARGO_NET_OFFLINE': 'true', 'CARGO_TARGET_DIR': record['target_directory']}
    raw = run(variant + '-library', ['cargo', 'build', '--locked', '--release', '-p', 'hypercurve', '--features', 'triangulation', '--lib', '--message-format=json'], Path(record['source_directory']), env)
    libraries[variant], features[variant] = {}, {}
    for line in raw.splitlines():
        event = json.loads(line)
        if event.get('reason') != 'compiler-artifact' or event['target']['name'] not in ['hypercurve', 'hyperreal']:
            continue
        name = event['target']['name']
        library = Path(next(path for path in event['filenames'] if path.endswith('.rlib')))
        retained = WORK / 'measured-libraries' / (variant + '-' + name + '.rlib')
        shutil.copyfile(library, retained)
        libraries[variant][name] = {'path': str(library), 'retained_path': str(retained), 'sha256': digest(library), 'bytes': library.stat().st_size}
        features[variant][name] = event['features']
    assert len(libraries[variant]) == 2
(OUT / 'libraries.json').write_text(json.dumps(libraries, indent=2) + '\n')
assert features['baseline'] == features['candidate']
for variant, crates in libraries.items():
    for mode in ['time', 'memory']:
        binary = WORK / (variant + '-' + mode)
        command = ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(OUT / 'probe.rs')]
        search = set()
        for name, row in crates.items():
            library = Path(row['path'])
            command += ['--extern', name + '=' + str(library)]
            search.add(library.parent)
            if (library.parent / 'deps').is_dir():
                search.add(library.parent / 'deps')
        for directory in sorted(search):
            command += ['-L', 'dependency=' + str(directory)]
        command += ['-o', str(binary)]
        if mode == 'memory':
            command += ['--cfg', 'feature="count-alloc"']
        run('build-' + variant + '-' + mode, command)
        binaries[variant, mode] = binary
# Every build is terminal before preflight or measurement; no Rust source edits.
for (variant, mode), binary in binaries.items():
    run('preflight-' + variant + '-' + mode, ['taskset', '-c', '0', str(binary), '42'])
for mode, count in [('time', 8192), ('memory', 1000)]:
    for index in range(1, 6):
        for variant in (['baseline', 'candidate'] if index % 2 else ['candidate', 'baseline']):
            label = f'{mode}-{index}-{variant}'
            raw = run(label, ['taskset', '-c', '0', '/usr/bin/time', '-v', str(binaries[variant, mode]), str(count)])
            rows = [json.loads(line) for line in raw.splitlines()]
            assert len(rows) == 96
            assert sum('name' in row for row in rows) == 48
            assert sum('witness' in row for row in rows) == 36
            assert sum('fixture' in row for row in rows) == 12
            rss = int(re.search(r'Maximum resident set size \(kbytes\): (\d+)', (OUT / (label + '.log')).read_text())[1])
            trials.append({'variant': variant, 'mode': mode, 'index': index, 'rows': rows, 'maximum_rss_kib': rss})
            (OUT / 'trials.json').write_text(json.dumps(trials, indent=2) + '\n')
for mode in ['time', 'memory']:
    selected = [trial for trial in trials if trial['mode'] == mode]
    witness = lambda trial: [row for row in trial['rows'] if 'witness' in row or 'fixture' in row]
    assert all(witness(trial) == witness(selected[0]) for trial in selected)
summaries = []
for mode in ['time', 'memory']:
    for name in [row['name'] for row in trials[0]['rows'] if 'name' in row]:
        fields = ['nanos'] if mode == 'time' else ['allocation_calls', 'allocated_bytes', 'retained_bytes_delta', 'peak_bytes_delta']
        variants = {}
        for variant in ['baseline', 'candidate']:
            rows = [next(row for row in trial['rows'] if row.get('name') == name) for trial in trials if trial['variant'] == variant and trial['mode'] == mode]
            variants[variant] = {field: {'samples': [row[field] for row in rows], 'median': statistics.median(row[field] for row in rows)} for field in fields}
        summaries.append({'mode': mode, 'name': name, 'variants': variants})
binary_records = {}
for (variant, mode), binary in binaries.items():
    sections = subprocess.check_output(['size', '-A', str(binary)], text=True)
    (OUT / (variant + '-' + mode + '-size.log')).write_text(sections)
    binary_records[variant + '-' + mode] = {'path': str(binary), 'bytes': binary.stat().st_size, 'sha256': digest(binary), 'sections': sections}
verify()
manifest = {
    'scope': 'Five fresh processes per variant/mode, alternating order on CPU 0. Twelve public arc-membership families with construction and first/warm/cloned queries. Production libraries contain no diagnostic Rust. All builds precede all measurements.',
    'focused_gate': 'diagnostics-77.json', 'source_records': records,
    'all_production_and_main_source_hashes_unchanged': True,
    'probe_sha256': probe_hash, 'libraries': libraries, 'features': features,
    'compiler': subprocess.check_output(['rustc', '-Vv'], text=True), 'host': tuple(os.uname()), 'cpuinfo': Path('/proc/cpuinfo').read_text().split('\n\n')[0],
    'commands': commands, 'trials': trials, 'summaries': summaries, 'binaries': binary_records,
    'limitations': 'An early resource gate, not full application admission. First means newly constructed fixture graphs in that process, with possibly shared warm constants. Cloned query timing excludes handle cloning. Timing has no counting allocator; memory counts requested Rust bytes, excludes allocator metadata and native C heaps, and includes retained output vectors. Uncertain and Approximate512Consumed results remain explicit witnesses. Five-sample min/max ranges are descriptive, not confidence intervals. Full suites, historical witnesses and report replay remain required if this gate is favorable.',
}
(OUT / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
print('Endpoint production resource trial complete; inspect all tradeoffs before retention.', flush=True)
