"""Matched first/warm/cloned public Aabb2 queries after focused gates complete."""
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
OUT = EVIDENCE / 'bbox-initial-pair-production-cost'
CANDIDATE = EVIDENCE / 'bbox-initial-pair-production'
SOURCE = ROOT / 'target/bbox-initial-pair-production/source'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
source = read(CANDIDATE / 'sources.json')
native = read(Path(source['main_source_record']))
status = read(CANDIDATE / 'status.json')
assert len(status) == 8 and all(command['exit_code'] == 0 for command in status)
OUT.mkdir()
(OUT / '.gitignore').write_text('/baseline-time\n/baseline-memory\n/candidate-time\n/candidate-memory\n')
shutil.copyfile(__file__, OUT / 'reproduce.py')
shutil.copyfile(EVIDENCE / 'bbox_initial_pair_production_probe.rs', OUT / 'probe.rs')
baseline = read(Path(source['base_source_record']))
BASE_SOURCE = Path('/tmp/drc-area-compaction-final/source')
commands = []
libraries = {}
ENV = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache', 'CARGO_NET_OFFLINE': 'true', 'CARGO_TARGET_DIR': str(ROOT / 'target')}

def verify():
    assert all(digest(SOURCE / name) == value for name, value in source['source_hashes'].items())
    assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
    assert all(digest(BASE_SOURCE / name) == value for name, value in baseline['source_hashes'].items())
    assert all(digest(Path(record['path'])) == record['sha256'] for record in libraries.values())

def run(name, command, *, env=None, cwd=ROOT):
    verify()
    start = time.monotonic()
    print('start', name, flush=True)
    with (OUT / (name + '.log')).open('w') as stderr, (OUT / (name + '.jsonl')).open('w') as stdout:
        result = subprocess.run(command, cwd=cwd, env=env, stdout=stdout, stderr=stderr, check=False)
    commands.append({'name': name, 'command': command, 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start})
    (OUT / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
    verify()
    assert result.returncode == 0, name
    print('finish', name, flush=True)
    return (OUT / (name + '.jsonl')).read_text()

features = 'triangulation,hyperreal/default,hyperreal/serde'
for variant, directory, target in [('baseline', BASE_SOURCE, Path('/tmp/drc-area-compaction-final/target')), ('candidate', SOURCE, ROOT / 'target/bbox-initial-pair-production/target')]:
    run('build-' + variant + '-library', ['cargo', 'build', '--locked', '--release', '-p', 'hypercurve', '--features', features, '--message-format=json'], env=ENV | {'CARGO_TARGET_DIR': str(target)}, cwd=directory)
for variant, path in [('baseline', OUT / 'build-baseline-library.jsonl'), ('candidate', OUT / 'build-candidate-library.jsonl')]:
    matches = []
    for line in path.read_text().splitlines():
        if not line.startswith('{'): continue
        event = json.loads(line)
        if event.get('reason') == 'compiler-artifact' and event['target']['name'] == 'hypercurve':
            matches.extend(Path(name) for name in event['filenames'] if name.endswith('.rlib'))
    assert len(matches) == 1
    library = matches[0]
    libraries[variant] = {'path': str(library), 'sha256': digest(library), 'bytes': library.stat().st_size}

binaries = {}
for variant, library in libraries.items():
    for mode in ['time', 'memory']:
        path = Path(library['path'])
        binary = OUT / (variant + '-' + mode)
        command = ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(OUT / 'probe.rs'), '-L', 'dependency=' + str(path.parent), '--extern', 'hypercurve=' + str(path), '-o', str(binary)]
        if (path.parent / 'deps').is_dir(): command += ['-L', 'dependency=' + str(path.parent / 'deps')]
        if mode == 'memory': command += ['--cfg', 'feature="count-alloc"']
        run('build-' + variant + '-' + mode, command)
        binaries[variant, mode] = binary
        run('preflight-' + variant + '-' + mode, ['taskset', '-c', '0', str(binary), '8'])

trials = []
for mode, count in [('time', 1024), ('memory', 256)]:
    for index in range(1, 6):
        for variant in (['baseline', 'candidate'] if index % 2 else ['candidate', 'baseline']):
            name = f'{mode}-{index}-{variant}'
            output = run(name, ['taskset', '-c', '0', '/usr/bin/time', '-v', str(binaries[variant, mode]), str(count)])
            rows = [json.loads(line) for line in output.splitlines()]
            assert len(rows) == 168
            assert len([row for row in rows if 'witness' in row]) == 72
            rss = int(re.search(r'Maximum resident set size \(kbytes\): (\d+)', (OUT / (name + '.log')).read_text())[1])
            trials.append({'mode': mode, 'index': index, 'variant': variant, 'rows': rows, 'maximum_rss_kib': rss})
for mode in ['time', 'memory']:
    rows = [trial for trial in trials if trial['mode'] == mode]
    first = [row for row in rows[0]['rows'] if 'witness' in row]
    assert all([row for row in trial['rows'] if 'witness' in row] == first for trial in rows)

summaries = []
for mode in ['time', 'memory']:
    fields = ['nanos'] if mode == 'time' else ['allocation_calls', 'allocated_bytes', 'retained_bytes_delta', 'peak_bytes_delta']
    for name in [row['name'] for row in trials[0]['rows'] if 'name' in row]:
        stats = {}
        for variant in ['baseline', 'candidate']:
            rows = [next(row for row in trial['rows'] if row.get('name') == name) for trial in trials if trial['mode'] == mode and trial['variant'] == variant]
            stats[variant] = {field: {'samples': [row[field] for row in rows], 'median': statistics.median(row[field] for row in rows)} for field in fields}
        summaries.append({'mode': mode, 'name': name, 'variants': stats})

binary_records = {}
for (variant, mode), binary in binaries.items():
    sections = subprocess.check_output(['size', '-A', str(binary)], text=True)
    (OUT / (variant + '-' + mode + '-size.log')).write_text(sections)
    binary_records[variant + '-' + mode] = {'bytes': binary.stat().st_size, 'sha256': digest(binary), 'sections': sections}
manifest = {
    'scope': 'Five fresh alternating processes per variant/mode, CPU 0, after fixed unmeasured 100 ms integer-only CPU conditioning before geometry construction; timing uses the system allocator, while separate memory binaries count requested Rust bytes. Six affine coordinate families and one/two/three/eight-point inputs; first, warm and cloned public box queries. Verification uses independent integer extrema in positive affine frames, outside measured query spans. Construction includes separately prepared expected values and is an unchanged-work control. Both libraries use admitted production inputs plus the isolated candidate change. This is a focused matched probe, not a full application or cold-start speedup claim.',
    'candidate_source_record': str(CANDIDATE / 'sources.json'),
    'baseline_source_record': source['base_source_record'],
    'source_unchanged': True,
    'probe_sha256': digest(OUT / 'probe.rs'),
    'libraries': libraries, 'binaries': binary_records,
    'compiler': subprocess.check_output(['rustc', '-Vv'], text=True),
    'cpuinfo': Path('/proc/cpuinfo').read_text().split('\n\n')[0],
    'host': tuple(os.uname()), 'commands': commands, 'trials': trials, 'summaries': summaries,
}
(OUT / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
print('Matched bounding-box initial-pair probe complete', flush=True)
