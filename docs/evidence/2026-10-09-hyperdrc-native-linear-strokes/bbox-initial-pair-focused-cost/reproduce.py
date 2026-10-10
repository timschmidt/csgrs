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
OUT = EVIDENCE / 'bbox-initial-pair-focused-cost'
CANDIDATE = EVIDENCE / 'bbox-initial-pair-candidate'
SOURCE = ROOT / 'target/native-bbox-initial-pair/source'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
source = read(CANDIDATE / 'sources.json')
native = read(Path(source['native_source_record']))
status = read(CANDIDATE / 'status.json')
assert len(status) == 6 and all(command['exit_code'] == 0 for command in status)
OUT.mkdir()
(OUT / '.gitignore').write_text('/baseline-time\n/candidate-time\n')
shutil.copyfile(__file__, OUT / 'reproduce.py')
shutil.copyfile(EVIDENCE / 'bbox_initial_pair_focused_probe.rs', OUT / 'probe.rs')
commands = []
libraries = {}
ENV = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache', 'CARGO_NET_OFFLINE': 'true', 'CARGO_TARGET_DIR': str(ROOT / 'target')}

def verify():
    assert all(digest(SOURCE / name) == value for name, value in source['source_hashes'].items())
    assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
    assert all(digest(Path(record['path'])) == record['sha256'] for record in libraries.values())

def run(name, command, *, env=None):
    verify()
    start = time.monotonic()
    print('start', name, flush=True)
    with (OUT / (name + '.log')).open('w') as stderr, (OUT / (name + '.jsonl')).open('w') as stdout:
        result = subprocess.run(command, cwd=ROOT, env=env, stdout=stdout, stderr=stderr, check=False)
    commands.append({'name': name, 'command': command, 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start})
    (OUT / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
    verify()
    assert result.returncode == 0, name
    print('finish', name, flush=True)
    return (OUT / (name + '.jsonl')).read_text()

previous = read(EVIDENCE / 'bbox-initial-pair-cost-corrected/manifest.json')
libraries = previous['libraries']
verify()
binaries = {}
for variant, library in libraries.items():
    for mode in ['time']:
        path = Path(library['path'])
        binary = OUT / (variant + '-' + mode)
        command = ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(OUT / 'probe.rs'), '-L', 'dependency=' + str(path.parent), '--extern', 'hypercurve=' + str(path), '-o', str(binary)]
        if (path.parent / 'deps').is_dir(): command += ['-L', 'dependency=' + str(path.parent / 'deps')]
        if mode == 'memory': command += ['--cfg', 'feature="count-alloc"']
        run('build-' + variant + '-' + mode, command)
        binaries[variant, mode] = binary
        run('preflight-' + variant + '-' + mode, ['taskset', '-c', '0', str(binary), '8', '0', '2'])

trials = []
cases = [('rational-pair-original-batch', 0, 2, 1024), ('rational-pair-large-batch', 0, 2, 32768), ('pi-eight-large-batch', 1, 8, 4096)]
for case, kind, size, count in cases:
    for index in range(1, 6):
        for variant in (['baseline', 'candidate'] if index % 2 else ['candidate', 'baseline']):
            name = f'{case}-{index}-{variant}'
            output = run(name, ['taskset', '-c', '0', '/usr/bin/time', '-v', str(binaries[variant, 'time']), str(count), str(kind), str(size)])
            rows = [json.loads(line) for line in output.splitlines()]
            assert len(rows) == 7
            assert len([row for row in rows if 'witness' in row]) == 3
            rss = int(re.search(r'Maximum resident set size \(kbytes\): (\d+)', (OUT / (name + '.log')).read_text())[1])
            trials.append({'case': case, 'index': index, 'variant': variant, 'rows': rows, 'maximum_rss_kib': rss})
for case, _, _, _ in cases:
    rows = [trial for trial in trials if trial['case'] == case]
    first = [row for row in rows[0]['rows'] if 'witness' in row]
    assert all([row for row in trial['rows'] if 'witness' in row] == first for trial in rows)

summaries = []
for case, _, _, count in cases:
    first = next(trial for trial in trials if trial['case'] == case)
    for name in [row['name'] for row in first['rows'] if 'name' in row]:
        stats = {}
        for variant in ['baseline', 'candidate']:
            rows = [next(row for row in trial['rows'] if row.get('name') == name) for trial in trials if trial['case'] == case and trial['variant'] == variant]
            stats[variant] = {'nanos': {'samples': [row['nanos'] for row in rows], 'median': statistics.median(row['nanos'] for row in rows)}}
        summaries.append({'case': case, 'name': name, 'iterations': count, 'variants': stats})

binary_records = {}
for (variant, mode), binary in binaries.items():
    sections = subprocess.check_output(['size', '-A', str(binary)], text=True)
    (OUT / (variant + '-' + mode + '-size.log')).write_text(sections)
    binary_records[variant + '-' + mode] = {'bytes': binary.stat().st_size, 'sha256': digest(binary), 'sections': sections}
manifest = {
    'scope': 'Five alternating fresh processes per version and case, CPU 0. Each process constructs and queries only one coordinate-family/point-count combination, avoiding allocator history from earlier singleton rows. Rational pairs repeat the original 1024 count and a longer 32768 count; pi-translated eight-point inputs use 4096. These focused follow-ups preserve the complete initial matrix evidence and do not replace full fixtures. System allocator timing only; original allocation measurements remain in the preceding record.',
    'candidate_source_record': str(CANDIDATE / 'sources.json'),
    'baseline_source_record': source['native_source_record'],
    'source_unchanged': True,
    'probe_sha256': digest(OUT / 'probe.rs'),
    'earlier_cost_record': str(EVIDENCE / 'bbox-initial-pair-cost-corrected/manifest.json'),
    'cases': cases,
    'libraries': libraries, 'binaries': binary_records,
    'compiler': subprocess.check_output(['rustc', '-Vv'], text=True),
    'cpuinfo': Path('/proc/cpuinfo').read_text().split('\n\n')[0],
    'host': tuple(os.uname()), 'commands': commands, 'trials': trials, 'summaries': summaries,
}
(OUT / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
print('Focused bounding-box initial-pair timing complete', flush=True)
