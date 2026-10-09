"""Matched public Point2 equality costs after the frozen early validation gate.

Use a new output directory. Baseline libraries were preserved before the trial;
their metadata and the candidate Cargo artifact log identify exact builds.
First means the first comparison of a newly constructed pair in that process,
not an empty global scalar cache. Explicit preparation is a separate cost row.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import statistics
import subprocess
import time

root = Path(__file__).resolve().parent
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('output', type=Path)
parser.add_argument('--candidate-evidence', type=Path, default=root / 'point-equality-candidate')
args = parser.parse_args()
candidate = args.candidate_evidence.resolve()
out = args.output.resolve()
out.mkdir(parents=True, exist_ok=False)
source = json.loads((candidate / 'sources.json').read_text())
baseline = json.loads((root / 'normal-form-ownership-candidate/sources.json').read_text())
changes = [p for p, h in source['source_hashes'].items() if baseline['source_hashes'].get(p) != h]
assert changes == ['crates/hypercurve/src/point/equality_tests.rs', 'crates/hypercurve/src/point.rs']
status = json.loads((candidate / 'status.json').read_text())
assert status[-1]['name'] == 'build-drc-library' and all(r['exit_code'] == 0 for r in status)
base_metadata = Path('/tmp/drc-normal-form-inline-baseline-libraries/libraries.json')
base = json.loads(base_metadata.read_text())
digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
info = lambda p: dict(bytes=p.stat().st_size, sha256=digest(p))
libraries = {'baseline': {name: Path(next(r['path'] for r in base['libraries'] if r['crate'] == name))
                          for name in ['hypercurve', 'hyperreal']}, 'candidate': {}}
features = {'baseline': {}, 'candidate': {}}
for variant, log in [('baseline', root / 'normal-form-ownership-candidate/build-board-smoke.log'),
                     ('candidate', candidate / 'build-drc-library.log')]:
    for line in log.read_text().splitlines():
        try:
            artifact = json.loads(line)
        except ValueError:
            continue
        if artifact.get('reason') == 'compiler-artifact' and artifact['target']['name'] in ['hypercurve', 'hyperreal']:
            name = artifact['target']['name']
            features[variant][name] = artifact['features']
            if variant == 'candidate':
                libraries[variant][name] = Path(next(p for p in artifact['filenames'] if p.endswith('.rlib')))
assert features['baseline'] == features['candidate'] and len(features['candidate']) == 2
def verify_sources():
    assert all(digest(Path(p)) == h for p, h in source['source_hashes'].items())
    assert all(digest(Path(r['path'])) == r['sha256'] for r in base['libraries'])
verify_sources()
shutil.copy2(root / 'point_equality_probe.rs', out / 'probe.rs')
shutil.copy2(base_metadata, out / 'baseline-libraries.json')
shutil.copy2(__file__, out / 'reproduce.py')
commands = []
binaries = {}
trials = []
def run(name, command):
    verify_sources()
    started = time.monotonic()
    with (out / (name + '.jsonl')).open('w') as stdout, (out / (name + '.log')).open('w') as stderr:
        code = subprocess.run(command, stdout=stdout, stderr=stderr).returncode
    commands.append(dict(name=name, command=command, exit_code=code, wall_seconds=time.monotonic() - started))
    (out / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
    verify_sources()
    assert code == 0, name
    return (out / (name + '.jsonl')).read_text()
for variant, crates in libraries.items():
    for mode in ['time', 'memory']:
        binary = out / f'{variant}-{mode}'
        command = ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(out / 'probe.rs')]
        search = set()
        for name, library in crates.items():
            command += ['--extern', name + '=' + str(library)]
            search.add(library.parent)
            if (library.parent / 'deps').is_dir():
                search.add(library.parent / 'deps')
        for path in sorted(search):
            command += ['-L', 'dependency=' + str(path)]
        command += ['-o', str(binary)]
        if mode == 'memory':
            command += ['--cfg', 'feature="count-alloc"']
        run(f'build-{variant}-{mode}', command)
        run(f'preflight-{variant}-{mode}', ['taskset', '-c', '0', str(binary), '42'])
        binaries[variant, mode] = binary
for mode, count in [('time', 4096), ('memory', 1000)]:
    for index in range(1, 6):
        for variant in (('baseline', 'candidate') if index % 2 else ('candidate', 'baseline')):
            label = f'{mode}-{index}-{variant}'
            text = run(label, ['taskset', '-c', '0', '/usr/bin/time', '-v', str(binaries[variant, mode]), str(count)])
            rows = [json.loads(line) for line in text.splitlines()]
            assert len(rows) == 112
            assert sum('name' in r for r in rows) == 56
            assert sum('witness' in r for r in rows) == 40
            assert sum('fixture' in r for r in rows) == 16
            rss = int(re.search(r'Maximum resident set size \(kbytes\): (\d+)', (out / (label + '.log')).read_text())[1])
            trials.append(dict(variant=variant, mode=mode, index=index, rows=rows, maximum_rss_kib=rss))
            print(label, 'passed', flush=True)
for mode in ['time', 'memory']:
    selected = [t for t in trials if t['mode'] == mode]
    witness = lambda t: [r for r in t['rows'] if 'witness' in r or 'fixture' in r]
    assert all(witness(t) == witness(selected[0]) for t in selected)
summaries = []
for mode in ['time', 'memory']:
    for name in [r['name'] for r in trials[0]['rows'] if 'name' in r]:
        fields = ['nanos'] if mode == 'time' else ['allocation_calls', 'allocated_bytes', 'retained_bytes_delta', 'peak_bytes_delta']
        variants = {}
        for variant in ['baseline', 'candidate']:
            rows = [next(r for r in t['rows'] if r.get('name') == name)
                    for t in trials if t['variant'] == variant and t['mode'] == mode]
            variants[variant] = {field: dict(samples=[r[field] for r in rows], median=statistics.median(r[field] for r in rows)) for field in fields}
        summaries.append(dict(mode=mode, name=name, variants=variants))
binary_records = {}
for (variant, mode), path in binaries.items():
    sections = subprocess.check_output(['size', '-A', str(path)], text=True)
    (out / f'{variant}-{mode}-size.log').write_text(sections)
    binary_records[f'{variant}-{mode}'] = dict(**info(path), sections=sections)
manifest = dict(
    scope=__doc__, candidate_validation=str(candidate / 'status.json'),
    source_change=changes, source_unchanged=True,
    baseline_source=str(root / 'normal-form-ownership-candidate/sources.json'),
    candidate_source=str(candidate / 'sources.json'),
    compiler=subprocess.check_output(['rustc', '-Vv'], text=True), host=tuple(os.uname()),
    cpuinfo=Path('/proc/cpuinfo').read_text().split('\n\n')[0],
    features=features, commands=commands, probe_sha256=digest(out / 'probe.rs'),
    libraries={v: {name: dict(path=str(p), **info(p)) for name, p in crates.items()} for v, crates in libraries.items()},
    binaries=binary_records, trials=trials, summaries=summaries,
    limitations='Five fresh processes per version and mode, CPU 0, alternating order. No counting allocator in timing binaries; memory counts requested Rust bytes and excludes allocator metadata/native heaps. Eight point families, including shared-handle and copied-coordinate controls. First is a new pair, not empty global caches. Construction and explicit numerical preparation are measured separately. This scalar/point early gate does not admit full application behavior or production source.',
)
(out / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
print('paired point-equality cost complete', flush=True)
