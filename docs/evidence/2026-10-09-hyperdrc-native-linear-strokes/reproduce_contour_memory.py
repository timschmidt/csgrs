"""Run only after all validation and timing jobs have exited.

Usage: python3 reproduce_contour_memory.py CANDIDATE_EVIDENCE NEW_OUTPUT_DIRECTORY
Uses the separately validated production source plus conic-frame repair as its
baseline. Rebuild that source and adjust the recorded paths on another host.
This compares the combined candidate, not one optimization in isolation.
"""

import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

candidate, out = map(Path, sys.argv[1:])
baseline = Path(__file__).resolve().parent / 'conic-frame-fix-isolated'
baseline_source = Path('/tmp/drc-linear-conic-frame-fix-isolated/source')
probe = Path(__file__).resolve().parent / 'contour_memory_probe.rs'
out.mkdir(parents=True, exist_ok=False)


def info(path):
    return dict(bytes=path.stat().st_size,
                sha256=hashlib.sha256(path.read_bytes()).hexdigest())


records = {name: json.loads((root / 'sources.json').read_text())
           for name, root in [('baseline', baseline), ('candidate', candidate)]}


def verify():
    for name, root in [('baseline', baseline_source), ('candidate', Path.cwd())]:
        assert all(info(root / path)['sha256'] == digest
                   for path, digest in records[name]['source_hashes'].items())


def library(root):
    artifacts = []
    for line in (root / 'build-board-smoke.log').read_text().splitlines():
        try:
            artifact = json.loads(line)
        except ValueError:
            continue
        if (artifact.get('reason') == 'compiler-artifact'
                and artifact.get('target', {}).get('name') == 'hypercurve'):
            artifacts.extend(Path(p) for p in artifact['filenames'] if p.endswith('.rlib'))
    assert len(artifacts) == 1, artifacts
    return artifacts[0]


verify()
variants = {}
for name, root in [('baseline', baseline), ('candidate', candidate)]:
    folder = out / name
    folder.mkdir()
    linked = library(root)
    binary = folder / 'contour_memory_probe'
    command = ['rustc', '--edition=2024', '-O', str(probe), '-L',
               'dependency=' + str(linked.parent), '--extern',
               'hypercurve=' + str(linked), '-o', str(binary)]
    with (folder / 'build.log').open('w') as log:
        code = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT).returncode
    (folder / 'build.json').write_text(json.dumps(dict(command=command, exit_code=code),
                                                  indent=2) + '\n')
    assert code == 0
    trials = []
    for index in range(1, 6):
        command = ['taskset', '-c', '0', '/usr/bin/time', '-v', str(binary)]
        stdout = folder / f'trial-{index}.jsonl'
        stderr = folder / f'trial-{index}.log'
        with stdout.open('w') as output, stderr.open('w') as log:
            code = subprocess.run(command, stdout=output, stderr=log).returncode
        rows = [json.loads(line) for line in stdout.read_text().splitlines()]
        assert code == 0 and len(rows) == 37
        assert all(row['witness'] and row['iterations'] == 1000 for row in rows[1:])
        rss = int(re.search(r'Maximum resident set size \(kbytes\): (\d+)',
                            stderr.read_text())[1])
        trials.append(dict(command=command, exit_code=code, sizes=rows[0],
                           rows=rows[1:], maximum_rss_kib=rss))
        print(name, index, 'passed', flush=True)
    assert all(trial['sizes'] == trials[0]['sizes'] for trial in trials)
    variants[name] = dict(trials=trials, binary={str(binary): info(binary)},
                          library={str(linked): info(linked)})
verify()
for before, after in zip(variants['baseline']['trials'][0]['rows'],
                         variants['candidate']['trials'][0]['rows']):
    assert (before['name'], before['iterations'], before['witness']) == (
        after['name'], after['iterations'], after['witness'])
(out / 'manifest.json').write_text(json.dumps(dict(
    scope='Five serialized fresh processes per variant, CPU 0. Six fixtures, each with 1000 independent contours built from shared immutable segment/scalar inputs; construction, first inside query, warm inside/outside queries, warm clone storage, and clone query. Winding oracles are 1 for each simple square/circle, 8 for eight CCW traversals, and 0 outside. Requested Rust allocator calls/bytes exclude allocator metadata and native C/GMP heaps. Whole-process RSS and probe executable bytes are separate observations. This is not a matched timing or library-size claim, nor an attribution to a single cache.',
    baseline_production_revision='0e9fd3e21',
    baseline_source_record=str(baseline / 'sources.json'),
    candidate_source_record=str(candidate / 'sources.json'),
    source_unchanged=True, probe=info(probe), variants=variants,
), indent=2) + '\n')
