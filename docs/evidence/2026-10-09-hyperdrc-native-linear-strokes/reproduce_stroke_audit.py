"""Replay stroke capability costs and independent complete-board source oracles.

Usage: python3 reproduce_stroke_audit.py COST_DIRECTORY BASELINE_LIBRARIES_JSON TRACK_ORACLE_DIRECTORY PAD_ORACLE_DIRECTORY NEW_OUTPUT_DIRECTORY
Run after all builds/tests/cost jobs exit. Baseline libraries must be rebuilt
from the recorded revision on other hosts. Different witnesses forbid speedup
claims. Complete source-oracle queries are untimed correctness audits.
"""
from pathlib import Path
import hashlib
import json
import re
import statistics
import subprocess
import sys

cost, baseline_path, track_oracle, pad_oracle, out = map(Path, sys.argv[1:6])
out.mkdir(parents=True, exist_ok=False)
record = Path(__file__).resolve().parent
previous = record.parent / '2026-10-09-hyperdrc-kicad-pad-sources'
load = lambda path: json.loads(path.read_bytes())
info = lambda path: dict(bytes=path.stat().st_size, sha256=hashlib.sha256(path.read_bytes()).hexdigest())
manifest = load(cost / 'manifest.json')
sources = manifest['source_hashes']
current = lambda: {p: info(Path(p))['sha256'] for p in sources}
assert manifest['source_unchanged'] and manifest['fixture_exit_code'] == 0 and current() == sources
baseline = load(baseline_path)
assert all(name in baseline for name in ['hyperreal', 'hyperlattice', 'hyperlimit', 'hypersolve', 'hypercurve', 'hyperdrc'])
baseline_dependency_dirs = sorted({str(Path(library['path']).parent) for library in baseline.values()})
for library in baseline.values():
    assert info(Path(library['path'])) == {k: v for k, v in library.items() if k != 'path'}
artifacts = [json.loads(line) for line in (cost / 'library-build.jsonl').read_text().splitlines()]
def artifact(name):
    return next(Path(f) for a in artifacts if a.get('reason') == 'compiler-artifact'
                and a['target']['name'] == name for f in a['filenames'] if f.endswith('.rlib'))
names = ['hyperdrc', 'hypercurve', 'hyperlimit', 'serde_json']
libraries = {'baseline': {name: Path(baseline[name]['path']) for name in names},
             'candidate': {name: artifact(name) for name in names}}
builds, identities, results = [], {}, {}
for variant, linked in libraries.items():
    folder = out / variant
    folder.mkdir()
    binaries = {}
    probes = {'cost_probe': record / 'cost_probe.rs', 'track_probe': record / 'track_probe.rs',
              'memory_probe': record / 'memory_probe.rs'}
    if variant == 'candidate':
        probes['pad_probe'] = previous / 'full_board_probe.rs'
    for name, source in probes.items():
        binary = folder / name
        command = ['rustc', '--edition=2024', '-O', str(source)]
        if variant == 'baseline':
            for directory in baseline_dependency_dirs:
                command += ['-L', f'dependency={directory}']
        else:
            command += ['-L', f'dependency={Path.cwd()}/target/release/deps']
        for library, path in linked.items():
            command += ['--extern', f'{library}={path}']
        command += ['-o', str(binary)]
        with (folder / (name + '-build.log')).open('w') as log:
            code = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT).returncode
        builds.append(dict(command=command, exit_code=code))
        (out / 'builds.json').write_text(json.dumps(builds, indent=2) + '\n')
        assert code == 0
        binaries[name] = binary
    trials = []
    for trial in range(1, 6):
        command = ['taskset', '-c', '0', '/usr/bin/time', '-v', str(binaries['cost_probe']), str(out / 'inputs')]
        with (folder / f'trial-{trial}.jsonl').open('w') as stdout, (folder / f'trial-{trial}.log').open('w') as stderr:
            code = subprocess.run(command, stdout=stdout, stderr=stderr).returncode
        rows = [json.loads(line) for line in (folder / f'trial-{trial}.jsonl').read_text().splitlines()]
        assert code == 0 and len(rows) == 18
        identity = [{k: v for k, v in row.items() if k != 'elapsed_ns'} for row in rows]
        if trial == 1:
            identities[variant] = identity
        else:
            assert identity == identities[variant]
        trials.append(dict(command=command, exit_code=code, rows=rows,
                           peak_rss_kib=int(re.search(r'Maximum resident set size \(kbytes\): (\d+)', (folder / f'trial-{trial}.log').read_text())[1])))
    (folder / 'rows.json').write_text(json.dumps(identities[variant], sort_keys=True, separators=(',', ':')) + '\n')
    memory_trials = []
    for trial in range(1, 6):
        command = ['taskset', '-c', '0', str(binaries['memory_probe'])]
        with (folder / f'memory-{trial}.jsonl').open('w') as stdout, (folder / f'memory-{trial}.log').open('w') as stderr:
            code = subprocess.run(command, stdout=stdout, stderr=stderr).returncode
        measured = [json.loads(line) for line in (folder / f'memory-{trial}.jsonl').read_text().splitlines()]
        assert code == 0 and len(measured) == 5 and all(row['witness'] for row in measured[1:])
        memory_trials.append(dict(command=command, exit_code=code, sizes=measured[0], rows=measured[1:]))
    assert all(t['sizes'] == memory_trials[0]['sizes'] for t in memory_trials)
    boards = []
    for kind, oracle in [('track', track_oracle)] + ([('pad', pad_oracle)] if variant == 'candidate' else []):
        for label in ['cparti-fpga', 'hvp109a']:
            binary = binaries['track_probe' if kind == 'track' else 'pad_probe']
            command = [str(binary), str(oracle / (label + '.json'))]
            result_path = folder / (kind + '-' + label + '.json')
            with result_path.open('w') as stdout, result_path.with_suffix('.log').open('w') as stderr:
                code = subprocess.run(command, stdout=stdout, stderr=stderr).returncode
            assert code == 0
            result = load(result_path)
            boards.append(dict(kind=kind, label=label, command=command, exit_code=code,
                               result={k: v if not isinstance(v, list) else len(v) for k, v in result.items()},
                               artifact=info(result_path), oracle=info(oracle / (label + '.json'))))
            print(variant, kind, label, boards[-1]['result'], flush=True)
    results[variant] = dict(trials=trials, complete_boards=boards, memory_trials=memory_trials,
                           medians_ns={r['name']: statistics.median(t['rows'][i]['elapsed_ns'] / r['iterations'] for t in trials)
                                       for i, r in enumerate(trials[0]['rows'])},
                           binaries={str(p): info(p) for p in binaries.values()},
                           libraries={str(p): info(p) for p in linked.values()})
deltas = []
for before, after in zip(identities['baseline'], identities['candidate']):
    assert {k: v for k, v in before.items() if k != 'witness'} == {k: v for k, v in after.items() if k != 'witness'}
    if before != after:
        deltas.append(dict(before=before, after=after))
(out / 'deltas.json').write_text(json.dumps(deltas, indent=2) + '\n')
result = dict(scope='Five serial fresh release processes per variant pinned to CPU 0; 18 unchanged inputs and 100 construction/loader iterations per row. Witness queries are outside timers. Different answers preclude matched speedup claims; RSS and executable sizes do not establish allocation counts or library-size savings. A separate counting-allocator probe records requested allocation calls/bytes, retained and peak live byte deltas for four identical 1000-iteration workloads; these counts exclude allocator metadata and witness queries. Complete source oracles cover root linear segments and the previously admitted native pad components/custom placements.',
              variants=results, changed_witnesses=len(deltas), source_hashes=sources, source_unchanged=current() == sources,
              probe_sources={str(p): info(p) for p in [record / 'cost_probe.rs', record / 'track_probe.rs', record / 'memory_probe.rs', previous / 'full_board_probe.rs']},
              oracle_generators={str(p): info(p) for p in [record / 'track_oracle.py', previous / 'full_board_oracle.py']})
(out / 'manifest.json').write_text(json.dumps(result, indent=2) + '\n')
assert result['source_unchanged']
assert all(b['result']['all_admitted'] for b in results['candidate']['complete_boards'])
assert all(row['witness'].get('area_matches', row['witness'].get('matches_source')) for row in identities['candidate'])
