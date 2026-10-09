"""Run after validation and all builds/tests exit; no matched-baseline claim."""
import hashlib, json, pathlib, re, statistics, subprocess, sys

root = pathlib.Path.cwd()
validation = pathlib.Path(sys.argv[1])
out = pathlib.Path(sys.argv[2])
out.mkdir(parents=True, exist_ok=False)
validation_sources = json.loads((validation / 'sources.json').read_text())
assert validation_sources['unchanged']
assert all(step['exit_code'] == 0 for step in json.loads((validation / 'status.json').read_text()))
source_hashes = validation_sources['source_hashes']
current_sources = lambda: {path: hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest() for path in source_hashes}
assert current_sources() == source_hashes
source = pathlib.Path(__file__).resolve().parent / 'cost/operation_probe.rs'
library_command = ['cargo', 'build', '--locked', '--release', '-p', 'hyperdrc', '--lib', '--message-format=json']
with (out / 'library-build.jsonl').open('w') as stdout, (out / 'library-build.log').open('w') as stderr:
    subprocess.run(library_command, stdout=stdout, stderr=stderr, check=True)
artifacts = [json.loads(line) for line in (out / 'library-build.jsonl').read_text().splitlines()]
rlib = next(pathlib.Path(filename) for artifact in artifacts if artifact.get('reason') == 'compiler-artifact' and artifact['target']['name'] == 'hyperdrc' and 'lib' in artifact['target']['kind'] for filename in artifact['filenames'] if filename.endswith('.rlib'))
limit = next(pathlib.Path(filename) for artifact in artifacts if artifact.get('reason') == 'compiler-artifact' and artifact['target']['name'] == 'hyperlimit' for filename in artifact['filenames'] if filename.endswith('.rlib'))
curve = next(pathlib.Path(filename) for artifact in artifacts if artifact.get('reason') == 'compiler-artifact' and artifact['target']['name'] == 'hypercurve' for filename in artifact['filenames'] if filename.endswith('.rlib'))
probe = out / 'operation-probe'
command = ['rustc', '--edition=2024', '-O', str(source), '-L', f'dependency={root / "target/release/deps"}', '--extern', f'hyperdrc={rlib}', '--extern', f'hyperlimit={limit}', '--extern', f'hypercurve={curve}', '-o', str(probe)]
with (out / 'build.log').open('w') as log:
    subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
fixture = root / re.search(r'Executable benches/fixture_smoke.rs \(([^)]+)\)', (validation / 'fixture-build.log').read_text())[1]
observations = []
for trial in range(5):
    command_run = ['taskset', '-c', '0', '/usr/bin/time', '-v', str(probe)]
    with (out / f'operation-{trial+1}.log').open('w') as log:
        subprocess.run(command_run, stdout=log, stderr=subprocess.STDOUT, check=True)
    data = (out / f'operation-{trial+1}.log').read_text()
    rows = []
    for line in data.splitlines():
        if not line.startswith('operation_'):
            continue
        name, iterations, nanoseconds, witness = line.split('\t')
        rows.append(dict(name=name, iterations=int(iterations), ns_per_query=int(nanoseconds)/int(iterations), witness=witness))
    assert len(rows) == 24
    observations.append(dict(rows=rows, peak_rss_kib=int(re.search(r'Maximum resident set size \(kbytes\): (\d+)', data)[1])))
identities = lambda run: [{k: v for k, v in row.items() if k != 'ns_per_query'} for row in run['rows']]
assert all(identities(run) == identities(observations[0]) for run in observations)
identity_bytes = (json.dumps(identities(observations[0]), sort_keys=True, separators=(',', ':')) + '\n').encode()
(out / 'rows.json').write_bytes(identity_bytes)
baseline_rows = json.loads((pathlib.Path(__file__).parent.parent / '2026-10-08-hyperdrc-handoff/cost/rows.json').read_text())
assert identities(observations[0])[:14] == baseline_rows

fixture_command = ['taskset', '-c', '0', '/usr/bin/time', '-v', str(fixture)]
with (out / 'fixture.log').open('w') as log:
    fixture_code = subprocess.run(fixture_command, stdout=log, stderr=subprocess.STDOUT).returncode
file_info = lambda p: dict(bytes=p.stat().st_size, sha256=hashlib.sha256(p.read_bytes()).hexdigest())
manifest = dict(
    scope='Candidate capability cost, not a matched speedup or allocation/library-size comparison.',
    library_build_command=library_command, build_command=command, probe_command=command_run, fixture_command=fixture_command,
    fixture_exit_code=fixture_code, observations=observations,
    medians_ns={row['name']: statistics.median(run['rows'][index]['ns_per_query'] for run in observations) for index, row in enumerate(observations[0]['rows'])},
    rows=dict(count=24, **file_info(out / 'rows.json')),
    binaries={str(p): file_info(p) for p in [probe, fixture, rlib]},
    probe_source=file_info(source),
    previous_fourteen_witness_rows_unchanged=True, source_hashes=source_hashes, source_unchanged=current_sources() == source_hashes,
    rustc=subprocess.run(['rustc', '-Vv'], capture_output=True, text=True, check=True).stdout,
    conditions='Five sequential fresh processes and one full fixture run pinned to CPU 0 after every build/test exits. Region rows measure the operation plus checked result bounds, using fixed warm inputs and new PCB output wrappers; internal native-kernel evidence may already be warm. Input construction, first/last exact area and report-coordinate fingerprint checks are outside query timers but inside process RSS. Classification, bounds, area and emptiness rows each time one checked query; native handoff times a wrapper clone plus checked consuming export. Single/mixed aggregation and replacement rows also query checked result bounds. Twenty-four witness rows and serialized bytes are identical across all five trials. The previous fourteen witness rows are unchanged. New warm Polygon-area rows query existing circles with 17 and 1,000,000 report samples; source construction and fingerprints are outside query timers. Cold rows include fresh circle construction, report samples, first area evaluation and drop. Report rows include native area gating and finite report allocation with 17/128 samples. Coverage rows time complete checks over fixed duplicated inputs; first/last exact areas, report fingerprints and expected finding counts are outside timers but within process RSS. The full two-board and Gerber fixture workload and its 120-second gate are unchanged.')
(out / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
assert manifest['source_unchanged']
print(json.dumps({k: manifest[k] for k in ['medians_ns', 'rows', 'fixture_exit_code']}, indent=2), flush=True)
raise SystemExit(fixture_code)
