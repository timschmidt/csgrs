"""Replay twelve checks on both full boards in two fresh processes.

Usage: python3 reproduce_report_replay.py COST_OUTPUT REPORT_OUTPUT
Run after all builds/tests/cost jobs exit. Preserve the exact previous input
paths and bytes; the retained synthetic seven-record IPC sidecar is unchanged.
Every complete report delta is retained for explicit stroke-source and finding review.
"""
import gzip
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

record = Path(__file__).resolve().parent
cost, out = [Path(p).resolve() for p in sys.argv[1:3]]
out.mkdir(parents=True,exist_ok=False)
load = lambda path: json.loads(path.read_bytes())
info = lambda path: dict(bytes=path.stat().st_size,sha256=hashlib.sha256(path.read_bytes()).hexdigest())
manifest = load(cost/'manifest.json')
sources = manifest['source_hashes']
current = lambda: {p:info(Path(p))['sha256'] for p in sources}
assert current() == sources and manifest['source_unchanged'] and manifest['fixture_exit_code'] == 0
previous = record.parent/'2026-10-09-hyperdrc-native-pad-primitives'
baseline = load(previous/'report-replay/manifest.json')
inputs = Path(baseline['trials'][0]['command'][-2])
inventory = lambda: {str(p.relative_to(inputs)):info(p) for p in sorted(inputs.rglob('*')) if p.is_file()}
assert inventory() == baseline['input_files']
for path, expected in baseline['archives'].items(): assert info(Path(path)) == expected
artifacts = [json.loads(line) for line in (cost/'library-build.jsonl').read_text().splitlines()]
def rlib(name):
    return next(Path(f) for a in artifacts if a.get('reason') == 'compiler-artifact' and a['target']['name'] == name for f in a['filenames'] if f.endswith('.rlib'))
linked = {name:rlib(name) for name in ['hyperdrc','clap','serde_json']}
assert info(linked['hyperdrc']) == manifest['binaries'][str(linked['hyperdrc'])]
source = previous/'full_report_replay.rs'
assert info(source) == baseline['source']
binary = out/'full_report_replay'
command = ['rustc','--edition=2024','-O',str(source),'-L',f'dependency={Path.cwd()}/target/release/deps']
for name,path in linked.items(): command += ['--extern',f'{name}={path}']
command += ['-o',str(binary)]
with (out/'build.log').open('w') as log:
    subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
trials = []
for trial in [1,2]:
    run = ['taskset','-c','0','/usr/bin/time','-v',str(binary),str(inputs),str(out/f'trial-{trial}')]
    with (out/f'trial-{trial}.log').open('w') as log:
        result = subprocess.run(run,stdout=log,stderr=subprocess.STDOUT)
    trials.append(dict(command=run,exit_code=result.returncode,peak_rss_kib=int(re.search(r'Maximum resident set size \(kbytes\): (\d+)',(out/f'trial-{trial}.log').read_text())[1])))
    (out/'trials.json').write_text(json.dumps(trials,indent=2)+'\n')
    assert result.returncode == 0
    print('Full report trial',trial,'passed',flush=True)
reports = {}
for label in ['cparti-fpga','hvp109a']:
    paths = [out/f'trial-{trial}/{label}.json' for trial in [1,2]]
    raw = paths[0].read_bytes(); assert raw == paths[1].read_bytes()
    before = gzip.decompress((record.parent/f'2026-10-09-hyperdrc-kicad-pad-sources/report-replay/trial-1-{label}.json.gz').read_bytes())
    assert before.count(b'"check_version":20') == 12
    expected = before.replace(b'"check_version":20',b'"check_version":21')
    report = json.loads(raw); assert len(report['violations']) == report['violation_count']
    for trial,path in enumerate(paths,1):
        (out/f'trial-{trial}-{label}.json.gz').write_bytes(gzip.compress(path.read_bytes(),mtime=0))
    reports[label] = dict(**info(paths[0]),identical_complete_bytes=True,baseline_only_version_changed=raw == expected,
                          baseline_version_fields=12,violations=report['violation_count'],summary=report['summary'],
                          compressed_artifacts={f'trial-{trial}-{label}.json.gz':info(out/f'trial-{trial}-{label}.json.gz') for trial in [1,2]})
result = dict(scope='Two complete public Report serializations for twelve selected checks on both preserved full boards, with a synthetic seven-record IPC sidecar and unchanged fixed input paths. No candidate normalization; every baseline delta requires explicit review. No universal or path-independent determinism claim.',
              command=command,build_exit_code=0,trials=trials,reports=reports,input_files=baseline['input_files'],archives=baseline['archives'],
              source=info(source),binaries={str(binary):info(binary)},linked_libraries={str(p):info(p) for p in linked.values()},
              cost_library_build=info(cost/'library-build.jsonl'),source_hashes=sources,source_unchanged=current() == sources)
(out/'manifest.json').write_text(json.dumps(result,indent=2)+'\n')
initial = dict(status='passed' if all(r['baseline_only_version_changed'] for r in reports.values()) else 'failed',
               reports={label:r['baseline_only_version_changed'] for label,r in reports.items()},
               expectation='Complete baseline report bytes except exactly twelve implementation-version fields 20 to 21 per board.')
(out/'initial-comparison.json').write_text(json.dumps(initial,indent=2)+'\n')
assert result['source_unchanged'] and inventory() == baseline['input_files']
print(json.dumps(reports,indent=2),flush=True)
print('Complete reports require explicit correctness delta review:', initial['status'], flush=True)
