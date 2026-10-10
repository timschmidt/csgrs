"""Replay the full independent interval oracle on the frozen refined index."""
import hashlib
import json
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'contact-index-directed-width-matrix'
OUT.mkdir()
SOURCE = ROOT / 'target/contact-index-directed-width/source'
source_record = json.loads((EVIDENCE / 'contact-index-directed-width/sources.json').read_text())
name = 'crates/hypercurve/src/curve_region_boolean.rs'
text = (SOURCE / name).read_text()
assert hashlib.sha256(text.encode()).hexdigest() == source_record['source_hashes'][name]
start = text.index('#[derive(Default)]\nstruct ContactPointIndex {')
body = text[start:text.index('\nfn replace_topology_vertex(', start)]
oracle = (EVIDENCE / 'contact-index-range-matrix/candidate.rs').read_text()
adapter = oracle[:oracle.index('#[derive(Default)]\nstruct ContactPointIndex {')]
main = oracle[oracle.index('\nfn main() {'):]
(OUT / 'probe.rs').write_text(adapter + body + main)
binary = ROOT / 'target/contact-index-directed-width/range-matrix'
commands = []
for phase, command in [('build', ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(OUT / 'probe.rs'), '-o', str(binary)]), ('run', ['timeout', '120', str(binary)])]:
    start_time = time.monotonic()
    with (OUT / (phase + '.log')).open('w') as log, (OUT / (phase + '.jsonl')).open('w') as rows:
        result = subprocess.run(command, cwd=ROOT, stdout=rows, stderr=log, check=False)
    commands.append({'name':phase, 'command':command, 'exit_code':result.returncode, 'wall_seconds':time.monotonic()-start_time})
    (OUT / 'commands.json').write_text(json.dumps(commands,indent=2)+'\n')
    assert result.returncode == 0
rows = json.loads((OUT / 'run.jsonl').read_text())
assert rows == {'values':70, 'intervals':2486, 'overlap_comparisons':6354216, 'shared_omissions':0, 'isolated_omissions':0, 'extras':0}
assert (SOURCE / name).read_text() == text
record = {'scope':'The entire frozen refined index and the unchanged independent interval-overlap oracle. Adapter boxes are valid conservative intervals, not a full public Boolean test.', 'source_record':'../contact-index-directed-width/sources.json', 'implementation_sha256':hashlib.sha256(body.encode()).hexdigest(), 'oracle_adapter_sha256':hashlib.sha256(adapter.encode()).hexdigest(), 'oracle_main_sha256':hashlib.sha256(main.encode()).hexdigest(), 'probe_sha256':hashlib.sha256((OUT/'probe.rs').read_bytes()).hexdigest(), 'commands':commands, 'results':rows, 'production_source_unchanged':True}
(OUT / 'manifest.json').write_text(json.dumps(record,indent=2)+'\n')
(OUT / 'reproduce.py').write_bytes(Path(__file__).read_bytes())
print('Refined index: all 6,354,216 interval comparisons pass.',flush=True)
