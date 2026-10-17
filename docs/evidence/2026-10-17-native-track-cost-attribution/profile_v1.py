"""Retain a short sampled call graph of our active aggregation diagnostic."""
import hashlib
import json
import subprocess
import time
from pathlib import Path

W = Path(__file__).resolve().parent
O = W / 'aggregation-profile-v1'
R = W / 'stage-observations-v1'
binary = R / 'candidate-stages'
board = R / 'projects/cparti/CPArti FPGA dev board.kicad_pcb'
def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()
matches = []
for p in Path('/proc').iterdir():
    if not p.name.isdigit():
        continue
    try:
        args = (p / 'cmdline').read_bytes().split(b'\0')
        if args[:3] == [str(binary).encode(), str(board).encode(), b'groups']:
            matches.append(p)
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        pass
assert len(matches) == 1, matches
process = matches[0]
assert sha(process / 'exe') == sha(binary)
out_before = (R / 'candidate-cparti-groups.stdout').read_text()
err_before = (R / 'candidate-cparti-groups.stderr').read_text()
assert 'copper-layers' not in out_before and err_before.splitlines()[-1].endswith('copper-layers')
O.mkdir(exist_ok=False)
commands = [
    ('record', ['perf', 'record', '-F', '199', '--call-graph', 'dwarf,16384', '-p', process.name,
                '-o', str(O / 'perf.data'), '--', 'sleep', '3']),
    ('report', ['perf', 'report', '-i', str(O / 'perf.data'), '--stdio', '--children', '--percent-limit', '0.5',
                '--call-graph', 'graph,0.5,caller']),
    ('flat', ['perf', 'report', '-i', str(O / 'perf.data'), '--stdio', '--no-children', '--percent-limit', '0.5',
              '--call-graph', 'none'])
]
record = {'runner_sha256': sha(__file__), 'pid': int(process.name), 'exe_sha256': sha(binary),
    'raw_stat_before': (process / 'stat').read_text(), 'stdout_before': out_before, 'stderr_before': err_before,
    'scope': 'Sampled CPU attribution only. The instrumented stage timing cannot establish production performance.', 'commands': []}
for name, command in commands:
    begin = time.monotonic()
    with (O / (name + '.stdout')).open('wb') as out, (O / (name + '.stderr')).open('wb') as err:
        r = subprocess.run(['timeout', '--kill-after=5s', '30s', *command], stdout=out, stderr=err)
    record['commands'].append({'name': name, 'command': command, 'exit_code': r.returncode,
        'elapsed_seconds': time.monotonic() - begin,
        **{s + '_sha256': sha(O / (name + '.' + s)) for s in ['stdout', 'stderr']}})
    if r.returncode:
        break
record['perf_data_sha256'] = sha(O / 'perf.data') if (O / 'perf.data').exists() else None
(O / 'result.json').write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps({'pid': int(process.name), 'exits': [c['exit_code'] for c in record['commands']]}), flush=True)
