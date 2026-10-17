"""Sample only this diagnostic's currently running CPArti candidate process."""
import hashlib
import json
import subprocess
import time
from pathlib import Path

W = Path(__file__).resolve().parent
O = W / 'aggregation-backtrace-v3'
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
command = ['gdb', '-q', '-nx', '-batch', '-iex', 'set debuginfod enabled off',
           '-iex', 'set auto-load safe-path /nonexistent', '-iex', 'set sysroot /',
           '-se', str(binary), '-p', process.name,
           '-ex', 'set pagination off', '-ex', 'thread apply all bt 60', '-ex', 'detach']
record = {'runner_sha256': sha(__file__), 'pid': int(process.name), 'command': command,
          'exe_sha256': sha(binary), 'raw_stat_before': (process / 'stat').read_text(),
          'stdout_before': out_before, 'stderr_before': err_before,
          'scope': 'Read-only stack snapshot of the verified aggregation stage. This instrumented diagnostic timing is not benchmark admission.'}
begin = time.monotonic()
with (O / 'stdout').open('wb') as out, (O / 'stderr').open('wb') as err:
    r = subprocess.run(['timeout', '--kill-after=5s', '30s', *command], stdout=out, stderr=err)
record.update({'exit_code': r.returncode, 'elapsed_seconds': time.monotonic() - begin,
               'stdout_sha256': sha(O / 'stdout'), 'stderr_sha256': sha(O / 'stderr')})
(O / 'result.json').write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps({'exit_code': r.returncode, 'pid': int(process.name), 'stage': 'copper-layers'}), flush=True)
