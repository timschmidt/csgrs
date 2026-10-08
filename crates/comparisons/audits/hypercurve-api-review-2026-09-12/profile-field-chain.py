import hashlib
import os
from pathlib import Path
import signal
import subprocess

artifact = Path(__file__).resolve().parent
binary = artifact / 'stationary-ph-offset-retained-constant-coordinate'
expected = __import__('json').loads((artifact / 'native-selected-sign-phase-followup.json').read_text())['binary_sha256']
assert hashlib.sha256(binary.read_bytes()).hexdigest() == expected
commands = artifact / 'profile-field-chain.gdb'
commands.write_text('''set debuginfod enabled off
set pagination off
set confirm off
starti
python
import gdb, json
binary = gdb.current_progspace().filename
base = None
for line in gdb.execute('info proc mappings', to_string=True).splitlines():
    parts = line.split()
    if len(parts) >= 5 and line.rstrip().endswith(binary) and int(parts[3], 16) == 0:
        base = int(parts[0], 16)
        break
assert base is not None, 'loaded binary mapping is required'
print('binary_load_base', hex(base))
queries = {}
next_id = 0
class Entry(gdb.Breakpoint):
    def stop(self):
        global next_id
        next_id += 1
        slot = int(gdb.parse_and_eval('$rsp')) - 0x178
        query = {'id': next_id, 'iteration': 0, 'defining_coefficients': int(gdb.parse_and_eval('$rdx')), 'predicate_coefficients': int(gdb.parse_and_eval('$r8'))}
        queries[slot] = query
        print('query_entry', json.dumps(query))
        return False
class Chain(gdb.Breakpoint):
    def __init__(self, address, endpoint):
        super().__init__('*' + hex(address), internal=True)
        self.endpoint = endpoint
    def stop(self):
        slot = int(gdb.parse_and_eval('$rsp'))
        query = queries.get(slot)
        assert query is not None, 'chain must retain its matching query entry'
        if self.endpoint == 'lower':
            query['iteration'] += 1
        first_count = int.from_bytes(gdb.selected_inferior().read_memory(slot + 0x60, 8).tobytes(), 'little')
        print('chain_endpoint', json.dumps(dict(query, endpoint=self.endpoint, first_coefficients=first_count, current_coefficients=int(gdb.parse_and_eval('$rdx')))))
        return False
Entry('*' + hex(base + 0x720620), internal=True)
Chain(base + 0x720dd5, 'lower')
Chain(base + 0x720e22, 'upper')
end
continue
thread apply all bt 45
''')
log = artifact / 'stationary-ph-field-chain-trace.log'
with log.open('w') as output:
    process = subprocess.Popen(['gdb', '--quiet', '--batch', '-x', str(commands), '--args', str(binary)], stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        process.wait(timeout=25)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
print('gdb exit', process.returncode)
