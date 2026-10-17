"""Qualify the isolated demo before running the unchanged cost protocol."""
import hashlib, json, subprocess, time
from pathlib import Path

W = Path(__file__).resolve().parent
O = W / 'qualification-tail-v5'

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def dump(p, value):
    Path(p).write_text(json.dumps(value, indent=2) + '\n')

assert read(W / 'consumer-validation-v1/result.json')['unexpected_exits'] == ['circuit-all']
assert read(W / 'snapshot-requalification-v1/result.json')['complete']
assert read(W / 'ui-isolation-v1/result.json')['complete']
assert read(W / 'ui-validation-v1/result.json')['unexpected_exits'] == ['ui-tests', 'ui-clippy', 'ui-wasm']
assert read(W / 'qualification-tail-v4/commands.json')[-1]['exit_code'] == 101
O.mkdir(exist_ok=False)
pins = {str(p): sha(p) for p in [Path(__file__), W / 'ui_v5.py', W / 'ui_common_v4.py',
    W / 'compare_v5.py', W / 'cost_probe_v2.rs', W / 'cost_body_v2.rs', W / 'cost-contract-v1.md',
    W / 'ui-isolation-v1/result.json', W / 'ui-isolation-contract-v1.md',
    W / 'snapshot-requalification-v1/result.json', W / 'consumer-validation-v1/result.json']}
dump(O / 'protocol.json', {'input_sha256': pins, 'started_epoch': time.time(),
    'order': ['ui-validation-v2', 'comparison-v1'], 'overlapping_workload_measurements': False,
    'preserved_failures': ['ui-validation-v1', 'qualification-tail-v4']})
rows = []
for runner in ['ui_v5.py', 'compare_v5.py']:
    assert all(sha(p) == h for p, h in pins.items())
    begun = time.time()
    print('START dependent ' + runner, flush=True)
    result = subprocess.run(['python3', str(W / runner)], cwd=W)
    row = {'runner': runner, 'exit_code': result.returncode,
        'started_epoch': begun, 'finished_epoch': time.time()}
    rows.append(row)
    dump(O / 'commands.json', rows)
    assert result.returncode == 0, row
dump(O / 'result.json', {'complete': True, 'commands': rows,
    'consumer_result_sha256': sha(W / 'consumer-validation-v1/result.json'),
    'snapshot_requalification_sha256': sha(W / 'snapshot-requalification-v1/result.json'),
    'ui_result_sha256': sha(W / 'ui-validation-v2/result.json'),
    'comparison_result_sha256': sha(W / 'comparison-v1/result.json'),
    'finished_epoch': time.time(), 'main_sources_modified': False, 'runtime_admitted': False})
print('Isolated demo and unchanged comparison protocol completed', flush=True)
