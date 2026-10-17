"""Run demo and matched measurements only after final consumer qualification."""
import hashlib, json, subprocess, time
from pathlib import Path

W = Path(__file__).resolve().parent
O = W / 'qualification-tail-v4'

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def dump(p, value):
    Path(p).write_text(json.dumps(value, indent=2) + '\n')

assert read(W / 'v4-qualification-reuse-v2/result.json')['complete']
O.mkdir(exist_ok=False)
pins = {str(p): sha(p) for p in [Path(__file__), W / 'ui_v4.py', W / 'ui_common_v3.py',
    W / 'compare_v4.py', W / 'cost_probe_v2.rs', W / 'cost-contract-v1.md',
    W / 'v4-qualification-reuse-v2/result.json', W / 'requalify_snapshots_v1.py',
    W / 'metadata-migration-v2/result.json', W / 'snapshot-replay-libraries-v1/result.json']}
start = time.time()
dump(O / 'protocol.json', {'input_sha256': pins, 'started_epoch': start,
    'native_wait_limit_seconds': 10800,
    'order': ['consumer-validation-v1', 'snapshot-requalification-v1', 'ui-validation-v1', 'comparison-v1'],
    'overlapping_workload_measurements': False})
result_path = W / 'consumer-validation-v1/result.json'
while not result_path.exists():
    assert time.time() - start < 10800
    time.sleep(5)
native = read(result_path)
assert native['complete'] and native['unexpected_exits'] == ['circuit-all'], native
rows = []
for runner in ['requalify_snapshots_v1.py', 'ui_v4.py', 'compare_v4.py']:
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
    'consumer_result_sha256': sha(result_path),
    'snapshot_requalification_sha256': sha(W / 'snapshot-requalification-v1/result.json'),
    'ui_result_sha256': sha(W / 'ui-validation-v1/result.json'),
    'comparison_result_sha256': sha(W / 'comparison-v1/result.json'),
    'finished_epoch': time.time(), 'main_sources_modified': False, 'runtime_admitted': False})
print('Final consumer, demo and comparison qualification completed', flush=True)
