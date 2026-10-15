"""Investigate adverse short-batch controls using the same frozen binaries."""
from pathlib import Path
import hashlib, json

E = Path(__file__).resolve().parent
phase = 'cost-confirmation-v1'
assert not (E/phase).exists()
cost = Path('/tmp/csgrs-strict-drilled-workflows/cost-v2')
result = json.loads((E/'cost-v2/result.json').read_text())
assert result['passed']
commands = []
for sample in range(5):
    for revision in (['baseline', 'candidate'] if sample % 2 == 0 else ['candidate', 'baseline']):
        commands.append([f'{sample}-{revision}', [str(cost/revision), revision, '4096']])
keep = json.loads((E/'cost-v2-preserve.json').read_text())
for p in [*cost.iterdir(), Path(__file__), E/'scalar-assessment-v1.json']:
    if p.is_file(): keep[str(p)] = hashlib.sha256(p.read_bytes()).hexdigest()
for suffix, value in [('.json', commands), ('-seed.json', None), ('-preserve.json', keep)]:
    p = E/(phase+suffix)
    assert not p.exists()
    p.write_text(json.dumps(value, indent=2)+'\n')
contract = json.loads((E/'cost-contract-v2.json').read_text())
contract['reason_for_confirmation'] = 'All 18 completed-contract short-batch medians were slower, up to 31.29%, with unchanged allocation counts. Some batch windows were tens of microseconds. Preserve those observations and investigate longer fixed batches before retention; this is not a rerun that discards unfavorable samples.'
contract['samples'] = 'Five fresh processes per revision, alternating order, 4096 measured operations per row and the unchanged 32 warmups. The same frozen cost-v2 binaries, source, dependencies, oracle, cases, APIs and construction boundaries are reused. This is the probe\'s preexisting maximum iteration count.'
contract['earlier_assessment'] = 'scalar-assessment-v1.json'
p = E/'cost-confirmation-contract-v1.json'
assert not p.exists()
p.write_text(json.dumps(contract, indent=2)+'\n')
print(json.dumps({'phase': phase, 'samples': 10, 'iterations_per_row': 4096}))
