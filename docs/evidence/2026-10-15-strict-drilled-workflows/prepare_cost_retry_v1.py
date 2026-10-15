"""Correct direct-rustc configuration declarations in a fresh cost trial."""
from pathlib import Path
import hashlib, json

E = Path(__file__).resolve().parent
old = '/tmp/csgrs-strict-drilled-workflows/cost-v1'
new = '/tmp/csgrs-strict-drilled-workflows/cost-v2'
assert not Path(new).exists()
result = json.loads((E/'cost-v1/result.json').read_text())
assert not result['passed'] and result['commands'][-1]['name'] == 'baseline-library'
commands = json.loads((E/'cost-v1.json').read_text())
for name, command in commands:
    command[:] = [new if arg == old else arg.replace(old+'/', new+'/') for arg in command]
    if name.endswith('-library'):
        command.extend(['--check-cfg', 'cfg(test)'])
keep = json.loads((E/'cost-v1-preserve.json').read_text())
keep[str(Path(__file__))] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
for suffix, value in [('.json', commands), ('-seed.json', None), ('-preserve.json', keep)]:
    path = E/('cost-v2'+suffix)
    assert not path.exists()
    path.write_text(json.dumps(value, indent=2)+'\n')
contract = json.loads((E/'cost-contract-v1.json').read_text())
contract['retry'] = 'cost-v1 failed before measurement: direct rustc omitted --check-cfg cfg(test). cost-v2 adds that expected configuration name to both library commands without enabling tests or relaxing -D warnings. Source copies, dependencies, probe, assertions, sample counts and all limits are unchanged; outputs use a fresh directory.'
path = E/'cost-contract-v2.json'
assert not path.exists()
path.write_text(json.dumps(contract, indent=2)+'\n')
print(json.dumps({'phase': 'cost-v2', 'commands': len(commands), 'sources_unchanged': True}))
