"""Materialize the predeclared five alternating pairs after preflight succeeds."""
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
BUILD = Path('/tmp/csgrs-factored-product-proofs/cost-build-v1')
TARGET = BUILD.with_name('cost-measurement-v1')
assert json.loads((E / 'cost-build-v1/result.json').read_text())['passed']
contract = json.loads((E / 'cost-contract-v1.json').read_text())
measurement = contract['measurement']
assert measurement['pairs'] == 5
preserve = json.loads((E / 'cost-build-v1-preserve.json').read_text())
for p in BUILD.rglob('*'):
    if p.is_file():
        preserve[str(p)] = hashlib.sha256(p.read_bytes()).hexdigest()
commands = [['prepare-target', ['mkdir', str(TARGET)]]]
for sample in range(5):
    order = ['baseline', 'candidate'] if sample % 2 == 0 else ['candidate', 'baseline']
    for label in order:
        for kind in ['controls', 'forms']:
            argument = 'candidate' if kind == 'controls' else label
            iterations = measurement['control_iterations_per_row' if kind == 'controls' else 'form_iterations_per_row']
            commands.append([f'{sample}-{label}-{kind}', [str(BUILD / (label + '-' + kind)), argument, str(iterations)]])
commands.append(['artifact-sections', ['size'] + [str(BUILD / (label + '-' + kind))
                                                for label in ['baseline', 'candidate']
                                                for kind in ['controls', 'forms']]])
for suffix, value in [('.json', commands), ('-seed.json', None),
                      ('-target.json', str(TARGET)), ('-preserve.json', preserve)]:
    p = E / ('cost-measurement-v1' + suffix)
    assert not p.exists()
    p.write_text(json.dumps(value, indent=2) + '\n')
print('Prepared', len(commands), 'measurement commands')
