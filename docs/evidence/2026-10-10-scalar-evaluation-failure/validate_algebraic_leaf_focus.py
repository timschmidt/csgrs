#!/usr/bin/env python3
"""Run focused algebraic leaf regression, replay and cost workloads sequentially."""
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
build = 'typed-api-prototype-v842-algebraic-leaf-build'
unit = 'typed-api-prototype-v840-algebraic-leaf-unit-tests'
expected = sha(E/build/'source-hashes.json.gz')
for phase in [build, unit, 'typed-api-prototype-v841-algebraic-leaf-clippy']:
    r = read(E/phase/'result.json')
    assert r['exit_code'] == 0 and r['source_unchanged'] and r['main_unchanged']
    assert sha(E/phase/'source-hashes.json.gz') == expected

phase = 'typed-api-prototype-v839-algebraic-leaf-repair-cap'
r = read(E/phase/'result.json')
assert r['exit_code'] == 0 and r['source_unchanged'] and r['main_unchanged']
assert sha(E/phase/'source-hashes.json.gz') == expected
assert '6 passed; 0 failed;' in (E/phase/'stdout.log').read_text()

for script, phase, extra, count in [
    ('reproduce_inverse_trig_budget.py', 'typed-api-prototype-v843-algebraic-leaf-inverse-replay', [unit, 'replay'], 384),
    ('reproduce_inverse_trig_budget.py', 'typed-api-prototype-v844-algebraic-leaf-inverse-cost', [unit, 'cost'], 54),
    ('reproduce_normal_form_budget.py', 'typed-api-prototype-v845-algebraic-leaf-normal-form-cost', [unit, 'cost'], 54),
    ('reproduce_evaluation_budget_cost.py', 'typed-api-prototype-v846-algebraic-leaf-default-cost', [], 54),
    ('reproduce_algebraic_budget_v2.py', 'typed-api-prototype-v847-algebraic-leaf-algebraic-cost', [unit, 'cost'], 54),
    ('reproduce_evaluation_budget.py', 'typed-api-prototype-v848-algebraic-leaf-evaluation-replay', [unit], 384),
    ('reproduce_algebraic_budget_v2.py', 'typed-api-prototype-v849-algebraic-leaf-algebraic-replay', [unit, 'replay'], 288),
    ('reproduce_algebraic_wide_budget.py', 'typed-api-prototype-v850-algebraic-leaf-wide-replay', [unit, 'replay'], 144),
    ('reproduce_normal_form_budget.py', 'typed-api-prototype-v851-algebraic-leaf-normal-form-replay', [unit, 'replay'], 384),
]:
    assert not (E/phase).exists()
    subprocess.run([sys.executable, str(E/script), build, phase, *extra], cwd=ROOT, check=True)
    summary = read(E/phase/'summary.json')
    assert summary['source_hashes_sha256'] == expected and summary['guards_unchanged']
    assert summary.get('processes', summary['cases']) == count
    for key, value in summary.get('artifacts_sha256', {}).items():
        assert sha(E/phase/key) == value
    print(phase, 'passed', flush=True)
