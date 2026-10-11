#!/usr/bin/env python3
"""Run focused inverse-trig regression, replay and cost workloads sequentially."""
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
build = 'typed-api-prototype-v793-inverse-trig-inline-build'
unit = 'typed-api-prototype-v791-inverse-trig-inline-tests'
expected = sha(E/build/'source-hashes.json.gz')
for phase in [build, unit, 'typed-api-prototype-v792-inverse-trig-inline-clippy']:
    r = read(E/phase/'result.json')
    assert r['exit_code'] == 0 and r['source_unchanged'] and r['main_unchanged']
    assert sha(E/phase/'source-hashes.json.gz') == expected

paths = re.findall(r'Running unittests src/lib\.rs \(([^)]+)\)', (E/unit/'stderr.log').read_text())
assert len(paths) == 1
binary = Path(paths[0])
phase = 'typed-api-prototype-v794-inverse-trig-repair-cap'
assert not (E/phase).exists()
subprocess.run([sys.executable, str(E/'validate_candidate_v2.py'), phase, '--exec', sys.executable,
    str(E/'run_limited_unit_test.py'), str(binary), sha(binary),
    'computable::node::inverse_trig_budget_tests::', '--nocapture', '--test-threads=1'], check=True, cwd=ROOT)
assert read(E/phase/'result.json')['exit_code'] == 0
assert '9 passed; 0 failed;' in (E/phase/'stdout.log').read_text()

for script, phase, extra, count in [
    ('reproduce_inverse_trig_budget.py', 'typed-api-prototype-v795-inverse-trig-replay', [unit, 'replay'], 384),
    ('reproduce_inverse_trig_budget.py', 'typed-api-prototype-v796-inverse-trig-cost', [unit, 'cost'], 54),
    ('reproduce_normal_form_budget.py', 'typed-api-prototype-v797-inverse-trig-normal-form-cost', [unit, 'cost'], 54),
    ('reproduce_evaluation_budget_cost.py', 'typed-api-prototype-v798-inverse-trig-default-cost', [], 54),
    ('reproduce_algebraic_budget_v2.py', 'typed-api-prototype-v799-inverse-trig-algebraic-cost', [unit, 'cost'], 54),
    ('reproduce_evaluation_budget.py', 'typed-api-prototype-v800-inverse-trig-evaluation-replay', [unit], 384),
    ('reproduce_algebraic_budget_v2.py', 'typed-api-prototype-v801-inverse-trig-algebraic-replay', [unit, 'replay'], 288),
    ('reproduce_algebraic_wide_budget.py', 'typed-api-prototype-v802-inverse-trig-wide-replay', [unit, 'replay'], 144),
    ('reproduce_normal_form_budget.py', 'typed-api-prototype-v803-inverse-trig-normal-form-replay', [unit, 'replay'], 384),
]:
    assert not (E/phase).exists()
    subprocess.run([sys.executable, str(E/script), build, phase, *extra], cwd=ROOT, check=True)
    summary = read(E/phase/'summary.json')
    assert summary['source_hashes_sha256'] == expected and summary['guards_unchanged']
    assert summary.get('processes', summary['cases']) == count
    for key, value in summary.get('artifacts_sha256', {}).items():
        assert sha(E/phase/key) == value
    print(phase, 'passed', flush=True)
