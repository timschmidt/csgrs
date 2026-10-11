#!/usr/bin/env python3
"""Validate borrowed integer scaling and replay frozen resource and cost probes."""
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
first = 'typed-api-prototype-v1352-integer-scale-direct-build'
expected = '32b816463eacd2d2cef2f905c0455557c015117690a8a8dd635229a57726d2b2'


def checked(phase):
    r = read(E/phase/'result.json')
    assert r['exit_code'] == 0 and r['source_unchanged'] and r['main_unchanged'], (phase, r)
    assert sha(E/phase/'source-hashes.json.gz') == expected
    print(phase, 'passed', flush=True)


def validate(phase, args):
    assert not (E/phase).exists()
    subprocess.run([sys.executable, str(E/'validate_candidate_v2.py'), phase, *args], cwd=ROOT, check=True)
    checked(phase)


checked(first)
paths = re.findall(r'Executable unittests src/lib\.rs \(([^)]+)\)', (E/first/'stderr.log').read_text())
assert len(paths) == 1
binary = Path(paths[0])
validate('typed-api-prototype-v1353-integer-scale-repair-cap', ['--exec', sys.executable, str(E/'run_limited_unit_test.py'), str(binary), sha(binary), 'computable::node::integer_scale_tests::', '--nocapture', '--test-threads=1'])
unit = 'typed-api-prototype-v1354-integer-scale-unit-tests'
validate(unit, ['test', '--release', '-p', 'hyperreal', '--lib', '--all-features'])
assert '1054 passed; 0 failed;' in (E/unit/'stdout.log').read_text()
validate('typed-api-prototype-v1355-integer-scale-clippy', ['clippy', '--release',
    '-p', 'hyperreal', '-p', 'hyperlimit', '-p', 'hyperlattice', '-p', 'hypercurve', '-p', 'hyperdrc',
    '--all-targets', '--all-features', '--', '-D', 'warnings'])
build = 'typed-api-prototype-v1356-integer-scale-build'
validate(build, ['build', '--release', '-p', 'hyperreal', '-p', 'hypercurve', '--all-features'])
for script, phase, extra, count in [
    ('reproduce_integer_scale_cost.py', 'typed-api-prototype-v1357-integer-scale-cost', [], 72),
    ('reproduce_inverse_trig_budget.py', 'typed-api-prototype-v1362-integer-scale-inverse-replay', [unit, 'replay'], 384),
    ('reproduce_inverse_trig_budget.py', 'typed-api-prototype-v1358-integer-scale-inverse-cost', [unit, 'cost'], 54),
    ('reproduce_normal_form_budget.py', 'typed-api-prototype-v1359-integer-scale-normal-form-cost', [unit, 'cost'], 54),
    ('reproduce_evaluation_budget_cost.py', 'typed-api-prototype-v1360-integer-scale-default-cost', [], 54),
    ('reproduce_algebraic_budget_v2.py', 'typed-api-prototype-v1361-integer-scale-algebraic-cost', [unit, 'cost'], 54),
    ('reproduce_evaluation_budget.py', 'typed-api-prototype-v1363-integer-scale-evaluation-replay', [unit], 384),
    ('reproduce_algebraic_budget_v2.py', 'typed-api-prototype-v1364-integer-scale-algebraic-replay', [unit, 'replay'], 288),
    ('reproduce_algebraic_wide_budget.py', 'typed-api-prototype-v1365-integer-scale-wide-replay', [unit, 'replay'], 144),
    ('reproduce_normal_form_budget.py', 'typed-api-prototype-v1366-integer-scale-normal-form-replay', [unit, 'replay'], 384),
]:
    assert not (E/phase).exists()
    subprocess.run([sys.executable, str(E/script), build, phase, *extra], cwd=ROOT, check=True)
    summary = read(E/phase/'summary.json')
    assert summary['source_hashes_sha256'] == expected and summary['guards_unchanged']
    assert summary.get('processes', summary['cases']) == count
    for key, value in summary.get('artifacts_sha256', {}).items():
        assert sha(E/phase/key) == value
    print(phase, 'passed', flush=True)
