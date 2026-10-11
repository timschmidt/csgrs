#!/usr/bin/env python3
"""Validate rational and domain preflight ordering, then replay the frozen resource and cost probes."""
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
first = 'typed-api-prototype-v1147-rational-preflight-repair-build'
expected = '2f5184c00b655901e7a89b44ccb63824a7aad7ce3790a11f7e82e7ca5da5192f'


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
validate('typed-api-prototype-v1148-rational-preflight-repair-cap', ['--exec', sys.executable,
    str(E/'run_limited_unit_test.py'), str(binary), sha(binary),
    'computable::node::rational_preflight_tests::', '--nocapture', '--test-threads=1'])
assert '10 passed; 0 failed;' in (E/'typed-api-prototype-v1148-rational-preflight-repair-cap/stdout.log').read_text()
unit = 'typed-api-prototype-v1149-rational-preflight-unit-tests'
validate(unit, ['test', '--release', '-p', 'hyperreal', '--lib', '--all-features'])
assert '1037 passed; 0 failed;' in (E/unit/'stdout.log').read_text()
validate('typed-api-prototype-v1150-rational-preflight-clippy', ['clippy', '--release',
    '-p', 'hyperreal', '-p', 'hyperlimit', '-p', 'hyperlattice', '-p', 'hypercurve', '-p', 'hyperdrc',
    '--all-targets', '--all-features', '--', '-D', 'warnings'])
build = 'typed-api-prototype-v1151-rational-preflight-build'
validate(build, ['build', '--release', '-p', 'hyperreal', '-p', 'hypercurve', '--all-features'])
for script, phase, extra, count in [
    ('reproduce_inverse_trig_budget.py', 'typed-api-prototype-v1152-rational-preflight-inverse-replay', [unit, 'replay'], 384),
    ('reproduce_inverse_trig_budget.py', 'typed-api-prototype-v1153-rational-preflight-inverse-cost', [unit, 'cost'], 54),
    ('reproduce_normal_form_budget.py', 'typed-api-prototype-v1154-rational-preflight-normal-form-cost', [unit, 'cost'], 54),
    ('reproduce_evaluation_budget_cost.py', 'typed-api-prototype-v1155-rational-preflight-default-cost', [], 54),
    ('reproduce_algebraic_budget_v2.py', 'typed-api-prototype-v1156-rational-preflight-algebraic-cost', [unit, 'cost'], 54),
    ('reproduce_evaluation_budget.py', 'typed-api-prototype-v1157-rational-preflight-evaluation-replay', [unit], 384),
    ('reproduce_algebraic_budget_v2.py', 'typed-api-prototype-v1158-rational-preflight-algebraic-replay', [unit, 'replay'], 288),
    ('reproduce_algebraic_wide_budget.py', 'typed-api-prototype-v1159-rational-preflight-wide-replay', [unit, 'replay'], 144),
    ('reproduce_normal_form_budget.py', 'typed-api-prototype-v1160-rational-preflight-normal-form-replay', [unit, 'replay'], 384),
]:
    assert not (E/phase).exists()
    subprocess.run([sys.executable, str(E/script), build, phase, *extra], cwd=ROOT, check=True)
    summary = read(E/phase/'summary.json')
    assert summary['source_hashes_sha256'] == expected and summary['guards_unchanged']
    assert summary.get('processes', summary['cases']) == count
    for key, value in summary.get('artifacts_sha256', {}).items():
        assert sha(E/phase/key) == value
    print(phase, 'passed', flush=True)
