#!/usr/bin/env python3
"""Check residual equivalence and costs sequentially on one frozen candidate."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
source_hash = '94999b4ae4cf57ea63431cc2c2671c96335fa23049870e1abaf222a0c226d080'

def checked(phase):
    r = read(E/phase/'result.json')
    assert r['exit_code'] == 0 and r['source_unchanged'] and r['main_unchanged'], phase
    assert sha(E/phase/'source-hashes.json.gz') == source_hash, phase

def validate(phase, args):
    assert not (E/phase).exists()
    subprocess.run([sys.executable, str(E/'validate_candidate_v2.py'), phase, *args], cwd=ROOT, check=True)
    checked(phase)

unit = 'typed-api-prototype-v1426-acos-residual-unit-tests'
checked(unit)
assert '1064 passed; 0 failed;' in (E/unit/'stdout.log').read_text()
subprocess.run([sys.executable, str(E/'validate_acos_warm_survey_v2.py')], cwd=ROOT, check=True)
caps = read(E/'acos-warm-survey-caps-v1434.json')
assert len(caps['tests']) == 8
for phase in caps['tests']:
    checked(phase)
validate('typed-api-prototype-v1435-acos-residual-clippy', ['clippy', '--release',
    '-p', 'hyperreal', '-p', 'hyperlimit', '-p', 'hyperlattice', '-p', 'hypercurve', '-p', 'hyperdrc',
    '--all-targets', '--all-features', '--', '-D', 'warnings'])
build = 'typed-api-prototype-v1436-acos-residual-build'
validate(build, ['build', '--release', '-p', 'hyperreal', '-p', 'hypercurve', '--all-features'])
for script, phase, extra, count in [
    ('reproduce_acos_warm_cost_v2.py', 'typed-api-prototype-v1437-acos-residual-targeted-cost', [], 72),
    ('reproduce_inverse_trig_budget.py', 'typed-api-prototype-v1438-acos-residual-inverse-cost', [unit, 'cost'], 54),
    ('reproduce_normal_form_budget.py', 'typed-api-prototype-v1439-acos-residual-normal-form-cost', [unit, 'cost'], 54),
    ('reproduce_evaluation_budget_cost.py', 'typed-api-prototype-v1440-acos-residual-default-cost', [], 54),
    ('reproduce_algebraic_budget_v2.py', 'typed-api-prototype-v1441-acos-residual-algebraic-cost', [unit, 'cost'], 54),
    ('reproduce_inverse_trig_budget.py', 'typed-api-prototype-v1442-acos-residual-inverse-replay', [unit, 'replay'], 384),
    ('reproduce_evaluation_budget.py', 'typed-api-prototype-v1443-acos-residual-evaluation-replay', [unit], 384),
    ('reproduce_algebraic_budget_v2.py', 'typed-api-prototype-v1444-acos-residual-algebraic-replay', [unit, 'replay'], 288),
    ('reproduce_algebraic_wide_budget.py', 'typed-api-prototype-v1445-acos-residual-wide-replay', [unit, 'replay'], 144),
    ('reproduce_normal_form_budget.py', 'typed-api-prototype-v1446-acos-residual-normal-form-replay', [unit, 'replay'], 384),
    ('reproduce_integer_scale_cost.py', 'typed-api-prototype-v1447-acos-residual-integer-cost', [], 72),
]:
    assert not (E/phase).exists()
    subprocess.run([sys.executable, str(E/script), build, phase, *extra], cwd=ROOT, check=True)
    summary = read(E/phase/'summary.json')
    assert summary['source_hashes_sha256'] == source_hash
    assert summary.get('guards_unchanged') is True
    assert summary.get('processes', summary.get('cases')) == count, (phase, summary.keys())
    for name, digest in summary.get('artifacts_sha256', {}).items():
        assert sha(E/phase/name) == digest
    print(phase, 'passed', flush=True)
print('Frozen residual checks and cost/replay processes completed.', flush=True)
