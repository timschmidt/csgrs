#!/usr/bin/env python3
"""Run fixed normal-form, algebraic and general budget corpora sequentially."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
EXPECTED = '900383daace847820b85b825e0ff0a64d5b9dd491eb988b68ad23559afcff3ed'
BUILD = 'typed-api-prototype-v605-normal-form-replay-build'
UNIT = 'typed-api-prototype-v603-normal-form-scalar-tests'
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def passing(phase):
    p = E / phase
    r = read(p / 'result.json')
    assert r['exit_code'] == 0 and r['source_unchanged'] and r['main_unchanged'], (phase, r)
    assert sha(p / 'source-hashes.json.gz') == EXPECTED


def main():
    subprocess.run([sys.executable, str(E / 'validate_candidate_v2.py'), UNIT,
                    'test', '--release', '-p', 'hyperreal', '--all-features', '--lib'], cwd=ROOT, check=True)
    passing(UNIT)
    clippy = 'typed-api-prototype-v604-normal-form-clippy'
    subprocess.run([sys.executable, str(E / 'validate_candidate_v2.py'), clippy,
                    'clippy', '--release', '-p', 'hyperreal', '-p', 'hyperlimit', '-p', 'hyperlattice',
                    '-p', 'hypercurve', '-p', 'hyperdrc', '--all-targets', '--all-features', '--', '-D', 'warnings'], cwd=ROOT, check=True)
    passing(clippy)
    subprocess.run([sys.executable, str(E / 'validate_candidate_v2.py'), BUILD,
                    'build', '--release', '-p', 'hyperreal', '-p', 'hypercurve', '--all-features'],
                   cwd=ROOT, check=True)
    passing(BUILD)
    for script, phase, extra in [
        ('reproduce_normal_form_budget.py', 'typed-api-prototype-v606-normal-form-replay', [UNIT, 'replay']),
        ('reproduce_normal_form_budget.py', 'typed-api-prototype-v607-normal-form-cost', [UNIT, 'cost']),
        ('reproduce_evaluation_budget_cost.py', 'typed-api-prototype-v608-normal-form-default-cost', []),
        ('reproduce_algebraic_budget_v2.py', 'typed-api-prototype-v609-normal-form-algebraic-cost', [UNIT, 'cost']),
        ('reproduce_evaluation_budget.py', 'typed-api-prototype-v610-normal-form-budget-replay', [UNIT]),
        ('reproduce_algebraic_budget_v2.py', 'typed-api-prototype-v611-normal-form-algebraic-replay', [UNIT, 'replay']),
        ('reproduce_algebraic_wide_budget.py', 'typed-api-prototype-v612-normal-form-algebraic-wide-replay', [UNIT]),
    ]:
        subprocess.run([sys.executable, str(E / script), BUILD, phase, *extra], cwd=ROOT, check=True)
        r = read(E / phase / 'summary.json')
        assert r['source_hashes_sha256'] == EXPECTED and r['guards_unchanged'], phase
    print('Normal-form trial completed with frozen probe inputs and sequential workloads.', flush=True)


if __name__ == '__main__':
    main()
