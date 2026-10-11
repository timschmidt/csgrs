#!/usr/bin/env python3
"""Run fixed normal-form, algebraic and general budget corpora sequentially."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
EXPECTED = '291569171baff3f4521505719d4110258e77d4ebf0510680e1d1e7ab73ae4cfa'
BUILD = 'typed-api-prototype-v589-normal-form-replay-build'
UNIT = 'typed-api-prototype-v588-normal-form-scalar-tests'
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def passing(phase):
    p = E / phase
    r = read(p / 'result.json')
    assert r['exit_code'] == 0 and r['source_unchanged'] and r['main_unchanged'], (phase, r)
    assert sha(p / 'source-hashes.json.gz') == EXPECTED


def main():
    passing(UNIT)
    subprocess.run([sys.executable, str(E / 'validate_candidate_v2.py'), BUILD,
                    'build', '--release', '-p', 'hyperreal', '-p', 'hypercurve', '--all-features'],
                   cwd=ROOT, check=True)
    passing(BUILD)
    for script, phase, extra in [
        ('reproduce_normal_form_budget.py', 'typed-api-prototype-v590-normal-form-replay', [UNIT, 'replay']),
        ('reproduce_normal_form_budget.py', 'typed-api-prototype-v591-normal-form-cost', [UNIT, 'cost']),
        ('reproduce_evaluation_budget_cost.py', 'typed-api-prototype-v592-normal-form-default-cost', []),
        ('reproduce_algebraic_budget_v2.py', 'typed-api-prototype-v593-normal-form-algebraic-cost', [UNIT, 'cost']),
        ('reproduce_evaluation_budget.py', 'typed-api-prototype-v594-normal-form-budget-replay', [UNIT]),
        ('reproduce_algebraic_budget_v2.py', 'typed-api-prototype-v595-normal-form-algebraic-replay', [UNIT, 'replay']),
        ('reproduce_algebraic_wide_budget.py', 'typed-api-prototype-v596-normal-form-algebraic-wide-replay', [UNIT]),
    ]:
        subprocess.run([sys.executable, str(E / script), BUILD, phase, *extra], cwd=ROOT, check=True)
        r = read(E / phase / 'summary.json')
        assert r['source_hashes_sha256'] == EXPECTED and r['guards_unchanged'], phase
    print('Normal-form trial completed with frozen probe inputs and sequential workloads.', flush=True)


if __name__ == '__main__':
    main()
