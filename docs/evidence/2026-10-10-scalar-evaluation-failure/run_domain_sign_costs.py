"""Measure seven unchanged scalar cost controls for the sign-domain extension."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    build = 'typed-api-prototype-v1624-domain-sign-release-build'
    units = 'typed-api-prototype-v1622-domain-sign-scalar-tests'
    for phase in [build, units]:
        result = read(E/phase/'result.json')
        assert result['exit_code'] == 0 and result['main_unchanged'] and result['source_unchanged']
    expected = read(E/'domain-sign-retaining-caps-v1621.json')['source_record_sha256']
    assert sha(E/build/'source-hashes.json.gz') == expected
    out = E/'domain-sign-cost-orchestration-v1637.json'
    assert not out.exists()
    rows = []
    commands = [
        ('atan', 'reproduce_atan_zero_term_cost.py', 'typed-api-prototype-v1631-domain-sign-atan-cost', []),
        ('integer', 'reproduce_integer_scale_cost.py', 'typed-api-prototype-v1632-domain-sign-integer-cost', []),
        ('inverse_trig', 'reproduce_inverse_trig_budget.py', 'typed-api-prototype-v1633-domain-sign-inverse-cost', [units, 'cost']),
        ('normal_form', 'reproduce_normal_form_budget.py', 'typed-api-prototype-v1634-domain-sign-normal-form-cost', [units, 'cost']),
        ('default', 'reproduce_evaluation_budget_cost.py', 'typed-api-prototype-v1635-domain-sign-default-cost', []),
        ('algebraic', 'reproduce_algebraic_budget_v2.py', 'typed-api-prototype-v1636-domain-sign-algebraic-cost', [units, 'cost']),
        ('acos', 'reproduce_acos_warm_cost_v2.py', 'typed-api-prototype-v1637-domain-sign-acos-cost', []),
    ]
    for family, helper, phase, extra in commands:
        assert not (E/phase).exists()
        command = [sys.executable, str(E/helper), build, phase, *extra]
        started = time.monotonic()
        result = subprocess.run(command, cwd=ROOT)
        row = {'family': family, 'phase': phase, 'command': command,
               'helper_sha256': sha(E/helper), 'exit_code': result.returncode,
               'seconds': time.monotonic()-started}
        rows.append(row)
        if result.returncode == 0:
            summary = read(E/phase/'summary.json')
            assert summary['source_hashes_sha256'] == expected and summary['guards_unchanged']
            row['processes'] = summary['processes']
            row['summary_sha256'] = sha(E/phase/'summary.json')
        out.write_text(json.dumps({'source_record_sha256': expected,
            'runner_sha256': sha(Path(__file__)), 'completed': len(rows) == len(commands) and result.returncode == 0,
            'results': rows}, indent=2)+'\n')
        assert result.returncode == 0, row
    assert sum(row['processes'] for row in rows) == 468
    print('All seven cost families completed: 468 separate processes.', flush=True)


if __name__ == '__main__':
    main()
