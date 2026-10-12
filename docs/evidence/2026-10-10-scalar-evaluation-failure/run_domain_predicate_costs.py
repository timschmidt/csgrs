"""Measure seven unchanged scalar cost controls for the conditional domain-predicate extraction."""
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
    build = 'typed-api-prototype-v1766-domain-predicate-release-build'
    units = 'typed-api-prototype-v1764-domain-predicate-scalar-tests'
    for phase in [build, units]:
        result = read(E/phase/'result.json')
        assert result['exit_code'] == 0 and result['main_unchanged'] and result['source_unchanged']
    expected = sha(E/'typed-api-prototype-v1761-domain-predicate-verus/source-hashes.json.gz')
    assert sha(E/build/'source-hashes.json.gz') == expected
    out = E/'domain-predicate-cost-orchestration-v1779.json'
    assert not out.exists()
    rows = []
    commands = [
        ('atan', 'reproduce_atan_zero_term_cost.py', 'typed-api-prototype-v1773-domain-predicate-atan-cost', []),
        ('integer', 'reproduce_integer_scale_cost.py', 'typed-api-prototype-v1774-domain-predicate-integer-cost', []),
        ('inverse_trig', 'reproduce_inverse_trig_budget.py', 'typed-api-prototype-v1775-domain-predicate-inverse-cost', [units, 'cost']),
        ('normal_form', 'reproduce_normal_form_budget.py', 'typed-api-prototype-v1776-domain-predicate-normal-form-cost', [units, 'cost']),
        ('default', 'reproduce_evaluation_budget_cost.py', 'typed-api-prototype-v1777-domain-predicate-default-cost', []),
        ('algebraic', 'reproduce_algebraic_budget_v2.py', 'typed-api-prototype-v1778-domain-predicate-algebraic-cost', [units, 'cost']),
        ('acos', 'reproduce_acos_warm_cost_v2.py', 'typed-api-prototype-v1779-domain-predicate-acos-cost', []),
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
