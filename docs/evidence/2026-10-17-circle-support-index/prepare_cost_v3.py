"""Keep the failed v2 encoder; encode only coordinates proved by the exact oracle."""
import hashlib
import json
from pathlib import Path

W = Path(__file__).resolve().parent

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

rows = json.loads((W / 'cost-preflight-v2/commands.json').read_text())
assert [(r['name'], r['exit_code']) for r in rows] == [('baseline-contracts', 0), ('candidate-contracts', 0), ('mixed-512-0-baseline', 101)]
error = (W / 'cost-preflight-v2/mixed-512-0-baseline.stderr').read_text()
assert 'circle_support_cost_probe.rs:76:45' in error and 'Option::unwrap()' in error
source = (W / 'cost_probe_v2.rs').read_text()
source = source.replace('let x = &base + &offset + Real::from(n as i64 * 32) * &scale;',
                        'let residual = &offset + Real::from(n as i64 * 32) * &scale;\n            let x = &base + &residual;')
source = source.replace('(first, second, base, symbolic, x)', '(first, second, symbolic, x, residual)')
source = source.replace('for (first, second, base, symbolic, x) in &input {', 'for (first, second, symbolic, x, residual) in &input {')
source = source.replace('let expected_x = x + Real::from(3) * &scale;',
                        'let expected_residual = residual + Real::from(3) * &scale;\n                let expected_x = x + Real::from(3) * &scale;')
source = source.replace('let x1 = first_point.x() - base;\n                let x2 = second_point.x() - base;',
                        '// Subtracting pi need not expose an exact_rational_ref. The\n                // preceding strict coordinate equalities prove this exact encoding.\n                let x1 = &expected_residual;\n                let x2 = &expected_residual;')
assert 'for (first, second, base' not in source and 'point.x() - base' not in source
with (W / 'cost_probe_v3.rs').open('x') as f:
    f.write(source)
builder = (W / 'build_cost_v2.py').read_text().replace('cost-build-v2', 'cost-build-v3').replace('cost_probe_v2.rs', 'cost_probe_v3.rs').replace('/csgrs-circle-support-cost-v2', '/csgrs-circle-support-cost-v3')
with (W / 'build_cost_v3.py').open('x') as f:
    f.write(builder)
measure = (W / 'measure_cost_v2.py').read_text().replace("phase + '-v2'", "phase + '-v3'").replace('cost-build-v2', 'cost-build-v3').replace('cost-preflight-v2', 'cost-preflight-v3')
with (W / 'measure_cost_v3.py').open('x') as f:
    f.write(measure)
with (W / 'cost-encoder-correction-v3.json').open('x') as f:
    json.dump({'complete': True, 'failed_attempt_preserved': 'cost-preflight-v2',
               'failure': 'Baseline probe encoder assumed subtracting pi exposed a rational carrier; strict coordinate and both circle equations had passed before encoding failed.',
               'correction': 'Encode the known rational residual only after the unchanged strict coordinate, two-circle and independent 3-4-5 assertions prove that exact coordinate. No kernel or semantic assertions changed.',
               'input_sha256': {str(p): sha(p) for p in [Path(__file__), W / 'cost_probe_v2.rs', W / 'cost_probe_v3.rs', W / 'cost-preflight-v2/commands.json']},
               'kernel_failure_claimed': False, 'performance_claimed': False}, f, indent=2)
    f.write('\n')
print('Prepared v3 encoder and fresh direct-rustc build; v2 artifacts preserved.')
