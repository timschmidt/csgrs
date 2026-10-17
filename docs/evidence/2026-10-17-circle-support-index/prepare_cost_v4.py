"""Extend the exact encoder to symbolic y carriers; keep both failed attempts."""
import hashlib
import json
from pathlib import Path
W = Path(__file__).resolve().parent

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

error = (W / 'cost-preflight-v3/mixed-512-0-baseline.stderr').read_text()
assert 'circle_support_cost_probe.rs:81:58' in error and 'Option::unwrap()' in error
source = (W / 'cost_probe_v3.rs').read_text()
anchor = '                let x1 = &expected_residual;'
extra = '''                // Preserve y witness order using a strict sign proof, then
                // prove the chosen rational encoding equals the actual value.
                let encode_y = |actual: &Real| {
                    let exact = match hyperlimit::compare_reals(actual, &Real::zero(), PredicatePolicy::STRICT).value() {
                        Some(std::cmp::Ordering::Greater) => expected_y.clone(),
                        Some(std::cmp::Ordering::Less) => -&expected_y,
                        other => panic!("nonzero secant y needs a strict sign proof: {other:?}"),
                    };
                    same(actual, &exact);
                    exact.exact_rational_ref().unwrap().to_string()
                };
'''
assert source.count(anchor) == 1
source = source.replace(anchor, extra + anchor)
source = source.replace('first_point.y().exact_rational_ref().unwrap().to_string()', 'encode_y(first_point.y())')
source = source.replace('second_point.y().exact_rational_ref().unwrap().to_string()', 'encode_y(second_point.y())')
with (W / 'cost_probe_v4.rs').open('x') as f:
    f.write(source)
builder = (W / 'build_cost_v3.py').read_text().replace('cost-build-v3', 'cost-build-v4').replace('cost_probe_v3.rs', 'cost_probe_v4.rs').replace('/csgrs-circle-support-cost-v3', '/csgrs-circle-support-cost-v4')
with (W / 'build_cost_v4.py').open('x') as f:
    f.write(builder)
measure = (W / 'measure_cost_v3.py').read_text().replace("phase + '-v3'", "phase + '-v4'").replace('cost-build-v3', 'cost-build-v4').replace('cost-preflight-v3', 'cost-preflight-v4')
with (W / 'measure_cost_v4.py').open('x') as f:
    f.write(measure)
heap = (W / 'heap_cost_v3.py').read_text().replace('heap-cost-v3', 'heap-cost-v4').replace('cost-matched-v3', 'cost-matched-v4').replace('cost-build-v3', 'cost-build-v4')
with (W / 'heap_cost_v4.py').open('x') as f:
    f.write(heap)
with (W / 'cost-encoder-correction-v4.json').open('x') as f:
    json.dump({'complete': True, 'failed_attempts_preserved': ['cost-preflight-v2', 'cost-preflight-v3'],
               'failure': 'The same structural-rational assumption also failed for a certified symbolic y coordinate in the baseline.',
               'correction': 'A strict sign proof selects positive or negative 4*scale, and an additional strict equality must prove that encoding equals the actual y. The original coordinate, circle-equation and 3-4-5 assertions remain unchanged.',
               'input_sha256': {str(p): sha(p) for p in [Path(__file__), W / 'cost_probe_v3.rs', W / 'cost_probe_v4.rs', W / 'cost-preflight-v3/commands.json']},
               'kernel_failure_claimed': False, 'performance_claimed': False}, f, indent=2)
    f.write('\n')
print('Prepared a strict value encoder for both coordinates; no kernel changes.')
