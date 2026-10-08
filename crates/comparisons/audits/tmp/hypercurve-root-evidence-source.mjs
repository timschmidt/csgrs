import {readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';

const files = [
    'hypersolve/src/lib.rs',
    'hypersolve/src/algebraic.rs',
    'hypersolve/src/algebraic_binary.rs',
    'hypersolve/src/algebraic_fiber.rs',
    'hypersolve/src/algebraic_mobius.rs',
    'hypersolve/src/algebraic_polynomial_image.rs',
    'hypersolve/src/algebraic_rational_image.rs',
    'hypersolve/src/algebraic_sqrt.rs',
    'hypersolve/src/algebraic_tensor_image.rs',
    'hypercurve/src/bezier_algebraic_image.rs',
    'hypercurve/src/bezier_arrangement.rs',
    'hypercurve/src/bezier_offset.rs',
    'hypercurve/src/bezier_parameter.rs',
    'hypercurve/src/bezier_tangent_order.rs',
];
const rows = files.map(path => {
    const hash = (root, beforeTests = false) => {
        let text = readFileSync(`${root}/${path}`, 'utf8');
        if (beforeTests) {
            const marker = '\nmod conversion_tests {';
            const boundary = text.indexOf(marker);
            if (boundary < 0) throw new Error(`Missing test boundary: ${path}`);
            text = text.slice(0, boundary);
        }
        return createHash('sha256').update(text).digest('hex');
    };
    const live = hash('/home/tim/Documents/GitHub/workspace');
    const frozen = hash('/tmp/hypercurve-dyadic-bernstein-control.a5ZKq2');
    const result = {path, live, frozen, equal: live === frozen};
    if (!result.equal) {
        result.pre_conversion_tests_live = hash('/home/tim/Documents/GitHub/workspace', true);
        result.pre_conversion_tests_frozen = hash('/tmp/hypercurve-dyadic-bernstein-control.a5ZKq2', true);
        result.pre_conversion_tests_equal = result.pre_conversion_tests_live === result.pre_conversion_tests_frozen;
    }
    return result;
});
console.log(JSON.stringify(rows, null, 2));
