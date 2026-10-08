from pathlib import Path
import difflib
import subprocess

a = Path(__file__).resolve().parent
w = a.parent/'hypercurve'
path = w/'src/bezier_offset.rs'
test = (a/'parallel-normal-source-angle-test.rs').read_text()

def change(s):
    start = s.index('    pub(crate) fn tangent_cross_dot_parallel_linear_combination_sign(')
    end = s.index('    pub(crate) fn tangent_cross_vector_sign(', start)
    old = s[start:end]
    signature_end = old.index('        // A retained oriented unit tangent')
    source_signature = old[:signature_end].replace('pub(crate) fn tangent_cross_dot_parallel_linear_combination_sign', 'fn tangent_cross_dot_parallel_source_linear_combination_sign')
    body = old[signature_end:]
    first = body.index('            let derivative_scale = match parallel')
    last = body.index('        let support = match self.independent_support_system', first)
    body = body[:first] + '            return Ok(Classification::Decided(source_sign));\n        }\n' + body[last:]
    first = body.index('        let derivative_scale = match parallel.parallel_derivative_scale_sign(')
    body = body[:first] + '        Ok(Classification::Decided(source_sign))\n    }\n\n'
    wrapper = old[:signature_end] + '''        let source_sign = match self.tangent_cross_dot_parallel_source_linear_combination_sign(
            parallel, parameter, cross_scale, dot_scale, policy,
        )? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let scale = match parallel.parallel_derivative_scale_sign(parameter, policy)? {
            Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        Ok(Classification::Decided(product_sign(source_sign, scale)))
    }

    /// Signs the same chord relation against the parallel's source tangent.
    /// Circle normal frames use this direction even where the parallel's
    /// derivative reverses. The traversal predicate above applies that scale
    /// only when the actual parallel tangent is requested.
'''
    s = s[:start] + wrapper + source_signature + body + s[end:]
    start = s.index('        } else if let Some(frame) = semicircle.data.frame.chord_normal() {', s.index('    fn selected_parallel_contact_order_to_real('))
    end = s.index('        } else {\n            let anchor = semicircle.source_parallel()', start)
    s = s[:start] + '''        } else if let Some(frame) = semicircle.data.frame.chord_normal() {
            match frame.anchor.tangent_cross_dot_parallel_source_linear_combination_sign(
                parallel, parameter, &(-(radial * &turn)), &tangential, policy,
            )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
''' + s[end:]
    start = s.index('    fn selected_chord_parallel_normal_contact_order_to_real(')
    end = s.index('    /// Returns the original mapped carrier', start)
    section = s[start:end]
    assert section.count('chord.tangent_cross_dot_parallel_linear_combination_sign(') == 1
    section = section.replace('chord.tangent_cross_dot_parallel_linear_combination_sign(', 'chord.tangent_cross_dot_parallel_source_linear_combination_sign(')
    section = section.replace('left_normal(T_parallel)', 'left_normal(T_source)').replace('(T_parallel dot T_chord)', '(T_source dot T_chord)').replace('(T_parallel x T_chord)', '(T_source x T_chord)')
    s = s[:start] + section + s[end:]
    return s + test

parent = subprocess.check_output(['git', 'show', 'HEAD:src/bezier_offset.rs'], cwd=w, text=True)
(a/'parallel-normal-source-angle-parent.rs').write_text(parent+test)
(a/'parallel-normal-source-angle-candidate.rs').write_text(change(parent))
working = path.read_text()
(a/'parallel-normal-source-angle-prior-working.rs').write_text(working)
path.write_text(change(working))
rustfmt = '/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt'
for target in [path, a/'parallel-normal-source-angle-parent.rs', a/'parallel-normal-source-angle-candidate.rs']:
    subprocess.run([rustfmt, '--edition', '2024', '--config', 'skip_children=true', str(target)], check=True)
candidate = (a/'parallel-normal-source-angle-candidate.rs').read_text()
(a/'parallel-normal-source-angle.patch').write_text(''.join(difflib.unified_diff(parent.splitlines(True), candidate.splitlines(True), fromfile='a/src/bezier_offset.rs', tofile='b/src/bezier_offset.rs')))
