from pathlib import Path
import hashlib, json, shutil, subprocess

A = Path(__file__).resolve().parent
source = Path('/tmp/hypercurve-circle-closure-probe5-2026-09-23')
root = Path('/tmp/hypercurve-fiber-ring-arithmetic-probe1-2026-09-23')
assert json.loads((A/'recognized-circle-evidence-20260923-v5-terminal.json').read_text())['all_processes_reaped']
assert not root.exists()
bindings = json.loads((A/'circle-closure-20260923-probe5-sources.json').read_text())
for name, sha in bindings.items():
    assert hashlib.sha256((source/name).read_bytes()).hexdigest() == sha, name
    (root/name).parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source/name, root/name)

p = root/'hypersolve/src/algebraic_fiber.rs'
s = p.read_text()
a = s.index('fn trim_local_image_polynomial(')
b = s.index('\nfn local_image_polynomial_add(', a)
s = s[:a]+'''// Image-ring operations need a degree bound, not an exact leading-degree decision.
fn trim_local_image_polynomial(polynomial: &mut LocalImagePolynomial) {
    while polynomial.len() > 1
        && polynomial.last().is_some_and(local_field_element_is_structurally_zero)
    {
        polynomial.pop();
    }
    if polynomial.is_empty() {
        polynomial.push(LocalFieldElement::zero());
    }
}
''' + s[b:]
s = s.replace('trim_local_image_polynomial(&mut result, field)?;', 'trim_local_image_polynomial(&mut result);')
s = s.replace('trim_local_image_polynomial(&mut determinant, field)?;', 'trim_local_image_polynomial(&mut determinant);')
s = s.replace('trim_local_image_polynomial(&mut image, field)?;', 'trim_local_polynomial(&mut image, field)?;')
s = s.replace('Some(modulus) => local_polynomial_remainder(product, modulus, field),', 'Some(modulus) => local_polynomial_remainder_untrimmed(product, modulus, field),')
a = s.index('fn local_polynomial_remainder(')
b = s.index('\nfn trim_local_polynomial(', a)
original = s[a:b]
body = original.replace('fn local_polynomial_remainder(', 'fn local_polynomial_remainder_untrimmed(', 1)
body = body.replace('    trim_local_polynomial(&mut dividend, field)?;', '    trim_local_image_polynomial(&mut dividend);')
body = body.replace('while dividend.len() >= divisor.len() && !(dividend.len() == 1 && dividend[0].is_zero(field)?) {', 'while dividend.len() >= divisor.len() {')
# A nonzero constant is a unit; its quotient ring has only the zero class.
body = body.replace('    let divisor_degree = divisor.len() - 1;', '    if divisor.len() == 1 { return Ok(local_image_polynomial_zero()); }\n    let divisor_degree = divisor.len() - 1;')
body = body.replace('    Ok(dividend)', '    trim_local_image_polynomial(&mut dividend);\n    Ok(dividend)')
s = s[:a]+'''fn local_polynomial_remainder(
    dividend: Vec<LocalFieldElement>,
    divisor: &[LocalFieldElement],
    field: &mut LocalAlgebraicField,
) -> Result<Vec<LocalFieldElement>, LocalFieldError> {
    let mut remainder = local_polynomial_remainder_untrimmed(dividend, divisor, field)?;
    trim_local_polynomial(&mut remainder, field)?;
    Ok(remainder)
}

'''+body+s[b:]
# Denominator clearing changes no degree and does not need a zero decision.
a = s.index('fn local_polynomial_clear_denominators(')
b = s.index('\nfn local_polynomial_remainder(', a)
clear = s[a:b].replace('    trim_local_polynomial(&mut polynomial, field)?;\n', '')
s = s[:a]+clear+s[b:]
p.write_text(s)
subprocess.run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt', '--edition', '2024', '--config', 'skip_children=true', str(p)], check=True)
bindings['hypersolve/src/algebraic_fiber.rs'] = hashlib.sha256(p.read_bytes()).hexdigest()
for name, sha in bindings.items(): assert hashlib.sha256((root/name).read_bytes()).hexdigest() == sha, name
(A/'fiber-ring-arithmetic-20260923-probe1-sources.json').write_text(json.dumps(bindings, indent=2)+'\n')
runner = (A/'run-circle-closure-probe5-20260923.py').read_text().replace(str(source),str(root)).replace('circle-closure-20260923-probe5','fiber-ring-arithmetic-20260923-probe1')
a = runner.index('selected=[')
b = runner.index('\n]\n', a)+3
runner = runner[:a]+"selected=[('selected_fiber_transverse_mapped_cut_inverts_by_point',120)]\n"+runner[b:]
(A/'run-fiber-ring-arithmetic-probe1-20260923.py').write_text(runner)
print('Prepared ring arithmetic diagnostic', bindings['hypersolve/src/algebraic_fiber.rs'])
