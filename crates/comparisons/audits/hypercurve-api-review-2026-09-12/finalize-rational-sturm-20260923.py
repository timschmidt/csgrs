from pathlib import Path
import re, shutil
repo=Path('/tmp/hypercurve-contact-isolation-2026-09-23/hypersolve')
audit=Path(__file__).resolve().parent
(audit/'contact-fiber-20260923-fraction_free_sturm.rs').write_bytes((repo/'src/algebraic_fiber.rs').read_bytes())
p=repo/'src/integer_interpolation/bivariate.rs'
s=p.read_text().replace('rational_fiber_sturm_sequence','regular_rational_fiber_sturm_rows')
start=s.index('/// A subresultant')
end=s.index('pub(crate) fn', start)
s=s[:start]+'''/// Signed regular subresultant rows after the source polynomial, beginning
/// with its derivative. One common positive rational scale makes the input
/// integral. Brown's regular PRS divides only by squares of preceding leading
/// coefficient polynomials; exact division certifies every constructed row.
/// The caller must certify those leading coefficients on its selected root.
/// Degree gaps and nonrational payloads retain the general field algorithm.
'''+s[end:]
s=s.replace('    let mut previous_scale = negated(&leading);\n','')
s=s.replace('''        let divisor = negated(&product(&leading, &power(&previous_scale, gap)));''','''        let divisor = product(&leading, &leading);''')
start=s.index('        previous_scale = if gap > 1 {')
end=s.index('        second = next;',start)
s=s[:start]+s[end:]
start=s.index('fn negated('); end=s.index('fn product(',start);s=s[:start]+s[end:]
start=s.index('fn power('); end=s.index('fn pseudo_remainder(',start);s=s[:start]+s[end:]
s=s.replace('''    let scale = power(leading, steps);
    for coefficient in &mut dividend {
        *coefficient = product(coefficient, &scale);
    }''','''    // A cancellation may skip a power during pseudo-division. Complete
    // the nominal square multiplier even when fewer eliminations were needed.
    for _ in 0..steps {
        for coefficient in &mut dividend {
            *coefficient = product(coefficient, leading);
        }
    }''')
p.write_text(s)
p=repo/'src/integer_interpolation.rs'
p.write_text(p.read_text().replace('rational_fiber_sturm_sequence','regular_rational_fiber_sturm_rows'))
p=repo/'src/algebraic_fiber.rs'
s=p.read_text().replace('rational_fiber_sturm_sequence','regular_rational_fiber_sturm_rows')
s=s.replace('    let start = std::time::Instant::now();\n','').replace('    eprintln!("PRS begin degree={}", first.len() - 1);\n','')
s=re.sub(r'    eprintln!\(\n        "PRS constructed.*?\n    \);\n','',s,flags=re.S)
s=re.sub(r'        eprintln!\(\n            "PRS specialized.*?\n        \);\n','',s,flags=re.S)
s=s.replace('''    proof.policy = field.policy;
    proof.observe_certainty(field.certainty);
    proof.refinement_steps += field.refinement_steps;
    *field = proof;
    eprintln!("PRS complete elapsed={:?}", start.elapsed());''','''    // Keep prior inverses and sign evidence; place the new STRICT decisions
    // first when an earlier policy-dependent query used the same polynomial.
    proof.signed_polynomials.append(&mut field.signed_polynomials);
    field.signed_polynomials = proof.signed_polynomials;
    field.root = proof.root;
    field.refinement_steps += proof.refinement_steps;
    field.observe_certainty(proof.certainty);''')
p.write_text(s)
fixture=repo/'tests/data/nonph_fillet_circle_contact.json'
fixture.parent.mkdir(parents=True,exist_ok=True)
shutil.copy2(repo/'examples/contact_fiber_probe_20260923.json',fixture)
print('Clean regular PRS candidate prepared.')
