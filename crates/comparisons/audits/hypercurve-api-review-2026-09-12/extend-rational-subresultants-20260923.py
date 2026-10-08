from pathlib import Path
root=Path('/tmp/hypercurve-gcd-replay-2026-09-23/hypersolve')
base=Path('/home/tim/Documents/GitHub/workspace/hypersolve')
p=root/'src/integer_interpolation/bivariate.rs'
s=(base/'src/integer_interpolation/bivariate.rs').read_text()
start=s.index('/// Signed regular')
end=s.index('\nfn trim(')
s=s[:start]+'''/// Subresultant rows after the first polynomial, beginning with the second.
/// Independent positive rational scales make the inputs integral. Brown's PRS
/// uses only exact polynomial division in Z[t], including abnormal degree gaps.
/// The caller must certify every retained leading coefficient at its selected
/// root before using the sequence there. No irreducibility is assumed.
pub(crate) fn rational_fiber_subresultant_rows(
    first: &[Vec<Real>],
    second: &[Vec<Real>],
) -> Option<Vec<Vec<Vec<Real>>>> {
    let first = integer_coefficients(first)?;
    let mut second = integer_coefficients(second)?;
    let gap = first.len().checked_sub(second.len())?;
    let mut next = pseudo_remainder(first, &second)?;
    if gap % 2 == 0 {
        negate(&mut next);
    }
    let mut leading = second.last()?.clone();
    let mut scale = power(&leading, gap);
    for value in &mut scale {
        *value = -std::mem::take(value);
    }
    let mut sequence = vec![second.clone()];
    while !next.is_empty() {
        let gap = second.len().checked_sub(next.len())?;
        sequence.push(next.clone());
        if next.len() == 1 {
            break;
        }
        let mut divisor = product(&leading, &power(&scale, gap));
        for value in &mut divisor {
            *value = -std::mem::take(value);
        }
        let remainder = pseudo_remainder(second, &next)?;
        let remainder = remainder
            .iter()
            .map(|coefficient| {
                integer_polynomial_exact_quotient(coefficient, &divisor)
                    .map(trim_integer_polynomial)
            })
            .collect::<Option<Polynomial>>()?;
        leading = next.last()?.clone();
        let negative_leading = leading.iter().map(|value| -value).collect::<Vec<_>>();
        scale = if gap > 1 {
            integer_polynomial_exact_quotient(
                &power(&negative_leading, gap),
                &power(&scale, gap - 1),
            )?
        } else {
            negative_leading
        };
        second = next;
        next = trim(remainder);
    }
    Some(
        sequence
            .into_iter()
            .map(|polynomial| {
                polynomial
                    .into_iter()
                    .map(|coefficient| {
                        coefficient
                            .into_iter()
                            .map(Rational::from_bigint)
                            .map(Real::from)
                            .collect()
                    })
                    .collect()
            })
            .collect(),
    )
}

fn integer_coefficients(coefficients: &[Vec<Real>]) -> Option<Polynomial> {
    let rationals = coefficients
        .iter()
        .flatten()
        .map(Real::exact_rational_ref)
        .collect::<Option<Vec<_>>>()?;
    let mut primitive = Rational::primitive_bigint_ratio(&rationals).into_iter();
    Some(trim(
        coefficients
            .iter()
            .map(|coefficient| {
                trim_integer_polynomial(
                    coefficient
                        .iter()
                        .map(|_| {
                            primitive
                                .next()
                                .expect("primitive integer ratios preserve coefficient count")
                        })
                        .collect(),
                )
            })
            .collect(),
    ))
}

fn power(polynomial: &[BigInt], exponent: usize) -> Vec<BigInt> {
    let mut result = vec![BigInt::one()];
    for _ in 0..exponent {
        result = product(&result, polynomial);
    }
    result
}

fn negate(polynomial: &mut Polynomial) {
    for coefficient in polynomial {
        for value in coefficient {
            *value = -std::mem::take(value);
        }
    }
}
''' + s[end:]
s=s.replace('Complete\n    // the nominal square multiplier', 'Complete\n    // the nominal leading-coefficient multiplier')
p.write_text(s)
p=root/'src/integer_interpolation.rs'
s=(base/'src/integer_interpolation.rs').read_text().replace('bivariate::regular_rational_fiber_sturm_rows','bivariate::rational_fiber_subresultant_rows');p.write_text(s)
p=root/'src/algebraic_fiber.rs';s=p.read_text()
start=s.index('/// Preserve a regular rational PRS')
end=s.index('\nfn local_sturm_sequence', start)
old=s[start:end]
old=old[old.index('fn rational_local_sturm_rows'):]
old=old.replace('rational_local_sturm_rows(', 'rational_local_subresultant_rows(').replace('    first: &[LocalFieldElement],\n', '    first: &[LocalFieldElement],\n    second: &[LocalFieldElement],\n',1)
old=old.replace('        .iter()\n        .any(|coefficient| coefficient.denominator.is_some())','        .iter()\n        .chain(second)\n        .any(|coefficient| coefficient.denominator.is_some())',1)
old=old.replace('    let coefficients = first\n        .iter()\n        .map(|coefficient| coefficient.numerator.clone())\n        .collect::<Vec<_>>();\n    let rows = crate::integer_interpolation::regular_rational_fiber_sturm_rows(&coefficients)?;', '''    let coefficients = |polynomial: &[LocalFieldElement]| {
        polynomial.iter().map(|coefficient| coefficient.numerator.clone()).collect::<Vec<_>>()
    };
    let rows = crate::integer_interpolation::rational_fiber_subresultant_rows(
        &coefficients(first), &coefficients(second),
    )?;''')
old=old.replace('    let mut sequence = Vec::with_capacity(rows.len());', '''    if first.last()?.is_zero(&mut proof).ok()? {
        return None;
    }
    let mut sequence = Vec::with_capacity(rows.len());''',1)
s=s[:start]+'''/// Specialize a rational subresultant sequence only after STRICT replay of
/// its leading coefficients. Exact polynomial division commutes with this
/// specialization while its divisors remain nonzero; a whole zero row proves
/// termination. A nonzero row losing degree retains the general field path.
'''+old+s[end:]
old='''    if let Some(rows) = rational_local_sturm_rows(&first, field) {
        let mut sequence = vec![first];
        sequence.extend(rows);
        return Ok(sequence);
    }
    let second = derivative_local_polynomial(&first, field)?;'''
new='''    let second = derivative_local_polynomial(&first, field)?;
    if let Some(mut rows) = rational_local_subresultant_rows(&first, &second, field) {
        // With consecutive degrees, every Brown PRS divisor is a square.
        // The signed Sturm rows therefore have signs +,-,-,+,+,-,-,...
        // after F. Degree gaps retain the general signed field sequence.
        if std::iter::once(first.len()).chain(rows.iter().map(Vec::len))
            .collect::<Vec<_>>().windows(2).all(|pair| pair[0] == pair[1] + 1)
        {
            for (index, row) in rows.iter_mut().enumerate() {
                if ((index + 1) / 2) % 2 == 1 {
                    for coefficient in row {
                        coefficient.negate();
                    }
                }
            }
            let mut sequence = vec![first];
            sequence.extend(rows);
            return Ok(sequence);
        }
    }'''
assert old in s;s=s.replace(old,new,1)
needle='''    while !local_polynomial_is_zero(&second, field)? {
        let remainder = local_polynomial_remainder(first, &second, field)?;'''
assert needle in s
s=s.replace(needle,'''    if let Some(mut rows) = rational_local_subresultant_rows(&first, &second, field) {
        if let Some(last) = rows.pop() {
            return Ok(last);
        }
    }
'''+needle,1)
# Diagnostic only; tests migrated before production qualification.
p.write_text(s)
