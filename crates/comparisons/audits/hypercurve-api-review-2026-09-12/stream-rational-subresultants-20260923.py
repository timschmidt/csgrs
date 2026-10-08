from pathlib import Path
repo=Path('/tmp/hypercurve-gcd-replay-2026-09-23/hypersolve')
p=repo/'src/integer_interpolation/bivariate.rs';s=p.read_text().replace('rational_fiber_subresultant_rows(', 'rational_fiber_subresultants(')
s=s.replace('    second: &[Vec<Real>],\n) -> Option<Vec<Vec<Vec<Real>>>> {','''    second: &[Vec<Real>],
    mut retain: impl FnMut(Vec<Vec<Real>>) -> Option<bool>,
) -> Option<()> {''',1)
s=s.replace('    let gap = first.len().checked_sub(second.len())?;','''    let mut emit = |row: &Polynomial| {
        retain(row.iter().map(|coefficient| {
            coefficient.iter().cloned().map(Rational::from_bigint).map(Real::from).collect()
        }).collect())
    };
    if !emit(&second)? {
        return Some(());
    }
    let gap = first.len().checked_sub(second.len())?;''',1)
s=s.replace('    let mut sequence = vec![second.clone()];\n','',1)
s=s.replace('        sequence.push(next.clone());\n        if next.len() == 1 {','''        if !emit(&next)? || next.len() == 1 {''',1)
start=s.index('    Some(\n        sequence')
end=s.index('\nfn integer_coefficients',start)
s=s[:start]+'''    Some(())
}
'''+s[end:]
s=s.replace('/// root before using the sequence there. No irreducibility is assumed.','''/// root before using the sequence there. The callback stops on `Some(false)`
/// after selected termination; `None` declines the sequence. Thus unused
/// rows are never constructed. No irreducibility is assumed.''')
p.write_text(s)
p=repo/'src/integer_interpolation.rs';s=p.read_text().replace('bivariate::rational_fiber_subresultant_rows','bivariate::rational_fiber_subresultants');p.write_text(s)
p=repo/'src/algebraic_fiber.rs';s=p.read_text()
start=s.index('    let rows = crate::integer_interpolation::rational_fiber_subresultant_rows(')
end=s.index('    // Keep prior inverses',start)
s=s[:start]+'''    let mut proof = LocalAlgebraicField::new(&field.root, PredicatePolicy::STRICT).ok()?;
    if first.last()?.is_zero(&mut proof).ok()? {
        return None;
    }
    let mut sequence = Vec::new();
    crate::integer_interpolation::rational_fiber_subresultants(
        &coefficients(first),
        &coefficients(second),
        |row| {
            let degree = row.len();
            let mut row = row
                .into_iter()
                .map(|coefficient| LocalFieldElement::from_polynomial(coefficient, &proof))
                .collect::<Result<Vec<_>, _>>()
                .ok()?;
            trim_local_polynomial(&mut row, &mut proof).ok()?;
            if local_polynomial_is_zero(&row, &mut proof).ok()? {
                return (!sequence.is_empty()).then_some(false);
            }
            if row.len() != degree {
                return None;
            }
            sequence.push(row);
            Some(true)
        },
    )?;
'''+s[end:]
# Avoid a temporary degree allocation in the Sturm regularity check.
a=s.index('        if std::iter::once(first.len())',s.index('fn local_sturm_sequence'))
b=s.index('        {',a)
s=s[:a]+'''        if std::iter::once(&first).chain(&rows).zip(&rows)
            .all(|(previous, next)| previous.len() == next.len() + 1)
'''+s[b:]
p.write_text(s)
