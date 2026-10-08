start=t.index('fn algebraic_selected_fiber_root_predicate_sign(');end=t.index('fn validate_selected_fiber_pair_base(',start);part=t[start:end]
part=part.replace('        let restricted = predicate.substitute_affine(', '        if probe { eprintln!("selected-predicate affine-begin"); }\n        let restricted = predicate.substitute_affine(',1)
part=part.replace('        if let Some(sign) =\n            bivariate_unit_square_strict_bernstein_sign(', '        if probe { eprintln!("selected-predicate affine-end bernstein-begin"); }\n        if let Some(sign) =\n            bivariate_unit_square_strict_bernstein_sign(',1)
part=part.replace('        if policy.has_bounded_exact_predicate_budget() {', '        if probe { eprintln!("selected-predicate bernstein-end"); }\n        if policy.has_bounded_exact_predicate_budget() {',1)
part=part.replace('        if let Some(enclosure) = RealInterval::evaluate_bivariate_power_basis(', '        if probe { eprintln!("selected-predicate dyadic-begin"); }\n        let coefficient_bits = refinement_steps.saturating_add(64).min(i32::MAX as usize) as i32;\n        let enclosure = RealInterval::__probe_bivariate_dyadic(',1)
needle2='''            &RealInterval {
                lower: latest.lower.clone(),
                upper: latest.upper.clone(),
            },
        )
        .and_then(|interval| interval.strict_nonzero_sign())
        {'''
replacement='''            &RealInterval {
                lower: latest.lower.clone(),
                upper: latest.upper.clone(),
            },
            -coefficient_bits,
        )
        .and_then(|interval| interval.strict_nonzero_sign());
        if probe { eprintln!("selected-predicate dyadic-end sign={:?}", enclosure); }
        if let Some(enclosure) = enclosure {'''
assert needle2 in part;part=part.replace(needle2,replacement,1);t=t[:start]+part+t[end:]
needle2='    fn evaluate_bivariate_power_basis(\n'
helper=r'''    fn __probe_bivariate_dyadic(
        polynomial: &BivariatePolynomial,
        first: &Self,
        second: &Self,
        precision: i32,
    ) -> Option<Self> {
        let round = |value: &Self| Some(Self {
            lower: Real::new(value.lower.certified_dyadic_interval(precision)?[0].clone()),
            upper: Real::new(value.upper.certified_dyadic_interval(precision)?[1].clone()),
        });
        let first = round(first)?;
        let second = round(second)?;
        let zero = || Self { lower: Real::zero(), upper: Real::zero() };
        let mut value = zero();
        for row in polynomial.coefficients.iter().rev() {
            let mut row_value = zero();
            for coefficient in row.iter().rev() {
                let [lower, upper] = coefficient.certified_dyadic_interval(precision)?;
                row_value = row_value.multiply(&second)?.add(&Self { lower: Real::new(lower), upper: Real::new(upper) });
            }
            value = value.multiply(&first)?.add(&row_value);
        }
        Some(value)
    }

'''
assert needle2 in t;t=t.replace(needle2,helper+needle2,1)
