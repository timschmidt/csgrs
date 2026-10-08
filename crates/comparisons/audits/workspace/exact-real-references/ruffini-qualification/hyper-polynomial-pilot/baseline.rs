fn multiply_exact_polynomials(first: &[Real], second: &[Real]) -> Vec<Real> {
    if first.is_empty() || second.is_empty() {
        return vec![Real::zero()];
    }
    let mut product = vec![Real::zero(); first.len() + second.len() - 1];
    for (first_power, first_coefficient) in first.iter().enumerate() {
        for (second_power, second_coefficient) in second.iter().enumerate() {
            product[first_power + second_power] += first_coefficient * second_coefficient;
        }
    }
    while product.len() > 1 && product.last().is_some_and(exact_real_is_zero) {
        product.pop();
    }
    product
}

fn exact_real_is_zero(value: &Real) -> bool {
    if let Some(coefficient) = value.exact_rational_ref() {
        return coefficient.is_zero();
    }
    match value.certified_sign_until(hyperlimit::PredicatePolicy::MAX_REFINEMENT_PRECISION) {
        CertifiedRealSign::Known {
            sign: RealSign::Zero,
            ..
        } => true,
        CertifiedRealSign::Known { .. } => false,
        CertifiedRealSign::Unknown { .. } => value
            .exact_rational_normal_form()
            .is_some_and(|value| value.is_zero()),
    }
}
