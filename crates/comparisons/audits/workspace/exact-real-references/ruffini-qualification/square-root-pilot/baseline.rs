fn exact_polynomial_square_root(mut polynomial: Vec<Real>) -> Option<Vec<Real>> {
    while polynomial.len() > 1 && polynomial.last().is_some_and(exact_real_is_zero) {
        polynomial.pop();
    }
    if exact_polynomial_is_zero(&polynomial) {
        return Some(vec![Real::zero()]);
    }
    let degree = polynomial.len() - 1;
    if !degree.is_multiple_of(2) {
        return None;
    }
    let root_degree = degree / 2;
    let leading = polynomial.last()?.clone().sqrt().ok()?;
    if exact_real_is_zero(&leading) {
        return None;
    }
    let twice_leading = Real::from(2_i8) * &leading;
    let twice_leading_reciprocal = strict_reciprocal(&twice_leading)?;
    let mut root = vec![Real::zero(); root_degree + 1];
    root[root_degree] = leading;
    for power in (0..root_degree).rev() {
        let product = multiply_exact_polynomials(&root, &root);
        let known = product
            .get(root_degree + power)
            .cloned()
            .unwrap_or_else(Real::zero);
        root[power] = (polynomial[root_degree + power].clone() - known) * &twice_leading_reciprocal;
    }
    exact_polynomial_is_zero(&subtract_exact_polynomials(
        &multiply_exact_polynomials(&root, &root),
        &polynomial,
    ))
    .then_some(root)
}
