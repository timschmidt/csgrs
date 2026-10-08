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
        // Coefficients through `power` are still exact zero. Only this diagonal
        // of the square is consumed; keep the full convolution's summation order.
        let mut known = Real::zero();
        for first_power in power + 1..root_degree {
            known += &root[first_power] * &root[root_degree + power - first_power];
        }
        root[power] = (polynomial[root_degree + power].clone() - known) * &twice_leading_reciprocal;
    }
    exact_polynomial_is_zero(&subtract_exact_polynomials(
        &multiply_exact_polynomials(&root, &root),
        &polynomial,
    ))
    .then_some(root)
}
