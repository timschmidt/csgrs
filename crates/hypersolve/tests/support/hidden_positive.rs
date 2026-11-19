fn hidden_positive() -> hyperreal::Real {
    use hyperreal::Real;
    // sin(e + 1) = sin(e)cos(1) + cos(e)sin(1), leaving precisely 2^-2048.
    (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin()
        + Real::from(2).powi_i64(-2048).unwrap()
}
