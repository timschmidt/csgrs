// Exact integer square-root floor/ceiling shared by primitive enclosures.
// The floor has at most 64 bits, so its square and ceiling fit u128.
pub(crate) fn integer_sqrt_bounds(value: u128) -> (u128, u128) {
    let lower = value.isqrt();
    (lower, lower + u128::from(lower * lower != value))
}
