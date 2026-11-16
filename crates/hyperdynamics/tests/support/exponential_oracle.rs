use hyperreal::Rational as Q;

// Positive exponential series through degree twelve. For 0 <= t < 14,
// every ratio in the omitted tail after its first term is at most t/14.
pub fn exp_bounds(t: Q) -> (Q, Q) {
    assert!(t >= Q::zero() && t < Q::from(14));
    let mut term = Q::from(1);
    let mut lower = term.clone();
    for k in 1..=12 {
        term = &term * &t / Q::from(k);
        lower = lower + &term;
    }
    let upper = &lower + (&term * &t / Q::from(13)) / (Q::from(1) - &t / Q::from(14));
    (lower, upper)
}
