// Exact rational enclosure utilities shared by native inverse kernels.
/// Interpret the integer value exactly in units of `2^p`, for rational enclosure endpoints.
/// Use a power-of-two denominator for negative p and an integer shift otherwise.
/// Unsigned absolute value avoids negating i32::MIN, but constructing such a
/// power can still require enormous memory. This is exact conversion, without
/// rounding, cancellation polling or a local allocation bound.
fn interval_dyadic(value: BigInt, p: Precision) -> Rational {
    if p < 0 {
        Rational::from_bigint_fraction(value, BigUint::one() << p.unsigned_abs() as usize)
            .expect("dyadic denominator is nonzero")
    } else {
        Rational::from_bigint(value << p as usize)
    }
}

/// Enclose an admitted Computable in exact rational endpoints at requested grid spacing `2^p`.
/// Return a singleton when exact_rational recognizes the value; that path ignores
/// p and adds no stop check. Otherwise widen the integer sample by one unit on
/// each side. Containment depends on the child's completed one-unit approximation
/// contract; exact endpoint arithmetic does not independently certify that sample.
/// The supplied signal reaches sampling, but this helper returns no abort status.
/// Callers must discard interrupted bounds and own domain/resource validation.
fn interval_of(c: &Computable, signal: &Option<Signal>, p: Precision) -> [Rational; 2] {
    if let Some(exact) = c.exact_rational() {
        return [exact.clone(), exact];
    }
    let a = c.approx_signal(signal, p);
    [interval_dyadic(&a - 1, p), interval_dyadic(a + 1, p)]
}

/// Select the lesser exact rational endpoint, consuming both candidates.
/// Equality selects b. This is value comparison, without tolerance, refinement
/// or a cancellation poll; integer comparison costs remain unbounded here.
fn interval_min(a: Rational, b: Rational) -> Rational {
    if a < b { a } else { b }
}

/// Select the greater exact rational endpoint, consuming both candidates.
/// Equality selects b. Use this to intersect lower bounds without introducing
/// rounded endpoint comparisons or a local cancellation/resource contract.
fn interval_max(a: Rational, b: Rational) -> Rational {
    if a > b { a } else { b }
}

/// Clip an exact rational endpoint to `[-k,k]`, assuming nonnegative k.
/// Monotonicity lets inverse solvers clip both endpoints of a root enclosure.
/// This helper does not validate k or prove that clipping the root preserves the
/// requested function's error allowance; each solver supplies that argument.
fn interval_clamp(a: Rational, k: &Rational) -> Rational {
    interval_max(-k.clone(), interval_min(k.clone(), a))
}
