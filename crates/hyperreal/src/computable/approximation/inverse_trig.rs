fn atan(signal: &Option<Signal>, i: &BigInt, p: Precision) -> BigInt {
    // Integral atan is used for atan(1/n), where division by n^2 each iteration
    // is cheaper and more stable than approximating a rational Computable child.
    // This is the arctangent-series kernel used by the Machin pi computation.
    if p >= 1 {
        return Zero::zero();
    }

    let iterations_needed: i32 = -p / 2 + 2; // conservative estimate > 0.
    // from Java implementation description:

    // Claim: each intermediate term is accurate
    // to 2*base^calc_precision.
    // Total rounding error in series computation is
    // 2*iterations_needed*base^calc_precision,
    // exclusive of error in op.

    let calc_precision = p - bound_log2(2 * iterations_needed) - 2;
    // Error in argument results in error of < 3/8 ulp.
    // Cumulative arithmetic rounding error is < 1/4 ulp.
    // Series truncation error < 1/4 ulp.
    // Final rounding error is <= 1/2 ulp.
    // Thus final error is < 1 ulp.

    let max_trunc_error: BigUint = BigUint::one() << (p - 2 - calc_precision);

    let scaled_1 = signed::ONE.deref() << (-calc_precision);
    let big_op_squared: BigInt = i * i;
    let inverse: BigInt = scaled_1 / i;

    let mut current_power = inverse.clone();
    let mut current_term = inverse.clone();
    let mut sum = inverse;

    let mut sign = 1;
    let mut n = 1;

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;
        current_power /= &big_op_squared;
        sign = -sign;
        let signed_n: BigInt = (n * sign).into();
        current_term = &current_power / signed_n;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

// Approximate atan(c) for |c| < 1/2.
fn atan_computable(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Kernel precondition: |c| is small. Larger atan inputs are reduced by
    // subtraction of atan(1/2) or the reciprocal identity before reaching here.
    // Reduce the argument before evaluating the series.
    if p >= 1 {
        return Zero::zero();
    }

    let iterations_needed: i32 = -p / 2 + 4;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_prec = calc_precision - 3;
    let op_appr = c.approx_signal(signal, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    // Borrowed magnitude checks matter here because tiny inverse-trig benches
    // run many short series from cold caches.
    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut current_term = scale(op_appr, op_prec - calc_precision);
    let mut sum = current_term.clone();
    let mut n = 1;

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;
        current_term = scale(current_term * &op_squared, op_prec);
        current_term *= -(n - 2);
        current_term /= n;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

fn atan_rational_small(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Same Taylor kernel as `atan_computable`, but exact rational leaves can
    // provide the working approximation directly. This removes a Computable
    // child approximation call from tiny and residual atan reductions.
    if p >= 1 {
        return Zero::zero();
    }

    let iterations_needed: i32 = -p / 2 + 4;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_prec = calc_precision - 3;
    let op_appr = ratio(r, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut current_term = scale(op_appr, op_prec - calc_precision);
    let mut sum = current_term.clone();
    let mut n = 1;

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;
        current_term = scale(current_term * &op_squared, op_prec);
        current_term *= -(n - 2);
        current_term /= n;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

fn atan_anchor_residual(r: &Rational, anchor_numerator: u8, anchor_denominator: u8) -> Rational {
    // atan(r) - atan(a/b) = atan((r-a/b)/(1+r*a/b)). For exact positive
    // rationals, compute the residual from numerator/denominator parts in one
    // pass instead of constructing several temporary Rational values.
    let numerator_positive = r.numerator() * anchor_denominator;
    let numerator_negative = r.denominator() * anchor_numerator;
    let numerator = BigInt::from_biguint(Sign::Plus, numerator_positive)
        - BigInt::from_biguint(Sign::Plus, numerator_negative);
    let denominator = r.denominator() * anchor_denominator + r.numerator() * anchor_numerator;
    Rational::from_bigint_fraction(numerator, denominator)
        .expect("atan anchor residual denominator is positive")
}

fn atan_sqrt_rational_small(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Same Taylor kernel as atan_rational_small for inputs known to satisfy
    // sqrt(r) <= 1/2. The sqrt is formed as an integer approximation directly
    // from the Rational, avoiding a transient Sqrt node followed by PrescaledAtan.
    if p >= 1 {
        return Zero::zero();
    }

    let iterations_needed: i32 = -p / 2 + 4;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_prec = calc_precision - 3;
    let sqrt_extra = 8;
    let op_appr = shift(ratio(r, 2 * op_prec - sqrt_extra).sqrt(), -(sqrt_extra / 2));
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut current_term = scale(op_appr, op_prec - calc_precision);
    let mut sum = current_term.clone();
    let mut n = 1;

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;
        current_term = scale(current_term * &op_squared, op_prec);
        current_term *= -(n - 2);
        current_term /= n;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

fn atan_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Exact rational atan keeps the public constructor shallow and performs
    // range reduction directly here. The identities are the same as
    // Computable::atan: odd symmetry, atan(x)=pi/2-atan(1/x) for x>=2, and
    // atan(x)=atan(1/2)+atan((x-1/2)/(1+x/2)) in the middle interval.
    crate::trace_dispatch!("computable_approx", "atan", "exact-rational-reduction");
    match r.sign() {
        Sign::NoSign => return Zero::zero(),
        Sign::Minus => return -atan_rational(signal, &(-r.clone()), p),
        Sign::Plus => {}
    }

    if r.numerator() == &BigUint::one() {
        let denominator = BigInt::from_biguint(Sign::Plus, r.denominator().clone());
        if denominator > *signed::ONE.deref() {
            return atan(signal, &denominator, p);
        }
    }

    let half = HALF_RATIONAL.deref();
    if r <= half {
        return atan_rational_small(signal, r, p);
    }

    if r.denominator() == &BigUint::one() && r.numerator() > &BigUint::one() {
        crate::trace_dispatch!("computable_approx", "atan", "large-integer-reciprocal");
        let extra = 3;
        let work_precision = p - extra;
        let half_pi = Computable::pi().approx_signal(signal, work_precision + 1);
        let denominator = BigInt::from_biguint(Sign::Plus, r.numerator().clone());
        let reduced = atan(signal, &denominator, work_precision);
        return scale(half_pi - reduced, -extra);
    }

    if r.msd_exact().is_some_and(|msd| msd >= 1) {
        let extra = 3;
        let work_precision = p - extra;
        let half_pi = Computable::pi().approx_signal(signal, work_precision + 1);
        let reciprocal = r.clone().inverse().expect("positive rational is nonzero");
        let reduced = atan_rational(signal, &reciprocal, work_precision);
        return scale(half_pi - reduced, -extra);
    }

    let extra = 3;
    let work_precision = p - extra;
    if r >= SEVEN_FOURTHS_RATIONAL.deref() && r <= TWO_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "atan", "two-anchor-shared");
        let anchor = Computable::atan2_constant().approx_signal(signal, work_precision);
        let residual = atan_anchor_residual(r, 2, 1);
        let reduced = atan_rational_small(signal, &residual, work_precision);
        return scale(anchor + reduced, -extra);
    }
    if r >= FOUR_THIRDS_RATIONAL.deref() && r <= TWO_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "atan", "three-halves-anchor-shared");
        let anchor = Computable::atan_three_halves_constant().approx_signal(signal, work_precision);
        let residual = atan_anchor_residual(r, 3, 2);
        let reduced = atan_rational_small(signal, &residual, work_precision);
        return scale(anchor + reduced, -extra);
    }
    if r <= TWO_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "atan", "unit-anchor-pi-quarter");
        let quarter_pi = Computable::pi().approx_signal(signal, work_precision + 2);
        let residual = atan_anchor_residual(r, 1, 1);
        let reduced = atan_rational_small(signal, &residual, work_precision);
        // Sampling pi two bits coarser gives its integer approximation exactly
        // the weight needed for a pi/4 anchor when both terms are shifted back
        // by `extra`: pi*2^(1-p)/8 = (pi/4)*2^(-p).
        return scale(quarter_pi + reduced, -extra);
    }

    let anchor = atan(signal, signed::TWO.deref(), work_precision);
    let residual = atan_anchor_residual(r, 1, 2);
    let reduced = atan_rational_small(signal, &residual, work_precision);
    scale(anchor + reduced, -extra)
}

fn asin_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Exact rational asin uses the same direct series as `asin_computable`, but
    // bypasses the child Computable approximation. This is only selected after
    // construction certifies a tiny/moderate positive input, so the series
    // remains convergent enough to beat the generic sqrt/atan transform.
    if p >= 1 {
        return Zero::zero();
    }

    let iterations_needed: i32 = -p / 2 + 4;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_prec = calc_precision - 3;
    let op_appr = ratio(r, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut current_term = scale(op_appr, op_prec - calc_precision);
    let mut sum = current_term.clone();
    let mut n = 0_i32;

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 1;
        current_term = scale(current_term * &op_squared, op_prec);
        let numerator = (2 * n - 1) * (2 * n - 1);
        let denominator = (2 * n) * (2 * n + 1);
        current_term *= numerator;
        current_term /= denominator;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

// Approximate asin(c) for small |c|.
fn asin_computable(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Dedicated tiny-argument asin series. It avoids the generic atan/sqrt
    // transform, which is overkill and slower when |x| is already very small.
    if p >= 1 {
        return Zero::zero();
    }

    let iterations_needed: i32 = -p / 2 + 4;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_prec = calc_precision - 3;
    let op_appr = c.approx_signal(signal, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    // Borrowed magnitude checks matter here because tiny inverse-trig benches
    // run many short series from cold caches.
    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut current_term = scale(op_appr, op_prec - calc_precision);
    let mut sum = current_term.clone();
    let mut n = 0_i32;

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 1;
        current_term = scale(current_term * &op_squared, op_prec);
        let numerator = (2 * n - 1) * (2 * n - 1);
        let denominator = (2 * n) * (2 * n + 1);
        current_term *= numerator;
        current_term /= denominator;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

fn asin_deferred(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Generic asin uses asin(x) = 2*atan(x / (sqrt(1-x^2)+1)).
    // Keeping this as one approximation node mirrors the deferred acos and
    // inverse-hyperbolic reductions: construction preserves the compact input
    // graph, while approximation still enters the same stable transform.
    let one = Computable::one();
    let denominator = one.clone().add(c.clone().square().negate()).sqrt().add(one);
    c.clone()
        .multiply(denominator.inverse())
        .atan()
        .shift_left(1)
        .approx_signal(signal, p)
}

fn acos_positive(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Positive-domain acos uses 2*atan(sqrt((1-x)/(1+x))). Keeping it as one
    // approximation node makes public construction cheap for endpoint-heavy
    // inverse trig rows while preserving the existing stable reduction.
    let one = Computable::one();
    let numerator = one.clone().add(c.clone().negate());
    let denominator = one.add(c.clone());
    numerator
        .multiply(denominator.inverse())
        .sqrt()
        .atan()
        .shift_left(1)
        .approx_signal(signal, p)
}

fn acos_positive_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Exact-rational endpoint inputs benefit from forming the half-angle
    // residual directly. This follows the same cancellation-avoiding reduction
    // as `acos_positive` while avoiding a temporary Computable division tree.
    let residual = (Rational::one() - r.clone()) / (Rational::one() + r.clone());
    if residual <= Rational::fraction(1, 4).expect("nonzero denominator") {
        crate::trace_dispatch!("computable_approx", "acos", "small-rational-residual");
        return atan_sqrt_rational_small(signal, &residual, p - 1);
    }
    Computable::sqrt_rational(residual)
        .atan()
        .shift_left(1)
        .approx_signal(signal, p)
}

fn acos_negative_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    Computable::pi().approx_signal(signal, p) - acos_positive_rational(signal, r, p)
}
