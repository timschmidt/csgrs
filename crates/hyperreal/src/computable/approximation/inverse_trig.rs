/// Approximate the principal angle `atan(1/i)` in units of `2^p` for nonzero integer `i`.
///
/// The division-only alternating series uses successive powers of `1/i^2`;
/// its geometric convergence schedule requires `abs(i) >= 2`. Unit denominators
/// instead use the native `atan(±1)` identity. This kernel supplies Machin-style
/// pi and shared arctangent constants without allocating a rational child node.
/// For `p >= 1`, the angle bound permits zero without examining the denominator;
/// that shortcut does not validate an invalid zero denominator.
///
/// A loop stop returns the accumulated partial sum, which the caller must not
/// publish or cache as certified. Setup precedes that poll. Working precisions,
/// iteration counters and shifts must fit; the iteration estimate is not an
/// enforced total-work bound. The output is an approximation, not a promise of
/// correct rounding of the exact angle.
fn atan(signal: &Option<Signal>, i: &BigInt, p: Precision) -> BigInt {
    // Integral atan is used for atan(1/n), where division by n^2 each iteration
    // is cheaper and more stable than approximating a rational Computable child.
    // This is the arctangent-series kernel used by the Machin pi computation.
    if p >= 1 {
        return Zero::zero();
    }
    // The series' geometric work/error bound requires |n| >= 2. Imported
    // unit denominators are valid, but need the native atan(+/-1) identity.
    if i.magnitude().is_one() {
        if should_stop(signal) {
            return Zero::zero();
        }
        return Computable::integer(i.clone())
            .atan()
            .approx_signal(signal, p);
    }

    let iterations_needed: i32 = -p / 2 + 2; // conservative estimate > 0.
    // from Java implementation description:

    // Claim: each intermediate term is accurate
    // to 2*base^calc_precision.
    // Total rounding error in series computation is
    // 2*iterations_needed*base^calc_precision,
    // exclusive of error in op.

    let calc_precision = p - bound_log2(2 * iterations_needed) - 2;
    // The denominator is exact. Guard precision accounts for integer division
    // in the retained powers and terms; the term threshold controls the tail.
    // The final scale operation rounds the accumulated integer once more.

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

/// Approximate principal `atan(c)`, certifying the local series range from its working sample.
///
/// For `p >= 1`, zero is valid for every finite real child. Otherwise, reserve
/// the sample's one-unit error when checking `abs(c) <= 1/2`. If the check fails,
/// use exact-rational reduction when available, or two half-angle steps that
/// avoid reselecting this kernel at a borderline cache rounding. An imported
/// prescaled tag alone does not justify the Taylor series.
///
/// The alternating recurrence uses guard precision and a term threshold before
/// scaling to `2^p` units. Stops can return a partial sum; the caller handles
/// abort publication. Setup and fixed guard bits supply no total-work bound,
/// and precision/counter arithmetic is unchecked.
fn atan_computable(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // This coarse result is valid for every finite real c: |atan(c)| < pi/2 < 2.
    if p >= 1 {
        return Zero::zero();
    }

    let iterations_needed: i32 = -p / 2 + 4;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_prec = calc_precision - 3;
    let op_appr = c.approx_signal(signal, op_prec);
    // A stored PrescaledAtan tag is not a range certificate. The working
    // sample has error at most one, so reserve that unit when proving |c| <= 1/2.
    if op_appr.magnitude().bits()
        > u64::try_from(-op_prec - 1).expect("atan series scale is positive")
    {
        if should_stop(signal) {
            return Zero::zero();
        }
        if let Some(rational) = c.exact_rational() {
            return atan_rational(signal, &rational, p);
        }
        // The ordinary reducer can select this same kernel from a valid
        // coarse sample near 1/2. Two half-angle steps instead guarantee a
        // strictly smaller argument, even at such cache-rounding boundaries.
        return c
            .clone()
            .atan_with_half_angle_reduction()
            .approx_signal(signal, p);
    }
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

/// Evaluate a retained arctangent source by invoking its native convergence reducer on demand.
///
/// The child remains authoritative while construction avoids eagerly allocating
/// the reduced graph. The principal real branch is defined for every finite
/// child; `signal` and `p` pass to the resulting expression. This wrapper adds
/// no cancellation poll or resource bound beyond the reducer and evaluation.
fn atan_deferred(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Preserve the source argument at construction time, then apply the same
    // convergence reduction previously performed eagerly by Computable::atan.
    // Recursive atan calls operate on strictly smaller residuals.
    c.clone().atan_reduced().approx_signal(signal, p)
}

/// Evaluate the alternating arctangent series for a caller-proved `abs(r) <= 1/2`.
///
/// Sample the exact rational directly at guarded precision, avoiding a temporary
/// Computable child and its cache. Unlike the general rational reducer, this
/// helper does not check the small-range precondition. Return an integer in
/// `2^p` units; coarse `p >= 1` uses the global principal-angle bound.
/// Loop cancellation leaves a partial sum for the caller to reject. Setup,
/// precision arithmetic and rational integer sizes have no local resource cap.
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

/// Form the exact tangent subtraction residual `(r - a/b) / (1 + r*a/b)`.
///
/// Require nonnegative `r`, nonnegative anchor numerator `a`, and positive
/// anchor denominator `b`. The implementation uses the stored magnitude parts
/// of `r`, so it is not a signed-rational formula for negative inputs. Positive
/// denominators justify the construction's expect; the principal arctangent
/// difference stays between `-pi/2` and `pi/2`, avoiding a branch correction.
/// Combining numerator/denominator products directly avoids temporary rational
/// quotients but does not impose a coefficient-growth budget.
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

/// Approximate `atan(sqrt(r))` for caller-proved `0 <= r <= 1/4`.
///
/// Form a nonnegative integer root sample with eight bits before the square
/// root, then shift four bits to the series scale. No temporary Sqrt node is
/// needed. When `sample_exact_square` is true, sample the retained rational `r`
/// for the squared argument; otherwise square and rescale the root sample.
/// The flag selects a calculation path, not a claim that the root is rational
/// or that its sampled square is exact.
///
/// The caller supplies any outer factor two by requesting precision `p - 1`.
/// This helper returns `2^p` units, assumes the range without checking it, and
/// can return a partial sum on cancellation. Its signed precision arithmetic
/// and integer allocations have no checked resource contract.
fn atan_sqrt_rational_small(
    signal: &Option<Signal>,
    r: &Rational,
    p: Precision,
    sample_exact_square: bool,
) -> BigInt {
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
    let op_squared = if sample_exact_square {
        // The series argument is sqrt(r), so its square is the retained exact
        // rational r. The asin complement already samples another rational and
        // benefits from reusing that division path instead of a wide square.
        ratio(r, op_prec)
    } else {
        // The standalone acos path squares the root sample it already owns
        // instead of requesting a second rational division.
        scale(&op_appr * &op_appr, op_prec)
    };

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

/// Reduce an exact rational's principal arctangent before evaluating a small residual.
///
/// Apply odd symmetry first. Unit numerators with denominator greater than one
/// use integral reciprocal atan; other inputs at most one half use the direct
/// series. Large positive integers
/// and certified magnitudes at least two use `pi/2 - atan(1/r)`. Middle inputs
/// select anchors in order: two, three halves, two thirds, then one. The two-thirds
/// anchor reuses `pi/4 - atan(1/5)`. Three guard bits combine cached anchor and
/// residual approximations into `2^p` units. Preserve branch and trace order.
///
/// Large noninteger inputs need a representable exact magnitude for reciprocal
/// routing. If that metadata is unavailable, the final half-anchor fallback
/// has no established small-residual range; this is an implementation limit,
/// not a proof of convergence for every rational coefficient size.
/// The reducer adds no entry stop poll or total-work limit, and its precision
/// subtractions must fit. Child/kernel aborts must not be published as certified.
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
    if r <= FOUR_FIFTHS_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "atan", "two-thirds-anchor-shared");
        // atan(2/3) = pi/4 - atan(1/5). This adds a table-reduction point
        // without a new transcendental constant: both terms already have
        // shared caches for the Machin pi path. On [1/2, 4/5] the exact
        // subtraction residual is at most 1/8 instead of the unit anchor's
        // 1/3, substantially shortening the medium-precision Taylor series.
        let quarter_pi = Computable::pi().approx_signal(signal, work_precision + 2);
        let atan_inv5 = Computable::atan_inv5_constant().approx_signal(signal, work_precision);
        let residual = atan_anchor_residual(r, 2, 3);
        let reduced = atan_rational_small(signal, &residual, work_precision);
        return scale(quarter_pi - atan_inv5 + reduced, -extra);
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

#[inline]
/// Return `(2n-1)^2` and `2n*(2n+1)` for the nth odd-power asin coefficient update.
///
/// For positive `i32` indices, widening before multiplication keeps both exact
/// products in `u64`, including indices beyond the old signed-word overflow
/// boundary. Asinh uses the same magnitudes and negates each successive term.
/// The positive-index precondition is a debug assertion, not a typed domain
/// check or a bound on the caller's iteration count.
fn inverse_sine_series_coefficients(n: i32) -> (u64, u64) {
    // Widen before multiplication: (2*n-1)^2 already exceeds i32 at n=23171.
    // For every positive i32 index, 2*n <= u32::MAX-1, so both products fit
    // u64. asinh applies the alternating sign to the BigInt term separately.
    debug_assert!(n > 0);
    let even = (n as u64) * 2;
    ((even - 1) * (even - 1), even * (even + 1))
}

#[test]
/// Compare both coefficient products with independent `u128` formulas at eight positive indices.
///
/// Include the old squared-coefficient overflow boundary and `i32::MAX`. These
/// sixteen assertions sample the width argument; they do not enumerate every
/// index or establish an entire series' approximation error.
fn inverse_sine_series_coefficients_cover_the_index_range() {
    for n in [1, 2, 23_170, 23_171, 65_535, 65_536, 1 << 30, i32::MAX] {
        let even = (n as u128) * 2;
        let (numerator, denominator) = inverse_sine_series_coefficients(n);
        assert_eq!(u128::from(numerator), (even - 1) * (even - 1));
        assert_eq!(u128::from(denominator), even * (even + 1));
    }
}

/// Approximate principal `asin(r)` for an admitted rational in `[-1, 1]`.
///
/// An exact binary magnitude at most -4 admits the small odd-power series.
/// Otherwise reduce the magnitude through `pi/2 - acos(abs(r))` and restore the
/// sign. For half-angle residual `(1-abs(r))/(1+abs(r))` at most one quarter,
/// combine guarded pi and square-root atan samples directly; larger residuals
/// use the generic expression's adaptive schedule. Zero has no magnitude
/// metadata and follows the complement route if presented directly here.
///
/// The coarse `p >= 1` shortcut uses the valid-domain angle bound without
/// checking that domain. Series cancellation may return a partial sum; the
/// caller handles abort status. Precision/counter arithmetic and temporary
/// rational work have no local checked resource bound.
fn asin_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Tiny exact rational inputs use the direct odd series. Larger magnitudes
    // retain the cancellation-safe pi/2 - acos(|x|) schedule, but build that
    // approximation graph only on a cold cache miss instead of during every
    // public construction. The outer node carries the original sign directly.
    if p >= 1 {
        return Zero::zero();
    }

    if r.msd_exact().is_none_or(|msd| msd > -4) {
        crate::trace_dispatch!("computable_approx", "asin", "rational-acos-complement");
        let sign = r.sign();
        let magnitude = if sign == Sign::Minus {
            -r.clone()
        } else {
            r.clone()
        };
        let residual =
            (Rational::one() - magnitude.clone()) / (Rational::one() + magnitude.clone());
        if residual > Rational::fraction(1, 4).expect("nonzero denominator") {
            // The generic acos residual branch has its own adaptive error
            // schedule. Preserve the former expression-level approximation
            // there; the endpoint range below is the branch that can safely
            // combine two directly sampled integer approximations.
            crate::trace_dispatch!(
                "computable_approx",
                "asin",
                "generic-rational-acos-complement"
            );
            let half_pi = Computable::pi().multiply(Computable::rational(HALF_RATIONAL.clone()));
            let reduced = half_pi.add(Computable::rational(magnitude).acos().negate());
            let result = reduced.approx_signal(signal, p);
            return if sign == Sign::Minus { -result } else { result };
        }

        // Sample both terms two guard bits finer. The pi approximation has
        // error below one working unit and the doubled atan approximation has
        // error below two; after the final division by eight their combined
        // error is below 3/8 of one result unit, leaving room for the final
        // half-unit rounding. At that scale
        // pi/2 - acos(x) = (pi - 2*acos(x))/2, so a single final shift
        // performs the combination without building an arithmetic graph.
        let extra = 2;
        let work_precision = p - extra;
        let pi = Computable::pi().approx_signal(signal, work_precision);
        crate::trace_dispatch!("computable_approx", "acos", "small-rational-residual");
        let acos = atan_sqrt_rational_small(signal, &residual, work_precision - 1, true);
        let result = scale(pi - (acos << 1), -(extra + 1));
        return if sign == Sign::Minus { -result } else { result };
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
        let (numerator, denominator) = inverse_sine_series_coefficients(n);
        current_term *= numerator;
        current_term /= denominator;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

/// Approximate principal `asin(c)` using a certified tiny-argument series or a deferred transform.
///
/// Require an admitted child in `[-1, 1]`. After the entry stop check, coarse
/// `p >= 1` needs no sample. Finer requests reserve one unit of sample error to
/// prove `abs(c) <= 1/8`; a failed range check selects the atan/sqrt transform.
/// A prescaled node or an estimated magnitude is not a range certificate.
///
/// The positive-coefficient odd-power recurrence shares its wide coefficient
/// helper with asinh and returns `2^p` units. Loop stops leave partial output
/// for the caller to reject. Fixed guard bits do not bound total work, and
/// intermediate precisions and counters must fit their signed words.
fn asin_computable(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Dedicated tiny-argument asin series. It avoids the generic atan/sqrt
    // transform, which is overkill and slower when |x| is already very small.
    if should_stop(signal) {
        return Zero::zero();
    }
    // |asin(x)| <= pi/2 throughout the valid domain, independently of whether
    // the tiny series applies. This coarse shortcut needs no operand sample.
    if p >= 1 {
        return Zero::zero();
    }

    let iterations_needed: i32 = -p / 2 + 4;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_prec = calc_precision - 3;
    let op_appr = c.approx_signal(signal, op_prec);
    // Reuse the series' own operand sample as its domain proof. With
    // k = -op_prec-3, bits(|a|) <= k implies |a| <= 2^k-1; including the
    // one-unit sample error gives |c| <= 1/8. No preliminary sample is needed.
    if op_appr.magnitude().bits()
        > u64::try_from(-op_prec - 3).expect("tiny-series scale is positive")
    {
        if should_stop(signal) {
            return Zero::zero();
        }
        return asin_deferred(signal, c, p);
    }
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
        let (numerator, denominator) = inverse_sine_series_coefficients(n);
        current_term *= numerator;
        current_term /= denominator;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

/// Evaluate principal asin on `[-1, 1]` via `2*atan(c/(sqrt(1-c^2)+1))`.
///
/// The positive denominator also covers both endpoints. Build the exact
/// reduction only when approximation is requested, preserving the compact
/// source until then. This wrapper assumes domain admission, passes `signal`
/// and `p` to the result, and adds no stop poll or independent work limit.
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

/// Evaluate principal acos for an admitted nonnegative child at most one.
///
/// Use `2*atan(sqrt((1-c)/(1+c)))`, whose residual is nonnegative and tends to
/// zero at the upper endpoint. Defer the exact quotient/root graph until digits
/// are requested. Domain validation belongs to the caller; this wrapper passes
/// `signal` and `p` through without an extra stop poll or resource limit.
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

/// Approximate principal acos for `0 <= r <= 1` using its exact half-angle residual.
///
/// Construct `(1-r)/(1+r)` as a rational. Residuals at most one quarter use the
/// small square-root atan kernel at `p - 1`, incorporating the outer factor two
/// without a separate graph. Larger residuals use native sqrt/atan reduction.
/// Return `2^p` units. The range is a caller precondition; the helper adds no
/// stop poll, checked precision subtraction, or coefficient-size bound.
fn acos_positive_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Exact-rational endpoint inputs benefit from forming the half-angle
    // residual directly. This follows the same cancellation-avoiding reduction
    // as `acos_positive` while avoiding a temporary Computable division tree.
    let residual = (Rational::one() - r.clone()) / (Rational::one() + r.clone());
    if residual <= Rational::fraction(1, 4).expect("nonzero denominator") {
        crate::trace_dispatch!("computable_approx", "acos", "small-rational-residual");
        return atan_sqrt_rational_small(signal, &residual, p - 1, false);
    }
    Computable::sqrt_rational(residual)
        .atan()
        .shift_left(1)
        .approx_signal(signal, p)
}

/// Approximate `acos(-r)` from the stored nonnegative magnitude `0 <= r <= 1`.
///
/// The payload is not the negative argument itself. Evaluate `pi - acos(r)`
/// with two guard bits and round the difference once to `2^p` units. An entry
/// stop returns an abort placeholder; the caller must also handle any later
/// child aborts. The precision subtraction is unchecked, and the two samples
/// do not constitute a separate total-work budget.
fn acos_negative_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    if should_stop(signal) {
        return Zero::zero();
    }
    // Each sample can err by one working unit, including a rescaled cached
    // sample. Two guard bits bound their combined error by half an output
    // unit; rounding once adds at most the remaining half, as in native add.
    let work_precision = p - 2;
    let pi = Computable::pi().approx_signal(signal, work_precision);
    let positive = acos_positive_rational(signal, r, work_precision);
    scale(pi - positive, -2)
}
