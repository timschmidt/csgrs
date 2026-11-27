/// Evaluate reduced-argument cosine by an even Taylor series, returning `2^p` units.
///
/// The nominal local range is `abs(c) < 1`; this routine does not verify it.
/// Internal tangent scheduling can also pass children with magnitude below two,
/// so that scheduling is not itself a certificate of the nominal series range.
/// Sample at `p - 2`, accumulate guarded terms, then round with `scale`.
/// For `p >= 1` return one: this coarse choice relies on positive cosine in the
/// local range, not merely the global bound `abs(cos(c)) <= 1`.
///
/// A stop returns one before the loop or a partial sum within it; the caller
/// must reject interrupted publication. Precision, counters and the product
/// `n*(n-1)` are unchecked signed arithmetic, with no total-work bound.
fn cos(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Nominal local range: |c| < 1. This series performs no range reduction;
    // tangent scheduling may also supply |c| < 2, as documented above.
    if p >= 1 {
        return signed::ONE.deref().clone();
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return signed::ONE.deref().clone();
    }

    //  Claim: each intermediate term is accurate
    //  to 2*2^calc_precision.
    //  Total rounding error in series computation is
    //  2*iterations_needed*2^calc_precision,
    //  exclusive of error in op.
    let calc_precision = p - bound_log2(2 * iterations_needed) - 4; // for error in op, truncation.
    let op_prec = p - 2;
    let op_appr = c.approx_signal(signal, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    // Error in argument results in error of < 1/4 ulp.
    // Cumulative arithmetic rounding error is < 1/16 ulp.
    // Series truncation error < 1/16 ulp.
    // Final rounding error is <= 1/2 ulp.
    // Thus final error is < 1 ulp.

    // Keep the truncation guard allocation-free across Taylor iterations.
    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut n = 0;
    let mut current_term = signed::ONE.deref() << (-calc_precision);
    let mut current_sum = current_term.clone();

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;

        // t_n = -t_(n-2) * x^2 / (n * (n - 1)).
        current_term = scale(current_term * &op_squared, op_prec);
        current_term /= -(n * (n - 1));

        current_sum += &current_term;
    }
    scale(current_sum, calc_precision - p)
}

// Evaluate the analytic continuations
//
//   sin(x) / x             = sum (-1)^k x^(2k) / (2k + 1)!
//   (1 - cos(x)) / x^2     = sum (-1)^k x^(2k) / (2k + 2)!
//
// for a constructor-certified |x| <= 3/4. On this interval the alternating
// terms decrease from the outset, so the first omitted term bounds the tail.
// The integral forms
//
//   sinc(x) = integral_0^1 cos(tx) dt
//   cosc(x) = integral_0^1 (1-t) cos(tx) dt
//
// bound the absolute derivatives by |x|/3 <= 1/4 and |x|/12 <= 1/16.
// Requesting x at p-2 therefore spends at most 1/4 output ulp on argument
// error. The remaining guard budget matches the established sin/cos kernels:
// at most 1/16 ulp each for fixed-point arithmetic and the omitted tail, plus
// the final 1/2-ulp scale operation, keeping the total strictly below one ulp.
/// Evaluate the even analytic continuation sinc or cosc for caller-certified `abs(c) <= 3/4`.
///
/// Selector one gives `sin(c)/c`, including value one at zero; selector two
/// gives `(1-cos(c))/c^2`, including value one half at zero. The selector is
/// only debug-asserted. Series evaluation avoids dividing by an unknown or zero
/// argument. Alternating terms and the integral derivative bounds motivate the
/// local tail/argument guards; a stored tag alone is not a range proof.
///
/// Return `2^p` units. For nonnegative p the selected coarse integers are one
/// and zero. A stop may return those placeholders or a partial sum; callers own
/// publication. Precision and factorial-factor products are unchecked, and no
/// work or memory budget is enforced here.
fn small_angle_quotient(
    signal: &Option<Signal>,
    c: &Computable,
    p: Precision,
    first_denominator: i32,
) -> BigInt {
    debug_assert!(first_denominator == 1 || first_denominator == 2);

    if first_denominator == 1 && p >= 0 {
        return signed::ONE.deref().clone();
    }
    if first_denominator == 2 && p >= 0 {
        return BigInt::zero();
    }
    if should_stop(signal) {
        return if first_denominator == 1 {
            signed::ONE.deref().clone()
        } else {
            BigInt::zero()
        };
    }

    let iterations_needed = -p / 2 + 4;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 4;
    let op_prec = p - 2;
    let op_appr = c.approx_signal(signal, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);
    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");

    let mut n = first_denominator;
    let mut current_term = signed::ONE.deref() << (-calc_precision);
    if first_denominator == 2 {
        current_term >>= 1;
    }
    let mut current_sum = current_term.clone();

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;
        current_term = scale(current_term * &op_squared, op_prec);
        current_term /= -(n * (n - 1));
        current_sum += &current_term;
    }

    scale(current_sum, calc_precision - p)
}

/// Evaluate the admitted small-angle sinc continuation, selecting its factorial series without dividing by c.
/// The constructor supplies `abs(c) <= 3/4`; value one at zero is part of the
/// same analytic function. Units, cancellation and resource limits are those
/// of small_angle_quotient with selector one.
fn sinc_small(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    small_angle_quotient(signal, c, p, 1)
}

/// Evaluate the admitted small-angle cosc continuation, selecting its factorial series without dividing by c squared.
/// The constructor supplies `abs(c) <= 3/4`; value one half at zero is retained.
/// Units, cancellation and resource limits are those of small_angle_quotient
/// with selector two.
fn cosc_small(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    small_angle_quotient(signal, c, p, 2)
}

/// Approximate cosine for a caller-admitted rational `abs(r) < 1` without a temporary Ratio node.
///
/// Feed the exact rational directly to the guarded even series. The source
/// remains exact; its working sample is an integer, not a replacement model.
/// The coarse return one requires this positive-cosine range. Range validation
/// is not performed here. Return `2^p` units; stopped/partial output requires
/// caller handling. Derived precisions and term-factor products are unchecked.
fn cos_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    if p >= 1 {
        return signed::ONE.deref().clone();
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return signed::ONE.deref().clone();
    }

    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_prec = p - 2;
    let op_appr = ratio(r, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut n = 0;
    let mut current_term = signed::ONE.deref() << (-calc_precision);
    let mut current_sum = current_term.clone();

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;

        current_term = scale(current_term * &op_squared, op_prec);
        current_term /= -(n * (n - 1));

        current_sum += &current_term;
    }
    scale(current_sum, calc_precision - p)
}

/// Choose a signed half-pi multiple for a small residual, without promising the nearest multiple.
///
/// Try the fixed rational sectors first. Otherwise estimate the quotient using
/// cached pi, then adjust it by at most one from a precision -1 residual sample.
/// For certified input exponents at least 4096, the guard request incorporates
/// saturating `-p + 16`; the underlying estimator enforces at least sixteen
/// extra bits. This anticipates the final residual's pi precision.
///
/// The quotient estimator and generic fallback use their own signal attachments;
/// only the explicit residual evaluation receives `signal`. This helper neither
/// polls a stop itself nor reports a typed failure, and it imposes no total-work
/// or allocation bound. Correct residuals, not unique nearest rounding, are the
/// relevant requirement for its consumers.
fn large_rational_half_pi_multiple(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Classify fixed sectors first, then use the constructor layer's rational
    // half-pi quotient estimate and correct its residual from cached pi.
    // A suitable small residual does not require a uniquely nearest multiple.
    if let Some(multiple) = fixed_small_large_rational_half_pi_multiple(r) {
        return multiple;
    }

    // The residual below will need pi at roughly `msd(r) - p` bits. Request
    // that precision while estimating the quotient so the shared pi cache can
    // serve the correction and final residual instead of running the
    // high-precision pi kernel twice. The 16 extra bits retain the original
    // quotient-estimation guard for coarse requests. The wider schedule starts
    // at a certified 4096-bit input exponent; smaller inputs retain sixteen
    // extra quotient bits. The estimator enforces that minimum itself.
    let quotient_guard_bits = if r.msd_exact().is_some_and(|msd| msd >= 4_096) {
        p.saturating_neg().saturating_add(16)
    } else {
        16
    };
    let mut multiple =
        Computable::half_pi_multiple_exact_rational_with_guard(r, quotient_guard_bits)
            .unwrap_or_else(|| Computable::rational(r.clone()).half_pi_multiple());
    let rough_appr = large_rational_half_pi_residual(signal, r, &multiple, -1);

    if rough_appr >= *crate::computable::signed::TWO {
        multiple += 1;
    } else if rough_appr <= -crate::computable::signed::TWO.deref().clone() {
        multiple -= 1;
    }

    multiple
}

/// Recognize fixed signed half-pi multiples on selected exact rational sectors.
///
/// For positive inputs, choose 2 on [7/2,39/10], 3 on [79/20,27/5], 4 on
/// [11/2,7], and 5 on (7,8); negative inputs reflect these choices. Earlier
/// branches own shared endpoints. The magnitude-exponent-two guard excludes
/// [8,17/2] despite the last comparison's written upper bound.
///
/// These choices leave residual magnitude below one. They are not all nearest:
/// 701/100 selects five although four is nearer. None means no table match,
/// not an invalid angle. Preserve comparison and trace order; comparisons retain
/// exact rational payloads and have no coefficient-size or cancellation budget.
fn fixed_small_large_rational_half_pi_multiple(r: &Rational) -> Option<BigInt> {
    let msd = r.msd_exact();

    // For |r| in [7/2, 39/10], the nearest half-pi multiple is exactly +/-2.
    // The interval lies strictly between 3*pi/4 and 5*pi/4. It covers the
    // historical promoted tangent tail immediately below the old 79/20
    // deferred-reduction boundary, while retaining an exact sector certificate.
    if msd == Some(1) {
        if r >= SEVEN_HALVES_RATIONAL.deref() && r <= THIRTY_NINE_TENTHS_RATIONAL.deref() {
            crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-multiple-2");
            return Some(BigInt::from(2));
        }
        if r <= NEG_SEVEN_HALVES_RATIONAL.deref() && r >= NEG_THIRTY_NINE_TENTHS_RATIONAL.deref() {
            crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-multiple-neg2");
            return Some(BigInt::from(-2));
        }
    }

    // For |r| in [79/20, 4], the nearest half-pi multiple is exactly +/-3.
    // This catches tan rows just below the large-rational threshold while
    // staying above 5*pi/4.
    if r >= SEVENTY_NINE_TWENTIETHS_RATIONAL.deref() && r <= FOUR_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-multiple-3");
        return Some(BigInt::from(3));
    }
    if r <= NEG_SEVENTY_NINE_TWENTIETHS_RATIONAL.deref() && r >= NEG_FOUR_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-multiple-neg3");
        return Some(BigInt::from(-3));
    }

    // For |r| in [4, 27/5], the nearest half-pi multiple is exactly +/-3.
    // The upper bound is kept conservatively below 7*pi/4, so the shortcut
    // remains a cheap certificate rather than a fresh pi-dependent comparison.
    if msd != Some(2) {
        return None;
    }
    if r >= FOUR_RATIONAL.deref() && r <= TWENTY_SEVEN_FIFTHS_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-multiple-3");
        return Some(BigInt::from(3));
    }
    if r <= NEG_FOUR_RATIONAL.deref() && r >= NEG_TWENTY_SEVEN_FIFTHS_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-multiple-neg3");
        return Some(BigInt::from(-3));
    }
    // For |r| in [11/2, 7], the nearest half-pi multiple is exactly +/-4.
    // The lower bound is above 7*pi/4 and the upper bound is below 9*pi/4,
    // covering the next promoted tan cluster without a pi-dependent probe.
    if r >= ELEVEN_HALVES_RATIONAL.deref() && r <= SEVEN_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-multiple-4");
        return Some(BigInt::from(4));
    }
    if r <= NEG_ELEVEN_HALVES_RATIONAL.deref() && r >= NEG_SEVEN_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-multiple-neg4");
        return Some(BigInt::from(-4));
    }
    // The earlier branch owns |r|=7 and the MSD guard excludes |r|>=8.
    // On the remaining (7,8) interval, +/-5 leaves a residual below one in
    // magnitude, although +/-4 is nearer near its lower endpoint. The written
    // 17/2 bound is further restricted by that preceding MSD guard.
    if r >= SEVEN_RATIONAL.deref() && r <= SEVENTEEN_HALVES_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-multiple-5");
        return Some(BigInt::from(5));
    }
    if r <= NEG_SEVEN_RATIONAL.deref() && r >= NEG_SEVENTEEN_HALVES_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-multiple-neg5");
        return Some(BigInt::from(-5));
    }
    None
}

/// Approximate the exact expression `r - multiple*pi/2` in `2^p` units without building its graph.
///
/// Sample r with three extra bits and pi more finely according to the integer
/// multiplier's bit length, then combine guarded integers. Multipliers ±4 reuse
/// a dedicated two-pi scaling path. This is an approximation of a generally
/// irrational residual, not an exact rational residual or a certified small-range
/// claim. Consumers must justify their selected multiple independently.
///
/// Pi sampling receives `signal`; rational arithmetic and scale operations add
/// no stop poll. Oversized multiplier bit lengths panic on conversion, and
/// precision subtractions remain unchecked with no total allocation bound.
fn large_rational_half_pi_residual(
    signal: &Option<Signal>,
    r: &Rational,
    multiple: &BigInt,
    p: Precision,
) -> BigInt {
    // Approximate r - multiple*pi/2 at precision p without allocating
    // Add(Multiply(Pi, k), Ratio(r)); retain the same exact defining expression
    // while performing its guarded integer subtraction directly.
    let extra = 3;
    let work_precision = p - extra;
    let rational = ratio(r, work_precision);
    if multiple == signed::FOUR.deref() || multiple == NEG_FOUR_BIGINT.deref() {
        crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-residual-two-pi");
        let pi_precision = work_precision - 6;
        let pi = Computable::pi().approx_signal(signal, pi_precision);
        let two_pi = scale(pi, pi_precision - work_precision + 1);
        let half_pi_multiple = if multiple.sign() == Sign::Minus {
            -two_pi
        } else {
            two_pi
        };
        return scale(rational - half_pi_multiple, -extra);
    }
    let multiple_msd = i32::try_from(multiple.magnitude().bits().saturating_sub(1))
        .expect("large trig quotient bits should fit in i32");
    let pi_precision = work_precision - multiple_msd - 4;
    let pi = Computable::pi().approx_signal(signal, pi_precision);
    let half_pi_multiple = scale(pi * multiple, pi_precision - work_precision - 1);
    scale(rational - half_pi_multiple, -extra)
}

/// Return the Euclidean residue of a signed half-pi multiple modulo four as a BigInt.
/// The double remainder maps negative multiples into 0..=3 for the quadrant
/// identities. This is integer arithmetic, not an angle or nearest-root decision.
fn large_rational_quadrant(multiple: &BigInt) -> BigInt {
    ((multiple % crate::computable::signed::FOUR.deref()) + crate::computable::signed::FOUR.deref())
        % crate::computable::signed::FOUR.deref()
}

/// Approximate `r - (2*multiple + quarter_sign)*pi/4` in `2^p` units.
///
/// Callers use quarter_sign ±1 to shift a half-pi residual toward a quarter-pi
/// anchor. Form that exact integer multiplier before selecting pi precision;
/// three extra result bits protect the final subtraction. The routine does not
/// validate the sign selector or prove the resulting argument small.
/// Pi sampling carries `signal`; bit-length conversion may panic and precision
/// arithmetic/integer allocation have no local resource bound.
fn large_rational_quarter_pi_residual(
    signal: &Option<Signal>,
    r: &Rational,
    multiple: &BigInt,
    quarter_sign: i8,
    p: Precision,
) -> BigInt {
    let extra = 3;
    let work_precision = p - extra;
    let rational = ratio(r, work_precision);
    let quarter_multiple: BigInt = (multiple << 1) + BigInt::from(quarter_sign);
    let multiple_msd = i32::try_from(quarter_multiple.magnitude().bits().saturating_sub(1))
        .expect("large trig quotient bits should fit in i32");
    let pi_precision = work_precision - multiple_msd - 5;
    let pi = Computable::pi().approx_signal(signal, pi_precision);
    let quarter_pi_multiple = scale(pi * quarter_multiple, pi_precision - work_precision - 2);
    scale(rational - quarter_pi_multiple, -extra)
}

/// Compute sine then cosine from one integer sample of a caller-admitted reduced angle.
///
/// `op_appr * 2^op_prec` represents the shared argument sample. Callers provide
/// its error bound and sufficient guard precision (normally `op_prec = p - 2`)
/// as well as a small angle; this routine validates neither. Reuse the squared
/// sample across separate odd/even series and return both results in `2^p` units.
/// Coarse or entry-stop output is `(0,1)`; loop stops can leave partial sums.
/// The caller handles aborts, unchecked precision/term factors and total work.
fn sin_cos_scaled_argument(
    signal: &Option<Signal>,
    op_appr: BigInt,
    op_prec: Precision,
    p: Precision,
) -> (BigInt, BigInt) {
    if p >= 1 {
        return (Zero::zero(), signed::ONE.deref().clone());
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return (Zero::zero(), signed::ONE.deref().clone());
    }

    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");

    let mut sin_n = 1;
    let mut sin_term = scale(op_appr, op_prec - calc_precision);
    let mut sin_sum = sin_term.clone();

    while sin_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        sin_n += 2;
        sin_term = scale(sin_term * &op_squared, op_prec);
        sin_term /= -(sin_n * (sin_n - 1));
        sin_sum += &sin_term;
    }

    let mut cos_n = 0;
    let mut cos_term = signed::ONE.deref() << (-calc_precision);
    let mut cos_sum = cos_term.clone();

    while cos_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        cos_n += 2;
        cos_term = scale(cos_term * &op_squared, op_prec);
        cos_term /= -(cos_n * (cos_n - 1));
        cos_sum += &cos_term;
    }

    (
        scale(sin_sum, calc_precision - p),
        scale(cos_sum, calc_precision - p),
    )
}

/// Round a ratio of equally scaled integer samples into `2^p` units, with ties away from zero.
///
/// Require a nonzero denominator and a nonpositive p whose negation fits i32.
/// Normalize the denominator sign, shift the numerator by -p, and round its
/// absolute quotient before restoring the sign. A zero denominator panics with
/// ArithmeticException. Precision preconditions are not checked here.
/// This rounds the supplied rational quotient; it does not propagate errors in
/// the source samples or detect an exact pole. No signal or allocation cap is used.
fn divide_scaled(numerator: BigInt, denominator: BigInt, p: Precision) -> BigInt {
    let abs_denominator = denominator.abs();
    if abs_denominator.is_zero() {
        panic!("ArithmeticException");
    }

    let scaled_numerator = if denominator.sign() == Sign::Minus {
        -numerator << -p
    } else {
        numerator << -p
    };
    let adjustment = &abs_denominator >> 1;

    if scaled_numerator.sign() == Sign::Minus {
        let rounded: BigInt = ((-scaled_numerator) + adjustment) / abs_denominator;
        -rounded
    } else {
        (scaled_numerator + adjustment) / abs_denominator
    }
}

/// Refine bounded trigonometric numerator/denominator samples before rounding their quotient.
///
/// The callback supplies both components at a common requested precision, with
/// one-unit errors and bounded numerator magnitude as in sine/cosine. A nonzero
/// sampled denominator determines extra precision proportional to twice its
/// negative leading-bit exponent; a zero sample requests eight more bits.
/// This is not a general quotient-error rule for arbitrary unbounded callbacks.
/// The caller must admit a nonpole and a nonpositive output precision.
///
/// A reschedule polls the stop flag, then panics if extra precision exceeds
/// 16384 bits. That guard is not total-work accounting or proof of a pole;
/// a ready quotient returns before those checks. Callback work, the leading-bit
/// cast and signed precision arithmetic are unchecked. Abort output is zero
/// without typed status.
fn divide_scaled_refining<F>(signal: &Option<Signal>, p: Precision, mut components: F) -> BigInt
where
    F: FnMut(Precision) -> (BigInt, BigInt),
{
    let mut working_precision = p - 8;

    loop {
        let (numerator, denominator) = components(working_precision);
        if !denominator.is_zero() {
            let denominator_msd =
                working_precision + denominator.magnitude().bits() as Precision - 1;
            // Quotient sensitivity is proportional to 1/d^2. Each bit that
            // |d| lies below one therefore costs two source bits.
            let required_precision = p + 2 * denominator_msd.min(0) - 8;
            if working_precision <= required_precision {
                return divide_scaled(numerator, denominator, p);
            }
            working_precision = required_precision;
        } else {
            working_precision -= 8;
        }

        if should_stop(signal) {
            return Zero::zero();
        }
        if p - working_precision > 16_384 {
            panic!("ArithmeticException");
        }
    }
}

/// Form a tangent quotient from the shared sine/cosine samples of a reduced argument.
///
/// This helper does no adaptive denominator refinement. The quarter-pi caller
/// supplies a small residual and extra precision so the cosine is safely separated
/// from zero. It inherits the sample/error preconditions of sin_cos_scaled_argument
/// and the precision/nonzero-denominator requirements of divide_scaled.
fn tan_scaled_argument(
    signal: &Option<Signal>,
    op_appr: BigInt,
    op_prec: Precision,
    p: Precision,
) -> BigInt {
    let (sin_appr, cos_appr) = sin_cos_scaled_argument(signal, op_appr, op_prec, p);
    divide_scaled(sin_appr, cos_appr, p)
}

/// Try a quarter-pi tangent identity when a sampled half-pi residual is far enough from zero.
///
/// Compare the precision -8 residual sample against ±128 to choose an anchor
/// side; this is a guarded sample test, not exact equality to a half-radian
/// threshold. Evaluate the smaller quarter residual with eight extra result bits,
/// then select `(1+t)/(1-t)` or `(t-1)/(1+t)` from quadrant parity and side.
/// The caller must supply a suitable half-pi multiple and nonpositive precision.
///
/// None declines this shortcut; Some is a computed integer, not a separate
/// certificate of cancellation, domain or resource completion. Divisions rely on
/// the reduced range; this path adds no adaptive quotient refinement or local budget.
fn tan_large_rational_quarter_pi(
    signal: &Option<Signal>,
    r: &Rational,
    multiple: &BigInt,
    p: Precision,
) -> Option<BigInt> {
    let quadrant = large_rational_quadrant(multiple).to_u8()?;

    let rough_residual = large_rational_half_pi_residual(signal, r, multiple, -8);
    let quarter_sign = if rough_residual >= *QUARTER_PI_TAN_RESIDUAL_THRESHOLD.deref() {
        1
    } else if rough_residual <= -QUARTER_PI_TAN_RESIDUAL_THRESHOLD.deref().clone() {
        -1
    } else {
        return None;
    };

    crate::trace_dispatch!("computable_approx", "tan", "quarter-pi-large-rational");
    let working_prec = p - 8;
    let op_prec = working_prec - 2;
    let delta = large_rational_quarter_pi_residual(signal, r, multiple, quarter_sign, op_prec);
    let tan_delta = tan_scaled_argument(signal, delta, op_prec, working_prec);
    let one = signed::ONE.deref() << -working_prec;
    let direct_tangent_form = (quadrant % 2 == 0) == (quarter_sign > 0);
    let (numerator, denominator) = if direct_tangent_form {
        (one.clone() + &tan_delta, one - tan_delta)
    } else {
        (tan_delta.clone() - &one, one + tan_delta)
    };
    Some(divide_scaled(numerator, denominator, p))
}

/// Evaluate cosine of `r - multiple*pi/2` using the direct guarded residual as its series input.
/// The caller must select a residual in the local small-angle range. This avoids
/// a composite Computable argument while retaining the exact r/pi definition.
/// Return `2^p` units; coarse/entry-stop output is one, and loop stops can return
/// partial sums. Precision and term-factor arithmetic have no checked work bound.
fn cos_large_rational_residual(
    signal: &Option<Signal>,
    r: &Rational,
    multiple: &BigInt,
    p: Precision,
) -> BigInt {
    // Same Taylor kernel as cos(|x| < 1), but the argument approximation comes
    // from the direct residual above instead of a child Computable node.
    if p >= 1 {
        return signed::ONE.deref().clone();
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return signed::ONE.deref().clone();
    }

    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_prec = p - 2;
    let op_appr = large_rational_half_pi_residual(signal, r, multiple, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut n = 0;
    let mut current_term = signed::ONE.deref() << (-calc_precision);
    let mut current_sum = current_term.clone();

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;
        current_term = scale(current_term * &op_squared, op_prec);
        current_term /= -(n * (n - 1));
        current_sum += &current_term;
    }
    scale(current_sum, calc_precision - p)
}

/// Evaluate sine of a caller-selected small residual `r - multiple*pi/2`.
/// Feed its guarded integer sample to the odd Taylor series without constructing
/// a reduced expression graph. Return `2^p` units; coarse/entry-stop output is
/// zero, and loop stops can return partial sums. Callers own residual admission,
/// abort publication and the unchecked precision/term-factor resource obligations.
fn sin_large_rational_residual(
    signal: &Option<Signal>,
    r: &Rational,
    multiple: &BigInt,
    p: Precision,
) -> BigInt {
    // Same Taylor kernel as sin(|x| < 1), fed by the direct large-rational
    // residual to avoid constructing a generic reduced Computable expression.
    if p >= 1 {
        return Zero::zero();
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return Zero::zero();
    }

    let calc_precision = p - bound_log2(2 * iterations_needed) - 4;
    let op_prec = p - 2;
    let op_appr = large_rational_half_pi_residual(signal, r, multiple, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut n = 1;
    let mut current_term = scale(op_appr.clone(), op_prec - calc_precision);
    let mut current_sum = current_term.clone();

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;
        current_term = scale(current_term * &op_squared, op_prec);
        current_term /= -(n * (n - 1));
        current_sum += &current_term;
    }
    scale(current_sum, calc_precision - p)
}

/// Evaluate sine then cosine of one selected rational-minus-half-pi residual.
/// Reuse its sample and square across the two series, returning both in `2^p`
/// units. The caller supplies the small-residual guarantee. Coarse/entry-stop
/// output is `(0,1)`; a loop stop is not a certified completed pair. Signed
/// precision and term-factor arithmetic and total integer work are unbounded here.
fn sin_cos_large_rational_residual(
    signal: &Option<Signal>,
    r: &Rational,
    multiple: &BigInt,
    p: Precision,
) -> (BigInt, BigInt) {
    if p >= 1 {
        return (Zero::zero(), signed::ONE.deref().clone());
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return (Zero::zero(), signed::ONE.deref().clone());
    }

    let calc_precision = p - bound_log2(2 * iterations_needed) - 4;
    let op_prec = p - 2;
    let op_appr = large_rational_half_pi_residual(signal, r, multiple, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");

    let mut sin_n = 1;
    let mut sin_term = scale(op_appr, op_prec - calc_precision);
    let mut sin_sum = sin_term.clone();

    while sin_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        sin_n += 2;
        sin_term = scale(sin_term * &op_squared, op_prec);
        sin_term /= -(sin_n * (sin_n - 1));
        sin_sum += &sin_term;
    }

    let mut cos_n = 0;
    let mut cos_term = signed::ONE.deref() << (-calc_precision);
    let mut cos_sum = cos_term.clone();

    while cos_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        cos_n += 2;
        cos_term = scale(cos_term * &op_squared, op_prec);
        cos_term /= -(cos_n * (cos_n - 1));
        cos_sum += &cos_term;
    }

    (
        scale(sin_sum, calc_precision - p),
        scale(cos_sum, calc_precision - p),
    )
}

/// Approximate cosine of a retained rational via a selected half-pi residual and quadrant identity.
/// Use cosine, negative sine, negative cosine or sine for quadrants zero through
/// three. Defer reduction until digits are requested, preserving the rational
/// source without constructing a generic reduced graph. Return `2^p` units;
/// this wrapper inherits the reducer/kernel limits and has no separate stop poll.
fn cos_large_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Defer half-pi reduction until evaluation and use the direct residual
    // kernels without recursing back through Computable::cos.
    let multiple = large_rational_half_pi_multiple(signal, r, p);
    match large_rational_quadrant(&multiple).to_u8() {
        Some(0) => cos_large_rational_residual(signal, r, &multiple, p),
        Some(1) => -sin_large_rational_residual(signal, r, &multiple, p),
        Some(2) => -cos_large_rational_residual(signal, r, &multiple, p),
        Some(3) => sin_large_rational_residual(signal, r, &multiple, p),
        _ => unreachable!("quadrant reduction is modulo four"),
    }
}

/// Approximate the exact expression `pi/2 - r` with two guard bits before final subtraction.
/// The input is rational; the residual generally is not. Sample half-pi and r
/// at a common working scale without building a composite node, then return
/// `2^p` units. This arithmetic helper does not certify a small residual. Pi
/// sampling receives `signal`; precision and allocation limits remain unchecked.
fn half_pi_minus_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Specialized residual for exact medium trig inputs. It performs the same
    // guarded subtraction as the generic Add(Offset(pi), -r) path, but without
    // allocating or querying a composite expression on every cold approximation.
    let extra = 2;
    let work_precision = p - extra;
    let half_pi = Computable::pi().approx_signal(signal, work_precision + 1);
    let rational = ratio(r, work_precision);
    scale(half_pi - rational, -extra)
}

/// Approximate `cos(pi/2 - r)` for caller-admitted exact `1 <= r < 3/2`.
/// The reduced angle lies in the positive small-angle range. Sample the retained
/// pi/rational difference directly and use the even series, returning `2^p`
/// units. Coarse/entry-stop output is one; loop stops can leave partial sums.
/// The helper does not recheck the interval or bound precision/counter work.
fn cos_half_pi_minus_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    if p >= 1 {
        return signed::ONE.deref().clone();
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return signed::ONE.deref().clone();
    }

    let calc_precision = p - bound_log2(2 * iterations_needed) - 4;
    let op_prec = p - 2;
    // Sample the pi/rational residual directly from cached pi. The residual
    // is generally irrational; the generic route would first build an Add tree.
    let op_appr = half_pi_minus_rational(signal, r, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    // Keep the truncation guard allocation-free across Taylor iterations.
    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut n = 0;
    let mut current_term = signed::ONE.deref() << (-calc_precision);
    let mut current_sum = current_term.clone();

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;

        current_term = scale(current_term * &op_squared, op_prec);
        current_term /= -(n * (n - 1));

        current_sum += &current_term;
    }
    scale(current_sum, calc_precision - p)
}

/// Approximate reduced-argument sine by a guarded odd Taylor series in `2^p` units.
///
/// The nominal schedule uses `abs(c) < 1`, without checking that range here;
/// internal tangent scheduling also admits magnitude below two. Sample at
/// `p - 2`, accumulate odd terms and round once to the output scale. A coarse
/// `p >= 1` zero is valid from the global sine bound, independently of local
/// series applicability. Entry stops return zero and loop stops partial sums.
/// The caller owns abort publication; precision/counter products are unchecked.
fn sin(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Kernel precondition: |c| < 1. The caller keeps large arguments out of this
    // Taylor loop so huge sin/cos rows spend time in reduction, not series setup.
    if p >= 1 {
        return Zero::zero();
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return Zero::zero();
    }

    //  Claim: each intermediate term is accurate
    //  to 2*2^calc_precision.
    //  Total rounding error in series computation is
    //  2*iterations_needed*2^calc_precision,
    //  exclusive of error in op.
    let calc_precision = p - bound_log2(2 * iterations_needed) - 4; // for error in op, truncation.
    let op_prec = p - 2;
    let op_appr = c.approx_signal(signal, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    // Error in argument results in error of < 1/4 ulp.
    // Cumulative arithmetic rounding error is < 1/16 ulp.
    // Series truncation error < 1/16 ulp.
    // Final rounding error is <= 1/2 ulp.
    // Thus final error is < 1 ulp.

    // Keep the truncation guard allocation-free across Taylor iterations.
    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut n = 1;
    let mut current_term = scale(op_appr.clone(), op_prec - calc_precision);
    let mut current_sum = current_term.clone();

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;

        // t_n = -t_(n-2) * x^2 / (n * (n - 1)).
        current_term = scale(current_term * &op_squared, op_prec);
        current_term /= -(n * (n - 1));

        current_sum += &current_term;
    }
    scale(current_sum, calc_precision - p)
}

/// Approximate sine for a caller-admitted exact rational `abs(r) < 1` without a temporary leaf node.
/// Sample the retained rational directly, then use the guarded odd series.
/// Return `2^p` units. Coarse zero follows the sine bound; stopped or partial
/// output must not be certified by the caller. No range check, typed precision
/// failure or total-work bound is supplied by this helper.
fn sin_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Direct exact-rational variant of `sin`. It avoids allocating a temporary
    // Computable leaf for PrescaledSinRational while still rounding the rational
    // argument exactly once at the requested guard precision.
    if p >= 1 {
        return Zero::zero();
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return Zero::zero();
    }

    let calc_precision = p - bound_log2(2 * iterations_needed) - 4;
    let op_prec = p - 2;
    let op_appr = ratio(r, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut n = 1;
    let mut current_term = scale(op_appr.clone(), op_prec - calc_precision);
    let mut current_sum = current_term.clone();

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;

        current_term = scale(current_term * &op_squared, op_prec);
        current_term /= -(n * (n - 1));

        current_sum += &current_term;
    }
    scale(current_sum, calc_precision - p)
}

/// Approximate sine of a retained rational through half-pi reduction and quadrant identities.
/// Use sine, cosine, negative sine or negative cosine for quadrants zero through
/// three. The selected residual need not come from a uniquely nearest multiple.
/// Return `2^p` units and inherit the reducer/series preconditions, abort handling
/// and precision/resource limits; this wrapper adds no stop poll.
fn sin_large_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Same lazy public-construction policy as cosine. The direct residual
    // arithmetic avoids allocating the generic reduced expression tree while
    // preserving the standard quadrant identities.
    let multiple = large_rational_half_pi_multiple(signal, r, p);
    match large_rational_quadrant(&multiple).to_u8() {
        Some(0) => sin_large_rational_residual(signal, r, &multiple, p),
        Some(1) => cos_large_rational_residual(signal, r, &multiple, p),
        Some(2) => -sin_large_rational_residual(signal, r, &multiple, p),
        Some(3) => -cos_large_rational_residual(signal, r, &multiple, p),
        _ => unreachable!("quadrant reduction is modulo four"),
    }
}

/// Approximate `sin(pi/2 - r)` for caller-admitted exact `1 <= r < 3/2`.
/// Sample the generally irrational difference directly before the odd series,
/// returning `2^p` units. Coarse/entry-stop output is zero; loop stops can return
/// partial sums. The interval is not rechecked, and signed precision/term-factor
/// arithmetic has no checked resource contract.
fn sin_half_pi_minus_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    if p >= 1 {
        return Zero::zero();
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return Zero::zero();
    }

    let calc_precision = p - bound_log2(2 * iterations_needed) - 4;
    let op_prec = p - 2;
    // Sample the pi/rational residual directly from cached pi. The residual
    // is generally irrational; the generic route would first build an Add tree.
    let op_appr = half_pi_minus_rational(signal, r, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    // Keep the truncation guard allocation-free across Taylor iterations.
    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut n = 1;
    let mut current_term = scale(op_appr.clone(), op_prec - calc_precision);
    let mut current_sum = current_term.clone();

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;

        current_term = scale(current_term * &op_squared, op_prec);
        current_term /= -(n * (n - 1));

        current_sum += &current_term;
    }
    scale(current_sum, calc_precision - p)
}

/// Evaluate sine then cosine of the same `pi/2 - r` sample for a cotangent quotient.
/// Require the medium rational range `1 <= r < 3/2`; reuse the residual sample
/// and square across the two series. Return a pair in `2^p` units, or `(0,1)`
/// for coarse/entry-stop requests. Loop stops leave partial pairs for the caller
/// to reject. Precision/term-factor arithmetic and total work are unchecked.
fn sin_cos_half_pi_minus_rational(
    signal: &Option<Signal>,
    r: &Rational,
    p: Precision,
) -> (BigInt, BigInt) {
    // Shared complement kernel for cot(pi/2-r): sample the pi/rational
    // residual once, then feed both Taylor sums. This keeps the medium tangent
    // shortcut from paying two pi/rational subtractions per approximation.
    if p >= 1 {
        return (Zero::zero(), signed::ONE.deref().clone());
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return (Zero::zero(), signed::ONE.deref().clone());
    }

    let calc_precision = p - bound_log2(2 * iterations_needed) - 4;
    let op_prec = p - 2;
    let op_appr = half_pi_minus_rational(signal, r, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");

    let mut sin_n = 1;
    let mut sin_term = scale(op_appr, op_prec - calc_precision);
    let mut sin_sum = sin_term.clone();

    while sin_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        sin_n += 2;
        sin_term = scale(sin_term * &op_squared, op_prec);
        sin_term /= -(sin_n * (sin_n - 1));
        sin_sum += &sin_term;
    }

    let mut cos_n = 0;
    let mut cos_term = signed::ONE.deref() << (-calc_precision);
    let mut cos_sum = cos_term.clone();

    while cos_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        cos_n += 2;
        cos_term = scale(cos_term * &op_squared, op_prec);
        cos_term /= -(cos_n * (cos_n - 1));
        cos_sum += &cos_term;
    }

    (
        scale(sin_sum, calc_precision - p),
        scale(cos_sum, calc_precision - p),
    )
}

/// Approximate `cot(pi/2 - r) = tan(r)` from shared complement samples for `1 <= r < 3/2`.
///
/// At nonpositive precision, refine cosine/sine components before dividing,
/// avoiding a generic inverse expression. Domain admission and abort handling
/// remain with the caller. For `p >= 1` this currently returns zero without a
/// magnitude proof; that is not valid throughout the stated interval. For example,
/// a cold tan(7/5) request at p=1 returns zero and later refinement changes it.
/// This is an existing coarse-precision error, not a certified zero result.
fn cot_half_pi_minus_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // tan(r) near pi/2 is cot(pi/2-r). Reusing the same pi/rational
    // residual sample for both numerator and denominator avoids the generic
    // PrescaledCot(Offset(pi/2, -r)) expression graph.
    if p >= 1 {
        return Zero::zero();
    }

    divide_scaled_refining(signal, p, |working_precision| {
        let (sin_appr, cos_appr) = sin_cos_half_pi_minus_rational(signal, r, working_precision);
        (cos_appr, sin_appr)
    })
}

// Compute tangent of |c| < 1.
// This uses the direct quotient tan(x) = sin(x) / cos(x),
// but computes both approximations locally to avoid building
// separate Computable trees for sin, cos, inverse, and multiply.
/// Approximate tangent through locally evaluated sine/cosine components and adaptive division.
///
/// The intended reduced range is `abs(c) < 1`, but native structural scheduling
/// can pass `abs(c) < 2`, including values near a pole. Neither this routine nor
/// its component kernels validates the full domain; a nonpole is required.
/// For nonpositive p, denominator refinement supplies extra component precision.
/// The coarse `p >= 1` zero shortcut is not valid for every scheduled argument:
/// a cold tan(3/2) request at p=1 demonstrates the existing error.
/// Abort publication and unchecked precision/term-factor costs remain caller obligations.
fn tan(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // The nominal reduced range is |c| < 1, but structural scheduling also
    // admits |c| < 2. The coarse-zero branch is not justified over that wider
    // range; nonpositive requests refine the denominator below.
    if p >= 1 {
        return Zero::zero();
    }

    divide_scaled_refining(signal, p, |working_precision| {
        (
            sin(signal, c, working_precision),
            cos(signal, c, working_precision),
        )
    })
}

/// Evaluate tangent for caller-admitted exact `abs(r) < 1` using direct rational sine/cosine samples.
/// Refine the denominator locally rather than building a Ratio child, inverse
/// and multiply graph. In this range the coarse p>=1 zero is within one unit;
/// this helper does not establish the range itself. Return `2^p` units and
/// inherit the quotient helper's abort, precision and resource limitations.
fn tan_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Same local quotient as `tan`, but both numerator and denominator consume
    // the stored exact rational directly. This keeps exact-rational tangent
    // approximation lazy without rebuilding child Ratio nodes.
    if p >= 1 {
        return Zero::zero();
    }

    divide_scaled_refining(signal, p, |working_precision| {
        (
            sin_rational(signal, r, working_precision),
            cos_rational(signal, r, working_precision),
        )
    })
}

/// Approximate tangent of an exact rational using half-pi reduction and local quotient identities.
///
/// Try the quarter-pi identity before adaptive sine/cosine division. Quadrant
/// transport preserves both component signs, then common sample scales cancel.
/// A rational angle cannot equal an irrational half-pi pole, but it can lie very
/// close to one and require extensive refinement.
///
/// The current `p >= 1` shortcut returns zero before reduction without a tangent
/// magnitude bound; tan(39/5) gives an incorrect cold result at p=1. It must not
/// be interpreted as a valid global coarse approximation. Finer evaluation still
/// inherits the reducer, arithmetic, cancellation and resource limitations.
fn tan_large_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Large tangent is evaluated as a local sin/cos quotient after the same
    // direct half-pi reduction used by sin/cos. This keeps exact large rationals
    // off the generic pi-reduction path and avoids constructing inverse nodes.
    crate::trace_dispatch!("computable_approx", "tan", "large-rational-direct-quotient");
    if p >= 1 {
        return Zero::zero();
    }

    let multiple = large_rational_half_pi_multiple(signal, r, p);
    if let Some(reduced) = tan_large_rational_quarter_pi(signal, r, &multiple, p) {
        return reduced;
    }
    divide_scaled_refining(signal, p, |working_precision| {
        let (sin_residual, cos_residual) =
            sin_cos_large_rational_residual(signal, r, &multiple, working_precision);
        match large_rational_quadrant(&multiple).to_u8() {
            Some(0) => (sin_residual, cos_residual),
            Some(1) => (cos_residual, -sin_residual),
            Some(2) => (-sin_residual, -cos_residual),
            Some(3) => (-cos_residual, sin_residual),
            _ => unreachable!("quadrant reduction is modulo four"),
        }
    })
}

// Compute cotangent of |c| < 1.
// This mirrors tan(x) = sin(x) / cos(x), but flips the quotient so
// tan(pi/2 - x) can avoid building an extra inverse Computable node.
/// Approximate cotangent of a nonzero reduced argument by refining cosine/sine components.
///
/// The intended magnitude is below one; zero is a pole. A small nonzero sine can
/// require substantial refinement, so it is not assumed safely separated merely
/// because a complement node exists. The caller must admit the domain and handle
/// aborted output. The current p>=1 zero shortcut is invalid for sufficiently
/// small arguments: imported cot(1/64) reproduces that coarse-precision error.
/// The refining quotient's precision guard is not a typed total resource budget.
fn cot(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Tangent complement reduction can leave sin(c) arbitrarily close to zero.
    // Nonpositive requests refine that denominator; the coarse-zero shortcut
    // has no valid global cotangent bound.
    if p >= 1 {
        return Zero::zero();
    }

    divide_scaled_refining(signal, p, |working_precision| {
        (
            cos(signal, c, working_precision),
            sin(signal, c, working_precision),
        )
    })
}
