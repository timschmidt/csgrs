/// Evaluate principal acosh for an admitted `c >= 1` through a near-one logarithm reduction.
///
/// Use `ln1p((c-1) + sqrt(c^2-1))` so a small result enters the stable log1p
/// path. The identity is valid throughout this domain; the name describes its
/// scheduling role, not a separately enforced upper bound. Build the exact
/// graph on demand, then pass `signal` and `p` to its approximation. This wrapper
/// performs no domain check, stop poll or independent resource accounting.
fn acosh_near_one(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Near one, acosh(x) is ln1p((x - 1) + sqrt(x^2 - 1)). Deferring the graph
    // keeps construction cheap for endpoint-adjacent scalar rows without
    // changing the cancellation-avoiding approximation identity.
    let one = Computable::one();
    let shifted = c.clone().add(one.clone().negate());
    let radicand = c.clone().square().add(one.negate());
    shifted
        .add(radicand.sqrt())
        .ln_1p()
        .approx_signal(signal, p)
}

/// Evaluate principal acosh for an admitted `c >= 1` as `ln(c + sqrt(c^2-1))`.
///
/// This deferred node serves inputs outside the near-one schedule without
/// allocating the sqrt/log graph during public construction. The identity also
/// holds at one. Return an integer in `2^p` units through the child expression;
/// domain checking, cancellation publication and resource limits belong to
/// its callers and underlying operations.
fn acosh_direct(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Large acosh inputs use ln(x + sqrt(x^2 - 1)); this node is used by Real
    // construction paths where allocating that graph eagerly is the bottleneck.
    let one = Computable::one();
    let radicand = c.clone().square().add(one.negate());
    c.clone().add(radicand.sqrt()).ln().approx_signal(signal, p)
}

/// Evaluate asinh through `ln1p(c + c^2/(sqrt(1+c^2)+1))` on demand.
///
/// Rationalizing `sqrt(1+c^2)-1` avoids that subtraction for small inputs.
/// The exact identity holds for every finite real child, although this node
/// serves the near-zero schedule and does not certify a uniform cost bound.
/// It builds the reduction at evaluation time and forwards `signal` and `p`,
/// with no local stop poll or total-work accounting.
fn asinh_near_zero(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Near zero, asinh(x) is evaluated through ln1p(x + x^2/(sqrt(1+x^2)+1)).
    // This deferred node removes construction overhead but preserves the
    // cancellation-resistant formula used by the public Real path.
    let square = c.clone().square();
    let one = Computable::one();
    let denominator = square.clone().add(one.clone()).sqrt().add(one);
    c.clone()
        .add(square.multiply(denominator.inverse()))
        .ln_1p()
        .approx_signal(signal, p)
}

/// Evaluate the direct identity `asinh(c) = ln(c + sqrt(1+c^2))` on demand.
///
/// The identity is real for every finite child. Callers use odd symmetry for
/// known negative inputs to avoid cancellation in the logarithm's argument;
/// this helper itself does not normalize the sign. It defers graph allocation
/// until digits are requested, passes `signal` and `p` through, and adds no
/// stop poll or independent resource limit.
fn asinh_direct(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Large asinh inputs use ln(x + sqrt(1+x^2)); deferring the direct identity
    // keeps scalar construction from allocating the sqrt/log graph eagerly.
    let radicand = c.clone().square().add(Computable::one());
    c.clone().add(radicand.sqrt()).ln().approx_signal(signal, p)
}

/// Route a sign-separated argument outside the small asinh series to the direct identity.
///
/// The caller supplies a proved `negative` flag from an exact rational or a
/// working sample separated from zero. Negate both input and result in that
/// case, implementing odd symmetry before the log/sqrt evaluation so a large
/// negative source does not cancel internally. This helper does not verify the
/// flag or add cancellation polls, precision checks or a total-work bound.
fn asinh_outside_series(
    signal: &Option<Signal>,
    c: &Computable,
    negative: bool,
    p: Precision,
) -> BigInt {
    // A separated series sample (or the exact rational) proves this sign.
    // Use odd symmetry before the existing logarithmic identity so a large
    // negative argument cannot cancel against sqrt(1+x^2).
    if negative {
        -asinh_direct(signal, &c.clone().negate(), p)
    } else {
        asinh_direct(signal, c, p)
    }
}

/// Approximate asinh with a sample-certified `abs(c) <= 1/2` series, otherwise reduce by sign.
///
/// Use `min(p, 0)` when sizing the range probe, so even a coarse request must
/// justify the small-input shortcut. Reserving the sample's one-unit error
/// proves the half bound. A failed check also separates the sign and selects
/// the direct logarithmic identity with odd symmetry; a stored tag alone does
/// not prove the range. Only after that check may `p >= 1` return zero.
///
/// The alternating odd series shares asin's coefficient magnitudes. It returns
/// `2^p` units, or an entry-stop placeholder/loop partial sum for the caller to
/// reject on abort. Precision and counter arithmetic is unchecked, and the
/// iteration estimate and guard bits do not impose a total-work limit.
fn asinh_computable(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Dedicated tiny-argument asinh series. It avoids constructing the generic
    // ln(x + sqrt(1+x^2)) or ln1p expression for exact tiny rational inputs.
    if should_stop(signal) {
        return Zero::zero();
    }

    let series_precision = p.min(0);
    let iterations_needed: i32 = -series_precision / 2 + 4;
    let calc_precision = series_precision - bound_log2(2 * iterations_needed) - 5;
    let op_prec = calc_precision - 3;
    let op_appr = c.approx_signal(signal, op_prec);
    // bits(|a|) <= -op_prec-1, including the sample's one-unit error,
    // proves |c| <= 1/2. An imported tag is not this range certificate.
    if op_appr.magnitude().bits()
        > u64::try_from(-op_prec - 1).expect("asinh series scale is positive")
    {
        if should_stop(signal) {
            return Zero::zero();
        }
        return asinh_outside_series(signal, c, op_appr.sign() == Sign::Minus, p);
    }
    // |asinh(c)| <= |c| <= 1/2 justifies the coarse approximation only
    // after the range proof, independently of the requested precision.
    if p >= 1 {
        return Zero::zero();
    }
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    // This is the asin recurrence with alternating sign:
    // asinh(x) = x - x^3/6 + 3x^5/40 - ...
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
        current_term = -current_term;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

/// Approximate asinh from an exact rational without a temporary sampled child node.
///
/// Zero or a known binary magnitude at most -2 admits the alternating series,
/// which means strict magnitude below one half for nonzero inputs. All other
/// magnitudes, including missing metadata, route to the direct identity after
/// sign normalization. The small range justifies coarse `p >= 1` returning zero.
///
/// Rational sampling and series setup precede the loop's cancellation poll;
/// partial sums are not certified abort results. Working precisions/counters
/// must fit, and this helper supplies no coefficient-size or total-work cap.
fn asinh_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Direct exact-rational variant of the tiny asinh series. It is the same
    // recurrence as `asinh_computable`, but it feeds the stored Rational
    // straight to the final precision request instead of allocating a temporary
    // Ratio node and child cache.
    // Only zero or a proved small magnitude admits the series. A magnitude
    // outside the metadata's i32 range is not a small-argument certificate.
    if r.sign() != Sign::NoSign && r.msd_exact().is_none_or(|msd| msd >= -1) {
        if should_stop(signal) {
            return Zero::zero();
        }
        return asinh_outside_series(
            signal,
            &Computable::rational(r.clone()),
            r.sign() == Sign::Minus,
            p,
        );
    }
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
        let (numerator, denominator) = inverse_sine_series_coefficients(n);
        current_term *= numerator;
        current_term /= denominator;
        current_term = -current_term;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

/// Evaluate principal atanh for admitted `-1 < c < 1` through `ln((1+c)/(1-c))/2`.
///
/// Defer the positive log-ratio graph until approximation is requested. Domain
/// checking must precede this node: endpoints are infinite and exterior real
/// arguments are outside the real branch. Forward `signal` and `p` to the
/// expression, adding no stop poll, typed domain check or separate work limit.
fn atanh_direct(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Endpoint atanh construction should not eagerly allocate the full exact
    // log-ratio graph. Approximation still uses the stable identity
    // atanh(x) = 1/2 * ln((1+x)/(1-x)).
    let one = Computable::one();
    let numerator = one.clone().add(c.clone());
    let denominator = one.add(c.clone().negate());
    numerator
        .multiply(denominator.inverse())
        .ln()
        .multiply(Computable::rational(HALF_RATIONAL.clone()))
        .approx_signal(signal, p)
}

/// Approximate principal atanh on `(-1, 1)` with a certified tiny series or a log ratio.
///
/// After an entry stop check, coarse requests use the child's small-argument
/// proof before returning zero; failure selects the direct identity. Finer
/// requests reuse the series' working sample, reserving its one-unit error to
/// prove `abs(c) <= 1/8`. This range check is distinct from full domain admission
/// and must not be inferred solely from a stored prescaled tag.
///
/// Accumulate `c + c^3/3 + c^5/5 + ...` with guarded integer powers and divisions,
/// then scale to `2^p` units. Loop stops leave partial output for the caller to
/// reject. Precision/counter arithmetic is unchecked; iteration planning is
/// neither a total-work bound nor a cancellation-latency guarantee.
fn atanh_computable(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Dedicated tiny-argument atanh series. Prove its domain before both the
    // fixed error budget and the coarse-zero shortcut, including stored nodes.
    if should_stop(signal) {
        return Zero::zero();
    }
    if p >= 1 {
        if c.inverse_series_argument_is_small(signal) || should_stop(signal) {
            return Zero::zero();
        }
        return atanh_direct(signal, c, p);
    }

    let iterations_needed: i32 = -p / 2 + 4;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_prec = calc_precision - 3;
    let op_appr = c.approx_signal(signal, op_prec);
    // The sample is already required by the series. bits(|a|) <= -op_prec-3
    // leaves one integer unit for its error and proves |c| <= 1/8.
    if op_appr.magnitude().bits()
        > u64::try_from(-op_prec - 3).expect("tiny-series scale is positive")
    {
        if should_stop(signal) {
            return Zero::zero();
        }
        return atanh_direct(signal, c, p);
    }
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    // Borrowed magnitude checks matter here because tiny inverse-hyperbolic
    // benches run many short series from cold caches.
    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut current_power = scale(op_appr, op_prec - calc_precision);
    let mut current_term = current_power.clone();
    let mut sum = current_term.clone();
    let mut n = 1_i32;

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;
        current_power = scale(current_power * &op_squared, op_prec);
        current_term = &current_power / n;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

/// Approximate principal atanh of an admitted exact rational in `(-1, 1)`.
///
/// Zero or a known binary magnitude at most -4 admits the direct odd series,
/// so nonzero series inputs have magnitude strictly below one eighth. Otherwise
/// re-enter the native rational constructor, whose exact log-ratio and shared
/// half-argument identities avoid recreating this inadmissible series schedule.
/// Missing magnitude metadata is not treated as evidence of smallness.
///
/// For the small range, `p >= 1` may return zero; finer requests sample the
/// rational directly and truncate successive odd-power quotients. Setup has no
/// initial stop poll on that path; loop stops return a partial sum for the caller
/// to reject. Signed precision arithmetic and integer work have no local cap.
fn atanh_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Direct exact-rational variant of the tiny atanh series. This mirrors the
    // direct rational trig kernels: preserve the symbolic rational payload and
    // round only once at the requested working precision.
    // A stored rational tag does not prove the tiny-series range. For nonzero
    // r, an exact MSD <= -4 proves |r| < 1/8 and justifies both this series
    // and the coarse-zero shortcut. Missing metadata is not a range proof.
    if r.sign() != Sign::NoSign && r.msd_exact().is_none_or(|msd| msd >= -3) {
        if should_stop(signal) {
            return Zero::zero();
        }
        // Native rational atanh uses this kernel only for MSD <= -4, so this
        // fallback instead reaches its exact logarithm reduction or shared
        // half-argument identities without recreating the inadmissible series node.
        return Computable::rational(r.clone())
            .atanh()
            .approx_signal(signal, p);
    }
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
    let mut current_power = scale(op_appr, op_prec - calc_precision);
    let mut current_term = current_power.clone();
    let mut sum = current_term.clone();
    let mut n = 1_i32;

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;
        current_power = scale(current_power * &op_squared, op_prec);
        current_term = &current_power / n;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}
