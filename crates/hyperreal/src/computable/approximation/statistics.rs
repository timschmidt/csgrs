/// Convert an integer planning estimate to the signed precision type, using MAX on any failed conversion.
/// The caller supplies a nonnegative squared-magnitude estimate. This is not a
/// checked precision-arithmetic API: subsequent additions to the MAX sentinel can
/// overflow, and negative out-of-range inputs would also map to MAX, not MIN.
fn to_prec(n: &BigInt) -> Precision {
    n.to_i32().unwrap_or(Precision::MAX)
}

/// Approximate the odd auxiliary series S(x)=sqrt(pi)*exp(x^2)*erf(x)/2 in units of 2^p.
/// Start with x and advance terms by 2*x^2/(2*n+3). The caller's erf graph supplies
/// the Gaussian and 2/sqrt(pi) factors; this helper does not directly return erf.
/// A -10 sample estimates squared magnitude, term count and guard bits. Evaluate
/// x with eight additional bits, then use rounded products and integer division.
///
/// Stop before adding a zero next term or when decreasing rounded-term magnitudes
/// satisfy the geometric-tail test cur*prev/(prev-cur) < 2^(p-4-calc_precision).
/// The ratios of the exact series terms decrease eventually; this implementation
/// uses rounded terms and planning guards, not a separately returned interval
/// certificate for accumulated rounding and truncation. Rescale the retained sum
/// at exit. A stop observed in the loop also exits with a partial sum for the
/// caller to reject; planning and the initial evaluations precede that poll.
///
/// Defined finite x gives a convergent mathematical series, but intermediate size
/// can grow like exp(x^2). There is no total-work or heap-byte limit. Unchecked
/// precision/counter arithmetic can overflow: in debug builds the retained series
/// at x=65536,p=0 overflows a magnitude guard, and p=MIN at x=1 overflows -p.
/// These are implementation limits, not typed exhaustion or a narrower definition
/// of erf. The helper does not validate arbitrary imported children.
fn erf_series(signal: &Option<Signal>, op: &Computable, p: Precision) -> BigInt {
    let rough_x = op.approx_signal(signal, -10);
    let x_sq_approx = (&rough_x * &rough_x) >> 20;

    let n_estimate = {
        let estimate = &x_sq_approx + BigInt::from(-p) + BigInt::from(10);
        if estimate < BigInt::one() {
            BigInt::one()
        } else {
            estimate
        }
    };
    let magnitude_bits = to_prec(&((&x_sq_approx * BigInt::from(3)) / BigInt::from(2))) + 2;
    let guard_bits = (n_estimate.magnitude().bits() as Precision) + magnitude_bits + 4;
    let calc_precision = p - guard_bits;
    let op_prec = calc_precision - 8;
    let op_appr = op.approx_signal(signal, op_prec);
    let max_trunc_error = signed::ONE.deref() << (p - 4 - calc_precision);

    let mut n: i64 = 0;
    let mut current_term = scale(op_appr.clone(), op_prec - calc_precision);
    let mut current_sum = current_term.clone();
    loop {
        if should_stop(signal) {
            break;
        }
        let prev = current_term;
        let mut t = scale(&prev * &op_appr, op_prec);
        t = scale(t * &op_appr, op_prec);
        t = (t * signed::TWO.deref()) / BigInt::from(2 * n + 3);
        n += 1;
        if t.is_zero() {
            break;
        }

        let prev_abs = prev.abs();
        let cur_abs = t.abs();
        if cur_abs < prev_abs {
            let denom = &prev_abs - &cur_abs;
            if &cur_abs * &prev_abs < &max_trunc_error * &denom {
                break;
            }
        }
        current_sum += &t;
        current_term = t;
    }
    scale(current_sum, calc_precision - p)
}

/// Evaluate the exact retained complement 1-erf(x) at precision p for a defined finite x.
/// Delegate cancellation and error control to the expression evaluator. This is
/// subtraction of exact expressions, not fixed-precision point estimates or a
/// separate tail asymptotic; small tails may require substantial internal refinement.
fn erfc(signal: &Option<Signal>, op: &Computable, p: Precision) -> BigInt {
    Computable::one()
        .add(op.clone().erf().negate())
        .approx_signal(signal, p)
}

/// Evaluate the standard normal survival function through the exact identity SF(x)=Phi(-x).
/// For finite x this is positive. Delegate the request and signal to the retained
/// CDF expression, without imposing the public Real adapter's practical input window.
fn normal_sf(signal: &Option<Signal>, op: &Computable, p: Precision) -> BigInt {
    op.clone().negate().pnorm().approx_signal(signal, p)
}

/// Evaluate the signed CDF difference Phi(hi)-Phi(lo) at precision p.
/// Finite endpoints need not be ordered here: reversed endpoints give a negative
/// difference. Public Real admission owns interval validation. The two CDFs remain
/// exact expressions until subtraction/refinement; this kernel adds no tail-specific
/// formula, separate certainty receipt or resource budget.
fn normal_interval(
    signal: &Option<Signal>,
    lo: &Computable,
    hi: &Computable,
    p: Precision,
) -> BigInt {
    hi.clone()
        .pnorm()
        .add(lo.clone().pnorm().negate())
        .approx_signal(signal, p)
}

/// Evaluate ln(Phi(x)) by composing the exact normal CDF with the logarithm.
/// For defined finite x the CDF is strictly between zero and one. This is not a
/// special log-tail expansion; all refinement, cancellation and resource limitations
/// come from the child graph, and raw imported domains are not checked here.
fn log_pnorm(signal: &Option<Signal>, op: &Computable, p: Precision) -> BigInt {
    op.clone().pnorm().ln().approx_signal(signal, p)
}

/// Evaluate ln(SF(x)) through the retained positive survival probability and logarithm.
/// For defined finite x the result is negative. No rounded probability is taken as
/// an exact log input, but proving a tiny tail numerically can require extra work;
/// this wrapper supplies no independent budget or domain validation.
fn log_normal_sf(signal: &Option<Signal>, op: &Computable, p: Precision) -> BigInt {
    op.clone().normal_sf().ln().approx_signal(signal, p)
}

/// Evaluate the analytic log density -x^2/2-ln(2*pi)/2 for a defined finite x.
/// Avoid constructing an exponentially small density only to take its logarithm.
/// All terms remain exact graph definitions; the final query delegates precision
/// and cancellation to them, without a separate total-work or storage bound.
fn log_dnorm(signal: &Option<Signal>, op: &Computable, p: Precision) -> BigInt {
    let neg_half_x_sq = op.clone().square().shift_left(-1).negate();
    let log_sqrt_2pi = Computable::pi().shift_left(1).ln().shift_left(-1);
    neg_half_x_sq
        .add(log_sqrt_2pi.negate())
        .approx_signal(signal, p)
}

/// Compare a dyadic trial with a valid interior normal quantile, or enclose it near the trial.
/// For working grid w, evaluate Phi((trial-2)*2^w), Phi((trial+2)*2^w), then
/// Phi(trial*2^w) only if needed. Each completed scalar sample has error at most
/// one unit; a difference greater than two therefore certifies strict CDF order.
/// Equal means the root is strictly inside the two-unit neighborhood, not that
/// the trial equals the root. Greater/Less certify the trial is above/below it.
///
/// Try the two endpoints first in the written order. The center separates a root
/// that equals an endpoint, avoiding reliance on an undecidable strict endpoint
/// comparison. A capped magnitude estimate chooses extra CDF guard bits only;
/// all decisions still require separated sampled enclosures. Refine by sixteen
/// bits with saturating subtraction, polling around child evaluations.
///
/// None reports an observed stop. It is not an undecidability or exhausted-budget
/// outcome: no iteration limit is imposed, and reaching MIN can repeat that same
/// precision indefinitely or encounter child arithmetic limits. Require a defined
/// probability strictly inside (0,1); this helper does not establish that domain.
fn normal_quantile_probe(
    signal: &Option<Signal>,
    p: &Computable,
    trial: &BigInt,
    working_precision: Precision,
) -> Option<std::cmp::Ordering> {
    use std::cmp::Ordering;

    let lower = Computable::integer(trial - 2)
        .shift_left(working_precision)
        .pnorm();
    let upper = Computable::integer(trial + 2)
        .shift_left(working_precision)
        .pnorm();
    let center = Computable::integer(trial.clone())
        .shift_left(working_precision)
        .pnorm();
    // This is a planning hint only. Small tail densities need extra CDF bits;
    // all decisions below still require disjoint numerical enclosures.
    let magnitude = scale(trial.abs(), working_precision)
        .to_u32()
        .unwrap_or(64)
        .min(64);
    let guard = ((magnitude + 2).pow(2) + 8) as Precision;
    let mut precision = working_precision.saturating_sub(guard).min(-16);
    loop {
        if should_stop(signal) {
            return None;
        }
        let probability = p.approx_signal(signal, precision);
        let lo = lower.approx_signal(signal, precision);
        let hi = upper.approx_signal(signal, precision);
        if should_stop(signal) {
            return None;
        }
        if &probability - &lo > BigInt::from(2) && &hi - &probability > BigInt::from(2) {
            return Some(Ordering::Equal);
        }
        if lo - &probability > BigInt::from(2) {
            return Some(Ordering::Greater);
        }
        if &probability - hi > BigInt::from(2) {
            return Some(Ordering::Less);
        }
        // If a bracket endpoint equals the root, its strict comparison cannot
        // terminate. The center is then separated from the root instead.
        let middle = center.approx_signal(signal, precision);
        if should_stop(signal) {
            return None;
        }
        if &middle - &probability > BigInt::from(2) {
            return Some(Ordering::Greater);
        }
        if probability - middle > BigInt::from(2) {
            return Some(Ordering::Less);
        }
        precision = precision.saturating_sub(16);
    }
}

/// Refine an inclusive dyadic bracket for a valid normal quantile on the output grid divided by four.
/// The caller supplies lower<=root<=upper in working-grid units and requires the
/// requested output precision to be working_precision+2. Probe integer midpoints;
/// certified orders replace the corresponding endpoint, while Equal permits
/// rounding trial/4 with at most one output unit of total error. Equal is an
/// interval witness, not scalar equality.
///
/// When width<=4, round (lower+upper)/8: half the bracket width plus rounding
/// is at most one output unit. An interrupted probe returns zero for the caller
/// to reject. The input bracket is trusted rather than validated, and nested CDF
/// work has no cumulative budget even though successful bisections reduce width.
fn normal_quantile_bisect(
    signal: &Option<Signal>,
    p: &Computable,
    mut lower: BigInt,
    mut upper: BigInt,
    working_precision: Precision,
) -> BigInt {
    use std::cmp::Ordering;

    while &upper - &lower > BigInt::from(4) {
        let trial = (&lower + &upper) >> 1;
        match normal_quantile_probe(signal, p, &trial, working_precision) {
            Some(Ordering::Less) => lower = trial,
            Some(Ordering::Greater) => upper = trial,
            Some(Ordering::Equal) => return scale(trial, -2),
            None => return BigInt::zero(),
        }
    }
    // The root lies in [lower, upper], whose width is at most one result ulp.
    // Round its exact midpoint: half the width plus half an ulp is <= one ulp.
    scale(lower + upper, -3)
}

/// Approximate the unique inverse of Phi for a defined probability in (0,1), with an untrusted seed.
/// Poll the stop first, then recognize exact half-probability or a retained rational
/// CDF inverse. For positive output precision, rescale the precision-zero result.
/// Otherwise work two bits finer. Below -16, recursively obtain a coarser certified
/// approximation, whose plus/minus-one bounds supply an inclusive bracket. One
/// Newton proposal uses the exact CDF residual divided by density; accept it only
/// inside that bracket and with a fresh Equal probe, otherwise bisect the bracket.
///
/// For precisions -16..=0, reject seeds whose exponent-plus-bit-count exceeds five
/// before scaling, and use zero for a zero/rejected seed. This is a planning guard,
/// not a magnitude restriction on the quantile. If the initial neighborhood is
/// unproved, expand lower/upper endpoints from -1/+1 until strict CDF orders supply
/// a bracket, then bisect. The seed or Newton convergence alone never certifies
/// an output, and the retained probability remains the authoritative definition.
///
/// Stop paths produce placeholders for the caller to reject. Neither probability
/// admission nor a cumulative work/heap-byte budget is supplied here. Precision
/// arithmetic is unchecked: for an unrecognized quantile, requesting MIN overflows
/// prec-2 in debug builds before refinement. Child limits and saturating probe
/// refinement likewise preclude a universal completion/resource guarantee.
fn normal_quantile(
    signal: &Option<Signal>,
    p: &Computable,
    seed: &BigInt,
    seed_prec: Precision,
    prec: Precision,
) -> BigInt {
    use std::cmp::Ordering;

    if should_stop(signal) {
        return BigInt::zero();
    }
    if let Some(exact) = p.exact_normal_quantile() {
        return Computable::rational(exact).approx_signal(signal, prec);
    }
    if prec > 0 {
        return scale(normal_quantile(signal, p, seed, seed_prec, 0), -prec);
    }
    let working_precision = prec - 2;
    if prec < -16 {
        let previous_precision = prec / 2 - 6;
        let previous = normal_quantile(signal, p, seed, seed_prec, previous_precision);
        if should_stop(signal) {
            return BigInt::zero();
        }
        let xn = Computable::integer(previous.clone()).shift_left(previous_precision);
        let residual = xn.clone().pnorm().add(p.clone().negate());
        let correction = residual.multiply(xn.clone().dnorm().inverse());
        let trial = xn
            .add(correction.negate())
            .approx_signal(signal, working_precision);
        let shift = previous_precision - working_precision;
        let lower = (&previous - 1) << shift;
        let upper = (&previous + 1) << shift;
        // A previous certified approximation supplies a bracket even if a
        // Newton proposal fails or leaves the interval. No convergence estimate
        // or fixed bound on the quantile's magnitude is used as a certificate.
        if trial >= lower
            && trial <= upper
            && normal_quantile_probe(signal, p, &trial, working_precision) == Some(Ordering::Equal)
        {
            return scale(trial, -2);
        }
        return normal_quantile_bisect(signal, p, lower, upper, working_precision);
    }

    // Avoid materializing a huge untrusted seed. This only selects a starting
    // point: every finite quantile remains reachable by the expanding search.
    let trial = if seed.is_zero()
        || i64::from(seed_prec) + i64::try_from(seed.bits()).unwrap_or(i64::MAX / 2) > 5
    {
        BigInt::zero()
    } else {
        scale(seed.clone(), seed_prec - working_precision)
    };
    if normal_quantile_probe(signal, p, &trial, working_precision) == Some(Ordering::Equal) {
        return scale(trial, -2);
    }

    let unit = BigInt::one() << working_precision.unsigned_abs();
    let mut lower = -&unit;
    let mut upper = unit;
    loop {
        match normal_quantile_probe(signal, p, &lower, working_precision) {
            Some(Ordering::Less) => break,
            Some(Ordering::Greater) => lower <<= 1,
            Some(Ordering::Equal) => return scale(lower, -2),
            None => return BigInt::zero(),
        }
    }
    loop {
        match normal_quantile_probe(signal, p, &upper, working_precision) {
            Some(Ordering::Greater) => break,
            Some(Ordering::Less) => upper <<= 1,
            Some(Ordering::Equal) => return scale(upper, -2),
            None => return BigInt::zero(),
        }
    }
    normal_quantile_bisect(signal, p, lower, upper, working_precision)
}
