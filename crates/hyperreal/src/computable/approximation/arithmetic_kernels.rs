/// Approximate `1/c` in units of `2^p`, assuming an admitted nonzero child.
///
/// Certified magnitude facts, or magnitude refinement when unavailable, size the
/// integer dividend and divisor. A divisor sample that rounds to zero triggers
/// eight more bits of refinement. The final magnitude quotient rounds to nearest
/// with ties away from zero before restoring the divisor's sign; this does not
/// promise a correctly rounded reciprocal of the exact child.
///
/// This is not domain validation: a child certified as zero currently returns
/// zero rather than a typed error. A stop observed after a zero divisor sample
/// also returns zero as an abort placeholder, not a certified approximation.
/// The caller must handle cancellation before publishing or caching a result.
/// Magnitude queries use the child's attached signal; samples use `signal`.
///
/// Precision arithmetic is unchecked. The 16,384 scale guard panics only after
/// a failed divisor sample is refined, so it is neither a general precision cap
/// nor a total work or allocation bound. A nonzero first sample bypasses it.
fn inverse(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // A raw planning hint may overestimate a near-cancelled denominator by an
    // arbitrary amount. Use an exact magnitude or refine before sizing the
    // reciprocal's numerator and denominator approximations.
    let (sign, planned_msd) = c.certified_sign_and_msd();
    if sign == Some(Sign::NoSign) {
        return Zero::zero();
    }
    let msd = planned_msd.flatten().unwrap_or_else(|| c.iter_msd());
    let inv_msd = 1 - msd;
    let digits_needed = inv_msd - p + 3;
    let mut prec_needed = msd - digits_needed;
    let mut log_scale_factor = -p - prec_needed;

    let scaled_divisor = loop {
        if log_scale_factor < 0 {
            return Zero::zero();
        }

        let scaled = c.approx_signal(signal, prec_needed);
        if !scaled.is_zero() {
            break scaled;
        }

        if should_stop(signal) {
            return Zero::zero();
        }

        // `iter_msd` is deliberately cheap and may overestimate a value whose
        // leading bits come from cancellation, such as a nested sqrt/log
        // reduction. Refine instead of dividing by a rounded-zero denominator.
        prec_needed -= 8;
        log_scale_factor += 8;
        if log_scale_factor > 16_384 {
            panic!("ArithmeticException");
        }
    };

    let dividend = signed::ONE.deref() << log_scale_factor;
    let abs_scaled_divisor = scaled_divisor.abs();
    let adj_dividend = dividend + (&abs_scaled_divisor >> 1);
    let result: BigInt = adj_dividend / abs_scaled_divisor;

    if scaled_divisor.sign() == Sign::Minus {
        -result
    } else {
        result
    }
}

/// Approximate `c1 + c2` in units of `2^p` using magnitude cutoffs before full addition.
///
/// Known zero forwards the other child. A cutoff of `p - 4` permits discarding
/// small contributions within the approximation error allowance; it does not
/// prove those contributions exactly zero. The certified same-sign branches
/// avoid treating possible cancellation as dominance. The fallback samples both
/// children at `p - 2`, sums their integers, and rounds with `scale`.
///
/// Read both certified facts left to right, then query missing magnitudes left
/// to right; retain that order for cache/proof reuse. Magnitude queries use each
/// child's attached signal; samples receive `signal`. There is no local stop
/// poll or total work bound,
/// and precision subtractions must fit their signed representation.
fn add(signal: &Option<Signal>, c1: &Computable, c2: &Computable, p: Precision) -> BigInt {
    // Addition first tries to bound an operand below the error allowance
    // at precision p. That dominates deep structural sums and avoids touching
    // tiny terms when signs/MSDs are already known.
    let extra = 4;
    let cutoff = p - extra;
    let (sign1, planning_msd1) = c1.certified_sign_and_msd();
    let (sign2, planning_msd2) = c2.certified_sign_and_msd();
    if sign1 == Some(Sign::NoSign) {
        return c2.approx_signal(signal, p);
    }
    if sign2 == Some(Sign::NoSign) {
        return c1.approx_signal(signal, p);
    }
    let msd1 = planning_msd1.flatten().or_else(|| c1.msd(cutoff));
    let msd2 = planning_msd2.flatten().or_else(|| c2.msd(cutoff));

    match (msd1, msd2) {
        (None, None) => return Zero::zero(),
        (None, Some(_)) if sign2.is_some_and(|sign| sign != Sign::NoSign) => {
            return scale(c2.approx_signal(signal, p - extra), -extra);
        }
        (Some(_), None) if sign1.is_some_and(|sign| sign != Sign::NoSign) => {
            return scale(c1.approx_signal(signal, p - extra), -extra);
        }
        (Some(left), Some(_right))
            if sign1 == sign2
                && sign2.is_some_and(|sign| sign != Sign::NoSign)
                && left < cutoff =>
        {
            return scale(c2.approx_signal(signal, p - extra), -extra);
        }
        (Some(_left), Some(right))
            if sign1 == sign2
                && sign1.is_some_and(|sign| sign != Sign::NoSign)
                && right < cutoff =>
        {
            return scale(c1.approx_signal(signal, p - extra), -extra);
        }
        _ => (),
    }

    scale(
        c1.approx_signal(signal, p - 2) + c2.approx_signal(signal, p - 2),
        -2,
    )
}

/// Return the leading-bit exponent of the dyadic sample `appr * 2^prec`.
///
/// For nonzero `appr`, this is `prec + bits(abs(appr)) - 1`, independent of
/// sign. It describes the sample, not necessarily the exact child's binade.
/// Callers supply nonzero samples: zero has no leading bit, though this helper
/// would mechanically compute `prec - 1` when representable.
///
/// The shared wide arithmetic checks the final exponent and panics if it does
/// not fit `Precision`; it never silently wraps that exponent.
fn msd_from_appr(prec: Precision, appr: &BigInt) -> Precision {
    crate::verified::word::checked_bit_length_msd(appr.magnitude().bits(), 1, false, prec)
        .expect("Approximation magnitude is outside the precision range")
}

/// Approximate `known * other` using an established magnitude anchor for `known`.
///
/// Sample `other` first at `p - known_msd - 3`; its nonzero sample's leading bit
/// then determines the precision needed from `known`. This asymmetric order
/// avoids first evaluating the expensive known-size child more finely than the
/// other operand requires. Rescale the integer product to units of `2^p`.
/// A zero sample returns zero at this demand, not an exact-zero certificate.
///
/// The anchor must come from the caller's certified/magnitude-query path, not
/// an arbitrary structural hint. Child samples carry `signal`; this helper adds
/// no stop poll or work limit, and its precision arithmetic must fit.
fn multiply_with_known_msd(
    signal: &Option<Signal>,
    known: &Computable,
    known_msd: Precision,
    other: &Computable,
    p: Precision,
) -> BigInt {
    // Evaluate the unknown-size operand first, then request only the precision
    // actually needed from the known-size operand. This asymmetric planning is
    // cheaper for products of exact scales and expensive transcendental nodes.
    let prec_other = p - known_msd - 3;
    let appr_other = other.approx_signal(signal, prec_other);

    if appr_other.sign() == Sign::NoSign {
        return Zero::zero();
    }

    let msd_other = msd_from_appr(prec_other, &appr_other);
    let prec_known = p - msd_other - 3;
    let appr_known = known.approx_signal(signal, prec_known);

    let scale_digits = prec_known + prec_other - p;
    scale(appr_known * appr_other, scale_digits)
}

/// Choose a magnitude anchor and approximate the product in units of `2^p`.
///
/// Inspect certified facts in operand order, then query missing magnitudes at
/// `(p >> 1) - 1` (floor division by two before subtracting one). A certified
/// zero short-circuits; two unresolved magnitudes at this cutoff permit a zero
/// approximation. With one known magnitude use it as the anchor; with two use
/// the larger, preferring the left on a tie. The asymmetric helper evaluates
/// the other operand first. These cutoffs are not exact-zero classifications.
///
/// Magnitude queries use the children's attached signals and samples receive
/// `signal`. Domain admission precedes these shortcuts. There is no local stop
/// poll, and fixed guard bits do not bound total work or precision arithmetic.
fn multiply(signal: &Option<Signal>, c1: &Computable, c2: &Computable, p: Precision) -> BigInt {
    // Prefer the operand with known larger magnitude as the precision anchor.
    // If neither magnitude is resolved at the planning cutoff, return a zero
    // approximation without evaluating both sides deeply.
    let half_prec = (p >> 1) - 1;
    let (sign1, msd1) = c1.certified_sign_and_msd();
    let (sign2, msd2) = c2.certified_sign_and_msd();
    if sign1 == Some(Sign::NoSign) || sign2 == Some(Sign::NoSign) {
        return Zero::zero();
    }
    let msd1 = msd1.flatten().or_else(|| c1.msd(half_prec));
    let msd2 = msd2.flatten().or_else(|| c2.msd(half_prec));

    match (msd1, msd2) {
        (None, None) => Zero::zero(),
        (Some(msd_op1), None) => multiply_with_known_msd(signal, c1, msd_op1, c2, p),
        (None, Some(msd_op2)) => multiply_with_known_msd(signal, c2, msd_op2, c1, p),
        (Some(msd_op1), Some(msd_op2)) if msd_op2 > msd_op1 => {
            multiply_with_known_msd(signal, c2, msd_op2, c1, p)
        }
        (Some(msd_op1), Some(_msd_op2)) => multiply_with_known_msd(signal, c1, msd_op1, c2, p),
    }
}

/// Approximate the fixed three-term sum of computable coefficients times rational values.
///
/// Evaluate indices zero, one, and two in that order at `p - 4`; each weighted
/// term requests eight additional coefficient guard bits relative to its exact
/// rational magnitude. Round the aggregate back by four bits. Truncation of
/// the three rational products is thus accounted for before the final rounding,
/// rather than building and rounding a separate general expression per term.
///
/// Nonzero rational magnitudes and intermediate precision arithmetic must fit
/// `i32`. The helper currently treats an unavailable rational magnitude as zero,
/// including an out-of-range exponent; that case is not a valid zero proof.
/// Zero weights skip their coefficients, so this kernel does not validate hidden
/// domain errors. Child evaluation receives `signal`; rational integer work has
/// no local cancellation poll, coefficient-size cap, or total work budget.
fn linear_combination3(signal: &Option<Signal>, form: &LinearCombination3, p: Precision) -> BigInt {
    /// Approximate one computable coefficient times an exact rational weight at precision `p`.
    ///
    /// Use the weight's binary magnitude to request eight coefficient guard bits,
    /// then multiply by its signed numerator and divide by its positive denominator
    /// with the required binary shift. Integer division truncates toward zero.
    /// An unavailable magnitude returns zero without evaluating the coefficient:
    /// this covers exact zero but also the unsupported out-of-range exponent case.
    /// The caller must ensure a representable magnitude for every nonzero weight.
    fn weighted(
        signal: &Option<Signal>,
        coefficient: &Computable,
        value: &Rational,
        p: Precision,
    ) -> BigInt {
        let Some(msd) = value.msd_exact() else {
            return Zero::zero();
        };
        // Eight coefficient guard bits leave ample room for the rational
        // scale and the truncating integer division below.
        let coefficient_precision = p - msd - 8;
        let approximation = coefficient.approx_signal(signal, coefficient_precision);
        let signed_numerator = BigInt::from_biguint(value.sign(), value.numerator().clone());
        let mut numerator = approximation * signed_numerator;
        let mut denominator = value.denominator().clone();
        let shift = coefficient_precision - p;
        if shift >= 0 {
            numerator <<= shift as usize;
        } else {
            denominator <<= shift.unsigned_abs() as usize;
        }
        numerator / BigInt::from(denominator)
    }

    // Compute each term four bits below the target. Even after the three
    // truncating rational scales, the final division by sixteen keeps the
    // aggregate absolute error below one target unit.
    let guard = 4;
    let precision = p - guard;
    let sum = weighted(signal, &form.coefficients[0], &form.values[0], precision)
        + weighted(signal, &form.coefficients[1], &form.values[1], precision)
        + weighted(signal, &form.coefficients[2], &form.values[2], precision);
    scale(sum, -guard)
}

/// Approximate `c^2` from one child sample rather than two product operands.
///
/// A certified zero sign short-circuits, but an inexact structural magnitude
/// hint is not used to choose working precision. Query the child's magnitude
/// at `(p >> 1) - 1`, sample at `p - msd - 3`, and rescale the integer square.
/// An absent magnitude or zero sample yields a zero approximation at this demand,
/// not an exact-zero proof. Querying the magnitude avoids compounding a binade
/// error across repeated squares.
///
/// Magnitude refinement uses the child's attached signal; the sample receives
/// `signal`. This kernel has no separate stop poll or work limit, and the signed
/// precision arithmetic must fit.
fn square(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Square can reuse one approximation of the child. Constructors create this
    // node for repeated powers so multiplication does not duplicate child work.
    let half_prec = (p >> 1) - 1;
    let (sign, _) = c.planning_sign_and_msd();
    if sign == Some(Sign::NoSign) {
        return Zero::zero();
    }
    // A square's structural MSD estimate can be one binade low, and repeatedly
    // doubling that estimate compounds the error in power chains. Ask for the
    // child's certified/cached MSD here; exact bounds remain allocation-free,
    // while inexact square chains refine before choosing a working precision.
    let msd = match c.msd(half_prec) {
        None => {
            return Zero::zero();
        }
        Some(msd) => msd,
    };
    let prec = p - msd - 3;

    let appr = c.approx_signal(signal, prec);

    if appr.sign() == Sign::NoSign {
        return Zero::zero();
    }

    let scale_digits = prec + prec - p;
    scale(&appr * &appr, scale_digits)
}

/// Approximate the exact rational `r` in units of `2^p` by integer division.
///
/// For negative `p`, shift the numerator by `-p` and truncate the quotient toward
/// zero. For nonnegative `p`, truncate to an integer first and then use `scale`
/// to round to the coarser binary grid, with ties toward positive infinity.
/// These stages satisfy the approximation role rather than promising a single
/// correctly rounded division of the original rational.
///
/// Negating `i32::MIN` is unsupported by the current precision arithmetic.
/// Shifts and division have no cancellation signal or local allocation budget.
fn ratio(r: &Rational, p: Precision) -> BigInt {
    // Exact rationals approximate by shifting the numerator/denominator ratio
    // directly; dyadic rationals make this path especially cheap.
    if p >= 0 {
        scale(r.shifted_big_integer(0), -p)
    } else {
        r.shifted_big_integer(-p)
    }
}

/// Evaluate the retained binary scale `c * 2^n` at precision `p` by requesting `c` at `p - n`.
///
/// This transports the precision without materializing a power-of-two factor.
/// The subtraction must fit `Precision`; the helper adds no checked failure,
/// stop poll, or resource limit to the child's evaluation with `signal`.
fn offset(signal: &Option<Signal>, c: &Computable, n: i32, p: Precision) -> BigInt {
    // x * 2^n at precision p is just x at precision p-n. This is why dyadic
    // scales are represented as Offset nodes instead of generic multiplication.
    c.approx_signal(signal, p - n)
}

/// Estimate the guard-bit count `ceil(log2(abs(n) + 1))` for series accumulation.
///
/// This uses binary64 logarithms only to plan guard precision; it does not
/// replace an authoritative scalar value with a float. It is not an exact
/// integer logarithm: rounding can add a guard bit, for example at `2^29 - 1`
/// on the audited host. The formula alone supplies no all-platform certification
/// of the floating-point bound.
///
/// Require `abs(n) + 1` to fit `i32`. Both `i32::MIN` and either sign of
/// `i32::MAX` violate that requirement; checked arithmetic builds panic.
fn bound_log2(n: i32) -> i32 {
    let abs_n = n.abs();
    let ln2 = 2.0_f64.ln();
    let n_plus_1: f64 = (abs_n + 1).into();
    let ans: f64 = (n_plus_1.ln() / ln2).ceil();
    ans as i32
}
