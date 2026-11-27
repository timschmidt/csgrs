// See docs/admission-inverse-isometric-latitude.md for the monotonicity,
// clipping and error proofs. All interval endpoints below are exact rationals.

/// Construct the exact forward isometric equation F(s,r) at rational s and positive rational r.
/// Here r is the squared polar/equatorial axis ratio and s = asinh(tan(phi)).
/// For r<1 subtract `sqrt(1-r)*atanh(sqrt(1-r)*tanh(s))`; for r>1 add
/// `sqrt(r-1)*atan(sqrt(r-1)*tanh(s))`. At r=1 the result is s.
///
/// Use odd symmetry for negative s and `(1-exp(-2s))/(1+exp(-2s))` for tanh
/// on the positive branch. The inverse solver restricts abs(s) to its current K,
/// avoiding exp of a potentially huge isometric input. Zero and spherical cases
/// return a rational directly. Positivity is a caller invariant, not checked here;
/// fresh expression construction attaches no supplied abort signal or work budget.
fn isometric_at(s: &Rational, r: &Rational) -> Computable {
    if s.sign() == Sign::NoSign || r.is_one() {
        return Computable::rational(s.clone());
    }
    if s.sign() == Sign::Minus {
        return isometric_at(&-s.clone(), r).negate();
    }
    let one = Computable::one();
    let decay = Computable::rational(-s.clone() * Rational::new(2)).exp();
    let tangent = one
        .clone()
        .add(decay.clone().negate())
        .multiply(one.add(decay).inverse());
    let oblate = r < &Rational::one();
    let eccentricity = Computable::rational(if oblate {
        Rational::one() - r
    } else {
        r - Rational::one()
    })
    .sqrt();
    let argument = eccentricity.clone().multiply(tangent);
    let correction = eccentricity.multiply(if oblate {
        argument.atanh()
    } else {
        argument.atan()
    });
    Computable::rational(s.clone()).add(if oblate {
        correction.negate()
    } else {
        correction
    })
}

/// Approximate the unique geodetic latitude defined by finite psi and finite log(b/a).
/// The stored logarithm gives positive `r = exp(2*log(b/a))`. Work in
/// `s = asinh(tan(phi))`, where the forward derivative lies between min(1,r)
/// and max(1,r), and transport rational r bounds in reversed order for negative s.
/// Narrow residual enclosures until their width is at most one eighth of the
/// positive derivative lower bound times the target root width; signed interval
/// division and certified residual signs then shrink the root bracket.
///
/// For p<=0, enclose the root clipped to `[-K,K]`, K=4-p, within width
/// `2^(p-4)`. Outward dyadic rounding at p-8 controls repeated denominator
/// growth. Clipping changes the final latitude by less than one eighth of a
/// `2^p` unit; the latitude map is 1-Lipschitz. Evaluate its midpoint via
/// `sign(s)*(pi/2-2*atan(exp(-abs(s))))` at p-3, then round to p. The exact
/// node keeps the original equation; no clipped midpoint replaces its value.
/// These error statements require completed child enclosures and valid inputs.
///
/// For p>=1 return zero, valid since abs(latitude)<pi/2. Stops also return zero,
/// so the caller must distinguish interruption before publishing an approximation.
/// Root/guard/refinement precision subtraction can panic on overflow; exact
/// arithmetic, child evaluations and repeated refinement have no total-work or
/// heap-byte bound. The stop flag is polled between iterations and passed to
/// sampling, not to every intermediate rational operation or construction.
fn inverse_isometric_latitude(
    signal: &Option<Signal>,
    psi: &Computable,
    log_axis_ratio: &Computable,
    p: Precision,
) -> BigInt {
    if should_stop(signal) || p >= 1 {
        return BigInt::zero();
    }
    crate::trace_dispatch!(
        "computable",
        "inverse_isometric_latitude",
        "interval-refinement"
    );
    let root_precision = p.checked_sub(4).expect("root precision fits i32");
    let k = Rational::from_bigint(BigInt::from(4_i64 - i64::from(p)));
    let width = interval_dyadic(BigInt::one(), root_precision);
    let mut lower = -k.clone();
    let mut upper = k.clone();
    let ratio_squared = log_axis_ratio.clone().shift_left(1).exp();
    let mut precision = root_precision
        .checked_sub(16)
        .expect("guard precision fits i32");
    while &upper - &lower > width {
        if should_stop(signal) {
            return BigInt::zero();
        }
        let midpoint = (&lower + &upper) * HALF_RATIONAL.deref();
        let (residual, minimum, maximum) = loop {
            if should_stop(signal) {
                return BigInt::zero();
            }
            let [rlo, rhi] = interval_of(&ratio_squared, signal, precision);
            if rlo.sign() == Sign::Plus {
                let minimum = interval_min(Rational::one(), rlo.clone());
                let maximum = interval_max(Rational::one(), rhi.clone());
                let [ylo, yhi] = interval_of(psi, signal, precision);
                let [first, second] = if midpoint.sign() == Sign::Minus {
                    [rhi, rlo]
                } else {
                    [rlo, rhi]
                };
                let flo =
                    interval_of(&isometric_at(&midpoint, &first), signal, precision)[0].clone();
                let fhi =
                    interval_of(&isometric_at(&midpoint, &second), signal, precision)[1].clone();
                let residual = [flo - yhi, fhi - ylo];
                if &residual[1] - &residual[0]
                    <= &minimum * &width * Rational::fraction(1, 8).unwrap()
                {
                    break (residual, minimum, maximum);
                }
            }
            precision = precision
                .checked_sub(32)
                .expect("refinement precision fits i32");
        };
        if should_stop(signal) {
            return BigInt::zero();
        }
        // F(mid) / F'(xi) = mid - root, with 0 < minimum <= F' <= maximum.
        let quotient_lo = residual[0].clone()
            / if residual[0].sign() == Sign::Minus {
                &minimum
            } else {
                &maximum
            };
        let quotient_hi = residual[1].clone()
            / if residual[1].sign() == Sign::Minus {
                &maximum
            } else {
                &minimum
            };
        // Outward dyadic rounding bounds denominator growth before the next
        // midpoint. This is enclosure arithmetic, not a rounded root value.
        let grid = root_precision - 4;
        let next_lower = interval_dyadic(ratio(&(&midpoint - quotient_hi), grid) - 1, grid);
        let next_upper = interval_dyadic(ratio(&(&midpoint - quotient_lo), grid) + 1, grid);
        lower = interval_max(lower, interval_clamp(next_lower, &k));
        upper = interval_min(upper, interval_clamp(next_upper, &k));
        if residual[0].sign() == Sign::Plus {
            upper = interval_min(upper, midpoint.clone());
        } else if residual[1].sign() == Sign::Minus {
            lower = interval_max(lower, midpoint);
        }
        debug_assert!(lower <= upper, "certified root intervals intersect");
    }
    let midpoint = (lower + upper) * HALF_RATIONAL.deref();
    let negative = midpoint.sign() == Sign::Minus;
    let magnitude = if negative { -midpoint } else { midpoint };
    let complement = Computable::rational(-magnitude).exp().atan().shift_left(1);
    let latitude = Computable::pi().shift_left(-1).add(complement.negate());
    let result = scale(latitude.approx_signal(signal, p - 3), -3);
    if negative { -result } else { result }
}

#[cfg(test)]
/// Direct kernel sentinels for coarse precision and an already asserted stop flag.
/// These cases return before tracing/refinement; they do not measure interruption
/// latency during an active solve or certify resource bounds.
mod isometric_precision_tests {
    use super::*;

    #[test]
    /// Require zero at i32::MAX precision and on an already stopped -512 request.
    /// Use finite psi=1 and ln(3/5) axis data. The assertions check the returned
    /// placeholders; source control flow, rather than a trace-count assertion,
    /// establishes that these requests return before refinement.
    fn coarse_precision_and_preexisting_abort_never_refine() {
        let psi = Computable::one();
        let log_axis = Computable::rational(Rational::fraction(3, 5).unwrap()).ln();
        assert_eq!(
            inverse_isometric_latitude(&None, &psi, &log_axis, i32::MAX),
            BigInt::zero()
        );
        let signal = Some(std::sync::Arc::new(std::sync::atomic::AtomicBool::new(
            true,
        )));
        assert_eq!(
            inverse_isometric_latitude(&signal, &psi, &log_axis, -512),
            BigInt::zero()
        );
    }
}
