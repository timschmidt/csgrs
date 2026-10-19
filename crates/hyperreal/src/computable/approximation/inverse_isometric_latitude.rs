// See docs/admission-inverse-isometric-latitude.md for the monotonicity,
// clipping and error proofs. All interval endpoints below are exact rationals.

fn isometric_dyadic(value: BigInt, p: Precision) -> Rational {
    if p < 0 {
        Rational::from_bigint_fraction(value, BigUint::one() << p.unsigned_abs() as usize)
            .expect("dyadic denominator is nonzero")
    } else {
        Rational::from_bigint(value << p as usize)
    }
}

fn isometric_interval(c: &Computable, signal: &Option<Signal>, p: Precision) -> [Rational; 2] {
    if let Some(exact) = c.exact_rational() {
        return [exact.clone(), exact];
    }
    let a = c.approx_signal(signal, p);
    [isometric_dyadic(&a - 1, p), isometric_dyadic(a + 1, p)]
}

fn isometric_min(a: Rational, b: Rational) -> Rational {
    if a < b { a } else { b }
}

fn isometric_max(a: Rational, b: Rational) -> Rational {
    if a > b { a } else { b }
}

fn isometric_clamp(a: Rational, k: &Rational) -> Rational {
    isometric_max(-k.clone(), isometric_min(k.clone(), a))
}

// F(s,r), for exact rational s and positive rational r. The root algorithm
// limits |s| to K before calling this, avoiding enormous exp(psi) intermediates.
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
    let width = isometric_dyadic(BigInt::one(), root_precision);
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
            let [rlo, rhi] = isometric_interval(&ratio_squared, signal, precision);
            if rlo.sign() == Sign::Plus {
                let minimum = isometric_min(Rational::one(), rlo.clone());
                let maximum = isometric_max(Rational::one(), rhi.clone());
                let [ylo, yhi] = isometric_interval(psi, signal, precision);
                let [first, second] = if midpoint.sign() == Sign::Minus {
                    [rhi, rlo]
                } else {
                    [rlo, rhi]
                };
                let flo = isometric_interval(&isometric_at(&midpoint, &first), signal, precision)
                    [0]
                .clone();
                let fhi = isometric_interval(&isometric_at(&midpoint, &second), signal, precision)
                    [1]
                .clone();
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
        let next_lower = isometric_dyadic(ratio(&(&midpoint - quotient_hi), grid) - 1, grid);
        let next_upper = isometric_dyadic(ratio(&(&midpoint - quotient_lo), grid) + 1, grid);
        lower = isometric_max(lower, isometric_clamp(next_lower, &k));
        upper = isometric_min(upper, isometric_clamp(next_upper, &k));
        if residual[0].sign() == Sign::Plus {
            upper = isometric_min(upper, midpoint.clone());
        } else if residual[1].sign() == Sign::Minus {
            lower = isometric_max(lower, midpoint);
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
mod isometric_precision_tests {
    use super::*;

    #[test]
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
