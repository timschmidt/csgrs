// Carlson duplication with outward dyadic rounding. Unlike a truncated
// correction polynomial, the stopping rule bounds the whole remaining
// integral by monotonicity. See docs/admission-elliptic-integrals.md.

// Floor and ceiling of numerator * 2^shift / denominator. The zero shortcut
// avoids allocating a huge denominator for a very coarse output request.
fn elliptic_quotient(numerator: BigUint, denominator: BigUint, shift: i64) -> [BigUint; 2] {
    debug_assert!(!denominator.is_zero());
    if numerator.is_zero() {
        return [BigUint::zero(), BigUint::zero()];
    }
    let (numerator, denominator) = if shift < 0 {
        if shift.unsigned_abs() >= numerator.bits() {
            return [BigUint::zero(), BigUint::one()];
        }
        (numerator, denominator << shift.unsigned_abs() as usize)
    } else {
        (numerator << shift as usize, denominator)
    };
    let (lower, remainder) = num::Integer::div_rem(&numerator, &denominator);
    let upper = &lower + BigUint::from(!remainder.is_zero() as u8);
    [lower, upper]
}

// A bound on the whole normalized integral, not a truncated correction.
// See docs/admission-carlson-mean-enclosure.md for the logarithm-series proof.
fn elliptic_rf_mean_enclosure(
    bounds: &[[BigUint; 2]; 3],
    q: Precision,
    output_grid: i64,
) -> Option<[BigUint; 2]> {
    let sum: BigUint = bounds.iter().map(|b| &b[0]).sum();
    let deviations = bounds.each_ref().map(|b| {
        let triple = &b[0] * 3_u8;
        if triple >= sum {
            triple - &sum
        } else {
            &sum - triple
        }
    });
    let largest = deviations.iter().max().unwrap();
    if largest >= &sum {
        return None;
    }
    let numerator: BigUint = deviations.iter().map(|d| d * d).sum();
    let denominator = (&sum * (&sum - largest)) << 2;
    if numerator >= denominator {
        return None;
    }
    let upper_sum: BigUint = bounds.iter().map(|b| &b[1]).sum();
    let root_lo = (&sum * 3_u8).sqrt();
    let root_hi = (upper_sum * 3_u8).sqrt() + BigUint::one();
    let shift = -i64::from(q) / 2 - output_grid;
    let lower = elliptic_quotient(BigUint::from(3_u8), root_hi, shift)[0].clone();
    let upper = elliptic_quotient(
        (&denominator * 5_u8 - &numerator * 4_u8) * 3_u8,
        root_lo * (&denominator - &numerator) * 5_u8,
        shift,
    )[1]
    .clone();
    Some([lower, upper])
}

fn elliptic_carlson(
    signal: &Option<Signal>,
    arguments: &[Computable; 3],
    second: bool,
    p: Precision,
) -> BigInt {
    if should_stop(signal) {
        return BigInt::zero();
    }
    crate::trace_dispatch!("computable", "elliptic_carlson", "interval-duplication");
    let values = [
        arguments[0].clone().square(),
        arguments[1].clone().exp(),
        arguments[2].clone().exp(),
    ];
    // Even negative grid precision keeps all square-root scale factors exact.
    // Restart with a finer grid if rounding obscures a denominator or stalls
    // the enclosure. A scheduling limit never substitutes for an error bound.
    // Integer square-root bounds lose half the grid's fractional bits. Start
    // with twice the requested accuracy plus guards, instead of repeatedly
    // exhausting grids that cannot possibly meet an ordinary fine request.
    // This is only a work schedule: every return still checks its enclosure.
    let mut q = p
        .min(0)
        .checked_sub(16)
        .and_then(|q| q.checked_mul(2))
        .expect("elliptic starting precision fits i32");
    loop {
        if should_stop(signal) {
            return BigInt::zero();
        }
        let mut bounds: [[BigUint; 2]; 3] = std::array::from_fn(|i| {
            let value = values[i].approx_signal(signal, q);
            [
                (&value - BigInt::one())
                    .max(BigInt::zero())
                    .to_biguint()
                    .unwrap(),
                (value + BigInt::one())
                    .max(BigInt::zero())
                    .to_biguint()
                    .unwrap(),
            ]
        });
        let largest_bits = bounds.iter().map(|b| b[1].bits()).max().unwrap();
        let iterations = largest_bits + u64::from(q.unsigned_abs()) + 64;
        let mut sum = [BigUint::zero(), BigUint::zero()];
        for iteration in 0..iterations {
            if should_stop(signal) {
                return BigInt::zero();
            }
            // RF's mean bound is costlier than one duplication step. Check
            // every fourth step; each check still bounds the whole integral.
            // RD keeps its existing per-step accumulated-sum checks.
            let enclosure = if !second && iteration % 4 != 0 {
                None
            } else {
                if second {
                    None
                } else {
                    elliptic_rf_mean_enclosure(&bounds, q, i64::from(p) - 3)
                }
                .or_else(|| {
                    let min = bounds.iter().map(|b| &b[0]).min().unwrap();
                    let max = bounds.iter().map(|b| &b[1]).max().unwrap();
                    let root_min = min.sqrt();
                    if root_min.is_zero() {
                        return None;
                    }
                    let root_max = max.sqrt() + BigUint::one();
                    let output_grid = i64::from(p) - 3;
                    let (shift, lower_denominator, upper_denominator) = if second {
                        (
                            -3 * i64::from(q) / 2 - 2 * iteration as i64 - output_grid,
                            max * &root_max,
                            min * &root_min,
                        )
                    } else {
                        (-i64::from(q) / 2 - output_grid, root_max, root_min)
                    };
                    let lower =
                        elliptic_quotient(BigUint::one(), lower_denominator, shift)[0].clone();
                    let upper =
                        elliptic_quotient(BigUint::one(), upper_denominator, shift)[1].clone();
                    let (lower, upper) = if second {
                        (
                            lower
                                + &elliptic_quotient(
                                    sum[0].clone(),
                                    BigUint::one(),
                                    i64::from(q) - output_grid,
                                )[0],
                            upper
                                + &elliptic_quotient(
                                    sum[1].clone(),
                                    BigUint::one(),
                                    i64::from(q) - output_grid,
                                )[1],
                        )
                    } else {
                        (lower, upper)
                    };
                    Some([lower, upper])
                })
            };
            if let Some([lower, upper]) = enclosure
                && &upper - &lower <= BigUint::from(2_u8)
            {
                // The enclosure width is <= 2^(p-2). Rounding its midpoint
                // adds at most 2^(p-1), strictly within one output ulp.
                return scale(BigInt::from(lower + upper), -4);
            }
            let lambda: [BigUint; 2] = std::array::from_fn(|side| {
                let mut total = BigUint::zero();
                for (a, b) in [(0, 1), (1, 2), (2, 0)] {
                    total += (&bounds[a][side] * &bounds[b][side]).sqrt();
                    if side == 1 {
                        total += BigUint::one();
                    }
                }
                total
            });
            if second {
                let root_lo = bounds[2][0].sqrt();
                if root_lo.is_zero() {
                    break;
                }
                let root_hi = bounds[2][1].sqrt() + BigUint::one();
                let shift = -5 * i64::from(q) / 2 - 2 * iteration as i64;
                sum[0] += &elliptic_quotient(
                    BigUint::from(3_u8),
                    root_hi * (&bounds[2][1] + &lambda[1]),
                    shift,
                )[0];
                sum[1] += &elliptic_quotient(
                    BigUint::from(3_u8),
                    root_lo * (&bounds[2][0] + &lambda[0]),
                    shift,
                )[1];
            }
            bounds = bounds.map(|[lo, hi]| {
                [
                    (lo + &lambda[0]) >> 2,
                    (hi + &lambda[1] + BigUint::from(3_u8)) >> 2,
                ]
            });
        }
        q = q
            .checked_mul(2)
            .expect("elliptic refinement precision fits i32");
    }
}
