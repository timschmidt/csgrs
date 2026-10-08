//! Bounded exact construction from integer-scaled root power sums.
//!
//! This constructs the full conjugate-pair carrier, including multiplicities.
//! It does not select a root, factor a carrier, or relax the caller's evidence.
use hyperreal::{Rational, Real};
use num::{BigInt, Integer, One, Signed, Zero};

use super::{AlgebraicRootArithmeticOp, MAX_BINARY_RESULTANT_DEGREE};

pub(super) fn binary_image(
    left: &[Real],
    right: &[Real],
    operation: AlgebraicRootArithmeticOp,
    degree: usize,
) -> Option<Vec<Real>> {
    let m = left.len().checked_sub(1)?;
    let n = right.len().checked_sub(1)?;
    if degree == 0 || degree > MAX_BINARY_RESULTANT_DEGREE || m.checked_mul(n)? != degree {
        return None;
    }
    let integers = |p: &[Real]| {
        p.iter()
            .map(|c| c.exact_rational_ref()?.to_big_integer())
            .collect::<Option<Vec<_>>>()
    };
    let a = integers(left)?;
    let mut b = integers(right)?;
    if a[m].is_zero() || b[n].is_zero() {
        return None;
    }
    // The represented divisor can be nonzero even if its reducible carrier
    // has another zero root. Keep the existing resultant path in that case.
    if operation == AlgebraicRootArithmeticOp::Divide {
        if b[0].is_zero() {
            return None;
        }
        b.reverse();
    }
    if operation == AlgebraicRootArithmeticOp::Negate {
        return None;
    }

    // L*alpha is integral for every root alpha of an integer polynomial
    // with leading coefficient L, including nonmonic and repeated carriers.
    let mut sa = scaled_root_power_sums(&a, degree);
    let mut sb = if a == b {
        sa.clone()
    } else {
        scaled_root_power_sums(&b, degree)
    };
    let scale = &a[m] * &b[n];
    let mut image_sums = vec![BigInt::zero(); degree + 1];
    match operation {
        AlgebraicRootArithmeticOp::Add | AlgebraicRootArithmeticOp::Subtract => {
            // A=L*alpha, B=M*beta: (LM)*(alpha +/- beta)=M*A +/- L*B.
            let mut ap = BigInt::one();
            let mut bp = BigInt::one();
            for k in 0..=degree {
                sa[k] *= &ap;
                sb[k] *= &bp;
                if operation == AlgebraicRootArithmeticOp::Subtract && k % 2 == 1 {
                    sb[k] = -std::mem::take(&mut sb[k]);
                }
                ap *= &b[n];
                bp *= &a[m];
            }
            for k in 1..=degree {
                let mut choose = 1_usize;
                for j in 0..=k {
                    image_sums[k] += &sa[k - j] * &sb[j] * choose;
                    if j < k {
                        choose = choose.checked_mul(k - j)? / (j + 1);
                    }
                }
            }
        }
        AlgebraicRootArithmeticOp::Multiply | AlgebraicRootArithmeticOp::Divide => {
            for k in 1..=degree {
                image_sums[k] = &sa[k] * &sb[k];
            }
        }
        AlgebraicRootArithmeticOp::Negate => return None,
    }

    // Newton reconstruction of the monic polynomial for the scaled images.
    let mut descending = vec![BigInt::one()];
    for k in 1..=degree {
        let mut sum = BigInt::zero();
        for j in 1..=k {
            sum += &descending[k - j] * &image_sums[j];
        }
        let (coefficient, remainder) = (-sum).div_rem(&BigInt::from(k));
        if !remainder.is_zero() {
            return None;
        }
        descending.push(coefficient);
    }
    descending.reverse();
    let mut power = BigInt::one();
    for coefficient in &mut descending {
        *coefficient *= &power;
        power *= &scale;
    }

    // Match the sampled resultant's signed primitive orientation, not just
    // its roots. Multiplication loses x-degree for zero roots of Q.
    let effective_n = if operation == AlgebraicRootArithmeticOp::Multiply {
        n - b.iter().take_while(|c| c.is_zero()).count()
    } else {
        n
    };
    let negative = (a[m].is_negative() && effective_n % 2 == 1)
        ^ (b[n].is_negative() && m % 2 == 1)
        ^ (operation == AlgebraicRootArithmeticOp::Subtract && degree % 2 == 1);
    if descending.last()?.is_negative() != negative {
        for coefficient in &mut descending {
            *coefficient = -std::mem::take(coefficient);
        }
    }
    let result = descending
        .into_iter()
        .map(Rational::from_bigint)
        .map(Real::from)
        .collect::<Vec<_>>();
    super::primitive_integer_polynomial(&result)
}

fn scaled_root_power_sums(p: &[BigInt], count: usize) -> Vec<BigInt> {
    let degree = p.len() - 1;
    let mut monic = vec![BigInt::one()];
    let mut power = BigInt::one();
    for i in 1..=degree {
        monic.push(&p[degree - i] * &power);
        power *= &p[degree];
    }
    let mut sums = vec![BigInt::from(degree)];
    for k in 1..=count {
        let mut sum = BigInt::zero();
        for i in 1..=degree.min(k - 1) {
            sum += &monic[i] * &sums[k - i];
        }
        if k <= degree {
            sum += &monic[k] * k;
        }
        sums.push(-sum);
    }
    sums
}

#[cfg(test)]
mod tests {
    use super::super::{
        primitive_integer_polynomial, resultant_polynomial_for_binary_image, sampled_binary_image,
    };
    use super::*;

    fn carriers() -> Vec<Vec<i64>> {
        vec![
            vec![-1, 1],
            vec![1, 2],
            vec![0, 1],
            vec![-2, 0, 1],
            vec![-3, 0, 1],
            vec![1, -2, 1],
            vec![0, -1, 1],
            vec![-2, 1, 1],
            vec![-2, 0, 0, 1],
            vec![-3, 0, 0, 2],
            vec![-1, 3, -3, 1],
            vec![0, -2, 0, 1],
            vec![2, -3, 0, 1],
            vec![1, 0, -10, 0, 1],
            vec![4, 0, -4, 0, 1],
            vec![0, 0, 1],
            vec![-2, 0, 0, 0, 0, 0, 0, 0, 0, 1],
            vec![-7, 0, 0, 0, 0, 0, 0, 1],
            vec![6, -5, 1],
            vec![1, 0, 1],
        ]
    }
    fn as_reals(p: &[i64], scale: i64, denominator: u64) -> Vec<Real> {
        p.iter()
            .map(|c| Real::from(Rational::fraction(c * scale, denominator).unwrap()))
            .collect()
    }
    fn wire(p: Option<Vec<Real>>) -> Option<Vec<String>> {
        p.map(|p| {
            p.iter()
                .map(|c| {
                    c.exact_rational_ref()
                        .unwrap()
                        .to_big_integer()
                        .unwrap()
                        .to_string()
                })
                .collect()
        })
    }
    #[test]
    fn integer_scaled_sums_match_known_roots() {
        let p = vec![BigInt::from(-3), BigInt::from(-1), BigInt::from(2)];
        // Roots 3/2,-1, so scaled roots are 3,-2.
        let sums = scaled_root_power_sums(&p, 9);
        for (k, s) in sums.iter().enumerate() {
            assert_eq!(
                *s,
                BigInt::from(3).pow(k as u32) + BigInt::from(-2).pow(k as u32)
            );
        }
    }
    #[test]
    fn unused_zero_divisor_root_uses_existing_fallback() {
        let a = as_reals(&[-2, 0, 1], 1, 1);
        let b = as_reals(&[0, -1, 1], 1, 1);
        assert!(binary_image(&a, &b, AlgebraicRootArithmeticOp::Divide, 4).is_none());
        assert_eq!(
            resultant_polynomial_for_binary_image(&a, &b, AlgebraicRootArithmeticOp::Divide, 4),
            sampled_binary_image(&a, &b, AlgebraicRootArithmeticOp::Divide, 4)
        );
    }
    #[test]
    fn audit_kernel_corpus() {
        let polys = carriers();
        let mut rows = 0;
        for (i, a) in polys.iter().enumerate() {
            for (j, b) in polys.iter().enumerate() {
                let degree = (a.len() - 1) * (b.len() - 1);
                if degree > 9 {
                    continue;
                }
                for (op, operation) in [
                    AlgebraicRootArithmeticOp::Add,
                    AlgebraicRootArithmeticOp::Subtract,
                    AlgebraicRootArithmeticOp::Multiply,
                    AlgebraicRootArithmeticOp::Divide,
                ]
                .into_iter()
                .enumerate()
                {
                    // Pure-zero divisors do not represent valid public division inputs.
                    if op == 3 && (j == 2 || j == 15) {
                        continue;
                    }
                    for (scale, (left_scale, right_scale)) in
                        [(1, 1), (-1, 1), (1, -1), (-1, -1)].into_iter().enumerate()
                    {
                        let left = as_reals(a, left_scale, 3);
                        let right = as_reals(b, right_scale, 5);
                        let primitive_left = primitive_integer_polynomial(&left).unwrap();
                        let primitive_right = primitive_integer_polynomial(&right).unwrap();
                        let candidate =
                            resultant_polynomial_for_binary_image(&left, &right, operation, degree);
                        let baseline = sampled_binary_image(
                            &primitive_left,
                            &primitive_right,
                            operation,
                            degree,
                        );
                        let direct =
                            binary_image(&primitive_left, &primitive_right, operation, degree);
                        println!(
                            "@POWER_SUMS {}",
                            serde_json::json!({"i":i,"j":j,"op":op,"scale":scale,
                        "left":a,"right":b,"candidate":wire(candidate),"baseline":wire(baseline),"direct":wire(direct)})
                        );
                        rows += 1;
                    }
                }
            }
        }
        println!(
            "@POWER_SUMS {}",
            serde_json::json!({"terminal":true,"rows":rows,"carriers":polys.len()})
        );
    }
}
