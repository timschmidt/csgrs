use super::*;
use num::{BigInt, BigRational, One, Zero};

fn real(q: BigRational) -> Real {
    Real::from(
        HyperRational::from_bigint_fraction(q.numer().clone(), q.denom().to_biguint().unwrap())
            .unwrap(),
    )
}

fn monomial_derivative(coefficients: &[BigRational], t: &BigRational, order: usize) -> BigRational {
    coefficients
        .iter()
        .enumerate()
        .skip(order)
        .fold(BigRational::zero(), |sum, (degree, c)| {
            let factor = (degree - order + 1..=degree).fold(BigInt::one(), |f, k| f * k);
            sum + c * BigRational::from_integer(factor) * t.pow((degree - order) as i32)
        })
}

#[test]
fn derivative_demand_exact_monomial_oracles() {
    let scales = [
        Real::one(),
        Real::pi(),
        Real::from(2).sqrt().unwrap(),
        Real::from(2).ln().unwrap(),
    ];
    for length in [0, 1, 2, 4, 9, 25] {
        for sparse in [false, true] {
            let coefficients: Vec<_> = (0..length)
                .map(|i| {
                    let n = if sparse && i % 3 != 1 {
                        0
                    } else {
                        (i as i64 * 7 % 11) - 5
                    };
                    BigRational::new(n.into(), (i + 1).into())
                })
                .collect();
            for scale in &scales {
                let exact: Vec<_> = coefficients
                    .iter()
                    .map(|c| real(c.clone()) * scale)
                    .collect();
                for t in [
                    BigRational::zero(),
                    BigRational::one(),
                    BigRational::new(1.into(), 2.into()),
                    BigRational::new(1.into(), 3.into()),
                ] {
                    for max_order in [0, 1, 2, 3, 8, 24, 80, 128] {
                        let values = evaluate_power_polynomial_derivatives(
                            &exact,
                            &real(t.clone()),
                            max_order,
                        )
                        .unwrap();
                        assert_eq!(values.len(), max_order + 1);
                        for (order, got) in values.iter().enumerate() {
                            let expected =
                                real(monomial_derivative(&coefficients, &t, order)) * scale;
                            assert_eq!(
                                got, &expected,
                                "length={length}, sparse={sparse}, t={t}, order={order}"
                            );
                        }
                        if t.is_zero() || t.is_one() {
                            assert_eq!(
                                evaluate_power_polynomial_endpoint_derivatives(
                                    &exact,
                                    t.is_one(),
                                    max_order
                                )
                                .unwrap(),
                                values
                            );
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn derivative_demand_preserves_all_rational_quotient_orders() {
    for scale in [Real::one(), Real::pi(), Real::from(2).sqrt().unwrap()] {
        let half_scale = (&scale / Real::from(2)).unwrap();
        let curve = RationalBezier2::try_new(
            vec![
                Point2::new(Real::zero(), scale.clone()),
                Point2::new(half_scale.clone(), half_scale),
            ],
            vec![Real::one(), Real::from(2)],
        )
        .unwrap();
        for t in [
            BigRational::zero(),
            BigRational::one(),
            BigRational::new(1.into(), 3.into()),
        ] {
            let parameter = real(t.clone());
            for max_order in [0, 1, 3, 24, 80, 128] {
                let Classification::Decided(values) =
                    curve.affine_derivative_values_at(&parameter, max_order, &CurveContext::STRICT)
                else {
                    panic!("rational quotient must be certified")
                };
                assert_eq!(values.len(), max_order + 1);
                let public = curve
                    .clone()
                    .derivatives_at(&parameter, max_order, &CurveContext::STRICT)
                    .unwrap();
                assert_eq!(public.len(), max_order);
                let mut factorial = BigInt::one();
                for (k, (x, y)) in values.iter().enumerate() {
                    if k > 0 {
                        factorial *= k;
                    }
                    let sign = if k % 2 == 0 { 1 } else { -1 };
                    let yq = BigRational::from_integer(&factorial * sign)
                        / (BigRational::one() + &t).pow((k + 1) as i32);
                    let xq = if k == 0 {
                        &t / (BigRational::one() + &t)
                    } else {
                        -&yq
                    };
                    assert_eq!(x, &(real(xq) * &scale), "x order {k}");
                    assert_eq!(y, &(real(yq) * &scale), "y order {k}");
                    if k > 0 {
                        assert_eq!(public[k - 1].dx(), x);
                        assert_eq!(public[k - 1].dy(), y);
                    }
                }
                if t.is_zero() || t.is_one() {
                    assert_eq!(
                        curve.endpoint_derivatives(t.is_one(), max_order, &CurveContext::STRICT),
                        Classification::Decided(values)
                    );
                }
            }
        }
    }
}

#[test]
fn derivative_demand_checked_capacity_and_exact_zero_tails() {
    for coefficients in [
        vec![],
        vec![Real::pi()],
        vec![Real::pi(), Real::zero(), Real::zero()],
    ] {
        assert!(
            evaluate_power_polynomial_derivatives(&coefficients, &Real::one(), usize::MAX)
                .is_none()
        );
        for at_end in [false, true] {
            assert!(
                evaluate_power_polynomial_endpoint_derivatives(&coefficients, at_end, usize::MAX)
                    .is_none()
            );
            let values =
                evaluate_power_polynomial_endpoint_derivatives(&coefficients, at_end, 128).unwrap();
            assert_eq!(values.len(), 129);
            assert!(values[1..].iter().all(|v| v == &Real::zero()));
        }
    }
}
