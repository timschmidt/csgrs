//! Observable enclosure contracts that a scalar-carrier migration must preserve.
use hyperinterval::RationalInterval;
use hyperreal::{Rational, Real, RealSign};
use hypersolve::RealInterval;
use hypersolve::real_interval::rational_interval_bernstein_strict_sign;

#[test]
fn straddling_products_are_conservative_but_not_a_drop_in_tight_hull() {
    let source = RealInterval {
        lower: Real::from(-1),
        upper: Real::from(1),
    };
    let tight_source = RationalInterval::new(Rational::new(-1), Rational::one()).unwrap();
    let product = source.multiply(&source).unwrap();
    let tight_product = tight_source.mul(&tight_source);
    assert_eq!(product.lower, Real::from(-2));
    assert_eq!(product.upper, Real::from(2));
    assert_eq!(
        tight_product.bounds(),
        Some((&Rational::new(-1), &Rational::one()))
    );
    println!("symbolic product=[-2,2]; tight rational product=[-1,1]");

    // Consume both generated results. Their difference survives the next operation.
    let square = product.square().unwrap();
    let tight_square = tight_product.square();
    assert_eq!(square.lower, Real::zero());
    assert_eq!(square.upper, Real::from(4));
    assert_eq!(
        tight_square.bounds(),
        Some((&Rational::zero(), &Rational::one()))
    );
    println!("generated square=[0,4]; tight rational generated square=[0,1]");
}

#[test]
fn ordered_rational_products_contain_independent_integer_corner_extrema() {
    let mut cases = 0;
    let mut wider = 0;
    for a in -3_i64..=3 {
        for b in a..=3 {
            for c in -3_i64..=3 {
                for d in c..=3 {
                    let left = RealInterval {
                        lower: a.into(),
                        upper: b.into(),
                    };
                    let right = RealInterval {
                        lower: c.into(),
                        upper: d.into(),
                    };
                    let result = left.multiply(&right).unwrap();
                    // Independent exact integer extrema, without either interval kernel.
                    let corners = [a * c, a * d, b * c, b * d];
                    let expected_lower = Rational::new(*corners.iter().min().unwrap());
                    let expected_upper = Rational::new(*corners.iter().max().unwrap());
                    let lower = result.lower.exact_rational_ref().unwrap();
                    let upper = result.upper.exact_rational_ref().unwrap();
                    assert!(lower <= &expected_lower && upper >= &expected_upper);
                    let tight = RationalInterval::new(a.into(), b.into())
                        .unwrap()
                        .mul(&RationalInterval::new(c.into(), d.into()).unwrap());
                    assert_eq!(tight.bounds(), Some((&expected_lower, &expected_upper)));
                    wider += usize::from(lower < &expected_lower || upper > &expected_upper);
                    cases += 1;
                }
            }
        }
    }
    assert_eq!(cases, 784);
    assert_eq!(wider, 81);
}

#[test]
fn nonrational_endpoints_retain_exact_symbolic_enclosures_through_composition() {
    let root = Real::from(2).sqrt().unwrap();
    assert!(root.exact_rational_ref().is_none());
    let source = RealInterval {
        lower: -root.clone(),
        upper: root.clone(),
    };
    let signs = RealInterval {
        lower: Real::from(-1),
        upper: Real::one(),
    };
    let product = source.multiply(&signs).unwrap();
    assert_eq!(product.lower, -Real::from(2) * &root);
    assert_eq!(product.upper, Real::from(2) * &root);
    let square = product.square().unwrap();
    assert_eq!(square.lower, Real::zero());
    assert_eq!(square.upper, Real::from(8));
    assert_eq!(square.strict_nonzero_sign(), None);
    let shifted = square.add(&RealInterval {
        lower: Real::one(),
        upper: Real::one(),
    });
    assert_eq!(shifted.strict_nonzero_sign(), Some(RealSign::Positive));
}

#[test]
fn quotient_domains_and_generated_bounds_remain_explicit() {
    let source = RealInterval {
        lower: Real::from(-1),
        upper: Real::one(),
    };
    let product = source.multiply(&source).unwrap();
    assert!(product.divide(&source).is_none());
    let denominator = RealInterval {
        lower: Real::from(2),
        upper: Real::from(4),
    };
    let quotient = product.divide(&denominator).unwrap();
    assert_eq!(quotient.lower, Real::from(-1));
    assert_eq!(quotient.upper, Real::one());
    let negative = RealInterval {
        lower: Real::from(-4),
        upper: Real::from(-2),
    };
    assert_eq!(product.divide(&negative).unwrap(), quotient);
    let zero_touch = RealInterval {
        lower: Real::zero(),
        upper: Real::one(),
    };
    assert!(product.divide(&zero_touch).is_none());
    assert!(RealInterval::from_values::<0>([]).is_none());
}

#[test]
fn bernstein_certification_is_a_polynomial_contract_not_controlwise_strict_sign() {
    // Degree-two Bernstein controls [1, 0, 1] yield 2(t-1/2)^2 + 1/2.
    // Requiring every control to have a strict sign would lose this certificate.
    let one = Rational::one();
    let zero = Rational::zero();
    assert_eq!(
        rational_interval_bernstein_strict_sign(vec![
            [one.clone(), one.clone()],
            [zero.clone(), zero.clone()],
            [one.clone(), one.clone()],
        ]),
        Some(RealSign::Positive)
    );
    // [1, -1/10, 1] has minimum 9/20. Mixed controls require subdivision.
    let minus_tenth = Rational::fraction(-1, 10).unwrap();
    let mixed = vec![
        [one.clone(), one.clone()],
        [minus_tenth.clone(), minus_tenth],
        [one.clone(), one.clone()],
    ];
    assert_eq!(
        rational_interval_bernstein_strict_sign(mixed.clone()),
        Some(RealSign::Positive)
    );
    let negative = mixed
        .into_iter()
        .map(|[lower, upper]| [-upper, -lower])
        .collect();
    assert_eq!(
        rational_interval_bernstein_strict_sign(negative),
        Some(RealSign::Negative)
    );
    // [1,-1,1] vanishes at 1/2, so a closed-domain strict sign is unavailable.
    let minus_one = -one.clone();
    assert_eq!(
        rational_interval_bernstein_strict_sign(vec![
            [one.clone(), one.clone()],
            [minus_one.clone(), minus_one],
            [one.clone(), one.clone()],
        ]),
        None
    );
    assert_eq!(
        rational_interval_bernstein_strict_sign(vec![[zero, one]]),
        None
    );
    assert_eq!(rational_interval_bernstein_strict_sign(vec![]), None);
}
