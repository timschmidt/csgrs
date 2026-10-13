use super::*;

const ROOTS: [i64; 9] = [2, 3, 5, 7, 11, 13, 17, 19, 23];

fn empty() -> PiLaurentPolynomial {
    PiLaurentPolynomial::monomial(0, RadicalCoefficient::rational(Rational::zero()))
}

fn raw(approximation: Approximation) -> Computable {
    Computable {
        internal: Arc::new(Node::new(
            approximation,
            BoundCache::Invalid,
            ExactSignCache::Unknown,
        )),
        signal: None,
    }
}

fn atom(index: i64) -> Computable {
    raw(Approximation::AtanRational(
        Rational::fraction(index + 1, 37).unwrap(),
    ))
}

fn components(value: &RadicalCoefficient) -> (Rational, Vec<(Rational, Rational)>) {
    assert!(value.atoms.is_empty());
    (value.rational.clone(), value.radicals.to_vec())
}

#[test]
fn radical_frontiers_cancel_and_regrow_in_order() {
    let mut value = RadicalCoefficient::rational(Rational::zero());
    for (index, root) in ROOTS[..8].iter().enumerate() {
        value
            .insert_radical(
                Rational::new(*root),
                Rational::new(i64::try_from(index + 1).unwrap()),
            )
            .unwrap();
        assert_eq!(value.radicals.len(), index + 1);
    }
    let before = components(&value);
    assert!(
        value
            .insert_radical(Rational::new(23), Rational::one())
            .is_none()
    );
    assert_eq!(components(&value), before);
    // Existing identities and zero entries remain allowed at the full frontier.
    value
        .insert_radical(Rational::new(23), Rational::zero())
        .unwrap();
    value
        .insert_radical(Rational::one(), Rational::new(41))
        .unwrap();
    for (index, root) in ROOTS[..8].iter().enumerate().rev() {
        value
            .insert_radical(
                Rational::new(*root),
                -Rational::new(i64::try_from(index + 1).unwrap()),
            )
            .unwrap();
        assert_eq!(value.radicals.len(), index);
    }
    assert_eq!(value.rational, Rational::new(41));
    for root in [23, 3, 2] {
        value
            .insert_radical(Rational::new(root), Rational::new(-7))
            .unwrap();
    }
    assert_eq!(
        value.radicals.as_slice(),
        [23, 3, 2]
            .map(|root| (Rational::new(root), Rational::new(-7)))
            .as_slice()
    );
}

#[test]
fn polynomial_frontiers_stay_sorted_after_cancel_and_regrow() {
    let mut value = empty();
    for power in [7, -4, 3, -2, 5, 0, 1, -6] {
        value
            .insert(power, RadicalCoefficient::rational(Rational::new(3)))
            .unwrap();
    }
    assert_eq!(
        value.terms.iter().map(|(p, _)| *p).collect::<Vec<_>>(),
        [-6, -4, -2, 0, 1, 3, 5, 7]
    );
    assert!(
        value
            .insert(9, RadicalCoefficient::rational(Rational::one()))
            .is_none()
    );
    assert_eq!(value.terms.len(), 8);
    value
        .insert(9, RadicalCoefficient::rational(Rational::zero()))
        .unwrap();
    for power in [0, 7, -6, 3, -2, 5, 1, -4] {
        value
            .insert(power, RadicalCoefficient::rational(Rational::new(-3)))
            .unwrap();
    }
    assert!(value.terms.is_empty());
    for power in [5, -5, 0] {
        value
            .insert(power, RadicalCoefficient::rational(Rational::new(11)))
            .unwrap();
    }
    assert_eq!(
        value.terms.iter().map(|(p, _)| *p).collect::<Vec<_>>(),
        [-5, 0, 5]
    );
}

#[test]
fn closed_roundtrips_preserve_scaled_single_and_spilled_forms() {
    for width in [0, 1, 2, 8] {
        for radical_width in [0, 1, 2, 8] {
            let mut value = empty();
            for index in 0..width {
                let mut coefficient = RadicalCoefficient::rational(Rational::new(41));
                for root in &ROOTS[..radical_width] {
                    coefficient
                        .insert_radical(Rational::new(*root), Rational::fraction(5, 7).unwrap())
                        .unwrap();
                }
                value.insert(index * 2 - 7, coefficient).unwrap();
            }
            for numerator in [0, -3, 2] {
                let scale = Rational::fraction(numerator, 11).unwrap();
                let scaled = value.clone().scaled(scale);
                let closed = ClosedPiLaurentPolynomial::from_polynomial(&scaled).unwrap();
                let restored = closed.to_polynomial();
                drop(closed);
                assert_eq!(
                    restored.terms.len(),
                    if numerator == 0 {
                        0
                    } else {
                        usize::try_from(width).unwrap()
                    }
                );
                for (index, (power, coefficient)) in restored.terms.iter().enumerate() {
                    assert_eq!(*power, i16::try_from(index).unwrap() * 2 - 7);
                    assert_eq!(
                        coefficient.rational,
                        Rational::fraction(41 * numerator, 11).unwrap()
                    );
                    assert_eq!(coefficient.radicals.len(), radical_width);
                    for ((root, amount), expected_root) in coefficient.radicals.iter().zip(ROOTS) {
                        assert_eq!(*root, Rational::new(expected_root));
                        assert_eq!(*amount, Rational::fraction(5 * numerator, 77).unwrap());
                    }
                    assert!(coefficient.atoms.is_empty());
                }
                // A second copy remains independent after the first is released.
                let again = ClosedPiLaurentPolynomial::from_polynomial(&restored).unwrap();
                drop(restored);
                assert_eq!(again.to_polynomial().terms.len(), scaled.terms.len());
            }
        }
    }
}

#[test]
fn radical_products_and_inverses_keep_exact_coefficients() {
    let left = RadicalCoefficient::rational(Rational::new(2))
        .add(RadicalCoefficient::radical(
            Rational::new(3),
            Rational::new(5),
        ))
        .unwrap();
    let right = RadicalCoefficient::rational(Rational::new(7))
        .add(RadicalCoefficient::radical(
            Rational::new(-2),
            Rational::new(5),
        ))
        .unwrap();
    assert_eq!(
        components(&left.multiply(&right).unwrap()),
        (
            Rational::new(-16),
            vec![(Rational::new(5), Rational::new(17))]
        )
    );
    let two_roots = RadicalCoefficient::radical(Rational::one(), Rational::new(2))
        .add(RadicalCoefficient::radical(
            Rational::one(),
            Rational::new(3),
        ))
        .unwrap();
    assert_eq!(
        components(&two_roots.clone().multiply(&two_roots).unwrap()),
        (Rational::new(5), vec![(Rational::new(6), Rational::new(2))])
    );
    assert!(two_roots.inverse().is_none());
    let unit = RadicalCoefficient::rational(Rational::new(2))
        .add(RadicalCoefficient::radical(
            Rational::one(),
            Rational::new(3),
        ))
        .unwrap();
    let inverse = unit.clone().inverse().unwrap();
    assert_eq!(
        components(&inverse),
        (
            Rational::new(2),
            vec![(Rational::new(3), Rational::new(-1))]
        )
    );
    assert_eq!(
        components(&unit.multiply(&inverse).unwrap()),
        (Rational::one(), vec![])
    );
    assert!(
        RadicalCoefficient::rational(Rational::zero())
            .inverse()
            .is_none()
    );
    assert!(RadicalCoefficient::atom(atom(0)).inverse().is_none());
    let opaque = RadicalCoefficient::atom(atom(1));
    assert!(opaque.clone().multiply(&opaque).is_none());
    assert!(
        opaque
            .multiply(&RadicalCoefficient::radical(
                Rational::one(),
                Rational::new(2)
            ))
            .is_none()
    );
}

#[test]
fn polynomial_products_cancel_cross_terms_and_keep_power_limits() {
    let make = |sign| {
        PiLaurentPolynomial::monomial(
            1,
            RadicalCoefficient::radical(Rational::one(), Rational::new(2)),
        )
        .add(PiLaurentPolynomial::monomial(
            0,
            RadicalCoefficient::radical(Rational::new(sign), Rational::new(3)),
        ))
        .unwrap()
    };
    let product = make(1).multiply(make(-1)).unwrap();
    assert_eq!(product.terms.len(), 2);
    assert_eq!(product.terms[0].0, 0);
    assert_eq!(components(&product.terms[0].1), (Rational::new(-3), vec![]));
    assert_eq!(product.terms[1].0, 2);
    assert_eq!(components(&product.terms[1].1), (Rational::new(2), vec![]));
    assert!(product.inverse_monomial().is_none());
    assert!(empty().inverse_monomial().is_none());
    let monomial =
        |power| PiLaurentPolynomial::monomial(power, RadicalCoefficient::rational(Rational::one()));
    assert!(monomial(i16::MAX).multiply(monomial(1)).is_none());
    assert!(monomial(i16::MIN).multiply(monomial(-1)).is_none());
    assert!(monomial(i16::MIN).inverse_monomial().is_none());
    assert_eq!(monomial(-7).inverse_monomial().unwrap().terms[0].0, 7);
}

#[test]
fn opaque_owners_are_released_on_clear_cancellation_and_decline() {
    for width in [1, 2, 8] {
        let mut value = empty();
        let mut weak = Vec::new();
        for index in 0..width {
            let owner = atom(index);
            weak.push(Arc::downgrade(&owner.internal));
            value
                .insert(
                    i16::try_from(index).unwrap(),
                    RadicalCoefficient::atom(owner),
                )
                .unwrap();
        }
        assert!(ClosedPiLaurentPolynomial::from_polynomial(&value).is_none());
        let clone = value.clone();
        assert!(weak.iter().all(|owner| owner.strong_count() == 2));
        let cleared = clone.scaled(Rational::zero());
        assert!(cleared.terms.is_empty());
        assert!(weak.iter().all(|owner| owner.strong_count() == 1));
        drop(cleared);
        if width == 8 {
            let extra = atom(17);
            let extra_weak = Arc::downgrade(&extra.internal);
            assert!(value.insert(9, RadicalCoefficient::atom(extra)).is_none());
            assert!(extra_weak.upgrade().is_none());
        }
        for index in (0..width).rev() {
            let mut opposite = value.terms[usize::try_from(index).unwrap()].1.clone();
            opposite.scale_in_place(&Rational::new(-1));
            value
                .insert(i16::try_from(index).unwrap(), opposite)
                .unwrap();
            assert!(weak[usize::try_from(index).unwrap()].upgrade().is_none());
        }
        assert!(value.terms.is_empty());
        assert!(weak.iter().all(|owner| owner.upgrade().is_none()));
    }
    let owner = atom(19);
    let weak = Arc::downgrade(&owner.internal);
    let left = PiLaurentPolynomial::monomial(i16::MAX, RadicalCoefficient::atom(owner));
    let right = PiLaurentPolynomial::monomial(1, RadicalCoefficient::rational(Rational::one()));
    assert!(left.multiply(right).is_none());
    assert!(weak.upgrade().is_none());
}

#[test]
fn opaque_cancellation_keeps_cold_warm_and_cloned_quadratic_facts() {
    let opaque = atom(5);
    let root = Computable::sqrt_rational(Rational::new(13));
    let sum = raw(Approximation::Add(
        raw(Approximation::Add(Computable::pi(), root)),
        opaque.clone(),
    ));
    let expression = raw(Approximation::Add(
        raw(Approximation::Add(
            sum.clone(),
            raw(Approximation::Negate(Computable::pi())),
        )),
        raw(Approximation::Negate(opaque)),
    ));
    let clone = expression.clone();
    let expected = Some((Rational::zero(), Rational::one(), Some(Rational::new(13))));
    for query in [&expression, &clone, &expression] {
        assert_eq!(query.exact_quadratic_surd_parts(), expected);
        assert!(sum.exact_quadratic_surd_parts().is_none());
        assert!(query.cached().is_none());
    }
}

#[test]
fn temporary_and_persistent_layouts_are_reported_separately() {
    println!(
        "TEMPORARY_LAURENT_LAYOUT\t{}\t{}\t{}\t{}\t{}\t{}\t{}",
        std::mem::size_of::<Rational>(),
        std::mem::size_of::<RadicalCoefficient>(),
        std::mem::size_of::<PiLaurentPolynomial>(),
        std::mem::size_of::<ClosedRadicalCoefficient>(),
        std::mem::size_of::<ClosedPiLaurentPolynomial>(),
        std::mem::size_of::<Computable>(),
        std::mem::size_of::<Node>(),
    );
    // The persistent representation still owns only closed rational data.
    let closed = ClosedPiLaurentPolynomial::from_polynomial(&empty()).unwrap();
    let _: Vec<(i16, ClosedRadicalCoefficient)> = closed.terms;
}
