use super::*;

const ROOTS: [i64; 8] = [2, 3, 5, 7, 11, 13, 17, 19];

fn coefficient(width: usize) -> RadicalCoefficient {
    let mut result = RadicalCoefficient::rational(Rational::new(29));
    for (index, root) in ROOTS[..width].iter().enumerate() {
        result
            .insert_radical(
                Rational::new(*root),
                Rational::new(i64::try_from(index + 1).unwrap()),
            )
            .unwrap();
    }
    result
}

fn components(value: &RadicalCoefficient) -> (Rational, Vec<(Rational, Rational)>) {
    assert!(value.atoms.is_empty());
    (value.rational.clone(), value.radicals.to_vec())
}

#[test]
fn closed_views_isolate_scaling_cancellation_and_frontier_failures() {
    for width in [1, 2, 8] {
        let source = PiLaurentPolynomial::monomial(0, coefficient(width));
        let expected = components(&source.terms[0].1);
        let closed = ClosedPiLaurentPolynomial::from_polynomial(&source).unwrap();
        let first = closed.to_polynomial();
        let second = closed.to_polynomial();
        if width == 1 {
            let shared = source.terms[0].1.radicals.as_ptr() == closed.terms[0].1.radicals.as_ptr()
                && first.terms[0].1.radicals.as_ptr() == second.terms[0].1.radicals.as_ptr();
            println!("RADICAL_STORAGE_SHARED\t{}", u8::from(shared));
        }
        let scale = Rational::fraction(-3, 11).unwrap();
        let scaled = first.scaled(scale.clone());
        assert_eq!(scaled.terms[0].1.rational, &expected.0 * &scale);
        assert_eq!(
            scaled.terms[0].1.radicals.as_slice(),
            expected
                .1
                .iter()
                .map(|(root, amount)| (root.clone(), amount * &scale))
                .collect::<Vec<_>>()
        );
        let mut cancelled = second;
        cancelled.terms[0]
            .1
            .insert_radical(Rational::new(2), Rational::new(-1))
            .unwrap();
        assert_eq!(cancelled.terms[0].1.radicals.as_slice(), &expected.1[1..]);
        let mut frontier = closed.to_polynomial();
        if width == 8 {
            assert!(
                frontier.terms[0]
                    .1
                    .insert_radical(Rational::new(23), Rational::new(1))
                    .is_none()
            );
            assert_eq!(components(&frontier.terms[0].1), expected);
        }
        assert_eq!(components(&source.terms[0].1), expected);
        assert_eq!(components(&closed.to_polynomial().terms[0].1), expected);
        assert!(
            closed
                .to_polynomial()
                .scaled(Rational::zero())
                .terms
                .is_empty()
        );
        assert_eq!(components(&closed.to_polynomial().terms[0].1), expected);
    }
}

#[test]
fn consuming_cached_products_and_inverses_preserves_live_certificates() {
    let mut value = RadicalCoefficient::radical(Rational::new(2), Rational::new(13));
    value.rational = Rational::new(3);
    let source = PiLaurentPolynomial::monomial(0, value);
    let expected = components(&source.terms[0].1);
    let closed = ClosedPiLaurentPolynomial::from_polynomial(&source).unwrap();
    let product = closed
        .to_polynomial()
        .multiply(closed.to_polynomial())
        .unwrap();
    assert_eq!(product.terms[0].1.rational, Rational::new(61));
    assert_eq!(
        product.terms[0].1.radicals.as_slice(),
        [(Rational::new(13), Rational::new(12))].as_slice()
    );
    let inverse = closed.to_polynomial().inverse_monomial().unwrap();
    assert_eq!(
        inverse.terms[0].1.rational,
        Rational::fraction(-3, 43).unwrap()
    );
    assert_eq!(
        inverse.terms[0].1.radicals.as_slice(),
        [(Rational::new(13), Rational::fraction(2, 43).unwrap())].as_slice()
    );
    let identity = inverse.multiply(closed.to_polynomial()).unwrap();
    assert_eq!(identity.terms[0].1.rational, Rational::one());
    assert!(identity.terms[0].1.radicals.is_empty());
    assert_eq!(components(&source.terms[0].1), expected);
    assert_eq!(components(&closed.to_polynomial().terms[0].1), expected);
}

#[test]
fn concurrent_cached_views_mutate_independently_and_outlive_the_graph() {
    let value = Computable {
        internal: Arc::new(Node::new(
            Approximation::Add(
                Computable::rational(Rational::new(3)),
                Computable::sqrt_rational(Rational::new(13)),
            ),
            BoundCache::Invalid,
            ExactSignCache::Unknown,
        )),
        signal: None,
    };
    let weak = Arc::downgrade(&value.internal);
    let clone = value.clone();
    let mut work = 512;
    let mut reused = false;
    let original = value
        .pi_laurent_polynomial_retained::<true>(64, &mut work, &mut None, &mut reused)
        .unwrap();
    let expected = (Rational::new(3), vec![(Rational::new(13), Rational::one())]);
    assert_eq!(components(&original.terms[0].1), expected);
    std::thread::scope(|scope| {
        for index in 1..=8 {
            let value = &value;
            let expected = &expected;
            scope.spawn(move || {
                for _ in 0..8 {
                    let mut work = 0;
                    let mut reused = false;
                    let view = value
                        .pi_laurent_polynomial_retained::<true>(
                            0,
                            &mut work,
                            &mut None,
                            &mut reused,
                        )
                        .unwrap();
                    assert!(reused);
                    assert_eq!(work, 0);
                    assert_eq!(components(&view.terms[0].1), *expected);
                    let changed = view.scaled(Rational::new(-index));
                    assert_eq!(changed.terms[0].1.rational, Rational::new(-3 * index));
                    assert_eq!(changed.terms[0].1.radicals[0].1, Rational::new(-index));
                }
            });
        }
    });
    assert_eq!(components(&original.terms[0].1), expected);
    assert!(value.cached().is_none());
    let view = value.internal.cache.pi_laurent_polynomial().unwrap();
    drop(clone);
    drop(value);
    assert!(weak.upgrade().is_none());
    assert_eq!(components(&original.terms[0].1), expected);
    assert_eq!(components(&view.terms[0].1), expected);
    let negated = view.scaled(Rational::new(-1));
    assert_eq!(negated.terms[0].1.rational, Rational::new(-3));
    assert_eq!(negated.terms[0].1.radicals[0].1, Rational::new(-1));
    assert_eq!(components(&original.terms[0].1), expected);
}
