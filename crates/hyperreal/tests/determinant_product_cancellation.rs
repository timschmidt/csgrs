//! Opposite identical products are scalar identities, even for opaque factors.
use hyperreal::{CertifiedRealSign, Rational, Real, RealSign};

fn factors() -> [Real; 2] {
    let origin = Real::from(6) - Real::from(2).sqrt().unwrap();
    [1, 3].map(|n| {
        Real::from(6)
            - Real::from(2)
                * ((Real::pi() * Real::from(n)) / Real::from(8))
                    .unwrap()
                    .sin()
            - &origin
    })
}

fn reductions(a: &Real, b: &Real, c: &Real, d: &Real) -> [Real; 3] {
    [
        Real::diff_of_products(a, b, c, d),
        Real::signed_product_sum([true, false], [[a, b], [c, d]]),
        Real::active_signed_product_sum([true, false], [[a, b], [c, d]]),
    ]
}

#[test]
fn reversed_symbolic_products_cancel_before_refinement() {
    let [a, b] = factors();
    for value in reductions(&a, &b, &b, &a) {
        assert_eq!(value.exact_rational(), Some(Rational::zero()));
        assert!(matches!(
            value.certified_sign_until(0),
            CertifiedRealSign::Known {
                sign: RealSign::Zero,
                ..
            }
        ));
    }
}

#[test]
fn independently_reconstructed_factors_keep_the_identity() {
    let [a, b] = factors();
    let [c, d] = factors();
    for value in reductions(&a, &b, &d, &c) {
        assert_eq!(value.exact_rational(), Some(Rational::zero()));
    }
}

#[test]
fn equal_ordered_symbolic_products_keep_the_identity() {
    let [a, b] = factors();
    for value in reductions(&a, &b, &a, &b) {
        assert_eq!(value.exact_rational(), Some(Rational::zero()));
    }
}

#[test]
fn nearby_unequal_factors_do_not_cancel() {
    let a = Real::from(2) + (Real::pi() / Real::from(8)).unwrap().sin();
    let b = Real::from(3) + Real::e().sin();
    let shifted = &a + (Real::one() / Real::from(1_000_000_000_000_i64)).unwrap();
    // a*b - b*(a+delta) = -b*delta; b > 2 and delta > 0.
    for value in reductions(&a, &b, &b, &shifted) {
        assert!(matches!(
            value.certified_sign_until(-512),
            CertifiedRealSign::Known {
                sign: RealSign::Negative,
                ..
            }
        ));
    }
}

#[test]
fn equal_sign_products_are_added() {
    let a = Real::from(2) + (Real::pi() / Real::from(8)).unwrap().sin();
    let b = Real::from(3) + Real::e().sin();
    for positive in [true, false] {
        for value in [
            Real::signed_product_sum([positive; 2], [[&a, &b], [&b, &a]]),
            Real::active_signed_product_sum([positive; 2], [[&a, &b], [&b, &a]]),
        ] {
            let expected = if positive {
                RealSign::Positive
            } else {
                RealSign::Negative
            };
            assert!(
                matches!(value.certified_sign_until(-512), CertifiedRealSign::Known { sign, .. } if sign == expected)
            );
        }
    }
}

#[test]
fn reversed_sign_order_keeps_the_identity() {
    let [a, b] = factors();
    for value in [
        Real::signed_product_sum([false, true], [[&a, &b], [&b, &a]]),
        Real::active_signed_product_sum([false, true], [[&a, &b], [&b, &a]]),
    ] {
        assert_eq!(value.exact_rational(), Some(Rational::zero()));
    }
}

#[test]
fn refinement_and_concurrent_readers_keep_structural_cancellation() {
    let [a, b] = factors();
    for precision in [0, -1100, 0] {
        let _ = a.certified_sign_until(precision);
        let _ = b.certified_sign_until(precision);
        std::thread::scope(|scope| {
            for _ in 0..4 {
                scope.spawn(|| {
                    for value in reductions(&a, &b, &b, &a) {
                        assert_eq!(value.exact_rational(), Some(Rational::zero()));
                    }
                });
            }
        });
    }
}

#[test]
fn other_arities_keep_their_mathematical_values() {
    let a = Real::from(2) + (Real::pi() / Real::from(8)).unwrap().sin();
    let b = Real::from(3) + Real::e().sin();
    let one = Real::one();
    let two = Real::from(2);
    for value in [
        Real::signed_product_sum([true, false, true], [[&a, &b], [&b, &a], [&one, &one]]),
        Real::active_signed_product_sum([true, false, true], [[&a, &b], [&b, &a], [&one, &one]]),
        Real::signed_product_sum([false, true], [[&a, &b, &one], [&b, &a, &two]]),
        Real::active_signed_product_sum([false, true], [[&a, &b, &one], [&b, &a, &two]]),
    ] {
        assert!(matches!(
            value.certified_sign_until(-512),
            CertifiedRealSign::Known {
                sign: RealSign::Positive,
                ..
            }
        ));
    }
    assert_eq!(Real::signed_product_sum::<0, 2>([], []), Real::zero());
    assert_eq!(
        Real::active_signed_product_sum::<0, 2>([], []),
        Real::zero()
    );
    assert_eq!(Real::signed_product_sum::<2, 0>([true; 2], [[], []]), two);
    assert_eq!(
        Real::active_signed_product_sum::<2, 0>([true; 2], [[], []]),
        two
    );
}

#[test]
fn empty_product_identity_is_shared_by_all_rational_entry_points() {
    use std::cmp::Ordering;
    for (signs, expected, ordering) in [
        ([true, true, true], 3, Ordering::Greater),
        ([true, false, true], 1, Ordering::Greater),
        ([false, true, false], -1, Ordering::Less),
        ([false, false, false], -3, Ordering::Less),
    ] {
        let terms: [[&Rational; 0]; 3] = [[], [], []];
        assert_eq!(
            Rational::signed_product_sum(signs, terms),
            Rational::new(expected)
        );
        assert_eq!(
            Rational::signed_product_sum_shared_denominator(signs, terms),
            Some(Rational::new(expected))
        );
        assert_eq!(
            Rational::signed_product_sum_ordering(signs, terms),
            ordering
        );
        assert_eq!(
            Real::signed_product_sum(signs, [[], [], []]),
            Real::from(expected)
        );
        assert_eq!(
            Real::active_signed_product_sum(signs, [[], [], []]),
            Real::from(expected)
        );
    }
    assert_eq!(
        Rational::signed_product_sum::<0, 0>([], []),
        Rational::zero()
    );
    assert_eq!(
        Rational::signed_product_sum_shared_denominator::<0, 0>([], []),
        Some(Rational::zero())
    );
    assert_eq!(
        Rational::signed_product_sum_ordering::<0, 0>([], []),
        Ordering::Equal
    );
    assert_eq!(
        Rational::signed_product_sum_ordering::<2, 0>([true, false], [[], []]),
        Ordering::Equal
    );
}
