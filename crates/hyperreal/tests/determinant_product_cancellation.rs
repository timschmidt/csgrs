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
