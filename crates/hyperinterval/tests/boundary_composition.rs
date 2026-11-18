//! Empty outputs remain operands, while invalid arguments retain their errors.

#[test]
fn public_errors_remain_distinct_and_keep_their_typed_source() {
    use core::error::Error;
    use hyperinterval::IntervalError as E;
    use std::collections::BTreeSet;

    let errors = [
        E::ReversedEndpoints,
        E::NonSingletonEndpoint,
        E::UnboundedImage,
        E::InvalidScale,
        E::Overflow,
        E::ContainsZero,
        E::EmptyDomain,
        E::Pole,
        E::UnsupportedBranch,
        E::PrecisionExhausted,
        E::WorkExhausted,
        E::UnqualifiedEnvironment,
        #[cfg(any(feature = "float-proposal", feature = "float-enclosure"))]
        E::NonFiniteEndpoint,
        #[cfg(feature = "rational")]
        E::NegativeRadius,
        #[cfg(feature = "rational")]
        E::NonPositiveLowerBound,
        #[cfg(feature = "hyperreal")]
        E::EnclosureUnavailable,
        #[cfg(all(feature = "rational", feature = "dyadic"))]
        E::NonDyadicEndpoint,
        #[cfg(feature = "dyadic")]
        E::SizeExhausted,
    ];
    let mut messages = BTreeSet::new();
    for error in errors {
        let message = error.to_string();
        assert!(!message.is_empty());
        assert!(
            messages.insert(message),
            "diagnostic collapsed for {error:?}"
        );
        assert!(error.source().is_none(), "invented scalar cause: {error:?}");
    }
    #[cfg(feature = "rational")]
    {
        let cause = hyperreal::Problem::DivideByZero;
        let error = E::Scalar(cause);
        assert_eq!(
            error.source().unwrap().downcast_ref::<hyperreal::Problem>(),
            Some(&cause)
        );
        assert!(error.to_string().contains(&cause.to_string()));
        assert!(messages.insert(error.to_string()));
    }
}

#[cfg(feature = "integer")]
#[test]
fn empty_integer_products_and_differences_compose_at_all_storage_widths() {
    use hyperinterval::IntegerRange as I;
    macro_rules! check {
        ($($t:ty),*) => { $( {
            let empty = I::<$t>::empty();
            let full = I::<$t>::new(<$t>::MIN, <$t>::MAX).unwrap();
            for (a, b) in [(&empty, &full), (&full, &empty), (&empty, &empty)] {
                let result = a.checked_mul(b).unwrap();
                assert_eq!(result.bounds(), None);
                assert_eq!(result.checked_add(&full).unwrap(), empty);
                assert_eq!(result.hull(&full), full);
                let difference = a.checked_sub(b).unwrap();
                assert_eq!(difference.bounds(), None);
                assert_eq!(difference.hull(&full), full);
            }
        } )* };
    }
    check!(
        i8, i16, i32, i64, i128, isize, u8, u16, u32, u64, u128, usize
    );
}

#[cfg(feature = "fixed")]
#[test]
fn empty_fixed_transforms_preserve_scale_and_denominator_validation() {
    use hyperinterval::{FixedInterval as F, IntervalError as E};
    macro_rules! check {
        ($($t:ty),*) => { $( {
            let empty = F::<$t, 0>::empty().unwrap();
            let reflected = empty.checked_neg().unwrap();
            assert_eq!(reflected.bounds(), None);
            let scaled = reflected.rescale::<1>().unwrap();
            assert_eq!(scaled.bounds(), None);
            assert_eq!(scaled.rescale::<0>().unwrap(), empty);
            assert_eq!(scaled.rescale::<{ <$t>::BITS }>(), Err(E::InvalidScale));
            for zero in [F::<$t, 1>::point(0).unwrap(), F::new(-1, 1).unwrap()] {
                assert_eq!(zero.reciprocal(), Err(E::ContainsZero));
                assert_eq!(scaled.checked_div(&zero), Err(E::ContainsZero));
            }
        } )* };
    }
    check!(i32, i64);
}

#[cfg(feature = "float-proposal")]
#[test]
fn empty_affine_proposals_still_validate_nonfinite_arguments() {
    use hyperinterval::{F32ProposalInterval, F64ProposalInterval, IntervalError as E};
    macro_rules! check {
        ($interval:ty, $float:ty) => {{
            let empty = <$interval>::empty();
            let transformed = empty.affine(-2.0, 3.0).unwrap();
            assert_eq!(transformed.estimates(), None);
            assert_eq!(transformed.affine(0.0, 1.0).unwrap(), empty);
            assert_eq!(
                transformed.affine(<$float>::NAN, 0.0),
                Err(E::NonFiniteEndpoint)
            );
            assert_eq!(
                transformed.affine(0.0, <$float>::INFINITY),
                Err(E::NonFiniteEndpoint)
            );
        }};
    }
    check!(F32ProposalInterval, f32);
    check!(F64ProposalInterval, f64);
}

#[cfg(feature = "dyadic")]
#[test]
fn split_and_extended_outputs_remain_valid_function_inputs() {
    use hyperinterval::{DomainCoverage, DyadicBudget, DyadicInterval as D};
    use hyperinterval::{ExtendedInterval as X, IntervalSet2 as S};

    let empty = S::<D>::empty();
    let mut no_work = DyadicBudget::new(0, 0);
    let reflected = empty.neg(&mut no_work).unwrap();
    assert_eq!(reflected, empty);
    let hull = reflected.hull(&mut no_work).unwrap();
    assert_eq!(hull, X::empty());
    assert!(hull.as_finite().is_none());
    for image in [hull.sin(-16, &mut no_work), hull.cos(-16, &mut no_work)] {
        let image = image.unwrap();
        assert_eq!(image.domain, DomainCoverage::Complete);
        assert_eq!(image.range, D::empty());
        assert_eq!(
            image.range.exp(-16, &mut no_work).unwrap().range,
            D::empty()
        );
    }

    let zero = D::point(0.into(), 0).unwrap();
    let one = D::point(1.into(), 0).unwrap();
    let mut budget = DyadicBudget::new(4096, 10_000_000);
    let point = S::from_interval(X::from_finite(zero.clone()));
    let hull = point.hull(&mut budget).unwrap();
    let sine = hull.sin(-16, &mut budget).unwrap();
    assert_eq!(sine.domain, DomainCoverage::Complete);
    assert_eq!(sine.range, zero);
    let cosine = X::from_finite(sine.range).cos(-16, &mut budget).unwrap();
    assert_eq!(cosine.domain, DomainCoverage::Complete);
    assert_eq!(cosine.range, one);
    assert!(
        point
            .neg(&mut budget)
            .unwrap()
            .contains_point(&0.into(), 0, &mut budget)
            .unwrap()
    );
}

#[cfg(feature = "float-enclosure")]
#[test]
fn strict_float_empty_affine_and_distinct_neighbors_compose() {
    use hyperinterval::{F32Enclosure, F64Enclosure, IntervalError as E};
    macro_rules! check {
        ($interval:ty, $float:ty) => {{
            let empty = <$interval>::empty();
            let transformed = empty.affine(-2.0, 3.0).unwrap();
            assert_eq!(transformed.bounds(), None);
            assert_eq!(transformed.affine(0.0, 1.0).unwrap(), empty);
            assert_eq!(
                transformed.affine(<$float>::NAN, 0.0),
                Err(E::NonFiniteEndpoint)
            );
            assert_eq!(
                transformed.affine(0.0, <$float>::INFINITY),
                Err(E::NonFiniteEndpoint)
            );
            let next = <$float>::from_bits((1.0 as $float).to_bits() + 1);
            let left = <$interval>::new(0.0, 1.0).unwrap();
            let right = <$interval>::new(next, 2.0).unwrap();
            assert!(left.disjoint(&right));
            assert!(right.disjoint(&left));
            assert_eq!(left.intersection(&right), empty);
            assert_eq!(left.intersection(&right).affine(1.0, 0.0).unwrap(), empty);
            let touching = <$interval>::new(1.0, 2.0).unwrap();
            assert!(!left.disjoint(&touching));
            assert!(!touching.disjoint(&left));
            assert_eq!(left.intersection(&touching).bounds(), Some((1.0, 1.0)));
        }};
    }
    check!(F32Enclosure, f32);
    check!(F64Enclosure, f64);
}

#[cfg(all(feature = "rational", feature = "dyadic"))]
#[test]
fn empty_rational_trig_preserves_complete_domain_without_spending_work() {
    use hyperinterval::{DomainCoverage, DyadicBudget, DyadicInterval, RationalInterval};
    let mut budget = DyadicBudget::new(0, 0);
    for precision in [i32::MIN, -32, 0, i32::MAX] {
        for image in [
            RationalInterval::empty().sin(precision, &mut budget),
            RationalInterval::empty().cos(precision, &mut budget),
        ] {
            let image = image.unwrap();
            assert_eq!(image.domain, DomainCoverage::Complete);
            assert_eq!(image.range, DyadicInterval::empty());
        }
        let tangent = RationalInterval::empty()
            .tan(precision, &mut budget)
            .unwrap();
        assert!(tangent.range.is_empty());
        assert_eq!(tangent.domain, DomainCoverage::Complete);
    }
}
