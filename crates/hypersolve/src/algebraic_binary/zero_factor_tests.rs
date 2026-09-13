use super::*;
use crate::SymbolId;

fn carrier(roots: &[i64], scale: i64) -> Vec<Real> {
    let mut p = vec![Real::from(scale)];
    for &root in roots {
        let mut next = vec![Real::zero(); p.len() + 1];
        for (i, c) in p.iter().enumerate() {
            next[i] = next[i].clone() - c * Real::from(root);
            next[i + 1] = next[i + 1].clone() + c;
        }
        p = next;
    }
    p
}

fn point(polynomial_coefficients: Vec<Real>, value: i64) -> AlgebraicRootRepresentation {
    AlgebraicRootRepresentation {
        constraint_index: 7,
        symbol: SymbolId(11),
        interval_index: 3,
        polynomial_coefficients,
        interval: IsolatedRootInterval {
            lower: Real::from(value),
            upper: Real::from(value),
            exact_root: Some(Real::from(value)),
            distinct_root_count: 1,
        },
        validation: AlgebraicRootValidationReport {
            status: AlgebraicRootValidationStatus::Valid,
            message: None,
        },
    }
}

fn divide(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
    policy: PredicatePolicy,
) -> AlgebraicRootBinaryTransformReport {
    let before = (left.clone(), right.clone());
    let report =
        transform_algebraic_roots_binary(left, right, AlgebraicRootArithmeticOp::Divide, policy);
    assert_eq!(before, (left.clone(), right.clone()));
    if let Some(root) = &report.representation {
        assert_eq!(
            validate_algebraic_root_representation(root, PredicatePolicy::STRICT).status,
            AlgebraicRootValidationStatus::Valid
        );
        assert_eq!(root.constraint_index, left.constraint_index);
        assert_eq!(root.symbol, left.symbol);
        assert_eq!(root.interval_index, left.interval_index);
    }
    report
}

#[test]
fn division_recovers_unused_common_zero_roots() {
    for left_roots in [&[0][..], &[0, 0], &[0, 1], &[0, -2]] {
        for right_roots in [&[0, 1][..], &[0, 0, -2], &[0, 1, 1]] {
            for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
                let left = point(carrier(left_roots, -3), left_roots[0]);
                let right = point(carrier(right_roots, -2), *right_roots.last().unwrap());
                let report = divide(&left, &right, policy);
                assert_eq!(
                    report.status,
                    AlgebraicRootBinaryTransformStatus::Transformed
                );
                assert_eq!(
                    report.representation.unwrap().interval.exact_root,
                    Some(Real::zero())
                );
            }
        }
    }
    let left = point(carrier(&[0, 2], 1), 2);
    let right = point(carrier(&[0, -2], 1), -2);
    let report = divide(&left, &right, PredicatePolicy::STRICT);
    assert_eq!(
        report.status,
        AlgebraicRootBinaryTransformStatus::Transformed
    );
    assert_eq!(
        report.representation.unwrap().interval.exact_root,
        Some(Real::from(-1))
    );
}

#[test]
fn division_preserves_signed_nonzero_resultants() {
    for m in 1..=3 {
        for k in 1..=2 {
            for a in [-2, 2] {
                for left_scale in [-3, 2] {
                    for right_scale in [-2, 3] {
                        let mut left_roots = vec![a];
                        left_roots.extend([3, -4].into_iter().take(m - 1));
                        let mut right_roots = vec![0; k];
                        right_roots.push(-1);
                        let left = point(carrier(&left_roots, left_scale), a);
                        let right = point(carrier(&right_roots, right_scale), -1);
                        let expected = resultant_polynomial_for_binary_image(
                            &left.polynomial_coefficients,
                            &right.polynomial_coefficients,
                            AlgebraicRootArithmeticOp::Divide,
                            m * (k + 1),
                        )
                        .unwrap();
                        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
                            let report = divide(&left, &right, policy);
                            assert_eq!(
                                report.status,
                                AlgebraicRootBinaryTransformStatus::Transformed
                            );
                            assert_eq!(
                                report.representation.unwrap().polynomial_coefficients,
                                expected
                            );
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn division_deflates_after_shared_oversized_square_free_reduction() {
    let mut roots = vec![0];
    roots.extend([1; 8]);
    for scale in [-5, 3] {
        let source = point(carrier(&roots, scale), 1);
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let report = divide(&source, &source, policy);
            assert_eq!(
                report.status,
                AlgebraicRootBinaryTransformStatus::Transformed
            );
            let root = report.representation.unwrap();
            assert_eq!(
                root.polynomial_coefficients,
                carrier(&[0, 1], scale.signum())
            );
            assert_eq!(root.interval.exact_root, Some(Real::one()));
        }
    }
}

#[test]
fn division_admits_only_the_reduced_degree_bound() {
    let right = point(carrier(&[0, 3, 4, 5], 1), 3);
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let admitted = divide(&point(carrier(&[0, 1, 2], 1), 1), &right, policy);
        assert_eq!(
            admitted.status,
            AlgebraicRootBinaryTransformStatus::Transformed
        );
        assert_eq!(
            admitted
                .representation
                .unwrap()
                .polynomial_coefficients
                .len(),
            10
        );
        let refused = divide(&point(carrier(&[0, 1, 2, 6], 1), 1), &right, policy);
        assert_eq!(
            refused.status,
            AlgebraicRootBinaryTransformStatus::UnsupportedDegree
        );
        assert!(refused.representation.is_none());
    }
}

#[test]
fn division_keeps_zero_interval_and_invalid_payload_guards() {
    let left = point(carrier(&[0], 1), 0);
    let mut right = point(carrier(&[0, 1], 1), 1);
    right.interval.lower = Real::zero();
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        assert_eq!(
            divide(&left, &right, policy).status,
            AlgebraicRootBinaryTransformStatus::DenominatorMayContainZero
        );
        assert_eq!(
            divide(&left, &point(carrier(&[0, 0], 1), 0), policy).status,
            AlgebraicRootBinaryTransformStatus::DenominatorMayContainZero
        );
    }
    right = point(carrier(&[0, 1], 1), 1);
    right.polynomial_coefficients.push(Real::zero());
    assert_eq!(
        divide(&left, &right, PredicatePolicy::STRICT).status,
        AlgebraicRootBinaryTransformStatus::InvalidEvidence
    );
    right = point(carrier(&[0, 1], 1), 2);
    assert_eq!(
        divide(&left, &right, PredicatePolicy::STRICT).status,
        AlgebraicRootBinaryTransformStatus::InvalidEvidence
    );
}

#[test]
fn divisor_deflation_borrows_or_reuses_owned_storage() {
    let coefficients = carrier(&[0, 0, -2], 1);
    let interval = point(coefficients.clone(), -2).interval;
    let mut borrowed = Cow::Borrowed(coefficients.as_slice());
    assert_eq!(
        remove_certified_divisor_zero_factors(&mut borrowed, &interval),
        2
    );
    assert!(matches!(borrowed, Cow::Borrowed(_)));
    assert_eq!(borrowed.as_ptr(), coefficients[2..].as_ptr());
    let mut owned = coefficients.clone();
    owned.reserve(8);
    let (ptr, capacity) = (owned.as_ptr(), owned.capacity());
    let mut owned = Cow::Owned(owned);
    assert_eq!(
        remove_certified_divisor_zero_factors(&mut owned, &interval),
        2
    );
    let Cow::Owned(owned) = owned else {
        panic!("must reuse owned carrier");
    };
    assert_eq!(owned.as_ptr(), ptr);
    assert_eq!(owned.capacity(), capacity);
    assert_eq!(owned.as_slice(), &coefficients[2..]);
}

#[test]
fn divisor_deflation_requires_strict_nonzero_and_nonconstant_residual() {
    let coefficients = carrier(&[0, 1], 1);
    let mut interval = point(coefficients.clone(), 1).interval;
    interval.lower = crate::test_support::terminal_zero();
    interval.exact_root = None;
    assert_eq!(
        interval_contains_zero(&interval, PredicatePolicy::STRICT),
        None
    );
    let mut polynomial = Cow::Borrowed(coefficients.as_slice());
    assert_eq!(
        remove_certified_divisor_zero_factors(&mut polynomial, &interval),
        0
    );
    assert_eq!(polynomial.as_ref(), coefficients.as_slice());
    let nonzero = point(carrier(&[1], 1), 1).interval;
    for coefficients in [
        carrier(&[0, 0], 1),
        carrier(&[1], 1),
        vec![],
        vec![Real::zero()],
    ] {
        let mut polynomial = Cow::Borrowed(coefficients.as_slice());
        assert_eq!(
            remove_certified_divisor_zero_factors(&mut polynomial, &nonzero),
            0
        );
        assert_eq!(polynomial.as_ref(), coefficients.as_slice());
    }
}
