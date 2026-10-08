#[cfg(test)]
mod finite_field_bernstein_regression {
    use super::*;

    fn decided<T>(value: Classification<T>) -> T {
        match value {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => {
                panic!("exact finite isolation declined: {reason:?}")
            }
        }
    }

    #[test]
    fn finite_field_isolation_reuses_original_polynomial_and_simple_root_proof() {
        let integers = [-3, -2, -1, 1, 2, 3];
        let coefficients = integers
            .iter()
            .fold(vec![Real::one()], |coefficients, root| {
                let mut product = vec![Real::zero(); coefficients.len() + 1];
                for (degree, coefficient) in coefficients.iter().enumerate() {
                    product[degree] -= coefficient * Real::from(*root);
                    product[degree + 1] += coefficient;
                }
                product
            });
        let extent = Real::from(10).sqrt().unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for scale in [Real::pi(), -Real::pi()] {
                let defining = decided(
                    BezierParameterPolynomial::try_new_power_basis(
                        coefficients.iter().map(|c| c * &scale).collect(),
                        &policy,
                    )
                    .unwrap(),
                );
                let result = decided(
                    isolate_roots_in_interval(
                        defining.coefficients().to_vec(),
                        &(-&extent),
                        &extent,
                        false,
                        BezierRootIsolationTrace2::default(),
                        &policy,
                    )
                    .unwrap(),
                );
                assert_eq!(result.trace().sturm_sequence_builds(), 0);
                assert_eq!(result.roots().len(), integers.len());
                for (root, expected) in result.roots().iter().zip(integers) {
                    let BezierParameter2::Algebraic(root) = root else {
                        panic!("nondyadic roots retain their original polynomial authority");
                    };
                    assert!(root.polynomial() == &defining);
                    assert_eq!(
                        compare_reals(root.interval().start(), &Real::from(expected), &policy),
                        Some(Ordering::Less)
                    );
                    assert_eq!(
                        compare_reals(root.interval().end(), &Real::from(expected), &policy),
                        Some(Ordering::Greater)
                    );
                    assert!(root.data.shared.sturm_sequence.get().is_none());
                }
                assert_eq!(
                    defining
                        .simple_root_classifications(result.roots(), &policy)
                        .unwrap(),
                    vec![Classification::Decided(true); 6]
                );
                for root in result.roots() {
                    let BezierParameter2::Algebraic(root) = root else {
                        unreachable!()
                    };
                    assert!(root.data.shared.sturm_sequence.get().is_none());
                }
                let empty = decided(
                    defining
                        .isolate_interval_roots(&extent, &Real::from(11).sqrt().unwrap(), &policy)
                        .unwrap(),
                );
                assert!(empty.is_empty());
            }
        }
    }

    #[test]
    fn finite_field_isolation_keeps_repeated_roots_and_closed_boundaries() {
        let scale = Real::from(3).sqrt().unwrap();
        let alpha = Real::from(2).sqrt().unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            // sqrt(3) * (t² - 2)² has just the two repeated roots ±sqrt(2).
            let defining = decided(
                BezierParameterPolynomial::try_new_power_basis(
                    [4, 0, -4, 0, 1].map(|c| Real::from(c) * &scale).to_vec(),
                    &policy,
                )
                .unwrap(),
            );
            for (lower, upper) in [(-&alpha, alpha.clone()), (Real::from(-2), Real::from(2))] {
                let roots = decided(
                    defining
                        .isolate_interval_roots(&lower, &upper, &policy)
                        .unwrap(),
                );
                assert_eq!(roots.len(), 2);
                for (root, expected) in roots.iter().zip([-&alpha, alpha.clone()]) {
                    assert_eq!(
                        root.cmp_by_refinement(&BezierParameter2::Exact(expected), &policy)
                            .unwrap(),
                        Classification::Decided(Ordering::Equal)
                    );
                }
                assert_eq!(
                    defining
                        .simple_root_classifications(&roots, &policy)
                        .unwrap(),
                    vec![Classification::Decided(false); 2]
                );
            }
        }
    }
}
