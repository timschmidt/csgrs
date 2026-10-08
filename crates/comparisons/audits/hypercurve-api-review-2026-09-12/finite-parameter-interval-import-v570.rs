// Unrun private importer regression for the ordered-interval migration.
#[cfg(test)]
mod finite_interval_import_regression {
    use super::*;

    #[test]
    fn unit_import_keeps_its_domain_after_generic_interval_construction() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for (lower, upper) in [(-2, -1), (1, 2)] {
                let mut representation = AlgebraicRootRepresentation {
                    constraint_index: 0,
                    symbol: hypersolve::SymbolId(0),
                    interval_index: 0,
                    polynomial_coefficients: vec![Real::from(-2), Real::zero(), Real::one()],
                    interval: hypersolve::IsolatedRootInterval {
                        lower: Real::from(lower),
                        upper: Real::from(upper),
                        exact_root: None,
                        distinct_root_count: 1,
                    },
                    validation: hypersolve::AlgebraicRootValidationReport {
                        status: hypersolve::AlgebraicRootValidationStatus::Valid,
                        message: None,
                    },
                };
                representation.validation = hypersolve::validate_algebraic_root_representation(
                    &representation,
                    hypersolve::PredicatePolicy::STRICT,
                );
                assert!(representation.is_valid());
                assert!(matches!(
                    BezierParameter2::from_algebraic_root_representation(&representation, &policy),
                    Err(CurveError::InvalidBezierParameter)
                ));
                assert!(matches!(
                    BezierParameter2::from_algebraic_root_representation_unbounded(
                        &representation,
                        &policy
                    ),
                    Ok(Classification::Decided(BezierParameter2::Algebraic(_)))
                ));
            }
        }
    }
}
