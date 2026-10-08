use criterion::Criterion;
use hyperreal::Real;
use hypersolve::{
    BivariatePolynomial, CurveIntersectionParameterLiftStatus, CurveIntersectionResultantConfig,
    CurveResultantParameter, linear_parameter_lifts_bivariate_polynomial_system,
};

fn polynomial(rows: &[&[i32]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|row| row.iter().copied().map(Real::from).collect())
            .collect(),
    )
}

pub(super) fn benchmarks(criterion: &mut Criterion) {
    let second = polynomial(&[&[0, 1], &[], &[-1]]); // u-t^2
    for (name, first) in [
        ("quadratic", polynomial(&[&[0, 0, 1], &[-1]])), // u^2-t
        (
            "specialized_degree_drop",
            polynomial(&[&[0, 1, -1], &[-1, 0, 1]]), // (t-1)u^2+u-t
        ),
    ] {
        for (axis_name, axis) in [
            ("first", CurveResultantParameter::First),
            ("second", CurveResultantParameter::Second),
        ] {
            let config = CurveIntersectionResultantConfig::default();
            let report =
                linear_parameter_lifts_bivariate_polynomial_system(&first, &second, axis, config);
            assert_eq!(
                report.status,
                CurveIntersectionParameterLiftStatus::Constructed
            );
            assert!(!report.maps.is_empty());
            assert!(
                report
                    .maps
                    .iter()
                    .all(|map| !map.denominator_coefficients.is_empty())
            );
            criterion.bench_function(
                &format!("linear_parameter_lifts/{name}/{axis_name}"),
                |bencher| {
                    bencher.iter(|| {
                        linear_parameter_lifts_bivariate_polynomial_system(
                            &first, &second, axis, config,
                        )
                    });
                },
            );
        }
    }
}
