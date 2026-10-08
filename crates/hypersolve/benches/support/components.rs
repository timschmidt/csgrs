use criterion::Criterion;
use hyperreal::Real;
use hypersolve::bivariate_arithmetic::bivariate_multiply;
use hypersolve::{
    BivariatePolynomial, BivariatePolynomialAxisFactorStatus, BivariatePolynomialComponentStatus,
    CurveIntersectionResultantConfig, CurveResultantParameter,
    extract_bivariate_polynomial_system_axis_factors,
    parameter_component_bivariate_polynomial_system,
};

fn polynomial(rows: &[&[i32]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|row| row.iter().copied().map(Real::from).collect())
            .collect(),
    )
}

pub(super) fn benchmarks(criterion: &mut Criterion) {
    for (name, component) in [
        ("rational", polynomial(&[&[0, 2], &[-1, 1], &[-1]])),
        ("implicit", polynomial(&[&[-1, 0, 1], &[0, -1]])),
    ] {
        let first = bivariate_multiply(&component, &polynomial(&[&[1, 1]]));
        let second = bivariate_multiply(&component, &polynomial(&[&[3, 1], &[1]]));
        for (axis_name, axis) in [
            ("first", CurveResultantParameter::First),
            ("second", CurveResultantParameter::Second),
        ] {
            let config = CurveIntersectionResultantConfig::default();
            let report =
                parameter_component_bivariate_polynomial_system(&first, &second, axis, config);
            assert!(matches!(
                report.status,
                BivariatePolynomialComponentStatus::Rational
                    | BivariatePolynomialComponentStatus::Implicit
            ));
            assert!(report.reduced_equations.is_some());
            criterion.bench_function(&format!("parameter_components/{name}/{axis_name}"), |b| {
                b.iter(|| {
                    parameter_component_bivariate_polynomial_system(&first, &second, axis, config)
                });
            });
        }
    }
    let first_primitive = polynomial(&[&[1, 2], &[3]]);
    let second_primitive = polynomial(&[&[4], &[5, 6]]);
    let content = bivariate_multiply(&polynomial(&[&[-2], &[1]]), &polynomial(&[&[3, 1]]));
    for (name, first, second, status) in [
        (
            "primitive",
            first_primitive.clone(),
            second_primitive.clone(),
            BivariatePolynomialAxisFactorStatus::Primitive,
        ),
        (
            "both_axes",
            bivariate_multiply(&content, &first_primitive),
            bivariate_multiply(&content, &second_primitive),
            BivariatePolynomialAxisFactorStatus::Reduced,
        ),
    ] {
        assert_eq!(
            extract_bivariate_polynomial_system_axis_factors(&first, &second).status,
            status
        );
        criterion.bench_function(&format!("axis_components/{name}"), |b| {
            b.iter(|| extract_bivariate_polynomial_system_axis_factors(&first, &second));
        });
    }
}
