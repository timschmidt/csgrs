// Frozen 8f5c14723 algorithms and differential cases. Production authority
// moves to Hyperinterval only after the migration gates pass.
use super::*;

#[test]
fn rational_horner_exact_differential() {
    for (polynomial, lower, upper) in crate::test_support::rational_horner_cases() {
        let point = AlgebraicPolynomialValueInterval { lower, upper };
        let old = legacy_evaluate_rational_polynomial_interval(&polynomial, &point);
        let new = candidate_evaluate_rational_polynomial_interval(&polynomial, &point);
        let production = super::evaluate_rational_polynomial_interval(&polynomial, &point);
        let endpoints = |value: Option<AlgebraicPolynomialValueInterval>| {
            value.map(|v| {
                (
                    v.lower.exact_rational_ref().unwrap().clone(),
                    v.upper.exact_rational_ref().unwrap().clone(),
                )
            })
        };
        let expected = endpoints(old);
        assert_eq!(endpoints(new), expected);
        assert_eq!(endpoints(production), expected);
    }
}

#[cfg(feature = "dispatch-trace")]
#[test]
fn rational_horner_dispatch_differential() {
    let count = crate::test_support::rational_horner_cases().len() * 2;
    crate::test_support::compare_horner_processes(
        "algebraic::interval_migration::rational_horner_dispatch_differential",
        count,
        |legacy| {
            for (polynomial, lower, upper) in crate::test_support::rational_horner_cases() {
                let point = AlgebraicPolynomialValueInterval { lower, upper };
                for _ in 0..2 {
                    hyperreal::dispatch_trace::reset();
                    hyperreal::dispatch_trace::reset_rational_stats();
                    let result = hyperreal::dispatch_trace::with_recording(|| {
                        if legacy {
                            legacy_evaluate_rational_polynomial_interval(&polynomial, &point)
                        } else {
                            candidate_evaluate_rational_polynomial_interval(&polynomial, &point)
                        }
                    });
                    let trace = hyperreal::dispatch_trace::take_trace();
                    println!("HORNER {trace:?} {result:?}");
                }
            }
        },
    );
}

fn legacy_evaluate_rational_polynomial_interval(
    polynomial: &[Real],
    point: &AlgebraicPolynomialValueInterval,
) -> Option<AlgebraicPolynomialValueInterval> {
    let point_lower = point.lower.exact_rational_ref()?;
    let point_upper = point.upper.exact_rational_ref()?;
    if point_lower > point_upper {
        return None;
    }
    let mut coefficients = polynomial.iter().rev();
    let leading = coefficients.next()?.exact_rational_ref()?;
    let mut lower = leading.clone();
    let mut upper = leading.clone();
    for coefficient in coefficients {
        let coefficient = coefficient.exact_rational_ref()?;
        (lower, upper) = crate::test_support::legacy_rational_interval_product(
            &lower,
            &upper,
            point_lower,
            point_upper,
        );
        lower = lower + coefficient;
        upper = upper + coefficient;
    }
    Some(AlgebraicPolynomialValueInterval {
        lower: Real::from(lower),
        upper: Real::from(upper),
    })
}

fn candidate_evaluate_rational_polynomial_interval(
    polynomial: &[Real],
    point: &AlgebraicPolynomialValueInterval,
) -> Option<AlgebraicPolynomialValueInterval> {
    let point_lower = point.lower.exact_rational_ref()?;
    let point_upper = point.upper.exact_rational_ref()?;
    let point =
        hyperinterval::RationalInterval::new(point_lower.clone(), point_upper.clone()).ok()?;
    let mut coefficients = polynomial.iter().rev();
    let leading = coefficients.next()?.exact_rational_ref()?;
    let mut value = hyperinterval::RationalInterval::point(leading.clone());
    for coefficient in coefficients {
        let coefficient = coefficient.exact_rational_ref()?;
        value = value.mul(&point);
        value = value.add(&hyperinterval::RationalInterval::point(coefficient.clone()));
    }
    let (lower, upper) = value.into_bounds()?;
    Some(AlgebraicPolynomialValueInterval {
        lower: Real::from(lower),
        upper: Real::from(upper),
    })
}
