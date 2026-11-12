// Frozen 8f5c14723 algorithms and differential cases. Production authority
// moves to Hyperinterval only after the migration gates pass.
use super::*;

#[test]
fn rational_horner_exclusion_exact_differential() {
    for (polynomial, lower, upper) in crate::test_support::rational_horner_cases() {
        assert_eq!(
            super::exact_rational_polynomial_interval_excludes_zero(&polynomial, &lower, &upper),
            legacy_exact_rational_polynomial_interval_excludes_zero(&polynomial, &lower, &upper)
        );
        assert_eq!(
            super::exact_rational_polynomial_derivative_interval_excludes_zero(
                &polynomial,
                &lower,
                &upper
            ),
            legacy_exact_rational_polynomial_derivative_interval_excludes_zero(
                &polynomial,
                &lower,
                &upper
            )
        );
        assert_eq!(
            candidate_exact_rational_polynomial_interval_excludes_zero(&polynomial, &lower, &upper),
            legacy_exact_rational_polynomial_interval_excludes_zero(&polynomial, &lower, &upper)
        );
        assert_eq!(
            candidate_exact_rational_polynomial_derivative_interval_excludes_zero(
                &polynomial,
                &lower,
                &upper
            ),
            legacy_exact_rational_polynomial_derivative_interval_excludes_zero(
                &polynomial,
                &lower,
                &upper
            )
        );
    }
}

#[cfg(feature = "dispatch-trace")]
#[test]
fn rational_horner_exclusion_dispatch_differential() {
    let count = crate::test_support::rational_horner_cases().len() * 4;
    crate::test_support::compare_horner_processes(
        "root_isolation::interval_migration::rational_horner_exclusion_dispatch_differential",
        count,
        |legacy| {
            for (polynomial, lower, upper) in crate::test_support::rational_horner_cases() {
                for _ in 0..2 {
                    for derivative in [false, true] {
                        hyperreal::dispatch_trace::reset();
                        hyperreal::dispatch_trace::reset_rational_stats();
                        let result = hyperreal::dispatch_trace::with_recording(|| {
                            match (legacy,derivative) {
                        (true,false)=>legacy_exact_rational_polynomial_interval_excludes_zero(&polynomial,&lower,&upper),
                        (true,true)=>legacy_exact_rational_polynomial_derivative_interval_excludes_zero(&polynomial,&lower,&upper),
                        (false,false)=>candidate_exact_rational_polynomial_interval_excludes_zero(&polynomial,&lower,&upper),
                        (false,true)=>candidate_exact_rational_polynomial_derivative_interval_excludes_zero(&polynomial,&lower,&upper),
                    }
                        });
                        println!(
                            "HORNER {:?} {result:?}",
                            hyperreal::dispatch_trace::take_trace()
                        );
                    }
                }
            }
        },
    );
}

fn legacy_exact_rational_polynomial_interval_excludes_zero(
    polynomial: &[Real],
    lower: &Real,
    upper: &Real,
) -> Option<bool> {
    let lower = lower.exact_rational_ref()?;
    let upper = upper.exact_rational_ref()?;
    if lower > upper {
        return None;
    }
    let mut coefficients = polynomial.iter().rev();
    let leading = coefficients.next()?.exact_rational_ref()?;
    let mut range_lower = leading.clone();
    let mut range_upper = leading.clone();
    for coefficient in coefficients {
        let coefficient = coefficient.exact_rational_ref()?;
        let (product_lower, product_upper) = crate::test_support::legacy_rational_interval_product(
            &range_lower,
            &range_upper,
            lower,
            upper,
        );
        range_lower = product_lower + coefficient;
        range_upper = product_upper + coefficient;
    }
    Some(range_lower.is_positive() || range_upper.is_negative())
}

fn legacy_exact_rational_polynomial_derivative_interval_excludes_zero(
    polynomial: &[Real],
    lower: &Real,
    upper: &Real,
) -> Option<bool> {
    let lower = lower.exact_rational_ref()?;
    let upper = upper.exact_rational_ref()?;
    if lower > upper {
        return None;
    }
    let mut coefficients = polynomial.iter().enumerate().skip(1).rev();
    let (degree, leading) = coefficients.next()?;
    let degree = HyperRational::new(i64::try_from(degree).ok()?);
    let leading = leading.exact_rational_ref()? * &degree;
    let mut range_lower = leading.clone();
    let mut range_upper = leading;
    for (degree, coefficient) in coefficients {
        let (product_lower, product_upper) = crate::test_support::legacy_rational_interval_product(
            &range_lower,
            &range_upper,
            lower,
            upper,
        );
        let degree = HyperRational::new(i64::try_from(degree).ok()?);
        let coefficient = coefficient.exact_rational_ref()? * degree;
        range_lower = product_lower + &coefficient;
        range_upper = product_upper + coefficient;
    }
    Some(range_lower.is_positive() || range_upper.is_negative())
}

fn candidate_exact_rational_polynomial_interval_excludes_zero(
    polynomial: &[Real],
    lower: &Real,
    upper: &Real,
) -> Option<bool> {
    let lower = lower.exact_rational_ref()?;
    let upper = upper.exact_rational_ref()?;
    let point = hyperinterval::RationalInterval::new(lower.clone(), upper.clone()).ok()?;
    let mut coefficients = polynomial.iter().rev();
    let leading = coefficients.next()?.exact_rational_ref()?;
    let mut range = hyperinterval::RationalInterval::point(leading.clone());
    for coefficient in coefficients {
        let coefficient = coefficient.exact_rational_ref()?;
        range = range.mul(&point);
        range = range.add(&hyperinterval::RationalInterval::point(coefficient.clone()));
    }
    let (range_lower, range_upper) = range.bounds()?;
    Some(range_lower.is_positive() || range_upper.is_negative())
}

fn candidate_exact_rational_polynomial_derivative_interval_excludes_zero(
    polynomial: &[Real],
    lower: &Real,
    upper: &Real,
) -> Option<bool> {
    let lower = lower.exact_rational_ref()?;
    let upper = upper.exact_rational_ref()?;
    let point = hyperinterval::RationalInterval::new(lower.clone(), upper.clone()).ok()?;
    let mut coefficients = polynomial.iter().enumerate().skip(1).rev();
    let (degree, leading) = coefficients.next()?;
    let degree = HyperRational::new(i64::try_from(degree).ok()?);
    let leading = leading.exact_rational_ref()? * &degree;
    let mut range = hyperinterval::RationalInterval::point(leading);
    for (degree, coefficient) in coefficients {
        range = range.mul(&point);
        let degree = HyperRational::new(i64::try_from(degree).ok()?);
        let coefficient = coefficient.exact_rational_ref()? * degree;
        range = range.add(&hyperinterval::RationalInterval::point(coefficient));
    }
    let (range_lower, range_upper) = range.bounds()?;
    Some(range_lower.is_positive() || range_upper.is_negative())
}
