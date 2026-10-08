use exactcore_hyper_comparison::{
    bivariate_eval, bivariate_resultant_eval, polynomial_binary_eval, polynomial_derivative_eval,
    polynomial_discriminant, polynomial_eval, polynomial_gcd_degree, polynomial_isolate_roots,
    polynomial_resultant, polynomial_root_count, polynomial_root_count_interval,
    polynomial_square_free_degree,
};
use hyperlimit::PredicatePolicy;
use hyperreal::{Rational, Real};
use hypersolve::{
    BivariatePolynomial, CurveIntersectionResultantConfig, CurveIntersectionResultantStatus,
    CurveResultantParameter, Expr, Problem, RootIsolationStatus, SymbolId,
    isolate_univariate_polynomial_expr, polynomial_has_one_distinct_root_in_open_interval,
    resultant_bivariate_polynomial_system, resultant_univariate_polynomials, square_free_part,
    subresultant_chain_univariate_polynomials,
};
use proptest::prelude::*;

fn rational(text: &str) -> Rational {
    text.parse()
        .unwrap_or_else(|error| panic!("failed to parse {text:?}: {error:?}"))
}

fn real(value: i64) -> Real {
    Real::from(value)
}

fn reals(coefficients: &[i64]) -> Vec<Real> {
    coefficients.iter().copied().map(real).collect()
}

fn assert_core_equals_real(core: String, hyper: &Real, context: &str) {
    let core = rational(&core);
    assert_eq!(
        hyper, &core,
        "{context}: exactCore={core}, Hyperreal={hyper:?}"
    );
}

fn eval_i64(coefficients: &[i64], x: i64) -> Real {
    Real::eval_poly(&reals(coefficients), &real(x))
}

fn add(left: &[i64], right: &[i64]) -> Vec<i64> {
    let mut result = vec![0; left.len().max(right.len())];
    for (index, value) in left.iter().enumerate() {
        result[index] += value;
    }
    for (index, value) in right.iter().enumerate() {
        result[index] += value;
    }
    result
}

fn subtract(left: &[i64], right: &[i64]) -> Vec<i64> {
    let mut result = vec![0; left.len().max(right.len())];
    for (index, value) in left.iter().enumerate() {
        result[index] += value;
    }
    for (index, value) in right.iter().enumerate() {
        result[index] -= value;
    }
    result
}

fn multiply(left: &[i64], right: &[i64]) -> Vec<i64> {
    let mut result = vec![0; left.len() + right.len() - 1];
    for (left_index, left_value) in left.iter().enumerate() {
        for (right_index, right_value) in right.iter().enumerate() {
            result[left_index + right_index] += left_value * right_value;
        }
    }
    result
}

fn derivative(mut coefficients: Vec<i64>, order: u32) -> Vec<i64> {
    for _ in 0..order {
        coefficients = coefficients
            .iter()
            .enumerate()
            .skip(1)
            .map(|(power, coefficient)| coefficient * power as i64)
            .collect();
        if coefficients.is_empty() {
            coefficients.push(0);
        }
    }
    coefficients
}

fn polynomial_expr(coefficients: &[i64], symbol: &Expr) -> Expr {
    coefficients
        .iter()
        .rev()
        .fold(Expr::zero(), |accumulator, coefficient| {
            accumulator * symbol.clone() + Expr::int(*coefficient)
        })
}

#[test]
fn evaluation_addition_subtraction_multiplication_and_composition_match() {
    let tables = [
        (vec![3], vec![-7, 2], -4),
        (vec![-6, 11, -6, 1], vec![2, -3, 1], 5),
        (vec![9, 0, -4, 0, 1], vec![-2, 0, 3], -3),
    ];

    for (left, right, x) in tables {
        assert_core_equals_real(
            polynomial_eval(&left, x).unwrap(),
            &eval_i64(&left, x),
            "polynomial evaluation",
        );
        for (operation, coefficients, name) in [
            (0, add(&left, &right), "addition"),
            (1, subtract(&left, &right), "subtraction"),
            (2, multiply(&left, &right), "multiplication"),
        ] {
            assert_core_equals_real(
                polynomial_binary_eval(operation, &left, &right, x).unwrap(),
                &eval_i64(&coefficients, x),
                name,
            );
        }

        let inner = eval_i64(&right, x);
        let composed = Real::eval_poly(&reals(&left), &inner);
        assert_core_equals_real(
            polynomial_binary_eval(4, &left, &right, x).unwrap(),
            &composed,
            "Horner composition",
        );
    }
}

#[test]
fn iterated_derivatives_match() {
    let coefficients = [17, -5, 0, 7, -3, 2];
    for order in 0..=7 {
        for x in [-4, -1, 0, 2, 9] {
            let expected = eval_i64(&derivative(coefficients.to_vec(), order), x);
            assert_core_equals_real(
                polynomial_derivative_eval(&coefficients, order, x).unwrap(),
                &expected,
                &format!("derivative order {order} at {x}"),
            );
        }
    }
}

#[test]
fn pseudo_remainder_value_matches_hypersolve_chain_first_step() {
    let cases = [
        (vec![-1, 0, 0, 1], vec![-1, 0, 1]),
        (vec![2, -3, 0, 2], vec![1, -1, 1]),
        (vec![-6, 11, -6, 1], vec![-2, 1]),
    ];
    for (left, right) in cases {
        let chain =
            subresultant_chain_univariate_polynomials(&reals(&left), &reals(&right), -128).unwrap();
        let signed_hyper_remainder = &chain.steps[0].remainder;
        for x in [-3, 0, 4] {
            // hypersolve stores the negated pseudo-remainder as the next PRS row.
            let expected = -Real::eval_poly(signed_hyper_remainder, &real(x));
            assert_core_equals_real(
                polynomial_binary_eval(3, &left, &right, x).unwrap(),
                &expected,
                "pseudo-remainder",
            );
        }
    }
}

#[test]
fn univariate_resultants_match_after_the_documented_argument_orientation() {
    let cases = [
        (vec![-2, 1], vec![-5, 1]),
        (vec![-1, 0, 1], vec![-1, 1]),
        (vec![2, -3, 0, 2], vec![1, 4, -2]),
        (vec![-6, 11, -6, 1], vec![2, 0, 1]),
    ];

    for (left, right) in cases {
        let core = polynomial_resultant(&left, &right).unwrap();
        let hyper = resultant_univariate_polynomials(&reals(&left), &reals(&right), -128)
            .unwrap()
            .resultant;
        // CORE 2.1's recursive `res` base case negates the otherwise-standard
        // Sylvester orientation for every nonconstant pair.
        let core_orientation = -hyper;
        assert_core_equals_real(core, &core_orientation, "univariate resultant");
    }
}

#[test]
fn gcd_and_square_free_degrees_match() {
    let gcd_cases = [
        (vec![-1, 0, 1], vec![-1, 1]),
        (vec![-6, 11, -6, 1], vec![2, -3, 1]),
        (vec![1, 0, 1], vec![-1, 1]),
        (vec![0, 0, 1], vec![0, 1]),
    ];
    for (left, right) in gcd_cases {
        let hyper =
            subresultant_chain_univariate_polynomials(&reals(&left), &reals(&right), -128).unwrap();
        assert_eq!(
            polynomial_gcd_degree(&left, &right),
            hyper.last_nonzero_degree as i32,
            "GCD degree for {left:?}, {right:?}",
        );
    }

    let square_free_cases = [
        vec![-6, 11, -6, 1],
        vec![-4, 0, 3, 1], // (x - 1) (x + 2)^2
        vec![1, -4, 6, -4, 1],
        vec![1, 0, 1],
    ];
    for coefficients in square_free_cases {
        let hyper = square_free_part(reals(&coefficients), PredicatePolicy::STRICT).unwrap();
        assert_eq!(
            polynomial_square_free_degree(&coefficients),
            hyper.len() as i32 - 1,
            "square-free degree for {coefficients:?}",
        );
    }
}

#[test]
fn sturm_root_counts_and_isolating_intervals_match() {
    let cases = [
        (vec![-2, 0, 1], 2),
        (vec![-2, 0, 0, 1], 1),
        (vec![1, 0, 1], 0),
    ];

    for (coefficients, expected_distinct_roots) in cases {
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", Real::zero());
        let report = isolate_univariate_polynomial_expr(
            0,
            &polynomial_expr(&coefficients, &x),
            &problem,
            PredicatePolicy::APPROXIMATE_512,
        );
        assert!(
            matches!(
                report.status,
                RootIsolationStatus::Isolated
                    | RootIsolationStatus::MultipleRoot
                    | RootIsolationStatus::NoRealRoots
            ),
            "unexpected isolation result for {coefficients:?}: {report:?}",
        );
        assert_eq!(report.intervals.len(), expected_distinct_roots);
        assert_eq!(
            polynomial_root_count(&coefficients),
            expected_distinct_roots as i32,
            "root count for {coefficients:?}",
        );

        let core_intervals = polynomial_isolate_roots(&coefficients).unwrap();
        assert_eq!(core_intervals.len(), report.intervals.len());
        for (lower, upper) in core_intervals {
            assert!(lower <= upper && lower.is_finite() && upper.is_finite());
            assert!(
                report.intervals.iter().any(|interval| {
                    let hyper_lower = interval.lower.to_f64_lossy().unwrap();
                    let hyper_upper = interval.upper.to_f64_lossy().unwrap();
                    hyper_lower <= upper && lower <= hyper_upper
                }),
                "CORE interval ({lower}, {upper}) did not overlap a Hyper interval: {:?}",
                report.intervals,
            );
        }
    }
}

#[test]
fn sturm_integer_interval_counts_match_hyper_open_interval_certification() {
    let coefficients = [-2, 0, 1];
    for (lower, upper, count) in [(-4, -2, 0), (-2, 0, 1), (0, 2, 1), (-4, 4, 2)] {
        assert_eq!(
            polynomial_root_count_interval(&coefficients, lower, upper),
            count
        );
        if count <= 1 {
            assert_eq!(
                polynomial_has_one_distinct_root_in_open_interval(
                    &reals(&coefficients),
                    &real(lower),
                    &real(upper),
                    PredicatePolicy::STRICT,
                ),
                Some(count == 1),
            );
        }
    }
}

#[test]
fn bivariate_evaluation_and_resultant_specializations_match() {
    // Coefficients are [first power][second power], each axis ascending.
    let left_flat = [0, 1, -1, 0]; // u - t
    let right_flat = [-1, 1, 1, 0]; // u + t - 1
    let left = BivariatePolynomial::new(vec![vec![real(0), real(1)], vec![real(-1), real(0)]]);
    let right = BivariatePolynomial::new(vec![vec![real(-1), real(1)], vec![real(1), real(0)]]);

    for first in [-3, 0, 2, 7] {
        for second in [-2, 1, 5] {
            let hyper = Real::eval_poly(
                &left
                    .coefficients
                    .iter()
                    .map(|row| Real::eval_poly(row, &real(second)))
                    .collect::<Vec<_>>(),
                &real(first),
            );
            assert_core_equals_real(
                bivariate_eval(&left_flat, 2, 2, first, second).unwrap(),
                &hyper,
                "bivariate evaluation",
            );
        }
    }

    for (eliminate_second, retained_parameter) in [
        (true, CurveResultantParameter::First),
        (false, CurveResultantParameter::Second),
    ] {
        let report = resultant_bivariate_polynomial_system(
            &left,
            &right,
            retained_parameter,
            CurveIntersectionResultantConfig::default(),
        );
        assert_eq!(report.status, CurveIntersectionResultantStatus::Constructed);
        for retained_value in [-4, 0, 2, 9] {
            let hyper = Real::eval_poly(&report.resultant_coefficients, &real(retained_value));
            assert_core_equals_real(
                bivariate_resultant_eval(
                    eliminate_second,
                    &left_flat,
                    &right_flat,
                    2,
                    2,
                    retained_value,
                )
                .unwrap(),
                &hyper,
                "bivariate resultant specialization",
            );
        }
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(128))]

    #[test]
    fn polynomial_evaluation_and_derivative_properties_match(
        coefficients in prop::collection::vec(-50_i64..=50, 1..8),
        x in -8_i64..=8,
        order in 0_u32..=4,
    ) {
        let hyper = eval_i64(&coefficients, x);
        let core = polynomial_eval(&coefficients, x).unwrap();
        prop_assert_eq!(hyper, rational(&core));

        let hyper_derivative = eval_i64(&derivative(coefficients.clone(), order), x);
        let core_derivative = polynomial_derivative_eval(&coefficients, order, x).unwrap();
        prop_assert_eq!(hyper_derivative, rational(&core_derivative));
    }

    #[test]
    fn linear_resultants_match_exactcore_argument_orientation(
        a0 in -100_i64..=100,
        a1 in -100_i64..=100,
        b0 in -100_i64..=100,
        b1 in -100_i64..=100,
    ) {
        prop_assume!(a1 != 0 && b1 != 0);
        let left = [a0, a1];
        let right = [b0, b1];
        let hyper = resultant_univariate_polynomials(&reals(&left), &reals(&right), -128)
            .unwrap()
            .resultant;
        let core = polynomial_resultant(&left, &right).unwrap();
        prop_assert_eq!(-hyper, rational(&core));
    }
}

#[test]
#[ignore = "exactCore res() negates the standard nonconstant resultant orientation"]
fn univariate_resultant_matches_the_standard_sylvester_orientation() {
    let left = [-2, 1];
    let right = [-5, 1];
    let core = polynomial_resultant(&left, &right).unwrap();
    let hyper = resultant_univariate_polynomials(&reals(&left), &reals(&right), -128)
        .unwrap()
        .resultant;
    assert_core_equals_real(core, &hyper, "standard resultant orientation");
}

#[test]
#[ignore = "known exactCore Sturm sequence loses one of three simple integer roots"]
fn exactcore_sturm_counts_three_simple_integer_roots() {
    let coefficients = [-6, 11, -6, 1];
    assert_eq!(polynomial_root_count(&coefficients), 3);
    assert_eq!(polynomial_isolate_roots(&coefficients).unwrap().len(), 3);
}

#[test]
#[ignore = "known exactCore res() constant-polynomial base case uses a nonstandard zero result"]
fn constant_polynomial_resultant_uses_the_standard_convention() {
    let left = [-2, 0, 1];
    let right = [3];
    let core = polynomial_resultant(&left, &right).unwrap();
    let hyper = resultant_univariate_polynomials(&reals(&left), &reals(&right), -128)
        .unwrap()
        .resultant;
    assert_core_equals_real(core, &hyper, "constant resultant");
}

#[test]
#[ignore = "exactCore disc() is explicitly a cheap resultant proxy and omits normalization"]
fn polynomial_discriminant_matches_the_standard_definition() {
    // x^3 - 2 has discriminant -108.
    let coefficients = [-2, 0, 0, 1];
    let derivative = [0, 0, 3];
    let resultant =
        resultant_univariate_polynomials(&reals(&coefficients), &reals(&derivative), -128)
            .unwrap()
            .resultant;
    let standard_discriminant = -resultant;
    assert_core_equals_real(
        polynomial_discriminant(&coefficients).unwrap(),
        &standard_discriminant,
        "cubic discriminant",
    );
}
