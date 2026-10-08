use std::cmp::Ordering;

use exactcore_hyper_comparison::{
    ExprBinary, ExprUnary, RationalBinary, RationalUnary, expr_binary, expr_constant,
    expr_floor_ceil, expr_sign_unary, expr_unary, rational_binary, rational_compare,
    rational_parts, rational_unary,
};
use hyperreal::{Rational, Real, RealSign};
use proptest::prelude::*;

fn rational(text: &str) -> Rational {
    text.parse()
        .unwrap_or_else(|error| panic!("failed to parse {text:?}: {error:?}"))
}

fn real(text: &str) -> Real {
    Real::new(rational(text))
}

fn oracle_rational(text: String) -> Rational {
    rational(&text)
}

fn real_f64(value: Real) -> f64 {
    value
        .to_f64_lossy()
        .expect("test value has a finite approximation")
}

fn assert_close(left: f64, right: f64, context: &str) {
    assert!(left.is_finite(), "{context}: exactCore returned {left}");
    assert!(right.is_finite(), "{context}: Hyperreal returned {right}");
    let scale = 1.0_f64.max(left.abs()).max(right.abs());
    assert!(
        (left - right).abs() <= 2.0e-12 * scale,
        "{context}: exactCore={left:.17e}, Hyperreal={right:.17e}"
    );
}

fn rational_text(numerator: i64, denominator: u64) -> String {
    format!("{numerator}/{denominator}")
}

#[test]
fn rational_construction_reduction_parts_and_queries_match() {
    let cases = [
        (0, 1),
        (1, 1),
        (-1, 1),
        (6, 8),
        (-42, 30),
        (i64::MAX, 97),
        (i64::MIN + 1, 65_537),
    ];

    for (numerator, denominator) in cases {
        let text = rational_text(numerator, denominator);
        let hyper = rational(&text);
        let (core_numerator, core_denominator) = rational_parts(&text).unwrap();
        let core = rational(&format!("{core_numerator}/{core_denominator}"));

        assert_eq!(hyper, core, "normalization mismatch for {text}");
        assert_eq!(hyper.is_zero(), core_numerator == "0");
        assert_eq!(hyper.is_one(), core_numerator == core_denominator);
        assert_eq!(hyper.is_integer(), core_denominator == "1");

        let denominator_value: u64 = core_denominator.parse().unwrap();
        assert_eq!(hyper.is_dyadic(), denominator_value.is_power_of_two());
    }
}

#[test]
fn rational_arithmetic_and_comparison_tables_match() {
    let cases = [
        ("1/3", "2/5"),
        ("-7/11", "13/17"),
        ("0", "19/23"),
        ("123456789/1000003", "-987654321/1000033"),
        ("9223372036854775807/97", "-9223372036854775807/89"),
    ];

    for (left_text, right_text) in cases {
        let left = rational(left_text);
        let right = rational(right_text);
        let expected = [
            (RationalBinary::Add, &left + &right),
            (RationalBinary::Subtract, &left - &right),
            (RationalBinary::Multiply, &left * &right),
            (RationalBinary::Divide, &left / &right),
        ];
        for (operation, hyper) in expected {
            let core = oracle_rational(rational_binary(operation, left_text, right_text).unwrap());
            assert_eq!(core, hyper, "{operation:?}({left_text}, {right_text})");
        }

        let core_ordering = rational_compare(left_text, right_text).unwrap();
        let hyper_ordering = match left.partial_cmp(&right).unwrap() {
            Ordering::Less => -1,
            Ordering::Equal => 0,
            Ordering::Greater => 1,
        };
        assert_eq!(
            core_ordering, hyper_ordering,
            "cmp({left_text}, {right_text})"
        );
    }
}

#[test]
fn rational_unary_operations_match() {
    for text in ["1", "-1", "7/11", "-123456789/97"] {
        let value = rational(text);

        let negated = oracle_rational(rational_unary(RationalUnary::Negate, text).unwrap());
        assert_eq!(negated, -&value, "negate({text})");

        let absolute = oracle_rational(rational_unary(RationalUnary::Absolute, text).unwrap());
        let hyper_absolute = if value.is_negative() {
            -&value
        } else {
            value.clone()
        };
        assert_eq!(absolute, hyper_absolute, "abs({text})");

        let reciprocal = oracle_rational(rational_unary(RationalUnary::Reciprocal, text).unwrap());
        assert_eq!(reciprocal, value.inverse().unwrap(), "inverse({text})");
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(256))]

    #[test]
    fn rational_binary_properties_match(
        a in -1_000_000_i64..=1_000_000,
        b in 1_u64..=10_000,
        c in -1_000_000_i64..=1_000_000,
        d in 1_u64..=10_000,
    ) {
        let left_text = rational_text(a, b);
        let right_text = rational_text(c, d);
        let left = rational(&left_text);
        let right = rational(&right_text);

        for (operation, hyper) in [
            (RationalBinary::Add, &left + &right),
            (RationalBinary::Subtract, &left - &right),
            (RationalBinary::Multiply, &left * &right),
        ] {
            let core = oracle_rational(rational_binary(operation, &left_text, &right_text).unwrap());
            prop_assert_eq!(
                core,
                hyper,
                "{:?}({}, {})",
                operation,
                left_text,
                right_text
            );
        }

        if !right.is_zero() {
            let core = oracle_rational(
                rational_binary(RationalBinary::Divide, &left_text, &right_text).unwrap()
            );
            prop_assert_eq!(
                core,
                &left / &right,
                "divide({}, {})",
                left_text,
                right_text
            );
        }

        let core_ordering = rational_compare(&left_text, &right_text).unwrap();
        let hyper_ordering = match left.partial_cmp(&right).unwrap() {
            Ordering::Less => -1,
            Ordering::Equal => 0,
            Ordering::Greater => 1,
        };
        prop_assert_eq!(core_ordering, hyper_ordering);
    }
}

#[test]
fn exact_real_arithmetic_constants_roots_and_elementary_functions_match() {
    let binary_cases = [
        (
            ExprBinary::Add,
            "7/3",
            "-5/11",
            0,
            real("7/3") + real("-5/11"),
        ),
        (
            ExprBinary::Subtract,
            "7/3",
            "-5/11",
            0,
            real("7/3") - real("-5/11"),
        ),
        (
            ExprBinary::Multiply,
            "7/3",
            "-5/11",
            0,
            real("7/3") * real("-5/11"),
        ),
        (
            ExprBinary::Divide,
            "7/3",
            "-5/11",
            0,
            (real("7/3") / real("-5/11")).unwrap(),
        ),
        (
            ExprBinary::IntegerPower,
            "7/3",
            "1",
            5,
            real("7/3").powi_i64(5).unwrap(),
        ),
        (
            ExprBinary::IntegerPower,
            "7/3",
            "1",
            -4,
            real("7/3").powi_i64(-4).unwrap(),
        ),
    ];
    for (operation, left, right, parameter, hyper) in binary_cases {
        assert_close(
            expr_binary(operation, left, right, parameter),
            real_f64(hyper),
            &format!("{operation:?}({left}, {right}, {parameter})"),
        );
    }

    assert_close(expr_constant(0), real_f64(Real::pi()), "pi");
    assert_close(expr_constant(1), real_f64(Real::e()), "e");

    let unary_cases: Vec<(ExprUnary, &str, u32, Real)> = vec![
        (ExprUnary::Negate, "7/3", 0, -real("7/3")),
        (ExprUnary::Absolute, "-7/3", 0, real("-7/3").abs()),
        (ExprUnary::Sqrt, "2", 0, real("2").sqrt().unwrap()),
        (ExprUnary::Cbrt, "-8", 0, real("-8").cbrt().unwrap()),
        (ExprUnary::RootN, "81", 4, real("81").root_n(4).unwrap()),
        (ExprUnary::Exp, "1/3", 0, real("1/3").exp().unwrap()),
        (ExprUnary::Ln, "7/3", 0, real("7/3").ln().unwrap()),
        (ExprUnary::Log2, "8", 0, real("8").log2().unwrap()),
        (ExprUnary::Log10, "1000", 0, real("1000").log10().unwrap()),
        (ExprUnary::Sin, "1/3", 0, real("1/3").sin()),
        (ExprUnary::Cos, "1/3", 0, real("1/3").cos()),
        (ExprUnary::Tan, "1/3", 0, real("1/3").tan().unwrap()),
        (ExprUnary::Asin, "1/3", 0, real("1/3").asin().unwrap()),
        (ExprUnary::Acos, "1/3", 0, real("1/3").acos().unwrap()),
        (ExprUnary::Atan, "1/3", 0, real("1/3").atan().unwrap()),
        (
            ExprUnary::Square,
            "-7/3",
            0,
            real("-7/3").powi_i64(2).unwrap(),
        ),
        (
            ExprUnary::Exp2,
            "1/3",
            0,
            real("2").pow(real("1/3")).unwrap(),
        ),
        (
            ExprUnary::Exp10,
            "1/3",
            0,
            real("10").pow(real("1/3")).unwrap(),
        ),
        (
            ExprUnary::Cot,
            "1/3",
            0,
            (real("1/3").cos() / real("1/3").sin()).unwrap(),
        ),
    ];

    for (operation, input, degree, hyper) in unary_cases {
        assert_close(
            expr_unary(operation, input, degree),
            real_f64(hyper),
            &format!("{operation:?}({input}, {degree})"),
        );
    }
}

#[test]
fn exact_real_sign_floor_and_ceil_match() {
    let sign_cases = [
        (ExprUnary::Sqrt, "2", 0, RealSign::Positive),
        (ExprUnary::Negate, "7/3", 0, RealSign::Negative),
        (ExprUnary::Sin, "0", 0, RealSign::Zero),
        (ExprUnary::Ln, "1/2", 0, RealSign::Negative),
        (ExprUnary::RootN, "-32", 5, RealSign::Negative),
    ];
    for (operation, input, degree, expected) in sign_cases {
        let core = expr_sign_unary(operation, input, degree);
        let expected_number = match expected {
            RealSign::Negative => -1,
            RealSign::Zero => 0,
            RealSign::Positive => 1,
        };
        assert_eq!(core, expected_number, "CORE sign {operation:?}({input})");

        let hyper = match operation {
            ExprUnary::Sqrt => real(input).sqrt().unwrap(),
            ExprUnary::Negate => -real(input),
            ExprUnary::Sin => real(input).sin(),
            ExprUnary::Ln => real(input).ln().unwrap(),
            ExprUnary::RootN => real(input).root_n(degree).unwrap(),
            _ => unreachable!(),
        };
        assert_eq!(hyper.refine_sign_until(-128), Some(expected));
    }

    for text in ["0", "1", "-1", "7/3", "-7/3", "1000000000001/10"] {
        let value = real(text);
        assert_eq!(
            expr_floor_ceil(false, text).unwrap(),
            value.floor_certified().unwrap().to_string(),
            "floor({text})"
        );
        assert_eq!(
            expr_floor_ceil(true, text).unwrap(),
            value.ceil_certified().unwrap().to_string(),
            "ceil({text})"
        );
    }
}

#[test]
fn shared_domain_failures_are_rejected() {
    assert!(real("-1").sqrt().is_err());
    assert!(real("0").ln().is_err());
    assert!(real("2").asin().is_err());
    assert!(real("0").inverse().is_err());
    assert!(rational("0").inverse().is_err());
}
