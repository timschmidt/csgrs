use hyperlimit::{Certainty, Point2, PredicateOutcome, PredicatePolicy, Sign};
use hyperreal::{Rational, Real};
use hypersolve::predicates::{Classification, PredicateBackend, StructuralPredicateBackend};
use num::{BigInt, BigUint, One};

fn argument(case: usize) -> Real {
    match case {
        0 => Rational::fraction(1, 3).unwrap().into(),
        1 => Real::from(2).sqrt().unwrap(),
        2 => Real::from(1).sin(),
        3 => Real::from(2).sqrt().unwrap() + Real::from(3).sqrt().unwrap(),
        _ => unreachable!(),
    }
}

fn pair(case: usize, delta: i32, reversed: bool) -> (Real, Real) {
    let a = argument(case).exp().unwrap().sqrt().unwrap();
    let q = if delta == 0 { Rational::zero() } else {
        Rational::from_bigint_fraction(BigInt::from(delta.signum()),
            BigUint::one() << delta.unsigned_abs() as usize).unwrap()
    };
    let b = (argument(case) * Real::from(Rational::fraction(1, 2).unwrap())
        + Real::from(q)).exp().unwrap();
    if reversed { (b, a) } else { (a, b) }
}

fn classified<T: core::fmt::Debug>(outcome: PredicateOutcome<T>) -> (String, String) {
    match outcome {
        PredicateOutcome::Decided { value, certainty, stage } => {
            assert_eq!(certainty, Certainty::Exact);
            (format!("{value:?}"), format!("{stage:?}"))
        }
        PredicateOutcome::Unknown { .. } => ("Unknown".into(), "Undecided".into()),
    }
}

fn main() {
    println!("case,delta,reversed,consumer,result,stage");
    for case in 0..4 {
        for delta in [0, -767, 767, -999, 999, -2048, 2048] {
            for reversed in [false, true] {
                let expected = if delta == 0 { Sign::Zero }
                    else if (delta < 0) != reversed { Sign::Positive } else { Sign::Negative };
                for consumer in ["sign", "ordering", "orientation", "solver"] {
                    // Independently constructed for each consumer; no prior
                    // classifier can seed the next one's scalar proof cache.
                    let (a, b) = pair(case, delta, reversed);
                    let (result, stage) = match consumer {
                        "sign" => classified(hyperlimit::classify_real_sign(&(a-b), PredicatePolicy::STRICT)),
                        "ordering" => classified(hyperlimit::compare_reals(&a, &b, PredicatePolicy::STRICT)),
                        "orientation" => classified(hyperlimit::orient2(
                            &Point2::new(0.into(), 0.into()), &Point2::new(1.into(), 0.into()), &Point2::new(0.into(), a-b),
                            PredicatePolicy::STRICT)),
                        "solver" => {
                            let report = StructuralPredicateBackend.classify_sign(&(a-b));
                            assert!(matches!(report.classification, Classification::Satisfied | Classification::Violated | Classification::Boundary | Classification::Unknown));
                            (format!("{:?}", report.classification), "Structural".into())
                        }
                        _ => unreachable!(),
                    };
                    if result != "Unknown" {
                        let valid = match (consumer, expected) {
                            ("ordering", Sign::Positive) => "Greater",
                            ("ordering", Sign::Negative) => "Less",
                            ("ordering", Sign::Zero) => "Equal",
                            ("solver", Sign::Positive) => "Satisfied",
                            ("solver", Sign::Negative) => "Violated",
                            ("solver", Sign::Zero) => "Boundary",
                            (_, Sign::Positive) => "Positive",
                            (_, Sign::Negative) => "Negative",
                            (_, Sign::Zero) => "Zero",
                        };
                        assert_eq!(result, valid, "incorrect certified consumer decision");
                    }
                    println!("{case},{delta},{reversed},{consumer},{result},{stage}");
                }
            }
        }
    }
}
