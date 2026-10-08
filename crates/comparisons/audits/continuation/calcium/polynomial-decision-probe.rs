use hyperlimit::{PredicateOutcome, PredicatePolicy, classify_real_sign};
use hyperreal::{Rational, Real};
use hypersolve::subresultant_chain_univariate_polynomials;
use num::{BigInt, BigUint, One};

fn coefficient(kind: usize) -> Real {
    match kind {
        0 => Real::zero(),
        1 => Real::one(),
        2 => Real::from(2).sqrt().unwrap(),
        3 => {
            let a = Real::one() + Real::from(2).sqrt().unwrap();
            a.ln().unwrap() * Real::from(2)
                - (Real::from(3) + Real::from(2) * Real::from(2).sqrt().unwrap()).ln().unwrap()
        }
        4..=6 => {
            let bits = [767, 999, 2048][kind - 4];
            let delta = Rational::from_bigint_fraction(BigInt::one(), BigUint::one() << bits).unwrap();
            let a = Real::from(1).sin();
            a.exp().unwrap().sqrt().unwrap()
                - (Real::from(1).sin() * Real::from(Rational::fraction(1, 2).unwrap())
                    + Real::from(delta)).exp().unwrap()
        }
        _ => unreachable!(),
    }
}

fn main() {
    println!("kind,degree,index,relation,floor,coefficient_known,outcome,result_degree,steps");
    for kind in 0..7 {
        for degree in 1..=4 {
            for index in 0..degree {
                for relation in ["self", "plus-one", "unknown-leading-control"] {
                    for floor in [-32, -128, -512] {
                        let value = coefficient(kind);
                        let known = matches!(classify_real_sign(&value, PredicatePolicy::STRICT), PredicateOutcome::Decided { .. });
                        let mut left = vec![Real::zero(); degree + 1];
                        left[index] = value.clone();
                        left[degree] = Real::one();
                        if relation == "unknown-leading-control" {
                            left[degree] = value;
                            left[0] += Real::one();
                        }
                        let mut right = left.clone();
                        if relation == "plus-one" { right[0] += Real::one(); }
                        match subresultant_chain_univariate_polynomials(&left, &right, floor) {
                            Ok(report) => {
                                if relation != "unknown-leading-control" {
                                    let expected = if relation == "self" { degree } else { 0 };
                                    assert_eq!(report.last_nonzero_degree, expected);
                                    assert_eq!(report.has_nonconstant_common_factor, expected > 0);
                                }
                                println!("{kind},{degree},{index},{relation},{floor},{known},Known,{},{}", report.last_nonzero_degree, report.steps.len());
                            }
                            Err(error) => println!("{kind},{degree},{index},{relation},{floor},{known},Unknown:{error:?},-1,0"),
                        }
                    }
                }
            }
        }
    }
}
