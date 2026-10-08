use hyperreal::{CertifiedRealEquality, Rational, Real};
use num::BigInt;

fn sine(k: i32) -> Real {
    let mut k = k.rem_euclid(24);
    let negative = k > 12;
    if negative {
        k -= 12;
    }
    if k > 6 {
        k = 12 - k;
    }
    let value = match k {
        0 => Real::zero(),
        1 => ((Real::from(6).sqrt().unwrap() - Real::from(2).sqrt().unwrap()) / Real::from(4))
            .unwrap(),
        2 => Real::from(Rational::fraction(1, 2).unwrap()),
        3 => (Real::from(2).sqrt().unwrap() / Real::from(2)).unwrap(),
        4 => (Real::from(3).sqrt().unwrap() / Real::from(2)).unwrap(),
        5 => ((Real::from(6).sqrt().unwrap() + Real::from(2).sqrt().unwrap()) / Real::from(4))
            .unwrap(),
        6 => Real::one(),
        _ => unreachable!(),
    };
    if negative { -value } else { value }
}

fn radical(k: i32, op: &str) -> Real {
    let s = sine(k);
    let c = sine(k + 6);
    match op {
        "sin" => s,
        "cos" => c,
        "tan" => (s / c).unwrap(),
        "cot" => (c / s).unwrap(),
        _ => unreachable!(),
    }
}

fn actual(k: i32, shift: usize, op: &str) -> Result<Real, hyperreal::Problem> {
    let turns = match shift {
        0 => BigInt::from(0),
        1 => BigInt::from(5),
        2 => BigInt::from(1) << 1024_usize,
        _ => unreachable!(),
    };
    let x =
        Real::from(Rational::fraction(k.into(), 12).unwrap() + Rational::from_bigint(turns * 2));
    match op {
        "sin" => Ok(x.sin_pi()),
        "cos" => Ok(x.cos_pi()),
        "tan" => x.tan_pi(),
        "cot" => x.cot_pi(),
        _ => unreachable!(),
    }
}

fn main() {
    println!("k,shift,op,precision,control,outcome");
    for k in 0..24 {
        for shift in 0..3 {
            for op in ["sin", "cos", "tan", "cot"] {
                let pole =
                    (op == "tan" && (k == 6 || k == 18)) || (op == "cot" && (k == 0 || k == 12));
                for precision in [-64, -256] {
                    for control in ["radical", "periodic", "perturbed"] {
                        let left = actual(k, shift, op);
                        let outcome = if pole {
                            assert!(matches!(left, Err(hyperreal::Problem::NotANumber)));
                            "Pole"
                        } else {
                            let left = left.expect("non-pole rational angle");
                            let right = match control {
                                "radical" => radical(k, op),
                                "periodic" => actual(k, 0, op).unwrap(),
                                "perturbed" => {
                                    radical(k, op)
                                        + Real::from(Rational::fraction(1, 1024).unwrap())
                                }
                                _ => unreachable!(),
                            };
                            match left.certified_eq_until(&right, precision) {
                                CertifiedRealEquality::Equal { .. } => {
                                    assert_ne!(control, "perturbed", "false equality");
                                    "Equal"
                                }
                                CertifiedRealEquality::NotEqual { .. } => {
                                    assert_eq!(control, "perturbed", "false inequality");
                                    "NotEqual"
                                }
                                CertifiedRealEquality::Unknown { .. } => "Unknown",
                            }
                        };
                        println!("{k},{shift},{op},{precision},{control},{outcome}");
                    }
                }
            }
        }
    }
}
