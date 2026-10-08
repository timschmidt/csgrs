use hyperreal::{CertifiedRealEquality, Rational, Real};

fn argument(case: usize) -> Real {
    match case {
        0 => Real::from(Rational::fraction(1, 3).unwrap()),
        1 => Real::from(2).sqrt().unwrap(),
        2 => Real::from(1).sin(),
        3 => Real::pi() - Real::from(3),
        4 => Real::from(2).ln().unwrap() + Real::from(3).ln().unwrap(),
        5 => Real::from(2).sqrt().unwrap() + Real::from(3).sqrt().unwrap(),
        6 => Real::from(2).sqrt().unwrap() * Real::from(32),
        7 => Real::from(2).sqrt().unwrap() * Real::from(Rational::fraction(1, 1024).unwrap()),
        _ => unreachable!(),
    }
}

fn main() {
    println!("case,negative,precision,control,outcome");
    for case in 0..8 {
        for negative in [false, true] {
            for precision in [-64, -256, -512] {
                for perturbed in [false, true] {
                    // Fresh construction per query, including separate operand
                    // factories for the two mathematically equivalent sides.
                    let x = argument(case) * Real::from(if negative { -1 } else { 1 });
                    let y = argument(case) * Real::from(if negative { -1 } else { 1 });
                    let left = x.exp().unwrap().sqrt().unwrap();
                    let mut right = (y * Real::from(Rational::fraction(1, 2).unwrap())).exp().unwrap();
                    if perturbed { right += Real::from(Rational::fraction(1, 1024).unwrap()); }
                    let outcome = match left.certified_eq_until(&right, precision) {
                        CertifiedRealEquality::Equal { .. } => {
                            assert!(!perturbed, "false equality");
                            "Equal"
                        }
                        CertifiedRealEquality::NotEqual { .. } => {
                            assert!(perturbed, "false inequality");
                            "NotEqual"
                        }
                        CertifiedRealEquality::Unknown { .. } => "Unknown",
                    };
                    println!("{case},{negative},{precision},{},{outcome}", if perturbed { "perturbed" } else { "identity" });
                }
            }
        }
    }
}
