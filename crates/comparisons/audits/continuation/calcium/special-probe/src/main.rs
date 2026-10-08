use hyperreal::{CertifiedRealEquality, Rational, Real};

fn argument(case: usize) -> Real {
    match case {
        0 => Real::from(-2),
        1 => Real::from(Rational::fraction(-1, 3).unwrap()),
        2 => Real::from(0),
        3 => Real::from(Rational::fraction(1, 3).unwrap()),
        4 => Real::from(2),
        5 => Real::from(2).sqrt().unwrap(),
        6 => Real::pi(),
        _ => unreachable!(),
    }
}

fn main() {
    // These are mathematical identities of total real functions. Shifting the
    // right side by a positive rational creates independent unequal controls.
    // No timings are compared with the native donor's different budget model.
    println!("kind,case,precision,control,outcome");
    for case in 0..7 {
        for precision in [-64, -256, -512] {
            for kind in ["complement", "erf_odd", "erfc_reflection"] {
                for perturb in [false, true] {
                    let x = argument(case);
                    let (left, mut right) = match kind {
                        "complement" => (x.clone().erf() + x.erfc(), Real::from(1)),
                        "erf_odd" => (x.clone().erf() + (-x).erf(), Real::from(0)),
                        "erfc_reflection" => (x.clone().erfc() + (-x).erfc(), Real::from(2)),
                        _ => unreachable!(),
                    };
                    if perturb {
                        right += Real::from(Rational::fraction(1, 1024).unwrap());
                    }
                    let outcome = match left.certified_eq_until(&right, precision) {
                        CertifiedRealEquality::Equal { .. } => {
                            assert!(!perturb, "false equality: {kind} {case}");
                            "Equal"
                        }
                        CertifiedRealEquality::NotEqual { .. } => {
                            assert!(perturb, "false inequality: {kind} {case}");
                            "NotEqual"
                        }
                        CertifiedRealEquality::Unknown { .. } => "Unknown",
                    };
                    let control = if perturb { "perturbed" } else { "identity" };
                    println!("{kind},{case},{precision},{control},{outcome}");
                }
            }
        }
    }
}
