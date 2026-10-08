use hyperreal::{CertifiedRealEquality, Real};
use std::time::Instant;

fn main() {
    // Exact oracle: for a > 0 and d > 0,
    // (a + sqrt(d))^2 = a^2 + d + 2*a*sqrt(d), and real log is injective.
    // The perturbed case is strictly different, not an approximate zero oracle.
    println!("kind,a,d,precision,outcome,elapsed_ns");
    for a in [1_i64, 2, 3] {
        for d in [2_i64, 3, 5, 6, 7, 10, 11, 13, 17, 19] {
            let root = Real::from(d).sqrt().unwrap();
            let x = Real::from(a) + root.clone();
            let expanded = Real::from(a * a + d) + Real::from(2 * a) * root;
            let twice_log = Real::from(2) * x.clone().ln().unwrap();
            let log_square = expanded.clone().ln().unwrap();
            let log_perturbed = (expanded.clone() + (Real::from(1) / Real::from(1024)).unwrap())
                .ln()
                .unwrap();
            let square = &x * &x;
            for precision in [-64, -256, -512] {
                for (kind, left, right, expected_equal) in [
                    ("algebraic_control", &square, &expanded, true),
                    ("log_identity", &twice_log, &log_square, true),
                    ("perturbed_control", &twice_log, &log_perturbed, false),
                ] {
                    let start = Instant::now();
                    let answer = left.certified_eq_until(right, precision);
                    let elapsed = start.elapsed().as_nanos();
                    let outcome = match answer {
                        CertifiedRealEquality::Equal { .. } => {
                            assert!(expected_equal, "false equality: {kind} a={a} d={d}");
                            "Equal"
                        }
                        CertifiedRealEquality::NotEqual { .. } => {
                            assert!(!expected_equal, "false inequality: {kind} a={a} d={d}");
                            "NotEqual"
                        }
                        CertifiedRealEquality::Unknown { .. } => "Unknown",
                    };
                    // Timings are diagnostic, not matched performance claims.
                    println!("{kind},{a},{d},{precision},{outcome},{elapsed}");
                }
            }
        }
    }
}
