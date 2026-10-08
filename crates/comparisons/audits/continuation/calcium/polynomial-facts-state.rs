#[allow(dead_code)]
mod corpus {
    include!("polynomial-decision-probe.rs");
    pub fn value(kind: usize) -> hyperreal::Real { coefficient(kind) }
}
#[allow(dead_code)]
mod normal {
    include!("polynomial-facts-trial/hypersolve/src/test_support.rs");
}
use hyperreal::{Real, RealSign, ZeroKnowledge};
use hypersolve::subresultant_chain_univariate_polynomials;
use std::sync::{Arc, Barrier, atomic::{AtomicBool, Ordering}};

fn run(left: &[Real], right: &[Real], degree: usize, shifted: bool) {
    let report = subresultant_chain_univariate_polynomials(left, right, -64)
        .expect("known leading nonzero must dominate unresolved lower coefficients");
    assert_eq!(report.last_nonzero_degree, if shifted { 0 } else { degree });
    assert_eq!(report.has_nonconstant_common_factor, !shifted);
    assert_eq!(report.steps.len(), 1);
    assert_eq!(report.steps[0].zero_remainder, !shifted);
}

fn main() {
    let mut queries = 0;
    let mut serializations = 0;
    for kind in [3, 6] {
        for degree in [1, 4] {
            for leading_kind in 0..4 {
                let mut coefficient = corpus::value(kind);
                let signal = Arc::new(AtomicBool::new(true));
                coefficient.abort(signal.clone());
                // An aborted approximation is not a valid numeric observation.
                // Discard it and test that the subsequent valid query recovers.
                let _ = coefficient.certified_sign_until(-512);
                signal.store(false, Ordering::Relaxed);
                let mut leading = match leading_kind {
                    0 | 1 => Real::one(),
                    _ => {
                        let x = normal::exact_normal_positive();
                        assert_eq!(x.zero_status(), ZeroKnowledge::Unknown);
                        x
                    }
                };
                if leading_kind % 2 == 1 { leading = -leading; }
                let mut left = vec![coefficient; degree + 1];
                left[degree] = leading;
                for shifted in [false, true] {
                    let mut right = left.clone();
                    if shifted { right[0] += Real::one(); }
                    let encoded_left = serde_json::to_string(&left).unwrap();
                    let encoded_right = serde_json::to_string(&right).unwrap();
                    run(&left, &right, degree, shifted); queries += 1;
                    let decoded_left: Vec<Real> = serde_json::from_str(&encoded_left).unwrap();
                    let decoded_right: Vec<Real> = serde_json::from_str(&encoded_right).unwrap();
                    run(&decoded_left, &decoded_right, degree, shifted); queries += 1;
                    let barrier = Barrier::new(4);
                    std::thread::scope(|scope| {
                        let handles: Vec<_> = (0..4).map(|worker| {
                            let (left, right, barrier) = (&left, &right, &barrier);
                            scope.spawn(move || {
                                barrier.wait();
                                for iteration in 0..8 {
                                    // Shared approximation/fact caches and independent
                                    // deserializations exercise different ownership histories.
                                    let floor = [-32, -128, -512][(worker + iteration) % 3];
                                    let _ = left[0].certified_sign_until(floor);
                                    if iteration % 2 == 0 {
                                        let a: Vec<Real> = serde_json::from_str(&serde_json::to_string(left).unwrap()).unwrap();
                                        let b: Vec<Real> = serde_json::from_str(&serde_json::to_string(right).unwrap()).unwrap();
                                        run(&a, &b, degree, shifted);
                                    } else { run(left, right, degree, shifted); }
                                }
                            })
                        }).collect();
                        for handle in handles { handle.join().unwrap(); }
                    });
                    queries += 32;
                    assert_eq!(serde_json::to_string(&left).unwrap(), encoded_left);
                    assert_eq!(serde_json::to_string(&right).unwrap(), encoded_right);
                    serializations += 2;
                }
            }
        }
    }
    // Earlier uncertainty must not prevent later valid precision from changing
    // an unknown-leading report into a certified degree.
    let value = corpus::value(6);
    let polynomial = [Real::one(), value];
    assert!(subresultant_chain_univariate_polynomials(&polynomial, &polynomial, -64).is_err());
    assert_eq!(polynomial[1].certified_sign_until(-2304).sign(), Some(RealSign::Negative));
    run(&polynomial, &polynomial, 1, false);
    queries += 2;
    println!("{{\"suite\":\"polynomial-facts-state\",\"queries\":{queries},\"unchanged_serializations\":{serializations},\"workers_per_case\":4}}");
}
