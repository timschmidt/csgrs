use hyperlimit::PredicatePolicy;
use hyperreal::Real;
use hypersolve::square_free_part;
use std::{hint::black_box, time::Instant};

fn polynomial(kind: usize, code: usize) -> Vec<Real> {
    let pair = match kind {
        0 => [Real::one(), Real::from(2)],
        1 => { let r = Real::from(2).sqrt().unwrap(); [r.clone(), -r] },
        2 => { let r = Real::from(2).ln().unwrap(); [r.clone(), r + Real::one()] },
        _ => panic!("invalid kind"),
    };
    let mut p = vec![Real::from(if code % 2 == 0 { 2 } else { -2 })];
    let mut digits = code;
    for root in [Real::zero(), pair[0].clone(), pair[1].clone()] {
        let exponent = digits % 3; digits /= 3;
        for _ in 0..exponent {
            let mut next = vec![Real::zero(); p.len() + 1];
            for (i, coefficient) in p.iter().enumerate() {
                next[i] -= coefficient * &root;
                next[i+1] += coefficient;
            }
            p = next;
        }
    }
    p
}

fn query(p: Vec<Real>, expected_degree: usize) -> bool {
    match black_box(square_free_part(black_box(p), PredicatePolicy::STRICT)) {
        Some(result) => {
            assert_eq!(result.len(), expected_degree+1);
            true
        }
        None => false,
    }
}

// Counters are observed outside timed work. CPU builds do not install an
// instrumented allocator; allocation-build clocks are never CPU evidence.
fn run(counters: Option<fn(bool) -> [usize; 4]>) {
    let args: Vec<_> = std::env::args().collect();
    assert_eq!(args.len(), 5);
    let kind: usize = args[1].parse().unwrap();
    let code: usize = args[2].parse().unwrap();
    let lifecycle = &args[3];
    let iterations: usize = args[4].parse().unwrap();
    assert!(kind < 3 && code < 27 && iterations > 0);
    assert!(["fresh", "retained"].contains(&lifecycle.as_str()));
    let expected_degree = [code % 3, (code / 3) % 3, (code / 9) % 3]
        .iter().filter(|&&e| e != 0).count();
    let expected_known = !(kind == 2 && [7,8,15,16,17,19,20,21,22,23,24,25,26].contains(&code));
    let p = polynomial(kind, code);
    for _ in 0..8 { assert_eq!(query(p.clone(), expected_degree), expected_known); }
    let before = counters.map_or([0;4], |f| f(true));
    let start = Instant::now();
    let mut known = 0;
    for _ in 0..iterations {
        let p = if lifecycle == "fresh" { polynomial(kind, code) } else { p.clone() };
        known += usize::from(query(p, expected_degree));
    }
    let elapsed_ns = start.elapsed().as_nanos();
    let after = counters.map_or([0;4], |f| f(false));
    assert_eq!(known, if expected_known { iterations } else { 0 });
    let requests = after[0] - before[0];
    let requested_bytes = after[1] - before[1];
    let live_delta = after[2] as i128 - before[2] as i128;
    let peak_delta = after[3].saturating_sub(before[2]);
    let mode = if counters.is_some() { "allocation" } else { "cpu" };
    println!("{{\"mode\":\"{mode}\",\"kind\":{kind},\"code\":{code},\"lifecycle\":\"{lifecycle}\",\"iterations\":{iterations},\"known\":{known},\"degree\":{expected_degree},\"elapsed_ns\":{elapsed_ns},\"requests\":{requests},\"requested_bytes\":{requested_bytes},\"live_delta\":{live_delta},\"peak_delta\":{peak_delta}}}");
}
