mod corpus {
    include!("polynomial-decision-probe.rs");
    pub fn value(kind: usize) -> hyperreal::Real { coefficient(kind) }
}
use hyperreal::Real;
use hypersolve::subresultant_chain_univariate_polynomials;
use std::{hint::black_box, time::Instant};

fn input(case: &str, degree: usize) -> (Vec<Real>, Vec<Real>) {
    let kind = match case {
        "rational-self" => 1,
        "radical-self" => 2,
        "log-self" | "log-plus-one" | "unknown-leading" => 3,
        "tiny-self" => 6,
        _ => panic!("unknown case"),
    };
    let mut p = vec![corpus::value(kind); degree + 1];
    if case != "unknown-leading" { p[degree] = Real::one(); }
    let mut q = p.clone();
    if case == "log-plus-one" { q[0] += Real::one(); }
    (p, q)
}

fn query(p: &[Real], q: &[Real], case: &str, degree: usize) -> bool {
    match black_box(subresultant_chain_univariate_polynomials(black_box(p), black_box(q), -64)) {
        Ok(report) => {
            let expected = if case == "log-plus-one" { 0 } else { degree };
            assert_eq!(report.last_nonzero_degree, expected);
            report.has_nonconstant_common_factor == (expected > 0)
        }
        Err(_) => false,
    }
}

fn main() {
    let args: Vec<_> = std::env::args().collect();
    let case = &args[1];
    let degree: usize = args[2].parse().unwrap();
    let lifecycle = &args[3];
    let iterations: usize = args[4].parse().unwrap();
    assert!(iterations > 0 && ["fresh", "retained"].contains(&lifecycle.as_str()));
    let (p, q) = input(case, degree);
    for _ in 0..8 { black_box(query(&p, &q, case, degree)); }
    let start = Instant::now();
    let mut known = 0;
    for _ in 0..iterations {
        let decided = if lifecycle == "fresh" {
            let (p, q) = input(case, degree);
            query(&p, &q, case, degree)
        } else { query(&p, &q, case, degree) };
        known += usize::from(decided);
    }
    let elapsed_ns = start.elapsed().as_nanos();
    println!("{{\"case\":\"{case}\",\"degree\":{degree},\"lifecycle\":\"{lifecycle}\",\"iterations\":{iterations},\"known\":{known},\"elapsed_ns\":{elapsed_ns}}}");
}
