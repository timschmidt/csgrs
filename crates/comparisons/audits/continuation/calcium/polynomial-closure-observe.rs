use hyperreal::Real;
use hyperlimit::{PredicatePolicy, compare_reals};
use hypersolve::square_free_part;
use std::cmp::Ordering;
fn product(a: &[Real], b: &[Real]) -> Vec<Real> {
    let mut out = vec![Real::zero(); a.len() + b.len() - 1];
    for (i, x) in a.iter().enumerate() { for (j, y) in b.iter().enumerate() { out[i + j] += x * y; } }
    out
}
fn status(a: &Real, b: &Real) -> &'static str {
    match compare_reals(a, b, PredicatePolicy::STRICT).value() {
        Some(Ordering::Equal) => "Equal", Some(_) => "NotEqual", None => "Unknown",
    }
}
fn main() {
    let mut known = 0;
    let mut blocked = 0;
    let mut degree_failures = 0;
    println!("kind,code,expected_degree,outcome,result_degree,coordinate_statuses,projective_statuses,root_statuses");
    for kind in 0..3 { for code in 0..27 {
        let pair = match kind {
            0 => [Real::one(), Real::from(2)],
            1 => { let x = Real::from(2).sqrt().unwrap(); [x.clone(), -x] },
            _ => { let x = Real::from(2).ln().unwrap(); [x.clone(), x + Real::one()] },
        };
        let roots = [Real::zero(), pair[0].clone(), pair[1].clone()];
        let mut selected = Vec::new();
        let mut p = vec![Real::from(if code % 2 == 0 { 2 } else { -2 })];
        let mut expected = p.clone(); let mut digits = code;
        for root in roots {
            let exponent = digits % 3; digits /= 3;
            let factor = [-root.clone(), Real::one()];
            for _ in 0..exponent { p = product(&p, &factor); }
            if exponent != 0 { expected = product(&expected, &factor); selected.push(root); }
        }
        let expected_degree = expected.len() - 1;
        if let Some(actual) = square_free_part(p, PredicatePolicy::STRICT) {
            let coordinates: Vec<_> = actual.iter().zip(&expected).map(|(a,b)| status(a,b)).collect();
            let projective: Vec<_> = actual.iter().zip(&expected).map(|(a,b)| status(&(a * expected.last().unwrap()), &(b * actual.last().unwrap()))).collect();
            let residuals: Vec<_> = selected.iter().map(|r| status(&Real::eval_poly(&actual, r), &Real::zero())).collect();
            println!("{kind},{code},{expected_degree},Known,{},{},{},{}", actual.len() - 1, coordinates.join(";"), projective.join(";"), residuals.join(";"));
            if kind == 2 && code == 5 {
                eprintln!("case2/5 actual={actual:?}\nexpected={expected:?}");
            }
            known += 1; degree_failures += usize::from(actual.len() != expected.len());
        } else {
            println!("{kind},{code},{expected_degree},Unknown,-1,,,"); blocked += 1;
        }
    } }
    println!("{{\"suite\":\"polynomial-closure-observe\",\"cases\":81,\"known\":{known},\"blocked\":{blocked},\"degree_failures\":{degree_failures}}}");
}
