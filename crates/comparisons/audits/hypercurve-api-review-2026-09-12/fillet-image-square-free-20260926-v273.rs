use hyperreal::Real;
use hyperlimit::PredicatePolicy;
use std::time::Instant;
fn main() {
    let text=std::fs::read_to_string(std::env::args().nth(1).expect("fixture path")).unwrap();
    let mut lines=text.lines();
    assert_eq!(lines.next(),Some("HPOLY1"));
    let count:usize=lines.next().unwrap().parse().unwrap();
    let coefficients=(0..count).map(|_|Real::from_json(lines.next().unwrap()).unwrap()).collect::<Vec<_>>();
    assert!(lines.next().is_none());
    println!("image replay admitted degree={} rationals={}",coefficients.len()-1,coefficients.iter().filter(|c|c.exact_rational_ref().is_some()).count());
    let start=Instant::now();
    let result=hypersolve::square_free_part(coefficients.clone(),PredicatePolicy::STRICT);
    println!("square_free_degree={:?} unchanged={} elapsed={:?}",result.as_ref().map(|p|p.len().saturating_sub(1)),result.as_ref()==Some(&coefficients),start.elapsed());
}
