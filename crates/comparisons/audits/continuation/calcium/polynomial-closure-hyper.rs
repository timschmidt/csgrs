use hyperreal::Real;
use hyperlimit::{PredicatePolicy, compare_reals};
use hypersolve::square_free_part;
use std::cmp::Ordering;

fn product(a: &[Real], b: &[Real]) -> Vec<Real> {
    let mut out = vec![Real::zero(); a.len() + b.len() - 1];
    for (i, x) in a.iter().enumerate() {
        for (j, y) in b.iter().enumerate() { out[i + j] += x * y; }
    }
    out
}
fn main() {
    let mut decided = 0;
    let mut unknown = 0;
    println!("kind,code,degree,outcome,result_degree,equal");
    for kind in 0..3 {
        for code in 0..27 {
            let pair = match kind {
                0 => [Real::one(), Real::from(2)],
                1 => { let x = Real::from(2).sqrt().unwrap(); [x.clone(), -x] },
                _ => { let x = Real::from(2).ln().unwrap(); [x.clone(), x + Real::one()] },
            };
            let roots = [Real::zero(), pair[0].clone(), pair[1].clone()];
            let mut p = vec![Real::from(if code % 2 == 0 { 2 } else { -2 })];
            let mut expected = p.clone();
            let mut digits = code;
            for root in roots {
                let exponent = digits % 3; digits /= 3;
                let factor = [-root, Real::one()];
                for _ in 0..exponent { p = product(&p, &factor); }
                if exponent != 0 { expected = product(&expected, &factor); }
            }
            let degree = p.len() - 1;
            if let Some(actual) = square_free_part(p, PredicatePolicy::STRICT) {
                let equal = actual.len() == expected.len() && actual.iter().zip(&expected).all(|(a,b)|
                    compare_reals(a, b, PredicatePolicy::STRICT).value() == Some(Ordering::Equal));
                assert!(equal, "kind={kind}, code={code}");
                println!("{kind},{code},{degree},Known,{},true", actual.len() - 1);
                decided += 1;
            } else {
                println!("{kind},{code},{degree},Unknown,-1,false"); unknown += 1;
            }
        }
    }
    println!("{{\"suite\":\"polynomial-closure-hyper\",\"cases\":81,\"decided\":{decided},\"unknown\":{unknown}}}");
}
