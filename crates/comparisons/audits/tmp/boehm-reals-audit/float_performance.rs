use hyperreal::{Rational as Q,Real};
use num::{BigInt,BigUint,One};
use std::{hint::black_box,time::Instant};
fn main(){let args:Vec<_>=std::env::args().collect();let case=args[1].as_str();let n:usize=args[2].parse().unwrap();
let q=match case {
"third"=>Q::fraction(1,3).unwrap(),
"word_dyadic"=>Q::fraction(3,8).unwrap(),
"wide_normal"=>Q::from_bigint_fraction((BigInt::one()<<200usize)+1,BigUint::one()<<180usize).unwrap(),
"wide_fraction"=>Q::from_bigint_fraction((BigInt::one()<<200usize)+1,(BigUint::one()<<180usize)+3u32).unwrap(),
"subnormal"=>Q::from_bigint_fraction((BigInt::one()<<54usize)-1,BigUint::one()<<1076usize).unwrap(),
"tiny_fraction"=>Q::from_bigint_fraction(BigInt::one(),(BigUint::one()<<1050usize)*3u32).unwrap(),
"underflow"=>Q::from_bigint_fraction(BigInt::one(),BigUint::one()<<1200usize).unwrap(),
_=>panic!("case")};
let f=||{let r=Real::new(black_box(q.clone()));black_box(r.to_f64_lossy());};for _ in 0..n.min(100){f();}let start=Instant::now();for _ in 0..n{f();}println!("{:.3}",start.elapsed().as_nanos()as f64/n as f64);
}
