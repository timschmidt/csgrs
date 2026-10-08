use boehm_reals::evaluation::bounded_rational::BoundedRational as B;
use hyperreal::Rational as Q;
use num::{BigInt, BigRational, One};
use std::{hint::black_box, time::Instant};

fn from_b(b: &B) -> BigRational {
    BigRational::new(b.numerator().to_string().parse().unwrap(), b.denominator().to_string().parse().unwrap())
}
fn from_h(h: &Q) -> BigRational {
    BigRational::new(BigInt::from_biguint(h.sign(), h.numerator().clone()), h.denominator().clone().into())
}
fn main() {
    let args: Vec<_> = std::env::args().collect();
    let library = args[1].as_str();
    let cold = args[2].ends_with("_cold");
    let case = args[2].strip_suffix("_cold").unwrap_or(&args[2]);
    let count: usize = args[3].parse().unwrap();
    let large = (BigInt::one() << 2048_usize) + 1_u32;
    let (x, y) = match case {
        "clone" | "cross_cancel" => (BigRational::new(large.clone(), 3.into()), BigRational::new(3.into(), large)),
        "add_small" | "mul_small" | "cmp_small" => (BigRational::new(2.into(), 7.into()), BigRational::new(3.into(), 11.into())),
        "same_denominator" => {
            let d = (BigInt::one() << 61_usize) - 1_u32;
            (BigRational::new(17.into(), d.clone()), BigRational::new(19.into(), d))
        }
        "add_large" | "cmp_large" => (BigRational::new(large.clone(), 3.into()), BigRational::new(large + 2, 5.into())),
        _ => panic!("case"),
    };
    let bx = B::new(x.numer().to_string().parse().unwrap(), x.denom().to_string().parse().unwrap()).unwrap();
    let by = B::new(y.numer().to_string().parse().unwrap(), y.denom().to_string().parse().unwrap()).unwrap();
    let hx = Q::from_bigint_fraction(x.numer().clone(), x.denom().to_biguint().unwrap()).unwrap();
    let hy = Q::from_bigint_fraction(y.numer().clone(), y.denom().to_biguint().unwrap()).unwrap();
    let operation = if case == "clone" { 0 } else if case.starts_with("cmp") { 1 } else if case == "cross_cancel" || case == "mul_small" { 2 } else { 3 };
    match operation {
        0 => { assert_eq!(from_b(&bx.clone()), x); assert_eq!(from_h(&hx.clone()), x); }
        1 => { assert_eq!(bx.compare_to(&by), x.cmp(&y)); assert_eq!(hx.partial_cmp(&hy), Some(x.cmp(&y))); }
        2 => { assert_eq!(from_b(&B::multiply(bx.clone(), by.clone())), &x * &y); assert_eq!(from_h(&(&hx * &hy)), &x * &y); }
        _ => { assert_eq!(from_b(&(&bx + &by)), &x + &y); assert_eq!(from_h(&(&hx + &hy)), &x + &y); }
    }
    let evaluate = || {
        if cold {
            if library == "donor" {
                let a = B::new(black_box(&bx).numerator().clone(), bx.denominator().clone()).unwrap();
                let b = B::new(black_box(&by).numerator().clone(), by.denominator().clone()).unwrap();
                match operation {
                    1 => { black_box(a.compare_to(&b)); }
                    2 => { black_box(B::multiply(a, b)); }
                    _ => { black_box(a + b); }
                }
            } else {
                let a = Q::from_bigint_fraction(black_box(&x).numer().clone(), x.denom().to_biguint().unwrap()).unwrap();
                let b = Q::from_bigint_fraction(black_box(&y).numer().clone(), y.denom().to_biguint().unwrap()).unwrap();
                match operation {
                    1 => { black_box(a.partial_cmp(&b)); }
                    2 => { black_box(a * b); }
                    _ => { black_box(a + b); }
                }
            }
            return;
        }
        match (library, operation) {
            ("donor", 0) => { black_box(black_box(&bx).clone()); }
            ("hyper", 0) => { black_box(black_box(&hx).clone()); }
            ("donor", 1) => { black_box(black_box(&bx).compare_to(black_box(&by))); }
            ("hyper", 1) => { black_box(black_box(&hx).partial_cmp(black_box(&hy))); }
            ("donor", 2) => { black_box(B::multiply(black_box(&bx).clone(), black_box(&by).clone())); }
            ("hyper", 2) => { black_box(black_box(&hx) * black_box(&hy)); }
            ("donor", 3) => { black_box(black_box(&bx) + black_box(&by)); }
            ("hyper", 3) => { black_box(black_box(&hx) + black_box(&hy)); }
            _ => panic!("library"),
        }
    };
    for _ in 0..count.min(100) { evaluate(); }
    let start = Instant::now();
    for _ in 0..count { evaluate(); }
    println!("{:.3}", start.elapsed().as_nanos() as f64 / count as f64);
}
