use hyperreal::{Computable, Rational};
use rug::{Integer, Rational as Q};
use std::{hint::black_box, time::Instant};

fn input(i: usize) -> (i64,u64) { (1048576 + (i%97) as i64, 3145728) }
#[inline(never)]
fn expression(op: &str, i: usize) -> Computable {
    let (n,d) = input(i);
    let x = Computable::rational(Rational::fraction(n,d).unwrap());
    match op { "sqrt" => x.sqrt(), "exp" => x.exp(), "sin" => x.sin(), _ => panic!("operation") }
}
fn main() {
    let args: Vec<_> = std::env::args().collect();
    if args[1] == "export" {
        for p in [53,200] {
            for op in ["sqrt","exp","sin"] {
                for i in 0..97 {
                    let (n,d) = input(i);
                    let v = expression(op,i).approx(-p).to_string().parse::<Integer>().unwrap();
                    let c = Q::from((v,Integer::from(1)<<p));
                    let e = Q::from((1,Integer::from(1)<<p));
                    let l = c.clone()-&e;
                    let u = c+e;
                    println!("{op} {p} {n} {d} {} {} {} {}",l.numer(),l.denom(),u.numer(),u.denom());
                }
            }
        }
        return;
    }
    let op = &args[1];
    let mode = &args[2];
    let p: i32 = args[3].parse().unwrap();
    let count: usize = args[4].parse().unwrap();
    let shared = expression(op,0);
    let run = |i| {
        let result = if mode == "warm" { black_box(&shared).approx(black_box(-p)) }
                     else { expression(black_box(op), black_box(i)).approx(-p) };
        black_box(result);
    };
    for i in 0..count.min(100) { run(i); }
    let start = Instant::now();
    for i in 0..count { run(i); }
    println!("{:.3}",start.elapsed().as_nanos() as f64/count as f64);
}
