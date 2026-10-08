use hyperreal::{Real, Rational};
use std::{hint::black_box, time::Instant};
#[path = "../cpu_clock.rs"] mod cpu_clock;

fn main() {
    let kind = std::env::args().nth(1).unwrap();
    let inputs: Vec<_> = (1..=256).map(|k| Real::new(Rational::fraction(k, 317).unwrap()).sin()).collect();
    let rational = Real::new(Rational::fraction(1, 3).unwrap());
    let pi = Real::pi(); let e = Real::e();
    let n = kind.strip_prefix("sum").map(|s| s.parse::<usize>().unwrap());
    let run = || {
        let value = match kind.as_str() {
            "rational-add" => black_box(&rational) + black_box(&rational),
            "symbolic-add" => black_box(&pi) + black_box(&e),
            "generic-add" => black_box(&inputs[0]) + black_box(&inputs[1]),
            "generic-mul" => black_box(&inputs[0]) * black_box(&inputs[1]),
            "sin" => black_box(&rational).clone().sin(),
            _ => Real::sum_refs(black_box(&inputs[..n.unwrap()]).iter()),
        };
        black_box(value);
    };
    run();
    let wall = Instant::now(); let start = cpu_clock::now_ns(); let mut count = 0_u64;
    let end;
    loop {
        for _ in 0..256 { run(); }
        count += 256;
        let now = cpu_clock::now_ns();
        if now-start >= 200_000_000 { end=now; break; }
    }
    println!("{kind}\t{count}\t{:.3}\t{:.3}", (end-start) as f64 / count as f64, wall.elapsed().as_nanos() as f64 / count as f64);
}
