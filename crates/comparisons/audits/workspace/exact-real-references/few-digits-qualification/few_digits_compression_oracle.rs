use rug::{Float, Integer, float::Round};
use std::{env, fs};

fn main() {
    let p = 8192;
    let mut checks = 0;
    for line in fs::read_to_string(env::args().nth(1).unwrap()).unwrap().lines() {
        let f: Vec<_> = line.split('\t').collect();
        assert_eq!(f.len(), 11);
        let integer = |s: &str| Integer::from_str_radix(s, 10).unwrap();
        let n = Float::with_val(p, integer(f[6]));
        let d = integer(f[7]);
        let mut lo = Float::with_val_round(p, &n / &d, Round::Down).0;
        let mut hi = Float::with_val_round(p, &n / &d, Round::Up).0;
        assert!(lo > 0 && hi < 1); // sin is increasing on this interval.
        lo.sin_round(Round::Down);
        hi.sin_round(Round::Up);
        let got = Float::with_val(p, integer(f[8])) / integer(f[9]); // exact dyadic
        let bits: i32 = f[2].parse().unwrap();
        let eps = Float::with_val(p, 1) >> bits;
        let projection_error = Float::with_val(p, &eps) >> 16;
        let budget = Float::with_val(p, &eps - &projection_error);
        assert!(Float::with_val(p, &got - &lo).abs() <= budget, "{} n={} bits={bits} seed={}", f[0], f[1], f[3]);
        assert!(Float::with_val(p, &got - &hi).abs() <= budget, "{} n={} bits={bits} seed={}", f[0], f[1], f[3]);
        checks += 1;
    }
    println!("PASS {checks} directed-MPFR Fibonacci-ratio sine checks; dyadic reporting error reserved from donor accuracy budget");
}
