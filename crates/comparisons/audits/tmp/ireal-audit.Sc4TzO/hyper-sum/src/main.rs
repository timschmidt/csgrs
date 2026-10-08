use hyperreal::{Rational, Real};
use std::{hint::black_box, time::Instant};
mod cpu_clock;

fn balanced(values: &[Real], reverse: bool) -> Real {
    match values.len() {
        0 => Real::zero(),
        1 => values[0].clone(),
        n => {
            let (left, right) = values.split_at(n / 2);
            if reverse { balanced(right, reverse) + balanced(left, reverse) }
            else { balanced(left, reverse) + balanced(right, reverse) }
        }
    }
}

fn expression(mode: &str, dependent: bool, n: usize, seed: u64) -> Real {
    let mut prefix = Real::zero();
    let inputs: Vec<_> = (1..=n).map(|k| {
        let k = k as u64;
        let value = Real::new(Rational::fraction(((k + seed) % 97 + 1) as i64, k % 29 + 31).unwrap()).sin();
        if dependent { prefix += value; prefix.clone() } else { value }
    }).collect();
    match mode {
        "public" => Real::sum_refs(inputs.iter()),
        "left" => inputs.iter().fold(Real::zero(), |sum, value| sum + value),
        "balanced" => balanced(&inputs, false),
        "reverse" => balanced(&inputs, true),
        _ => panic!("unknown mode"),
    }
}

fn main() {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let mode = &args[0]; let shape = &args[1];
    let dependent = shape == "dependent";
    assert!(dependent || shape == "independent");
    let n: usize = args[2].parse().unwrap();
    let bits: i32 = args[3].parse().unwrap();
    assert!((1..=512).contains(&bits));
    let phase = &args[4];
    let retained = expression(mode, dependent, n, black_box(1));
    let first = retained.certified_dyadic_interval(-bits).unwrap();
    let width = (&first[1] - &first[0]).to_string();
    assert!(first[0] > Rational::zero());
    if phase == "oracle" {
        let lower = format!("{}/{}", first[0].numerator(), first[0].denominator());
        let upper = format!("{}/{}", first[1].numerator(), first[1].denominator());
        let status = std::process::Command::new("/tmp/ireal-audit.Sc4TzO/sum-oracle-cli")
            .args([if dependent {"1"} else {"0"}, &n.to_string(), "1", &lower, &upper])
            .status().unwrap();
        assert!(status.success(), "independent MPFR finite-sum enclosure failed");
        println!("PASS\t{mode}\t{shape}\t{n}\t{bits}\twidth={width}");
        return;
    }
    let run = || {
        let cold;
        let input = if phase == "cold" || phase == "once" {
            cold = expression(black_box(mode), black_box(dependent), black_box(n), black_box(1));
            &cold
        } else { &retained };
        black_box(input.certified_dyadic_interval(black_box(-bits)).unwrap());
    };
    if phase == "once" { run(); return; }
    assert!(phase == "cold" || phase == "warm");
    let wall_start = Instant::now(); let start = cpu_clock::now_ns(); let mut count = 0_u64;
    let end;
    loop {
        run(); count += 1;
        if phase == "cold" || count % 64 == 0 {
            let now = cpu_clock::now_ns();
            if now - start >= 200_000_000 { end = now; break; }
        }
    }
    println!("{mode}\t{shape}\t{n}\t{bits}\t{phase}\t{count}\t{:.3}\t{:.3}", (end-start) as f64 / count as f64, wall_start.elapsed().as_nanos() as f64 / count as f64);
}
