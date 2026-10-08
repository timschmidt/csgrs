use hyperreal::{Real, Rational};
use std::{hint::black_box, time::Instant};
#[path = "../cpu_clock.rs"] mod cpu_clock;

fn expression(n: usize, step: i32, reverse: bool) -> Real {
    let factor = if step >= 0 { Real::new(Rational::fraction(1, 1_u64 << step).unwrap()) }
        else { Real::from(1_u64 << -step) };
    let mut prefix = Real::zero(); let mut scale = Real::one();
    let mut values: Vec<_> = (1..=n).map(|k| {
        prefix += Real::new(Rational::fraction(((k + 1) % 97 + 1) as i64, (k % 29 + 31) as u64).unwrap()).sin();
        let value = if step == 0 {prefix.clone()} else {&prefix * &scale};
        scale *= &factor;
        value
    }).collect();
    if reverse { values.reverse(); }
    values.into_iter().sum()
}

fn main() {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let n: usize = args[0].parse().unwrap();
    let step: i32 = args[1].parse().unwrap();
    let order = args.get(2).map(String::as_str).unwrap_or("forward");
    assert!(order == "forward" || order == "reverse");
    let phase = args.get(3).map(String::as_str).unwrap_or("bench");
    let bits: i32 = args.get(4).map(|s| s.parse().unwrap()).unwrap_or(128);
    assert!((1..=512).contains(&bits));
    assert!((-8..=8).contains(&step));
    let run = || black_box(expression(black_box(n), black_box(step), black_box(order == "reverse")).certified_dyadic_interval(-bits).unwrap());
    let initial = run();
    assert!(initial[0] > Rational::zero());
    let max_width = Rational::from_bigint_fraction(2.into(), Rational::one().denominator() << bits as usize).unwrap();
    assert!(&initial[1] - &initial[0] <= max_width);
    if phase == "oracle" {
        let lower = format!("{}/{}", initial[0].numerator(), initial[0].denominator());
        let upper = format!("{}/{}", initial[1].numerator(), initial[1].denominator());
        let status = std::process::Command::new("/tmp/ireal-audit.Sc4TzO/scaled-sum-oracle")
            .args([&n.to_string(), &step.to_string(), "1", &lower, &upper]).status().unwrap();
        assert!(status.success(), "independent scaled finite-sum enclosure failed");
        println!("PASS\t{n}\t{step}\t{order}\t{bits}");
        return;
    }
    if phase == "once" { run(); return; }
    assert_eq!(phase, "bench");
    let wall = Instant::now(); let start = cpu_clock::now_ns(); let mut count = 0_u64;
    let end;
    loop {
        run(); count += 1;
        let now = cpu_clock::now_ns();
        if now - start >= 200_000_000 { end = now; break; }
    }
    println!("{n}\t{step}\t{order}\t{bits}\t{count}\t{:.3}\t{:.3}",(end-start) as f64 / count as f64, wall.elapsed().as_nanos() as f64 / count as f64);
}
