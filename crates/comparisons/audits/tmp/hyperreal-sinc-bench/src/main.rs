use std::hint::black_box;
use std::time::{Duration, Instant};

fn old_opaque_zero() -> hyperreal_old::Real {
    let angle = hyperreal_old::Real::e();
    let sine = angle.clone().sin();
    let cosine = angle.cos();
    sine.clone() * sine + cosine.clone() * cosine - hyperreal_old::Real::one()
}

fn new_opaque_zero() -> hyperreal_new::Real {
    let angle = hyperreal_new::Real::e();
    let sine = angle.clone().sin();
    let cosine = angle.cos();
    sine.clone() * sine + cosine.clone() * cosine - hyperreal_new::Real::one()
}

fn run(name: &str, iterations: u32, mut operation: impl FnMut()) -> Duration {
    let start = Instant::now();
    for _ in 0..iterations {
        operation();
    }
    let elapsed = start.elapsed();
    println!(
        "{name}: {:.3} us/op ({iterations} iterations)",
        elapsed.as_secs_f64() * 1e6 / f64::from(iterations)
    );
    elapsed
}

fn main() {
    println!("old outcome: {:?}", old_opaque_zero().sinc().map(|_| "ok"));
    println!("new outcome: {:?}", new_opaque_zero().sinc().map(|_| "ok"));

    let iterations = 1_000;
    run("old cold sinc", iterations, || {
        let _ = black_box(old_opaque_zero().sinc());
    });
    run("new cold sinc", iterations, || {
        let _ = black_box(new_opaque_zero().sinc());
    });
    run("old cold sinc_pi", iterations, || {
        let _ = black_box(old_opaque_zero().sinc_pi());
    });
    run("new cold sinc_pi", iterations, || {
        let _ = black_box(new_opaque_zero().sinc_pi());
    });
    run("old cold cosc", iterations, || {
        let _ = black_box(old_opaque_zero().cosc());
    });
    run("new cold cosc", iterations, || {
        let _ = black_box(new_opaque_zero().cosc());
    });

    let old = old_opaque_zero();
    let new = new_opaque_zero();
    run("old shared sinc", iterations, || {
        let _ = black_box(old.clone().sinc());
    });
    run("new shared sinc", iterations, || {
        let _ = black_box(new.clone().sinc());
    });

    run("new cold sinc plus f64", iterations, || {
        let value = new_opaque_zero().sinc().expect("new removable limit");
        black_box(value.to_f64_lossy());
    });
}
