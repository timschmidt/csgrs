use std::{hint::black_box, time::Instant};

fn time(mut run: impl FnMut(), iterations: usize) {
    for _ in 0..iterations.min(100) { run(); }
    let start = Instant::now();
    for _ in 0..iterations { run(); }
    println!("{:.3}", start.elapsed().as_nanos() as f64 / iterations as f64);
}

macro_rules! bench {
    ($lib:ident, $case:expr, $arg:expr, $iters:expr) => {{
        use $lib::{Computable, Rational, Real};
        match $case {
            "clone" | "cached" => {
                let mut c = Computable::pi();
                for _ in 0..$arg { c = c.add(Computable::pi()); }
                black_box(c.approx(-128));
                if $case == "clone" {
                    time(|| { black_box(c.clone()); }, $iters);
                } else {
                    time(|| { black_box(c.approx(-128)); }, $iters);
                }
            },
            "sqrt_cold" => time(|| {
                let c = Computable::rational(Rational::new(2)).sqrt();
                black_box(c.approx(-($arg as i32)));
            }, $iters),
            "cos_cold" => time(|| {
                let c = Computable::rational(Rational::fraction(1,4).unwrap()).cos();
                black_box(c.approx(-($arg as i32)));
            }, $iters),
            "real_format" => {
                let c = Real::new(Rational::new(2)).sqrt().unwrap() + Real::new(Rational::new(3)).sqrt().unwrap();
                let expected = format!("{c:.*e}", $arg);
                assert!(expected.starts_with("3.146264369941972"));
                time(|| {black_box(format!("{c:.*e}", $arg));}, $iters);
            },
            "layout" => println!("Real={} Rational={} Computable={}", size_of::<Real>(),size_of::<Rational>(),size_of::<Computable>()),
            _ => panic!("unknown benchmark"),
        }
    }};
}

fn main() {
    let args: Vec<_> = std::env::args().collect();
    let arg: usize = args[3].parse().unwrap();
    let iterations: usize = args[4].parse().unwrap();
    match args[1].as_str() {
        "realistic" => bench!(realistic, args[2].as_str(), arg, iterations),
        "hyper" => bench!(hyperreal, args[2].as_str(), arg, iterations),
        _ => panic!("unknown library"),
    }
}
