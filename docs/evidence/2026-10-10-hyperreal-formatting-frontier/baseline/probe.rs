use hyperreal::Real;
use std::fmt::Write;

fn unresolved_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    &sine * &sine + &cosine * &cosine - Real::one()
}

fn value(family: &str) -> Real {
    match family {
        "zero" => Real::zero(),
        "rational" => "7/8".parse().unwrap(),
        "pi" => Real::pi(),
        "sqrt_two" => Real::from(2).sqrt().unwrap(),
        "tiny" => "1e-400".parse().unwrap(),
        "negative_tiny" => "-1e-400".parse().unwrap(),
        "huge" => "1e400".parse().unwrap(),
        "unresolved_zero" => unresolved_zero(),
        "unresolved_delta" => (Real::one() + unresolved_zero()) - Real::one(),
        "original_hypot" => {
            let delta = (Real::one() + unresolved_zero()) - Real::one();
            Real::hypot2(&delta, &Real::zero()).unwrap()
        },
        "positive_offset" => unresolved_zero() + "1e-40".parse::<Real>().unwrap(),
        "negative_offset" => unresolved_zero() - "1e-40".parse::<Real>().unwrap(),
        _ => panic!("unknown family"),
    }
}

fn main() {
    let args: Vec<_> = std::env::args().collect();
    assert_eq!(args.len(), 4);
    eprintln!("stage=constructing");
    let value = value(&args[1]);
    eprintln!("stage=constructed");
    match args[2].as_str() {
        "fresh" => {},
        "prepared" => {
            eprintln!("stage=preparing precision=-128");
            let available = value.certified_rational_interval(-128).is_some();
            eprintln!("stage=prepared interval_available={available}");
        },
        _ => panic!("unknown preparation"),
    }
    eprintln!("stage=formatting");
    let mut output = String::new();
    let result = match args[3].as_str() {
        "fixed" => write!(output, "{:.6}", value.fixed_decimal()),
        "alternate" => write!(output, "{value:#.6}"),
        "lower" => write!(output, "{value:.6e}"),
        "upper" => write!(output, "{value:.6E}"),
        _ => panic!("unknown format"),
    };
    eprintln!("stage=returned result={result:?}");
    println!("{output}");
}
