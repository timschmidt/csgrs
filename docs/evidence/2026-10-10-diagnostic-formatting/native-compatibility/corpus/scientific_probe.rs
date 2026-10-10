use hyperreal::{Real, ScientificCase};
use std::sync::Arc;

const FAMILIES: [&str; 18] = [
    "zero",
    "rational",
    "pi",
    "sqrt_two",
    "tiny",
    "negative_tiny",
    "huge",
    "unresolved_zero",
    "unresolved_delta",
    "original_hypot",
    "positive_offset",
    "negative_offset",
    "pi_scaled_huge",
    "pi_scaled_tiny",
    "e",
    "ln_two",
    "radical_sum",
    "rounding_tie",
];

fn scalar(text: &str) -> Real {
    text.parse().unwrap()
}

fn unresolved_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    &sine * &sine + &cosine * &cosine - Real::one()
}

fn value(family: &str) -> Real {
    match family {
        "zero" => Real::zero(),
        "rational" => scalar("7/8"),
        "pi" => Real::pi(),
        "sqrt_two" => Real::from(2).sqrt().unwrap(),
        "tiny" => scalar("1e-400"),
        "negative_tiny" => scalar("-1e-400"),
        "huge" => scalar("1e400"),
        "unresolved_zero" => unresolved_zero(),
        "unresolved_delta" => (Real::one() + unresolved_zero()) - Real::one(),
        "original_hypot" => {
            let delta = (Real::one() + unresolved_zero()) - Real::one();
            Real::hypot2(&delta, &Real::zero()).unwrap()
        },
        "positive_offset" => unresolved_zero() + scalar("1e-40"),
        "negative_offset" => unresolved_zero() - scalar("1e-40"),
        "pi_scaled_huge" => Real::pi() * scalar("1e400"),
        "pi_scaled_tiny" => Real::pi() * scalar("1e-400"),
        "e" => Real::e(),
        "ln_two" => Real::from(2).ln().unwrap(),
        "radical_sum" => Real::from(2).sqrt().unwrap() + Real::from(3).sqrt().unwrap(),
        "rounding_tie" => unresolved_zero() + scalar("1.234565"),
        _ => panic!("unknown family"),
    }
}

fn row(
    value: &Real,
    family: &str,
    stage: &str,
    places: usize,
    floor: i32,
    case: ScientificCase,
) -> String {
    let result = match value.try_scientific(places, floor, case) {
        Ok(text) => format!("{{\"kind\":\"ok\",\"text\":\"{text}\"}}"),
        Err(problem) => format!("{{\"kind\":\"error\",\"problem\":\"{problem:?}\"}}"),
    };
    format!(
        "{{\"family\":\"{family}\",\"stage\":\"{stage}\",\"places\":{places},\"floor\":{floor},\"case\":\"{case:?}\",\"result\":{result}}}"
    )
}

fn main() {
    let reverse = std::env::args().nth(1).as_deref() == Some("reverse");
    let mut families = FAMILIES.to_vec();
    if reverse {
        families.reverse();
    }
    for family in families {
        for stage in [
            "fresh",
            "prepared_128",
            "prepared_512",
            "cloned_prepared_512",
        ] {
            for places in [0, 5, 6, 20] {
                for floor in [0, -16, -64, -128, -256, -512] {
                    for case in [ScientificCase::Lower, ScientificCase::Upper] {
                        // Each row gets new query objects. Shared constants and
                        // constructor-internal caches can already be warm.
                        let mut value = value(family);
                        if stage != "fresh" {
                            let precision = if stage == "prepared_128" { -128 } else { -512 };
                            assert!(value.certified_rational_interval(precision).is_some());
                        }
                        if stage == "cloned_prepared_512" {
                            value = value.clone();
                        }
                        println!("{}", row(&value, family, stage, places, floor, case));
                    }
                }
            }
        }
    }
    let shared = Arc::new([
        ("original_hypot", value("original_hypot")),
        ("positive_offset", value("positive_offset")),
        ("negative_offset", value("negative_offset")),
        ("pi_scaled_huge", value("pi_scaled_huge")),
        ("radical_sum", value("radical_sum")),
    ]);
    let mut threads = Vec::new();
    for (index, preparation) in [-16, -128, -512, -256].into_iter().enumerate() {
        let shared = shared.clone();
        threads.push(std::thread::spawn(move || {
            let stage = format!("shared_thread_{index}");
            let mut rows = Vec::new();
            for (family, value) in shared.iter() {
                assert!(value.certified_rational_interval(preparation).is_some());
                for places in [0, 6, 20] {
                    for floor in [-64, -128, -256] {
                        for case in [ScientificCase::Lower, ScientificCase::Upper] {
                            rows.push(row(value, family, &stage, places, floor, case));
                        }
                    }
                }
            }
            rows
        }));
    }
    for thread in threads {
        for row in thread.join().unwrap() {
            println!("{row}");
        }
    }
}
