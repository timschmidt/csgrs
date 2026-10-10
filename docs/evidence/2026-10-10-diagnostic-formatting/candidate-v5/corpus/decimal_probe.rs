use hyperreal::Real;
use std::io::{self, BufRead};
use std::sync::{Arc, atomic::AtomicBool};

fn scalar(token: &str) -> Real {
    token.parse().unwrap()
}

fn unresolved_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    &sine * &sine + &cosine * &cosine - Real::one()
}

fn main() {
    let legacy = std::env::args().nth(1).as_deref() == Some("legacy");
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        let fields: Vec<_> = line.split('\t').collect();
        let [id, family, token, places, floor, preparation] = fields[..] else {
            panic!("invalid row");
        };
        let places: usize = places.parse().unwrap();
        let floor = floor.parse().unwrap();
        let mut value = match family {
            "rational" => scalar(token),
            "pi" | "aborted" => Real::pi(),
            "e" => Real::e(),
            "sqrt_two" => Real::from(2).sqrt().unwrap(),
            "ln_two" => Real::from(2).ln().unwrap(),
            "scaled_pi" => Real::pi() * scalar(token),
            "unresolved" => unresolved_zero() + scalar(token),
            _ => panic!("unknown family"),
        };
        if preparation == "prepared" {
            assert!(value.certified_rational_interval(-512).is_some());
        }
        if family == "aborted" {
            value.abort(Arc::new(AtomicBool::new(true)));
        }
        if legacy {
            assert_eq!(family, "rational");
            println!("{id}\tok\t{value:#.places$}");
        } else {
            match value.try_decimal(places, floor) {
                Ok(text) => println!("{id}\tok\t{text}"),
                Err(problem) => println!("{id}\terror\t{problem:?}"),
            }
        }
    }
}
