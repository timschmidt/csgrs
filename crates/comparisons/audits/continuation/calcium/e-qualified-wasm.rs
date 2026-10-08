use hyperreal::{Computable, Rational};
use num::{BigInt, bigint::Sign};
use std::hint::black_box;
use std::sync::Mutex;

#[allow(dead_code)]
mod kernel {
    include!(concat!(env!("CARGO_MANIFEST_DIR"), "/kernel.rs"));
}

static OUTPUT: Mutex<Vec<u32>> = Mutex::new(Vec::new());

#[unsafe(no_mangle)]
pub extern "C" fn plan(p: i32) -> u32 {
    assert!((-262_144..=16).contains(&p));
    kernel::e_terms_for_precision(p)
}

#[unsafe(no_mangle)]
pub extern "C" fn evaluate(p: i32, route: u32) -> u32 {
    assert!((-262_144..=16).contains(&p));
    let value: BigInt = match route {
        0 => kernel::e(p),
        1 => Computable::e().approx(p),
        2 => Computable::pi().approx(p),
        3 => Computable::rational(Rational::one()).exp().approx(p),
        _ => panic!("bounded route"),
    };
    let (sign, words) = value.to_u32_digits();
    assert_ne!(sign, Sign::Minus);
    let count = u32::try_from(words.len()).unwrap();
    *OUTPUT.lock().unwrap() = words;
    count
}

#[unsafe(no_mangle)]
pub extern "C" fn answer_word(index: u32) -> u32 {
    OUTPUT
        .lock()
        .unwrap()
        .get(index as usize)
        .copied()
        .unwrap_or(0)
}

#[unsafe(no_mangle)]
pub extern "C" fn time_loop(p: i32, route: u32, count: u32) -> u32 {
    assert!((-262_144..=16).contains(&p));
    assert!((1..=1_048_576).contains(&count));
    let public = if route == 3 {
        Computable::pi()
    } else {
        Computable::e()
    };
    let mut result = 0;
    for _ in 0..count {
        match route {
            0 => result ^= black_box(kernel::e_terms_for_precision(black_box(p))),
            1 => drop(black_box(kernel::e(black_box(p)))),
            2 | 3 => drop(black_box(public.approx(black_box(p)))),
            _ => panic!("bounded route"),
        }
    }
    result
}
