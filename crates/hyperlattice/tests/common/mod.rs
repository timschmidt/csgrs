#![allow(dead_code)]

use realistic_blas::{Real, sin};

pub fn r(value: i32) -> Real {
    value.into()
}

pub fn unknown_zero() -> Real {
    let one = r(1);
    sin(one.clone()) - sin(one)
}
