//! Fuzz exact numeric text parsing, including decimal scientific notation.

#![no_main]

use hyperreal::{Rational, Real};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let Ok(text) = core::str::from_utf8(data) else {
        return;
    };

    let rational = text.parse::<Rational>();
    let real = text.parse::<Real>();

    if let Ok(rational) = rational {
        assert_eq!(real.expect("Real accepts every Rational literal"), rational);
    }
});
