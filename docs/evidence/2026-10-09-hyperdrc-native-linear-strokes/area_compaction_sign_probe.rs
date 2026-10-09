//! Check whether reconstruction preserves an already certified near-zero sign.
use hyperreal::{Real, RealSign};

fn main() {
    let pi = Real::pi();
    let lower = pi.certified_rational_interval(-1024).unwrap()[0].clone();
    let delta = &pi - Real::new(lower);
    assert_eq!(
        delta.certified_sign_until(-1100).sign(),
        Some(RealSign::Positive)
    );
    let square = &delta * &delta;
    for scale in [-7, 1] {
        let value = &square * Real::from(scale);
        let before = value.certified_sign_until(0).sign();
        let compact = value.compact_pi_laurent().unwrap();
        println!(
            "scale={scale}\tbefore={before:?}\timmediate={:?}\tafter_0={:?}\tafter_512={:?}",
            compact.immediate_sign(),
            compact.certified_sign_until(0).sign(),
            compact.certified_sign_until(-512).sign(),
        );
    }
}
