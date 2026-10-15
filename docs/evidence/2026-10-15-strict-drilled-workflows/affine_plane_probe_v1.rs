//! The next scalar frontier, reduced from the retained native plane trace.
use hyperreal::{CertifiedRealSign, Real, RealSign};

fn main() {
    let origin = Real::from(6) - Real::from(2).sqrt().unwrap();
    let u = Real::from(6) - Real::from(2) * (Real::pi() / Real::from(8)).unwrap().sin();
    let v = Real::from(6)
        - Real::from(2)
            * ((Real::pi() * Real::from(3)) / Real::from(8))
                .unwrap()
                .sin();
    let nx = Real::from(6) * (&v - &origin);
    let ny = Real::from(-6) * (&u - &origin);
    let offset = -Real::dot2_refs([&nx, &ny], [&origin, &origin]);
    let one = Real::one();
    let value = Real::signed_product_sum([true; 3], [[&nx, &u], [&ny, &v], [&offset, &one]]);
    // Expanding h(v-o)u - h(u-o)v - [h(v-o)o-h(u-o)o] cancels every term.
    // These are frontier observations, not assertions of completed geometry.
    println!(
        "affine_zero_normal_form={:?}",
        value.exact_rational_normal_form()
    );
    for precision in [0, -512, 0] {
        println!(
            "affine_zero_sign_{precision}={:?}",
            value.certified_sign_until(precision)
        );
    }
    let shifted = value + (Real::one() / Real::from(1_000_000_000_000_i64)).unwrap();
    let sign = shifted.certified_sign_until(-512);
    println!("positive_displacement_sign={sign:?}");
    assert!(matches!(
        sign,
        CertifiedRealSign::Known {
            sign: RealSign::Positive,
            ..
        }
    ));
}
