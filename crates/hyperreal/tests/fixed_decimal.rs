use hyperreal::Real;

fn cancellation() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    &sine * &sine + &cosine * &cosine - Real::one()
}

#[test]
fn unresolved_zero_and_native_distance_need_no_nonzero_certificate() {
    let value = cancellation();
    assert_eq!(format!("{:.6}", value.fixed_decimal()), "0.000000");
    let distance = Real::hypot2(&value, &Real::zero()).unwrap();
    assert_eq!(format!("{:.6}", distance.fixed_decimal()), "0.000000");
}

#[test]
fn explicit_fractional_precision_and_sign_are_preserved() {
    for (text, expected) in [
        ("0", "0.000"),
        ("1/8", "0.125"),
        ("-1/8", "-0.125"),
        ("2", "2.000"),
        ("1/3", "0.333"),
    ] {
        let value: Real = text.parse().unwrap();
        assert_eq!(format!("{:.3}", value.fixed_decimal()), expected);
    }
    assert_eq!(format!("{:+.0}", Real::one().fixed_decimal()), "+1");
    assert_eq!(format!("{}", Real::one().fixed_decimal()), "1");
}

#[test]
fn symbolic_values_use_the_requested_decimal_places() {
    assert_eq!(format!("{:.6}", Real::pi().fixed_decimal()), "3.141593");
    assert_eq!(
        format!("{:.6}", Real::from(2).sqrt().unwrap().fixed_decimal()),
        "1.414214"
    );
}

#[test]
fn tiny_symbolic_values_keep_fixed_notation_after_cache_warming() {
    for sign in [1, -1] {
        let tiny: Real = "1e-100".parse().unwrap();
        let value = Real::from(sign) * Real::pi() * tiny;
        let before = format!("{:.6}", value.fixed_decimal());
        let _ = format!("{value:.110e}");
        assert_eq!(format!("{:.6}", value.fixed_decimal()), before);
        assert_eq!(before, if sign > 0 { "0.000000" } else { "-0.000000" });
    }
}

#[test]
fn large_values_do_not_switch_notation() {
    let value: Real = "1e40".parse().unwrap();
    assert_eq!(
        format!("{:.2}", value.fixed_decimal()),
        format!("1{}.00", "0".repeat(40))
    );
}
