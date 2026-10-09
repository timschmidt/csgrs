use hyperreal::Real;

fn main() {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let unresolved = &sine * &sine + &cosine * &cosine - Real::one();
    let delta = (Real::one() + unresolved) - Real::one();
    eprintln!("before hypot");
    let distance = Real::hypot2(&delta, &Real::zero()).unwrap();
    eprintln!("after hypot; before format");
    #[cfg(fixed_decimal)]
    let formatted = format!("{:.6}", distance.fixed_decimal());
    #[cfg(not(fixed_decimal))]
    let formatted = format!("{distance:#.6}");
    eprintln!("after format: {formatted}");
    assert_eq!(formatted, "0.000000");
}
