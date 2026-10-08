use hyperreal::Real;

fn main() {
    let q = |n: i32, d: i32| (Real::from(n) / Real::from(d)).unwrap();
    let alpha = q(1, 11).sqrt().unwrap();
    let beta = q(1, 13).sqrt().unwrap();
    let dx = Real::one() - &alpha;
    let squared = &dx * &dx + &beta * &beta;
    let reduced = q(167, 143) - Real::from(2) * &alpha;
    let speed = squared.clone().sqrt().unwrap();
    let other_speed = reduced.clone().sqrt().unwrap();
    let nx = (-&beta / &speed).unwrap();
    let ny = (&dx / &speed).unwrap();
    for (name, value) in [
        ("squared-speed", &squared - &reduced),
        ("speed", &speed - &other_speed),
        ("nx-square-times-speed", &nx * &nx * &squared - &beta * &beta),
        ("nx-square-times-reduced", &nx * &nx * &reduced - &beta * &beta),
        ("ny-square-times-reduced", &ny * &ny * &reduced - &dx * &dx),
        ("normal-unit", &nx * &nx + &ny * &ny - Real::one()),
        ("normal-x-equivalence", &nx + (&beta / &other_speed).unwrap()),
    ] {
        println!("{name}: zero={:?} tower={:?} rational={:?}", value.zero_status(), value.quadratic_tower_sign(), value.exact_rational_normal_form());
    }
    println!("normal-x-annihilator={:?}", nx.quadratic_tower_annihilating_polynomial());
}
