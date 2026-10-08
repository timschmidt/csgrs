use hyperreal::{Real, RealSign};

fn main() {
    let a = Real::from(3).sqrt().unwrap();
    let b = Real::from(7).sqrt().unwrap();
    let sum = &a + &b;
    println!("sum tower={:?}", sum.quadratic_tower_sign());
    assert_eq!(sum.quadratic_tower_sign(), Some(RealSign::Positive));
}
