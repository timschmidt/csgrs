use hyperlimit::{Point2, Real, Sign, orient2};

fn main() {
    let a = Point2::new(Real::from(0), Real::from(0));
    let b = Point2::new(Real::from(1), Real::from(0));
    let c = Point2::new(Real::from(0), Real::from(1));

    let orientation = orient2(&a, &b, &c);
    assert_eq!(orientation.value(), Some(Sign::Positive));
    println!("{orientation:?}");
}
