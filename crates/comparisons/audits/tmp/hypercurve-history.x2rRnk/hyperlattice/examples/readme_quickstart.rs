use hyperlattice::{Matrix3, Real, Vector3, sqrt};

fn r(value: i32) -> Real {
    value.into()
}

fn main() -> hyperlattice::BlasResult<()> {
    let vector = Vector3::new([r(3), r(4), r(0)]);
    assert_eq!(vector.dot(&vector), r(25));
    assert_eq!(sqrt(vector.dot(&vector))?, r(5));

    let identity = Matrix3::identity();
    assert_eq!(identity.clone() * vector.clone(), vector);
    assert_eq!(identity.inverse()?, Matrix3::identity());
    Ok(())
}
