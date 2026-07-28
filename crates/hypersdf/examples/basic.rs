use hyperlimit::{Plane3, Point3};
use hyperreal::Real;
use hypersdf::{Sdf, SdfExpr, SdfPointLocation};

fn r(value: i32) -> Real {
    Real::from(value)
}

fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

fn main() {
    let sphere = SdfExpr::sphere(p(0, 0, 0), r(25));
    let slab = SdfExpr::slab(Plane3::new(p(0, 0, 1), r(0)), r(3));
    let field = Sdf::new(sphere.intersection(slab).offset(r(1)));

    assert_eq!(
        field.classify_point(&p(0, 0, 0)).location,
        SdfPointLocation::Inside
    );
    assert_eq!(
        field.classify_point(&p(0, 0, 4)).location,
        SdfPointLocation::Boundary
    );
    assert_eq!(
        field.classify_point(&p(8, 0, 0)).location,
        SdfPointLocation::Outside
    );
}
