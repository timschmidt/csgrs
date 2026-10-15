//! Scalar product identities remain available to native plane construction.
use hyperlattice::{Point3, Real};
use hypermesh::Plane;
use hyperreal::{CertifiedRealSign, RealSign};

fn vertical_triangle() -> [Point3; 3] {
    let origin = Real::from(6) - Real::from(2).sqrt().unwrap();
    let x = Real::from(6) - Real::from(2) * (Real::pi() / Real::from(8)).unwrap().sin();
    let y = Real::from(6)
        - Real::from(2)
            * ((Real::pi() * Real::from(3)) / Real::from(8))
                .unwrap()
                .sin();
    [
        Point3::new(origin.clone(), origin, Real::from(-1)),
        Point3::new(x.clone(), y.clone(), Real::from(-1)),
        Point3::new(x, y, Real::from(5)),
    ]
}

#[test]
fn vertical_symbolic_triangle_has_exact_horizontal_normal() {
    let [a, b, c] = vertical_triangle();
    let plane = Plane::from_points(&a, &b, &c);
    assert_eq!(plane.normal.z, Real::zero());
    // The first two XY displacements have opposite nonzero signs.
    for coordinate in [&plane.normal.x, &plane.normal.y] {
        assert!(matches!(
            coordinate.certified_sign_until(-512),
            CertifiedRealSign::Known {
                sign: RealSign::Negative,
                ..
            }
        ));
    }
}

#[test]
fn near_vertical_symbolic_triangle_keeps_its_nonzero_component() {
    let [a, b, mut c] = vertical_triangle();
    c.x = &c.x + (Real::one() / Real::from(1_000_000_000_000_i64)).unwrap();
    let plane = Plane::from_points(&a, &b, &c);
    // n_z = -delta * (b.y-a.y), with delta > 0 and b.y-a.y < 0.
    assert!(matches!(
        plane.normal.z.certified_sign_until(-512),
        CertifiedRealSign::Known {
            sign: RealSign::Positive,
            ..
        }
    ));
}
