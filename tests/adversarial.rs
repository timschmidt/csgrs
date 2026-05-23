use csgrs::{csg::CSG, mesh::Mesh, mesh::plane::Plane, sketch::Profile};
use hyperlattice::{Point3, Real, Vector3};

fn r(value: f64) -> Real {
    Real::try_from(value).expect("test values must be finite")
}

fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

#[test]
fn exact_near_vertices_are_not_tolerance_merged() {
    let mesh = Mesh::<()>::cube(r(1.0), ());
    let near = p3(1.0e-12, 0.0, 0.0);

    assert!(!mesh.contains_vertex(&near));
}

#[test]
fn boolean_pipeline_accepts_hyperreal_transforms() {
    let a = Mesh::<()>::cube(r(2.0), ()).center();
    let b = Mesh::<()>::sphere(r(1.1), 16, 8, ()).translate(r(0.4), r(0.2), r(0.1));

    let result = a.difference(&b).union(&b.intersection(&a));

    assert!(!result.polygons.is_empty());
    assert!(result.bounding_box().maxs.x > result.bounding_box().mins.x);
}

#[test]
fn plane_split_uses_exact_hyperreal_side_classification() {
    let mut mesh = Mesh::<()>::cube(r(2.0), ());
    let polygon = mesh.polygons.remove(0);
    let plane = Plane::from_normal(Vector3::x(), r(1.0e-12));
    let (_cf, _cb, front, back) = plane.split_polygon(&polygon);

    assert!(!front.is_empty() || !back.is_empty());
}

#[test]
fn profile_offset_and_extrude_keep_hyperreal_scalars() {
    let profile = Profile::<()>::square(r(2.0), ());
    #[cfg(feature = "offset")]
    let profile = profile.offset(r(0.125));
    let mesh = profile.extrude(r(0.75));

    assert!(!mesh.polygons.is_empty());
    assert!(mesh.bounding_box().maxs.z >= r(0.75));
}
