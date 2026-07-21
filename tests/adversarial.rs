use csgrs::{
    csg::CSG,
    mesh::{Mesh, Triangle, plane::Plane},
    polygon_mesh::Polygon,
    sketch::Profile,
};
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
fn public_mesh_construction_accepts_only_explicit_triangles() {
    let triangle = Triangle::new(
        vec![
            csgrs::vertex::Vertex::new(p3(0.0, 0.0, 0.0), Vector3::z()),
            csgrs::vertex::Vertex::new(p3(1.0, 0.0, 0.0), Vector3::z()),
            csgrs::vertex::Vertex::new(p3(0.0, 1.0, 0.0), Vector3::z()),
        ],
        (),
    );
    let mesh = Mesh::from_triangles(vec![triangle]).expect("one explicit triangle");

    assert_eq!(mesh.triangles().len(), 1);
    assert_eq!(mesh.triangles()[0].vertices().len(), 3);
}

#[test]
fn boolean_pipeline_accepts_hyperreal_transforms() {
    let a = Mesh::<()>::cube(r(2.0), ()).center();
    let b = Mesh::<()>::cube(r(1.1), ()).translate(r(0.4), r(0.2), r(0.1));

    let result = a.difference(&b).union(&b.intersection(&a));

    assert!(!result.triangles().is_empty());
    assert!(result.bounding_box().maxs.x > result.bounding_box().mins.x);
}

#[test]
fn translated_union_does_not_emit_origin_fallback_vertices() {
    let left = Mesh::<()>::cube(r(1.0), ()).translate(r(10.0), r(0.0), r(0.0));
    let right = Mesh::<()>::cube(r(1.0), ()).translate(r(13.0), r(0.0), r(0.0));

    let result = left.union(&right);

    assert!(!result.triangles().is_empty());
    for polygon in result.triangles() {
        for vertex in polygon.vertices() {
            let x = vertex
                .position
                .x
                .to_f64_lossy()
                .expect("cube coordinates should export to finite f64");
            assert!(
                x >= 10.0,
                "union emitted an un-translated or origin fallback vertex: x={x}, position={:?}",
                vertex.position
            );
        }
    }
}

#[test]
fn plane_split_uses_exact_hyperreal_side_classification() {
    let mesh = Mesh::<()>::cube(r(2.0), ());
    let triangle = mesh.triangles()[0].clone();
    let polygon = Polygon::new(triangle.vertices().to_vec(), ());
    let plane = Plane::from_normal(Vector3::x(), r(1.0e-12));
    let (_cf, _cb, front, back) = plane.split_polygon(&polygon);

    assert!(!front.is_empty() || !back.is_empty());
}

#[test]
fn profile_offset_and_extrude_keep_hyperreal_scalars() {
    let profile = Profile::square(r(2.0));
    #[cfg(feature = "offset")]
    let profile = profile.offset(r(0.125));
    let mesh = profile.extrude(r(0.75), ());

    assert!(!mesh.triangles().is_empty());
    assert!(mesh.bounding_box().maxs.z >= r(0.75));
}
