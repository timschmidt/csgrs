mod support;

use csgrs::{
    float_types::EPSILON,
    mesh::{Mesh, polygon::Polygon, vertex::Vertex},
    traits::CSG,
};
use nalgebra::{Point3, Vector3};

use crate::support::{approx_eq, bounding_box};

#[test]
fn from_polygons_and_to_polygons() {
    let poly: Polygon<()> = Polygon::new(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
        ],
        None,
    );
    let csg: Mesh<()> = Mesh::from_polygons(&[poly.clone()], None);
    assert_eq!(csg.polygons.len(), 1);
    assert_eq!(csg.polygons[0].vertices.len(), 3);
}

#[test]
fn union() {
    let cube1: Mesh<()> = Mesh::cube(2.0, None).translate(-1.0, -1.0, -1.0); // from -1 to +1 in all coords
    let cube2: Mesh<()> = Mesh::cube(1.0, None).translate(0.5, 0.5, 0.5);

    let union_csg = cube1.union(&cube2);
    assert!(
        !union_csg.polygons.is_empty(),
        "Union of two cubes should produce polygons"
    );

    // Check bounding box => should now at least range from -1 to (0.5+1) = 1.5
    let bb = bounding_box(&union_csg.polygons);
    assert!(approx_eq(bb[0], -1.0, 1e-8));
    assert!(approx_eq(bb[1], -1.0, 1e-8));
    assert!(approx_eq(bb[2], -1.0, 1e-8));
    assert!(approx_eq(bb[3], 1.5, 1e-8));
    assert!(approx_eq(bb[4], 1.5, 1e-8));
    assert!(approx_eq(bb[5], 1.5, 1e-8));
}

#[test]
fn difference() {
    // Subtract a smaller cube from a bigger one
    let big_cube: Mesh<()> = Mesh::cube(4.0, None).translate(-2.0, -2.0, -2.0); // radius=2 => spans [-2,2]
    let small_cube: Mesh<()> = Mesh::cube(2.0, None).translate(-1.0, -1.0, -1.0); // radius=1 => spans [-1,1]

    let result = big_cube.difference(&small_cube);
    assert!(
        !result.polygons.is_empty(),
        "Subtracting a smaller cube should leave polygons"
    );

    // Check bounding box => should still be [-2,-2,-2, 2,2,2], but with a chunk removed
    let bb = bounding_box(&result.polygons);
    // At least the bounding box remains the same
    assert!(approx_eq(bb[0], -2.0, 1e-8));
    assert!(approx_eq(bb[3], 2.0, 1e-8));
}

#[test]
fn union2() {
    let c1: Mesh<()> = Mesh::cube(2.0, None); // cube from (-1..+1) if that's how you set radius=1 by default
    let c2: Mesh<()> = Mesh::sphere(1.0, 16, 8, None); // default sphere radius=1
    let unioned = c1.union(&c2);
    // We can check bounding box is bigger or at least not smaller than either shape's box
    let bb_union = unioned.bounding_box();
    let bb_cube = c1.bounding_box();
    let bb_sphere = c2.bounding_box();
    assert!(bb_union.mins.x <= bb_cube.mins.x.min(bb_sphere.mins.x));
    assert!(bb_union.maxs.x >= bb_cube.maxs.x.max(bb_sphere.maxs.x));
}

#[test]
fn intersect() {
    let c1: Mesh<()> = Mesh::cube(2.0, None);
    let c2: Mesh<()> = Mesh::sphere(1.0, 16, 8, None);
    let isect = c1.intersection(&c2);
    let bb_isect = isect.bounding_box();
    // The intersection bounding box should be smaller than or equal to each
    let bb_cube = c1.bounding_box();
    let bb_sphere = c2.bounding_box();
    assert!(bb_isect.mins.x >= bb_cube.mins.x - EPSILON);
    assert!(bb_isect.mins.x >= bb_sphere.mins.x - EPSILON);
    assert!(bb_isect.maxs.x <= bb_cube.maxs.x + EPSILON);
    assert!(bb_isect.maxs.x <= bb_sphere.maxs.x + EPSILON);
}

#[test]
fn intersect2() {
    let sphere: Mesh<()> = Mesh::sphere(1.0, 16, 8, None);
    let cube: Mesh<()> = Mesh::cube(2.0, None);

    let intersection = sphere.intersection(&cube);
    assert!(
        !intersection.polygons.is_empty(),
        "Sphere ∩ Cube should produce the portion of the sphere inside the cube"
    );

    // Check bounding box => intersection is roughly a sphere clipped to [-1,1]^3
    let bb = bounding_box(&intersection.polygons);
    // Should be a region inside the [-1,1] box
    for &val in &bb[..3] {
        assert!(val >= -1.0 - 1e-1);
    }
    for &val in &bb[3..] {
        assert!(val <= 1.0 + 1e-1);
    }
}

#[test]
fn inverse() {
    let c1: Mesh<()> = Mesh::cube(2.0, None);
    let inv = c1.inverse();
    // The polygons are flipped
    // We can check just that the polygon planes are reversed, etc.
    // A full check might compare the polygons, but let's do a quick check on one polygon.
    let orig_poly = &c1.polygons[0];
    let inv_poly = &inv.polygons[0];
    assert!(approx_eq(
        orig_poly.plane.normal().x,
        -inv_poly.plane.normal().x,
        EPSILON
    ));
    assert!(approx_eq(
        orig_poly.plane.normal().y,
        -inv_poly.plane.normal().y,
        EPSILON
    ));
    assert!(approx_eq(
        orig_poly.plane.normal().z,
        -inv_poly.plane.normal().z,
        EPSILON
    ));
    assert_eq!(
        c1.polygons.len(),
        inv.polygons.len(),
        "Inverse should keep the same polygon count, but flip them"
    );
}

#[test]
fn cube() {
    let c: Mesh<()> = Mesh::cube(2.0, None);
    // By default, corner at (0,0,0)
    // We expect 6 faces, each 4 vertices = 6 polygons
    assert_eq!(c.polygons.len(), 6);
    // Check bounding box
    let bb = c.bounding_box();
    assert!(approx_eq(bb.mins.x, 0.0, EPSILON));
    assert!(approx_eq(bb.maxs.x, 2.0, EPSILON));
}
