#![cfg(feature = "chull-io")]
mod support;

use csgrs::{mesh::Mesh, traits::CSG};

#[test]
fn convex_hull() {
    // If we take a shape with some random points, the hull should just enclose them
    let c1: Mesh<()> = Mesh::sphere(1.0, 16, 8, None);
    // The convex_hull of a sphere's sampling is basically that same shape, but let's see if it runs.
    let hull = c1.convex_hull();
    // The hull should have some polygons
    assert!(!hull.polygons.is_empty());
}

#[test]
fn minkowski_sum() {
    // Minkowski sum of two cubes => bigger cube offset by edges
    let c1: Mesh<()> = Mesh::cube(2.0, None).center();
    let c2: Mesh<()> = Mesh::cube(1.0, None).center();
    let sum = c1.minkowski_sum(&c2);
    let bb_sum = sum.bounding_box();
    // Expect bounding box from -1.5..+1.5 in each axis if both cubes were centered at (0,0,0).
    assert!(support::approx_eq(bb_sum.mins.x, -1.5, 0.01));
    assert!(support::approx_eq(bb_sum.maxs.x, 1.5, 0.01));
}
