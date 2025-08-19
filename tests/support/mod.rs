//! Test support library
//! Provides various helper functions & utilities for tests.

use csgrs::{
    float_types::Real,
    mesh::{polygon::Polygon, vertex::Vertex},
};
use nalgebra::{Point3, Vector3};

/// Returns the approximate bounding box `[min_x, min_y, min_z, max_x, max_y, max_z]`
/// for a set of polygons.
pub fn bounding_box(polygons: &[Polygon<()>]) -> [Real; 6] {
    let mut min_x = Real::MAX;
    let mut min_y = Real::MAX;
    let mut min_z = Real::MAX;
    let mut max_x = Real::MIN;
    let mut max_y = Real::MIN;
    let mut max_z = Real::MIN;

    for poly in polygons {
        for v in &poly.vertices {
            let p = v.pos;
            if p.x < min_x {
                min_x = p.x;
            }
            if p.y < min_y {
                min_y = p.y;
            }
            if p.z < min_z {
                min_z = p.z;
            }
            if p.x > max_x {
                max_x = p.x;
            }
            if p.y > max_y {
                max_y = p.y;
            }
            if p.z > max_z {
                max_z = p.z;
            }
        }
    }

    [min_x, min_y, min_z, max_x, max_y, max_z]
}

/// Quick helper to compare floating-point results with an acceptable tolerance.
pub fn approx_eq(a: Real, b: Real, eps: Real) -> bool {
    (a - b).abs() < eps
}

/// Helper to make a simple Polygon in 3D with given vertices.
pub fn make_polygon_3d(points: &[[Real; 3]]) -> Polygon<()> {
    let mut verts = Vec::new();
    for p in points {
        let pos = Point3::new(p[0], p[1], p[2]);
        // For simplicity, just store an arbitrary normal; Polygon::new re-computes the plane anyway.
        let normal = Vector3::z();
        verts.push(Vertex::new(pos, normal));
    }
    Polygon::new(verts, None)
}
