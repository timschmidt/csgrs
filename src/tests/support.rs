//! Shared imports and helpers for unit tests.

pub use crate::csg::CSG;
pub use crate::errors::ValidationError;
pub use crate::float_types::{FRAC_PI_2, PI, Real, tolerance};
pub use crate::mesh::Mesh;
pub use crate::mesh::bsp::Node;
pub use crate::mesh::plane::Plane;
pub use crate::polygon::Polygon;
pub use crate::sketch::Profile;
pub use crate::vertex::{Vertex, VertexCluster};
pub use hashbrown::HashMap;
pub use nalgebra::{Matrix4, Point3, Vector3};

/// A small, custom metadata type to demonstrate usage.
/// We derive PartialEq so we can assert equality in tests.
#[derive(Debug, Clone, PartialEq)]
pub struct MyMetaData {
    pub id: u32,
    pub label: String,
}

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
            let p = v.position;
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

pub fn make_polygon_3d(points: &[[Real; 3]]) -> Polygon<()> {
    let mut verts = Vec::new();
    for p in points {
        let pos = Point3::new(p[0], p[1], p[2]);
        let normal = Vector3::z();
        verts.push(Vertex::new(pos, normal));
    }
    Polygon::new(verts, ())
}

pub fn polygon_from_xy_points(xy_points: &[[Real; 2]]) -> Polygon<()> {
    let vertices = xy_points
        .iter()
        .map(|p| Vertex::new(Point3::new(p[0], p[1], 0.0), Vector3::z()))
        .collect();
    Polygon::new(vertices, ())
}
