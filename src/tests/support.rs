//! Shared imports and helpers for unit tests.

pub use crate::csg::CSG;
pub use crate::errors::ValidationError;
pub(crate) use crate::hyper_math::hreal_from_f64;
pub use crate::mesh::Mesh;
pub use crate::mesh::Polygon;
pub use crate::mesh::plane::Plane;
pub use crate::sketch::Profile;
pub use crate::vertex::Vertex;
pub use hyperlattice::{Matrix4, Point3, Real, Vector3};

pub fn r(value: impl crate::hyper_math::IntoReal) -> Real {
    hreal_from_f64(value).expect("test values must be finite hyperreals")
}

pub fn tolerance() -> Real {
    r(1.0e-9)
}

pub fn p3(
    x: impl crate::hyper_math::IntoReal,
    y: impl crate::hyper_math::IntoReal,
    z: impl crate::hyper_math::IntoReal,
) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

pub fn v3(
    x: impl crate::hyper_math::IntoReal,
    y: impl crate::hyper_math::IntoReal,
    z: impl crate::hyper_math::IntoReal,
) -> Vector3 {
    Vector3::from_xyz(r(x), r(y), r(z))
}

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
    let Some(first) = polygons
        .iter()
        .flat_map(|poly| poly.vertices.iter())
        .map(|vertex| &vertex.position)
        .next()
    else {
        return [
            Real::zero(),
            Real::zero(),
            Real::zero(),
            Real::zero(),
            Real::zero(),
            Real::zero(),
        ];
    };
    let mut min_x = first.x.clone();
    let mut min_y = first.y.clone();
    let mut min_z = first.z.clone();
    let mut max_x = first.x.clone();
    let mut max_y = first.y.clone();
    let mut max_z = first.z.clone();

    for poly in polygons {
        for v in &poly.vertices {
            let p = &v.position;
            if p.x < min_x {
                min_x = p.x.clone();
            }
            if p.y < min_y {
                min_y = p.y.clone();
            }
            if p.z < min_z {
                min_z = p.z.clone();
            }
            if p.x > max_x {
                max_x = p.x.clone();
            }
            if p.y > max_y {
                max_y = p.y.clone();
            }
            if p.z > max_z {
                max_z = p.z.clone();
            }
        }
    }

    [min_x, min_y, min_z, max_x, max_y, max_z]
}

/// Quick helper to compare floating-point results with an acceptable tolerance.
pub fn approx_eq(
    a: impl crate::hyper_math::IntoReal,
    b: impl crate::hyper_math::IntoReal,
    eps: impl crate::hyper_math::IntoReal,
) -> bool {
    (r(a) - r(b)).abs() < r(eps)
}

pub fn hr(value: impl crate::hyper_math::IntoReal) -> Real {
    r(value)
}

pub fn make_polygon_3d<T>(points: &[[T; 3]]) -> Polygon<()>
where
    T: Clone + crate::hyper_math::IntoReal,
{
    let mut verts = Vec::new();
    for p in points {
        let pos = p3(p[0].clone(), p[1].clone(), p[2].clone());
        let normal = Vector3::z();
        verts.push(Vertex::new(pos, normal));
    }
    Polygon::new(verts, ())
}

pub fn polygon_from_xy_points<T>(xy_points: &[[T; 2]]) -> Polygon<()>
where
    T: Clone + crate::hyper_math::IntoReal,
{
    let vertices = xy_points
        .iter()
        .map(|p| Vertex::new(p3(p[0].clone(), p[1].clone(), r(0.0)), Vector3::z()))
        .collect();
    Polygon::new(vertices, ())
}
