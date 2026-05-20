//! JavaScript wrapper for mesh planes.

use crate::float_types::{Real, hunit_cross_vector3, hunit_vector3};
use crate::mesh::plane::Plane;
use crate::vertex::Vertex;
use crate::wasm::{
    matrix_js::Matrix4Js, point_js::Point3Js, polygon_js::PolygonJs, vector_js::Vector3Js,
    vertex_js::VertexJs,
};
use nalgebra::{Point3, Vector3};
use wasm_bindgen::prelude::*;

fn wasm_plane_from_normal_or_default(normal: Vector3<Real>, offset: Real) -> Plane {
    let normal = hunit_vector3(&normal).unwrap_or_else(Vector3::z);
    let offset = if offset.is_finite() { offset } else { 0.0 };
    Plane::from_normal(normal, offset)
}

fn wasm_plane_from_points_or_default(
    point_a: Point3<Real>,
    point_b: Point3<Real>,
    point_c: Point3<Real>,
) -> Plane {
    let Some(normal) = hunit_cross_vector3(&(point_b - point_a), &(point_c - point_a)) else {
        return Plane::from_normal(Vector3::z(), 0.0);
    };
    if !point_a.coords.iter().all(|value| value.is_finite())
        || !point_b.coords.iter().all(|value| value.is_finite())
        || !point_c.coords.iter().all(|value| value.is_finite())
    {
        return Plane::from_normal(Vector3::z(), 0.0);
    }

    // JS input remains finite boundary data; the support normal is constructed
    // as a checked hyperlattice unit cross product before native plane topology
    // is built, following Yap, "Towards Exact Geometric Computation,"
    // Computational Geometry 7(1-2), 1997
    // (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    Plane::from_vertices(vec![
        Vertex::new(point_a, normal),
        Vertex::new(point_b, normal),
        Vertex::new(point_c, normal),
    ])
}

fn validate_plane_vertices(vertices: &[VertexJs]) -> Result<(), &'static str> {
    if vertices.len() < 3 {
        return Err("Plane.fromVertices requires at least 3 vertices");
    }
    Ok(())
}

#[wasm_bindgen]
pub struct PlaneJs {
    pub(crate) inner: Plane,
}

#[wasm_bindgen]
impl PlaneJs {
    #[wasm_bindgen(constructor)]
    pub fn new(vertices: Vec<VertexJs>) -> Result<PlaneJs, JsValue> {
        PlaneJs::from_vertices(vertices)
    }

    #[wasm_bindgen(js_name = FromVertices)]
    pub fn from_vertices(vertices: Vec<VertexJs>) -> Result<PlaneJs, JsValue> {
        validate_plane_vertices(&vertices).map_err(JsValue::from_str)?;

        // JS vertices are already finite boundary wrappers. `Plane::from_vertices`
        // selects support points with hyperlattice/hyperreal predicates, following
        // Yap, "Towards Exact Geometric Computation," Computational Geometry
        // 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
        let verts: Vec<Vertex> = vertices.into_iter().map(|v| v.inner).collect();

        let plane = Plane::from_vertices(verts);
        Ok(PlaneJs { inner: plane })
    }

    pub fn points(&self) -> Vec<Point3Js> {
        vec![
            Point3Js::from(self.inner.point_a),
            Point3Js::from(self.inner.point_b),
            Point3Js::from(self.inner.point_c),
        ]
    }

    // Constructor: Create a plane from three vertices
    #[wasm_bindgen(js_name = FromComponents)]
    pub fn from_components(
        ax: f64,
        ay: f64,
        az: f64,
        bx: f64,
        by: f64,
        bz: f64,
        cx: f64,
        cy: f64,
        cz: f64,
    ) -> Self {
        let point_a: Point3<Real> = Point3::new(ax as Real, ay as Real, az as Real);
        let point_b: Point3<Real> = Point3::new(bx as Real, by as Real, bz as Real);
        let point_c: Point3<Real> = Point3::new(cx as Real, cy as Real, cz as Real);

        let plane = wasm_plane_from_points_or_default(point_a, point_b, point_c);
        Self { inner: plane }
    }

    #[wasm_bindgen(js_name = FromPoints)]
    pub fn from_points(a: &Point3Js, b: &Point3Js, c: &Point3Js) -> Self {
        let point_a: Point3<Real> = a.into();
        let point_b: Point3<Real> = b.into();
        let point_c: Point3<Real> = c.into();

        let plane = wasm_plane_from_points_or_default(point_a, point_b, point_c);
        Self { inner: plane }
    }

    // Constructor: Create a plane from a normal vector and an offset
    #[wasm_bindgen(js_name = FromNormalComponents)]
    pub fn from_normal_components(nx: f64, ny: f64, nz: f64, offset: Real) -> Self {
        let normal: Vector3<Real> = Vector3::new(nx as Real, ny as Real, nz as Real);
        let plane = wasm_plane_from_normal_or_default(normal, offset);
        Self { inner: plane }
    }

    #[wasm_bindgen(js_name = FromNormal)]
    pub fn from_normal(normal: &Vector3Js, offset: Real) -> Self {
        let n: Vector3<Real> = normal.into();
        let plane = wasm_plane_from_normal_or_default(n, offset);
        Self { inner: plane }
    }

    // Get the plane's normal vector as an array [nx, ny, nz]
    #[wasm_bindgen(js_name=normal)]
    pub fn normal(&self) -> Vector3Js {
        let n = self.inner.normal();
        Vector3Js::from(n)
    }

    // Get the plane's offset (distance from origin along the normal)
    #[wasm_bindgen(js_name=offset)]
    pub fn offset(&self) -> f64 {
        self.inner.offset() as f64
    }

    // Flip the plane's orientation (negate normal and offset)
    #[wasm_bindgen(js_name=flip)]
    pub fn flip(&mut self) {
        self.inner.flip();
    }

    // Orient a point relative to the plane (FRONT, BACK, COPLANAR)
    #[wasm_bindgen(js_name = orientPoint)]
    pub fn orient_point(&self, p: &Point3Js) -> i8 {
        let point: Point3<Real> = p.into();
        self.inner.orient_point(&point)
    }

    // Orient a point relative to the plane (FRONT, BACK, COPLANAR)
    #[wasm_bindgen(js_name=orientPointComponents)]
    pub fn orient_point_components(&self, x: f64, y: f64, z: f64) -> i8 {
        let point: Point3<Real> = Point3::new(x as Real, y as Real, z as Real);
        self.inner.orient_point(&point)
    }

    // Orient another plane relative to this plane (FRONT, BACK, COPLANAR, SPANNING)
    #[wasm_bindgen(js_name=orientPlane)]
    pub fn orient_plane(&self, other: &PlaneJs) -> i8 {
        self.inner.orient_plane(&other.inner)
    }

    // Classify a polygon relative to the plane
    #[wasm_bindgen(js_name=classifyPolygon)]
    pub fn classify_polygon(&self, polygon_js: &PolygonJs) -> i8 {
        self.inner.classify_polygon(&polygon_js.inner)
    }

    // Split a polygon with the plane, returning the result as an object
    //#[wasm_bindgen(js_name=splitPolygon)]
    // pub fn split_polygon(&self, polygon_js: &PolygonJs) -> JsValue {}

    // Get the transformation matrices to project this plane onto the XY-plane and back
    #[wasm_bindgen(js_name=toXYTransform)]
    pub fn to_xy_transform(&self) -> JsValue {
        let (to_xy, from_xy) = self.inner.to_xy_transform();

        let to_xy_js = Matrix4Js { inner: to_xy };
        let from_xy_js = Matrix4Js { inner: from_xy };

        let obj = js_sys::Object::new();
        js_sys::Reflect::set(&obj, &"toXY".into(), &JsValue::from(to_xy_js)).unwrap();
        js_sys::Reflect::set(&obj, &"fromXY".into(), &JsValue::from(from_xy_js)).unwrap();
        obj.into()
    }
}

// Rust-only conversions (not visible to JS)
impl From<Plane> for PlaneJs {
    fn from(p: Plane) -> Self {
        PlaneJs { inner: p }
    }
}

impl From<&PlaneJs> for Plane {
    fn from(p: &PlaneJs) -> Self {
        p.inner.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::float_types::{hvector3_dot, tolerance};

    #[test]
    fn plane_js_from_vertices_rejects_short_input_without_panic() {
        let result = std::panic::catch_unwind(|| validate_plane_vertices(&[]));
        assert!(result.is_ok());
        assert!(result.unwrap().is_err());
    }

    #[test]
    fn plane_js_from_points_uses_hyperlattice_checked_cross_product() {
        let plane = wasm_plane_from_points_or_default(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        assert!(hvector3_dot(&plane.normal(), &Vector3::z()).unwrap() > 1.0 - tolerance());
        assert!((plane.offset()).abs() <= tolerance());

        let degenerate = wasm_plane_from_points_or_default(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(tolerance() * 0.25, 0.0, 0.0),
            Point3::new(tolerance() * 0.5, 0.0, 0.0),
        );
        assert!(
            hvector3_dot(&degenerate.normal(), &Vector3::z()).unwrap() > 1.0 - tolerance()
        );
        assert_eq!(degenerate.offset(), 0.0);

        let hostile = wasm_plane_from_points_or_default(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(Real::NAN, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        assert!(hvector3_dot(&hostile.normal(), &Vector3::z()).unwrap() > 1.0 - tolerance());
        assert_eq!(hostile.offset(), 0.0);
    }
}
