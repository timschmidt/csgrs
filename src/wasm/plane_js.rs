use crate::float_types::Real;
use crate::mesh::{plane::Plane, vertex::Vertex};
use crate::wasm::{
    matrix_js::Matrix4Js, point_js::Point3Js, polygon_js::PolygonJs, vector_js::Vector3Js,
    vertex_js::VertexJs,
};
use nalgebra::{Point3, Vector3};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct PlaneJs {
    pub(crate) inner: Plane,
}

#[wasm_bindgen]
impl PlaneJs {
    #[wasm_bindgen(js_name = FromVertices)]
    pub fn from_vertices(vertices: Vec<VertexJs>) -> PlaneJs {
        // Require at least 3 vertices (Plane::from_vertices will index [0..2])
        if vertices.len() < 3 {
            panic!("Plane.fromVertices requires at least 3 vertices");
        }

        // Strip the JS wrappers to get the underlying Vertex<Real> values
        let verts: Vec<Vertex> = vertices.into_iter().map(|v| v.inner).collect();

        let plane = Plane::from_vertices(verts);
        PlaneJs { inner: plane }
    }

    // Constructor: Create a plane from three vertices
    #[wasm_bindgen(js_name = FromComponents)]
    pub fn new_from_components(
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

        let normal = (point_b - point_a).cross(&(point_c - point_a)).normalize();

        // Convert Points to Vertices with default normals
        let vertex_a = Vertex::new(point_a, normal);
        let vertex_b = Vertex::new(point_b, normal);
        let vertex_c = Vertex::new(point_c, normal);

        let plane = Plane::from_vertices(vec![vertex_a, vertex_b, vertex_c]);
        Self { inner: plane }
    }

    #[wasm_bindgen(js_name = FromPoints)]
    pub fn new_from_points(a: &Point3Js, b: &Point3Js, c: &Point3Js) -> Self {
        let point_a: Point3<Real> = a.into();
        let point_b: Point3<Real> = b.into();
        let point_c: Point3<Real> = c.into();

        let normal = (point_b - point_a).cross(&(point_c - point_a)).normalize();

        let vertex_a = Vertex::new(point_a, normal);
        let vertex_b = Vertex::new(point_b, normal);
        let vertex_c = Vertex::new(point_c, normal);

        let plane = Plane::from_vertices(vec![vertex_a, vertex_b, vertex_c]);
        Self { inner: plane }
    }

    // Constructor: Create a plane from a normal vector and an offset
    #[wasm_bindgen(js_name = newFromNormalComponents)]
    pub fn new_from_normal_components(nx: f64, ny: f64, nz: f64, offset: Real) -> Self {
        let normal: Vector3<Real> = Vector3::new(nx as Real, ny as Real, nz as Real);
        let plane = Plane::from_normal(normal, offset);
        Self { inner: plane }
    }

    #[wasm_bindgen(js_name = newFromNormal)]
    pub fn new_from_normal(normal: &Vector3Js, offset: Real) -> Self {
        let n: Vector3<Real> = normal.into();
        let plane = Plane::from_normal(n, offset);
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
