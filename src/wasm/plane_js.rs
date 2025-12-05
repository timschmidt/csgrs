use crate::mesh::{plane::Plane, vertex::Vertex};
use crate::wasm::polygon_js::PolygonJs;
use nalgebra::{Point3, Vector3};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct PlaneJs {
    pub(crate) inner: Plane,
}

#[wasm_bindgen]
impl PlaneJs {
    // Constructor: Create a plane from three vertices
    #[wasm_bindgen(constructor)]
    pub fn new_from_vertices(
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
        let point_a = Point3::new(ax, ay, az);
        let point_b = Point3::new(bx, by, bz);
        let point_c = Point3::new(cx, cy, cz);

        let normal = (point_b - point_a).cross(&(point_c - point_a)).normalize();

        // Convert Points to Vertices with default normals
        let vertex_a = Vertex::new(point_a, normal);
        let vertex_b = Vertex::new(point_b, normal);
        let vertex_c = Vertex::new(point_c, normal);

        let plane = Plane::from_vertices(vec![vertex_a, vertex_b, vertex_c]);
        Self { inner: plane }
    }

    // Constructor: Create a plane from a normal vector and an offset
    #[wasm_bindgen(js_name=newFromNormal)]
    pub fn new_from_normal(nx: f64, ny: f64, nz: f64, offset: f64) -> Self {
        let normal = Vector3::new(nx, ny, nz);
        let plane = Plane::from_normal(normal, offset);
        Self { inner: plane }
    }

    // Get the plane's normal vector as an array [nx, ny, nz]
    #[wasm_bindgen(js_name=normal)]
    pub fn normal(&self) -> JsValue {
        let n = self.inner.normal();
        serde_wasm_bindgen::to_value(&[n.x, n.y, n.z]).unwrap()
    }

    // Get the plane's offset (distance from origin along the normal)
    #[wasm_bindgen(js_name=offset)]
    pub fn offset(&self) -> f64 {
        self.inner.offset()
    }

    // Flip the plane's orientation (negate normal and offset)
    #[wasm_bindgen(js_name=flip)]
    pub fn flip(&mut self) {
        self.inner.flip();
    }

    // Orient a point relative to the plane (FRONT, BACK, COPLANAR)
    #[wasm_bindgen(js_name=orientPoint)]
    pub fn orient_point(&self, x: f64, y: f64, z: f64) -> i8 {
        let point = Point3::new(x, y, z);
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

        // Convert Matrix4 to flat arrays for easier JS consumption
        let to_xy_flat: Vec<f64> =
            to_xy.as_slice().iter().copied().map(|v| v as f64).collect();
        let from_xy_flat: Vec<f64> =
            from_xy.as_slice().iter().copied().map(|v| v as f64).collect();

        let obj = js_sys::Object::new();
        js_sys::Reflect::set(
            &obj,
            &"toXY".into(),
            &js_sys::Float64Array::from(to_xy_flat.as_slice()),
        )
        .unwrap();
        js_sys::Reflect::set(
            &obj,
            &"fromXY".into(),
            &js_sys::Float64Array::from(from_xy_flat.as_slice()),
        )
        .unwrap();

        obj.into()
    }
}
