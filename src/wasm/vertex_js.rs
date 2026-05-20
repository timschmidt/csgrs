//! JavaScript wrapper for mesh vertices.

use crate::float_types::Real;
use crate::vertex::Vertex;
use crate::wasm::{point_js::Point3Js, vector_js::Vector3Js};
use nalgebra::{Point3, Vector3};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct VertexJs {
    pub(crate) inner: Vertex,
}

#[wasm_bindgen]
impl VertexJs {
    #[wasm_bindgen(constructor)]
    pub fn new(position: &Point3Js, normal: &Vector3Js) -> VertexJs {
        VertexJs::from_position_normal(position, normal)
    }

    #[wasm_bindgen(js_name = fromPositionNormal)]
    pub fn from_position_normal(position: &Point3Js, normal: &Vector3Js) -> VertexJs {
        let p: Point3<Real> = position.into();
        let n: Vector3<Real> = normal.into();
        VertexJs {
            inner: Vertex::new(p, n),
        }
    }

    #[wasm_bindgen(js_name = fromComponents)]
    pub fn from_components(x: f64, y: f64, z: f64) -> VertexJs {
        // Component inputs are JS primitive boundary data. Reuse the point and
        // vector wrappers so promotion/rejection stays centralized in
        // hyperreal/hyperlattice adapters, following Yap, "Towards Exact
        // Geometric Computation," Computational Geometry 7(1-2), 1997
        // (<https://doi.org/10.1016/0925-7721(95)00040-2>).
        let position = Point3Js::new(x, y, z);
        let normal = Vector3Js::new(0.0, 0.0, 1.0);
        VertexJs {
            inner: Vertex::new(position.inner, normal.inner),
        }
    }

    #[wasm_bindgen(js_name = position)]
    pub fn position(&self) -> Point3Js {
        Point3Js::from(self.inner.position)
    }

    #[wasm_bindgen(js_name = normal)]
    pub fn normal(&self) -> Vector3Js {
        Vector3Js::from(self.inner.normal)
    }

    #[wasm_bindgen(js_name = toString)]
    pub fn to_string_js(&self) -> String {
        format!(
            "<Vertex position=({}, {}, {}) normal=({}, {}, {})>",
            self.inner.position.x,
            self.inner.position.y,
            self.inner.position.z,
            self.inner.normal.x,
            self.inner.normal.y,
            self.inner.normal.z
        )
    }

    #[wasm_bindgen(js_name = toArray)]
    pub fn to_array(&self) -> Vec<f64> {
        vec![
            self.inner.position.x as f64,
            self.inner.position.y as f64,
            self.inner.position.z as f64,
            self.inner.normal.x as f64,
            self.inner.normal.y as f64,
            self.inner.normal.z as f64,
        ]
    }
}

// Rust-only conversions
impl From<Vertex> for VertexJs {
    fn from(v: Vertex) -> Self {
        VertexJs { inner: v }
    }
}

impl From<&VertexJs> for Vertex {
    fn from(v: &VertexJs) -> Self {
        v.inner
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vertex_js_from_components_reuses_hyperreal_point_boundary() {
        let vertex = VertexJs::from_components(f64::NAN, 1.0, f64::INFINITY);
        assert_eq!(vertex.inner.position, Point3::origin());
        assert_eq!(vertex.inner.normal, Vector3::z());
    }
}
