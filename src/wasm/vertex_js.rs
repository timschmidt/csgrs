use crate::float_types::Real;
use crate::mesh::vertex::Vertex;
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
    pub fn new(x: f64, y: f64, z: f64) -> VertexJs {
        VertexJs {
            inner: Vertex::new(
                Point3::new(x as Real, y as Real, z as Real),
                Vector3::new(0.0, 0.0, 1.0), // default normal
            ),
        }
    }

    #[wasm_bindgen(js_name = fromPositionNormal)]
    pub fn from_position_normal(position: &Point3Js, normal: &Vector3Js) -> VertexJs {
        let pos: Point3<Real> = position.into();
        let n: Vector3<Real> = normal.into();
        VertexJs {
            inner: Vertex::new(pos, n),
        }
    }

    #[wasm_bindgen(js_name = position)]
    pub fn position(&self) -> Point3Js {
        Point3Js::from(self.inner.pos)
    }

    #[wasm_bindgen(js_name = normal)]
    pub fn normal(&self) -> Vector3Js {
        Vector3Js::from(self.inner.normal)
    }

	#[wasm_bindgen(js_name = toArray)]
    pub fn to_array(&self) -> Vec<f64> {
        vec![
            self.inner.pos.x as f64,
            self.inner.pos.y as f64,
            self.inner.pos.z as f64,
            self.inner.normal.x as f64,
            self.inner.normal.y as f64,
            self.inner.normal.z as f64,
        ]
    }
}
