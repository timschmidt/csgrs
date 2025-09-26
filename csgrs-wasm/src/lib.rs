use wasm_bindgen::prelude::*;

use csgrs::mesh::vertex::Vertex;
use csgrs::float_types::Real;
use nalgebra::{Point3, Vector3};


#[wasm_bindgen]
pub struct VertexJs {
    inner: Vertex,
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

    pub fn to_array(&self) -> Vec<f64> {
        vec![self.inner.pos.x as f64, 
             self.inner.pos.y as f64, 
             self.inner.pos.z as f64,
             self.inner.normal.x as f64, 
             self.inner.normal.y as f64, 
             self.inner.normal.z as f64]
    }
}

