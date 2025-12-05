use crate::float_types::Real;
use nalgebra::Vector3;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct Vector3Js {
    pub(crate) inner: Vector3<Real>,
}

#[wasm_bindgen]
impl Vector3Js {
    #[wasm_bindgen(constructor)]
    pub fn new(x: f64, y: f64, z: f64) -> Vector3Js {
        Vector3Js {
            inner: Vector3::new(x as Real, y as Real, z as Real),
        }
    }

    #[wasm_bindgen(getter)]
    pub fn x(&self) -> f64 {
        self.inner.x as f64
    }

    #[wasm_bindgen(getter)]
    pub fn y(&self) -> f64 {
        self.inner.y as f64
    }

    #[wasm_bindgen(getter)]
    pub fn z(&self) -> f64 {
        self.inner.z as f64
    }
}

// Rust-only conversions
impl From<Vector3<Real>> for Vector3Js {
    fn from(v: Vector3<Real>) -> Self {
        Vector3Js { inner: v }
    }
}

impl From<&Vector3Js> for Vector3<Real> {
    fn from(v: &Vector3Js) -> Self {
        v.inner
    }
}
