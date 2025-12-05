use crate::float_types::Real;
use nalgebra::Point3;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct Point3Js {
    pub(crate) inner: Point3<Real>,
}

#[wasm_bindgen]
impl Point3Js {
    #[wasm_bindgen(constructor)]
    pub fn new(x: f64, y: f64, z: f64) -> Point3Js {
        Point3Js {
            inner: Point3::new(x as Real, y as Real, z as Real),
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

// Rust-only conversions (not visible to JS)
impl From<Point3<Real>> for Point3Js {
    fn from(p: Point3<Real>) -> Self {
        Point3Js { inner: p }
    }
}

impl From<&Point3Js> for Point3<Real> {
    fn from(p: &Point3Js) -> Self {
        p.inner
    }
}
