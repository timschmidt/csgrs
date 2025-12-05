use crate::float_types::Real;
use nalgebra::Matrix4;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct Matrix4Js {
    pub(crate) inner: Matrix4<f64>,
}

#[wasm_bindgen]
impl Matrix4Js {
    #[wasm_bindgen(constructor)]
    pub fn new(
        m00: f64,
        m01: f64,
        m02: f64,
        m03: f64,
        m10: f64,
        m11: f64,
        m12: f64,
        m13: f64,
        m20: f64,
        m21: f64,
        m22: f64,
        m23: f64,
        m30: f64,
        m31: f64,
        m32: f64,
        m33: f64,
    ) -> Matrix4Js {
        Matrix4Js {
            inner: Matrix4::new(
                m00 as Real,
                m01 as Real,
                m02 as Real,
                m03 as Real,
                m10 as Real,
                m11 as Real,
                m12 as Real,
                m13 as Real,
                m20 as Real,
                m21 as Real,
                m22 as Real,
                m23 as Real,
                m30 as Real,
                m31 as Real,
                m32 as Real,
                m33 as Real,
            ),
        }
    }
}
