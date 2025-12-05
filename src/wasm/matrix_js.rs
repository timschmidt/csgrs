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
        m34: f64,
        m41: f64,
        m42: f64,
        m43: f64,
        m44: f64,
    ) -> Matrix4Js {
        Matrix4Js {
            inner: Matrix4::new(
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
                m34 as Real,
                m41 as Real,
                m42 as Real,
                m43 as Real,
                m44 as Real,
            ),
        }
    }

    #[wasm_bindgen(js_name = toArray)]
    pub fn to_array(&self) -> Vec<f64> {
        vec![
            self.inner.m11 as f64,
            self.inner.m12 as f64,
            self.inner.m13 as f64,
            self.inner.m14 as f64,
            self.inner.m21 as f64,
            self.inner.m22 as f64,
            self.inner.m23 as f64,
            self.inner.m24 as f64,
            self.inner.m31 as f64,
            self.inner.m32 as f64,
            self.inner.m33 as f64,
            self.inner.m34 as f64,
            self.inner.m41 as f64,
            self.inner.m42 as f64,
            self.inner.m43 as f64,
            self.inner.m44 as f64,
        ]
    }
}
