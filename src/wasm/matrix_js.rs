//! JavaScript wrapper for 4x4 transform matrices.

use crate::{float_types::Real, wasm::finite_matrix4};
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
        let matrix = finite_matrix4([
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
        ])
        .unwrap_or_else(Matrix4::identity);
        Matrix4Js { inner: matrix }
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

// Rust-only conversions
impl From<Matrix4<Real>> for Matrix4Js {
    fn from(v: Matrix4<Real>) -> Self {
        let values = [
            v.m11, v.m12, v.m13, v.m14, v.m21, v.m22, v.m23, v.m24, v.m31, v.m32, v.m33,
            v.m34, v.m41, v.m42, v.m43, v.m44,
        ];
        Matrix4Js {
            inner: finite_matrix4(values).unwrap_or_else(Matrix4::identity),
        }
    }
}

impl From<&Matrix4Js> for Matrix4<Real> {
    fn from(v: &Matrix4Js) -> Self {
        finite_matrix4([
            v.inner.m11,
            v.inner.m12,
            v.inner.m13,
            v.inner.m14,
            v.inner.m21,
            v.inner.m22,
            v.inner.m23,
            v.inner.m24,
            v.inner.m31,
            v.inner.m32,
            v.inner.m33,
            v.inner.m34,
            v.inner.m41,
            v.inner.m42,
            v.inner.m43,
            v.inner.m44,
        ])
        .unwrap_or_else(Matrix4::identity)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matrix_js_rejects_nonfinite_constructor_values() {
        let matrix = Matrix4Js::new(
            1.0,
            0.0,
            0.0,
            0.0,
            Real::NAN,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
        );

        assert_eq!(matrix.inner, Matrix4::identity());
    }

    #[test]
    fn matrix_js_conversion_rejects_corrupted_nonfinite_inner_matrix() {
        let matrix = Matrix4Js {
            inner: Matrix4::new(
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                Real::INFINITY,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
            ),
        };

        let converted = Matrix4::<Real>::from(&matrix);
        assert_eq!(converted, Matrix4::identity());
    }
}
