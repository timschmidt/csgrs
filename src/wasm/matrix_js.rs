//! JavaScript wrapper for 4x4 transform matrices.

use crate::wasm::{finite_matrix4, real_from_js, real_to_js};
use hyperlattice::{Matrix4, Real};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct Matrix4Js {
    pub(crate) inner: Matrix4,
}

fn matrix_from_js_values(values: [f64; 16]) -> Option<Matrix4> {
    let mut promoted: [Real; 16] = std::array::from_fn(|_| Real::zero());
    for (slot, value) in promoted.iter_mut().zip(values) {
        *slot = real_from_js(value)?;
    }
    finite_matrix4(promoted)
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
        let matrix = matrix_from_js_values([
            m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33, m34, m41, m42, m43, m44,
        ])
        .unwrap_or_else(Matrix4::identity);
        Matrix4Js { inner: matrix }
    }

    #[wasm_bindgen(js_name = toArray)]
    pub fn to_array(&self) -> Vec<f64> {
        self.inner
            .0
            .iter()
            .flat_map(|row| row.iter().map(real_to_js))
            .collect()
    }
}

// Rust-only conversions
impl From<Matrix4> for Matrix4Js {
    fn from(v: Matrix4) -> Self {
        Matrix4Js { inner: v }
    }
}

impl From<&Matrix4Js> for Matrix4 {
    fn from(v: &Matrix4Js) -> Self {
        v.inner.clone()
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
            f64::NAN,
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
    fn matrix_js_conversion_preserves_hyperlattice_matrix() {
        let matrix = Matrix4Js {
            inner: Matrix4::identity(),
        };

        let converted = Matrix4::from(&matrix);
        assert_eq!(converted, Matrix4::identity());
    }
}
