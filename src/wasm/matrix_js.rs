//! JavaScript wrapper for 4x4 transform matrices.

use crate::wasm::{finite_matrix4, real_from_js, real_to_js};
use hyperlattice::{Matrix4, Real};
use wasm_bindgen::prelude::*;

/// Owned native 4-by-4 matrix with row-major JS ingress and finite display-array egress.
/// Construction checks finite numeric inputs, not affine form or invertibility.
#[wasm_bindgen]
pub struct Matrix4Js {
    /// Exact native coefficients, including symbolic values supplied through Rust conversion.
    pub(crate) inner: Matrix4,
}

/// Promotes all sixteen row-major binary64 values exactly, stopping at the first nonfinite value.
fn matrix_from_js_values(values: [f64; 16]) -> Option<Matrix4> {
    let mut promoted: [Real; 16] = std::array::from_fn(|_| Real::zero());
    for (slot, value) in promoted.iter_mut().zip(values) {
        *slot = real_from_js(value)?;
    }
    finite_matrix4(promoted)
}

#[wasm_bindgen]
impl Matrix4Js {
    /// Imports sixteen finite numbers in positional row-major order; invalid inputs return a JS string.
    /// Historical argument labels are irregular: rows are `(m11,m12,m13,m20)`,
    /// `(m21,m22,m23,m30)`, `(m31,m32,m33,m34)` and `(m41,m42,m43,m44)`.
    /// No affine, nonsingular or rigid-transform precondition is enforced here.
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
    ) -> Result<Matrix4Js, JsValue> {
        let matrix = matrix_from_js_values([
            m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33, m34, m41, m42, m43, m44,
        ])
        .ok_or_else(|| JsValue::from_str("matrix components must be finite"))?;
        Ok(Matrix4Js { inner: matrix })
    }

    /// Allocates sixteen row-major lossy numeric views; any unavailable finite component becomes NaN.
    /// The returned array is a copy and carries no exact-replay or error-bound guarantee.
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
    /// Moves native coefficients into the wrapper without finite conversion or validation.
    fn from(v: Matrix4) -> Self {
        Matrix4Js { inner: v }
    }
}

impl From<&Matrix4Js> for Matrix4 {
    /// Clones the native matrix without projecting through the array getter.
    fn from(v: &Matrix4Js) -> Self {
        v.inner.clone()
    }
}

/// Host-side promotion and native-copy witnesses, independent of generated JS execution.
#[cfg(test)]
mod tests {
    use super::*;

    /// Rejects an interior NaN through the promotion helper without invoking a JS exception branch.
    #[test]
    fn matrix_js_rejects_nonfinite_constructor_values() {
        let matrix = matrix_from_js_values([
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
        ]);

        assert!(matrix.is_none());
    }

    /// Requires the Rust wrapper conversion to preserve the native identity matrix exactly.
    #[test]
    fn matrix_js_conversion_preserves_hyperlattice_matrix() {
        let matrix = Matrix4Js {
            inner: Matrix4::identity(),
        };

        let converted = Matrix4::from(&matrix);
        assert_eq!(converted, Matrix4::identity());
    }
}
