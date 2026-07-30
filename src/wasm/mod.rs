//! WebAssembly bindings and JavaScript-facing conversion helpers.

use crate::hyper_math::{hreal_from_f64, hreal_to_f64};
use hyperlattice::{Matrix4, Point3, Real, Vector3};

pub mod curve_js;
pub mod matrix_js;
pub mod mesh_js;
pub mod plane_js;
pub mod point_js;
pub mod vector_js;

pub(crate) fn finite_matrix4(values: [Real; 16]) -> Option<Matrix4> {
    Some(Matrix4::from_row_major(values))
}

pub(crate) fn real_from_js(value: f64) -> Option<Real> {
    hreal_from_f64(value).ok()
}

pub(crate) fn real_from_js_named(
    value: f64,
    name: &str,
) -> Result<Real, wasm_bindgen::JsValue> {
    real_from_js(value)
        .ok_or_else(|| wasm_bindgen::JsValue::from_str(&format!("{name} must be finite")))
}

pub(crate) fn real_to_js(value: &Real) -> f64 {
    hreal_to_f64(value).unwrap_or(f64::NAN)
}

pub(crate) fn point3_from_js(x: f64, y: f64, z: f64) -> Result<Point3, wasm_bindgen::JsValue> {
    Ok(Point3::new(
        real_from_js_named(x, "x")?,
        real_from_js_named(y, "y")?,
        real_from_js_named(z, "z")?,
    ))
}

pub(crate) fn vector3_from_js(
    x: f64,
    y: f64,
    z: f64,
) -> Result<Vector3, wasm_bindgen::JsValue> {
    Ok(Vector3::from_xyz(
        real_from_js_named(x, "x")?,
        real_from_js_named(y, "y")?,
        real_from_js_named(z, "z")?,
    ))
}

#[cfg(test)]
pub(crate) fn tolerance() -> Real {
    real_from_js(1.0e-9).expect("finite wasm test tolerance")
}
