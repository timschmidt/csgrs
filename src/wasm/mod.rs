//! WebAssembly bindings and JavaScript-facing conversion helpers.

use serde_json::Value as JsonValue;
use serde_wasm_bindgen::from_value;
use wasm_bindgen::prelude::*;

use crate::hyper_math::{Real, hreal_from_f64, hreal_to_f64};
use hyperlattice::{Matrix4, Point3, Vector3};

pub mod matrix_js;
pub mod mesh_js;
pub mod metaballs_js;
pub mod plane_js;
pub mod point_js;
pub mod polygon_js;
pub mod sketch_js;
pub mod vector_js;
pub mod vertex_js;

fn js_metadata_to_string(metadata: JsValue) -> Result<Option<String>, JsValue> {
    if metadata.is_undefined() || metadata.is_null() {
        return Ok(None);
    }

    // Convert arbitrary JS value -> serde_json::Value
    let json: JsonValue = from_value(metadata).map_err(|e| {
        JsValue::from_str(&format!("Failed to serialize metadata from JS: {:?}", e))
    })?;

    // Store it as a JSON string in Rust metadata
    let s = serde_json::to_string(&json).map_err(|e| {
        JsValue::from_str(&format!("Failed to stringify metadata as JSON: {}", e))
    })?;

    Ok(Some(s))
}

pub(crate) fn finite_matrix4(values: [Real; 16]) -> Option<Matrix4> {
    Some(Matrix4::from_row_major(values))
}

pub(crate) fn real_from_js(value: f64) -> Option<Real> {
    hreal_from_f64(value).ok()
}

pub(crate) fn real_from_js_or_zero(value: f64) -> Real {
    real_from_js(value).unwrap_or_else(Real::zero)
}

pub(crate) fn real_to_js(value: &Real) -> f64 {
    hreal_to_f64(value).unwrap_or(0.0)
}

pub(crate) fn point3_from_js_or_origin(x: f64, y: f64, z: f64) -> Point3 {
    match (real_from_js(x), real_from_js(y), real_from_js(z)) {
        (Some(x), Some(y), Some(z)) => Point3::new(x, y, z),
        _ => Point3::origin(),
    }
}

pub(crate) fn vector3_from_js_or_zero(x: f64, y: f64, z: f64) -> Vector3 {
    match (real_from_js(x), real_from_js(y), real_from_js(z)) {
        (Some(x), Some(y), Some(z)) => Vector3::from_xyz(x, y, z),
        _ => Vector3::zeros(),
    }
}

#[cfg(test)]
pub(crate) fn tolerance() -> Real {
    real_from_js(1.0e-9).expect("finite wasm test tolerance")
}
