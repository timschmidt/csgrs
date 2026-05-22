//! WebAssembly bindings and JavaScript-facing conversion helpers.

use serde_json::Value as JsonValue;
use serde_wasm_bindgen::from_value;
use wasm_bindgen::prelude::*;

use crate::float_types::{Real, hreal_from_f64};
use nalgebra::Matrix4;

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

pub(crate) fn finite_matrix4(values: [Real; 16]) -> Option<Matrix4<Real>> {
    values
        .iter()
        .all(|value| hreal_from_f64(*value).is_ok())
        .then(|| {
            Matrix4::new(
                values[0], values[1], values[2], values[3], values[4], values[5], values[6],
                values[7], values[8], values[9], values[10], values[11], values[12],
                values[13], values[14], values[15],
            )
        })
}
