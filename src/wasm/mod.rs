use serde_json::Value as JsonValue;
use serde_wasm_bindgen::from_value;
use wasm_bindgen::prelude::*;

pub mod matrix_js;
pub mod mesh_js;
pub mod metaballs_js;
pub mod plane_js;
pub mod polygon_js;
pub mod sketch_js;
pub mod vertex_js;

// Optional: better panic messages in the browser console.
#[cfg(feature = "console_error_panic_hook")]
#[wasm_bindgen(start)]
pub fn init_panic_hook() {
    console_error_panic_hook::set_once();
}

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
