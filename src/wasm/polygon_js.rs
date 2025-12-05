use crate::mesh::polygon::Polygon;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct PolygonJs {
    pub(crate) inner: Polygon<String>,
}
