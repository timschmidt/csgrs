use wasm_bindgen::prelude::*;
use crate::mesh::polygon::Polygon;

#[wasm_bindgen]
pub struct PolygonJs {
    pub(crate) inner: Polygon<String>,
}
