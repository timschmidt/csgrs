use nalgebra::Matrix4;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct Matrix4Js {
    pub(crate) inner: Matrix4<f64>,
}
