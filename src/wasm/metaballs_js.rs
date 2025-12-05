// use crate::mesh::metaballs::MetaBall;
// use crate::float_types::Real;
// use wasm_bindgen::prelude::*;

// #[wasm_bindgen]
// #[derive(Serialize, Deserialize)]
// pub struct MetaBallJs {
// inner: MetaBall,
// }
//
// #[wasm_bindgen]
// impl MetaBallJs {
// #[wasm_bindgen(constructor)]
// pub fn new(center_x: Real, center_y: Real, center_z: Real, radius: Real) -> Self {
// let center = Point3::new(center_x, center_y, center_z);
// let meta_ball = MetaBall::new(center, radius);
// Self { inner: meta_ball }
// }
//
// #[wasm_bindgen(getter)]
// pub fn center_x(&self) -> Real {
// self.inner.center.x
// }
//
// #[wasm_bindgen(getter)]
// pub fn center_y(&self) -> Real {
// self.inner.center.y
// }
//
// #[wasm_bindgen(getter)]
// pub fn center_z(&self) -> Real {
// self.inner.center.z
// }
//
// #[wasm_bindgen(getter)]
// pub fn radius(&self) -> Real {
// self.inner.radius
// }
// }
