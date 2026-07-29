//! JavaScript wrapper for native Hypermesh planes.

use crate::wasm::{
    point_js::Point3Js, real_from_js_or_zero, real_to_js, vector_js::Vector3Js,
};
use hyperlattice::Vector3;
use hypermesh::Plane;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct PlaneJs {
    pub(crate) inner: Plane,
}

#[wasm_bindgen]
impl PlaneJs {
    #[wasm_bindgen(js_name = FromComponents)]
    pub fn from_components(
        ax: f64,
        ay: f64,
        az: f64,
        bx: f64,
        by: f64,
        bz: f64,
        cx: f64,
        cy: f64,
        cz: f64,
    ) -> Self {
        let a = Point3Js::new(ax, ay, az).inner;
        let b = Point3Js::new(bx, by, bz).inner;
        let c = Point3Js::new(cx, cy, cz).inner;
        Self {
            inner: Plane::from_points(&a, &b, &c),
        }
    }

    #[wasm_bindgen(js_name = FromPoints)]
    pub fn from_points(a: &Point3Js, b: &Point3Js, c: &Point3Js) -> Self {
        Self {
            inner: Plane::from_points(&a.inner, &b.inner, &c.inner),
        }
    }

    #[wasm_bindgen(js_name = FromNormalComponents)]
    pub fn from_normal_components(nx: f64, ny: f64, nz: f64, offset: f64) -> Self {
        Self {
            inner: Plane::from_coefficients(
                real_from_js_or_zero(nx),
                real_from_js_or_zero(ny),
                real_from_js_or_zero(nz),
                real_from_js_or_zero(offset),
            ),
        }
    }

    #[wasm_bindgen(js_name = FromNormal)]
    pub fn from_normal(normal: &Vector3Js, offset: f64) -> Self {
        let [x, y, z] = normal.inner.0.clone();
        Self {
            inner: Plane::from_coefficients(x, y, z, real_from_js_or_zero(offset)),
        }
    }

    pub fn normal(&self) -> Vector3Js {
        Vector3Js::from(Vector3::from_xyz(
            self.inner.normal.x.clone(),
            self.inner.normal.y.clone(),
            self.inner.normal.z.clone(),
        ))
    }

    pub fn offset(&self) -> f64 {
        real_to_js(&self.inner.offset)
    }

    pub fn flip(&mut self) {
        self.inner = self.inner.inverted();
    }

    #[wasm_bindgen(js_name = reflectionMatrix)]
    pub fn reflection_matrix(&self) -> Result<crate::wasm::matrix_js::Matrix4Js, JsValue> {
        self.inner
            .reflection_matrix()
            .map(Into::into)
            .map_err(|error| JsValue::from_str(&error.to_string()))
    }
}
