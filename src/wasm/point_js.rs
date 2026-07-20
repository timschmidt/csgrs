//! JavaScript wrapper for 3D points.

use crate::wasm::{point3_from_js_or_origin, real_to_js};
use hyperlattice::Point3;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct Point3Js {
    pub(crate) inner: Point3,
}

#[wasm_bindgen]
impl Point3Js {
    #[wasm_bindgen(constructor)]
    pub fn new(x: f64, y: f64, z: f64) -> Point3Js {
        // JS coordinates are primitive boundary data. Promote through
        // hyperreal first and fall back to the origin if a caller supplies
        // NaN/Inf, following Yap's exact-geometric-computation boundary split
        // (<https://doi.org/10.1016/0925-7721(95)00040-2>).
        Point3Js {
            inner: point3_from_js_or_origin(x, y, z),
        }
    }

    #[wasm_bindgen(getter)]
    pub fn x(&self) -> f64 {
        real_to_js(&self.inner.x)
    }

    #[wasm_bindgen(getter)]
    pub fn y(&self) -> f64 {
        real_to_js(&self.inner.y)
    }

    #[wasm_bindgen(getter)]
    pub fn z(&self) -> f64 {
        real_to_js(&self.inner.z)
    }

    #[wasm_bindgen(js_name = toString)]
    pub fn to_string_js(&self) -> String {
        format!(
            "<Point3({}, {}, {})>",
            self.inner.x, self.inner.y, self.inner.z
        )
    }
}

// Rust-only conversions (not visible to JS)
impl From<Point3> for Point3Js {
    fn from(p: Point3) -> Self {
        Point3Js { inner: p }
    }
}

impl From<&Point3Js> for Point3 {
    fn from(p: &Point3Js) -> Self {
        p.inner.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn constructor_rejects_nonfinite_js_boundary_coordinates() {
        let point = Point3Js::new(f64::NAN, 2.0, f64::INFINITY);
        assert_eq!(point.inner, Point3::origin());

        let finite = Point3Js::new(1.0, 2.0, 3.0);
        assert_eq!(finite.x(), 1.0);
        assert_eq!(finite.y(), 2.0);
        assert_eq!(finite.z(), 3.0);
    }
}
