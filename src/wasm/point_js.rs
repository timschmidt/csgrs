//! JavaScript wrapper for 3D points.

use crate::float_types::{Real, hreal_from_f64, hreal_to_f64};
use nalgebra::Point3;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct Point3Js {
    pub(crate) inner: Point3<Real>,
}

#[wasm_bindgen]
impl Point3Js {
    #[wasm_bindgen(constructor)]
    pub fn new(x: f64, y: f64, z: f64) -> Point3Js {
        // JS coordinates are primitive boundary data. Promote through
        // hyperreal first and fall back to the origin if a caller supplies
        // NaN/Inf, following Yap's exact-geometric-computation boundary split
        // (<https://doi.org/10.1016/0925-7721(95)00040-2>).
        let point = match (hreal_from_f64(x), hreal_from_f64(y), hreal_from_f64(z)) {
            (Ok(x), Ok(y), Ok(z)) => Point3::new(
                hreal_to_f64(&x).unwrap_or(0.0),
                hreal_to_f64(&y).unwrap_or(0.0),
                hreal_to_f64(&z).unwrap_or(0.0),
            ),
            _ => Point3::origin(),
        };
        Point3Js { inner: point }
    }

    #[wasm_bindgen(getter)]
    pub fn x(&self) -> f64 {
        self.inner.x as f64
    }

    #[wasm_bindgen(getter)]
    pub fn y(&self) -> f64 {
        self.inner.y as f64
    }

    #[wasm_bindgen(getter)]
    pub fn z(&self) -> f64 {
        self.inner.z as f64
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
impl From<Point3<Real>> for Point3Js {
    fn from(p: Point3<Real>) -> Self {
        Point3Js { inner: p }
    }
}

impl From<&Point3Js> for Point3<Real> {
    fn from(p: &Point3Js) -> Self {
        p.inner
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
        assert_eq!(finite.inner, Point3::new(1.0, 2.0, 3.0));
    }
}
