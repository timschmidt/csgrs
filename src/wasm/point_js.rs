//! JavaScript wrapper for 3D points.

use crate::wasm::{point3_from_js, real_to_js};
use hyperlattice::Point3;
use wasm_bindgen::prelude::*;

/// Owned native 3D point; coordinate getters expose lossy views without changing storage.
/// Rust conversions can retain symbolic coordinates beyond finite JS-number ingress.
#[wasm_bindgen]
pub struct Point3Js {
    /// Authoritative native point, independent of numeric getter projections.
    pub(crate) inner: Point3,
}

#[wasm_bindgen]
impl Point3Js {
    /// Imports each finite JS number as its exact binary value; rejects NaN/infinity with an axis-named string.
    #[wasm_bindgen(constructor)]
    pub fn new(x: f64, y: f64, z: f64) -> Result<Point3Js, JsValue> {
        // JS coordinates are primitive boundary data. Promote through
        // hyperreal first and reject NaN/Inf rather than fabricating an origin,
        // following Yap's exact-geometric-computation boundary split
        // (<https://doi.org/10.1016/0925-7721(95)00040-2>).
        Ok(Point3Js {
            inner: point3_from_js(x, y, z)?,
        })
    }

    /// Returns a lossy X view, or NaN if no finite binary64 view is available.
    #[wasm_bindgen(getter)]
    pub fn x(&self) -> f64 {
        real_to_js(&self.inner.x)
    }

    /// Returns a lossy Y view, or NaN if no finite binary64 view is available.
    #[wasm_bindgen(getter)]
    pub fn y(&self) -> f64 {
        real_to_js(&self.inner.y)
    }

    /// Returns a lossy Z view, or NaN if no finite binary64 view is available.
    #[wasm_bindgen(getter)]
    pub fn z(&self) -> f64 {
        real_to_js(&self.inner.z)
    }

    /// Formats native scalar displays for inspection; not an exact replay serialization.
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
    /// Moves native coordinates into the wrapper without projection or validation.
    fn from(p: Point3) -> Self {
        Point3Js { inner: p }
    }
}

impl From<&Point3Js> for Point3 {
    /// Clones authoritative coordinates without passing through numeric getters.
    fn from(p: &Point3Js) -> Self {
        p.inner.clone()
    }
}

/// Host-side finite-promotion and getter witnesses; not a generated JavaScript runtime test.
#[cfg(test)]
mod tests {
    use super::*;

    /// Checks invalid values through the promotion helper and finite construction/getters directly.
    /// Does not invoke the constructor's JS error branch on the host.
    #[test]
    fn constructor_rejects_nonfinite_js_boundary_coordinates() {
        assert!(crate::wasm::real_from_js(f64::NAN).is_none());
        assert!(crate::wasm::real_from_js(f64::INFINITY).is_none());

        let finite = Point3Js::new(1.0, 2.0, 3.0).expect("finite point");
        assert_eq!(finite.x(), 1.0);
        assert_eq!(finite.y(), 2.0);
        assert_eq!(finite.z(), 3.0);
    }
}
