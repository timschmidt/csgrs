//! WebAssembly bindings and JavaScript-facing conversion helpers.
//!
//! Native carriers retain exact scalar data. JavaScript numbers enter as their
//! exact finite binary64 values; numeric getters are lossy views and return NaN
//! when no finite view is available. For example, `0.1` is not the rational 1/10.
//! Context results expose decision certainty separately from construction history
//! and source fidelity. Some value/plane helpers still flatten failures to strings
//! or consume receipts; their individual contracts describe those boundaries.

use crate::hyper_math::{hreal_from_f64, hreal_to_f64};
use hyperlattice::{Matrix4, Point3, Real, Vector3};

/// Query statuses, construction qualifications and explicit operation receipts.
pub mod context_js;
/// Native filled-region modeling and curve/mesh outcome bindings.
pub mod curve_js;
/// Host-owned JavaScript interchange diagnostics and TypeScript declarations.
mod io_error_js;
/// Exact matrix storage with row-major numeric ingress and display egress.
pub mod matrix_js;
/// Native triangle-mesh modeling, queries and finite interchange bindings.
pub mod mesh_js;
/// Native plane construction and reflection with optional explicit receipts.
pub mod plane_js;
/// Exact point storage exposed through finite numeric coordinate views.
pub mod point_js;
/// Native vector arithmetic, strict predicates and finite scalar views.
pub mod vector_js;

/// Maps the JS policy flag to strict or terminal-authorizing context; selection is not consumption.
pub(crate) const fn geometry_context(approximate_512: bool) -> crate::GeometryContext {
    if approximate_512 {
        crate::GeometryContext::APPROXIMATE_512
    } else {
        crate::GeometryContext::STRICT
    }
}

/// Wraps sixteen native values as a row-major matrix, always returning `Some`.
/// Despite its historical name, performs no validation or finite projection;
/// JavaScript-number finiteness is checked before callers reach this helper.
pub(crate) fn finite_matrix4(values: [Real; 16]) -> Option<Matrix4> {
    Some(Matrix4::from_row_major(values))
}

/// Promotes a finite binary64 value exactly; collapses invalid-number errors to `None`.
pub(crate) fn real_from_js(value: f64) -> Option<Real> {
    hreal_from_f64(value).ok()
}

/// Promotes one finite number or returns a field-named JS string, not a typed error object.
pub(crate) fn real_from_js_named(
    value: f64,
    name: &str,
) -> Result<Real, wasm_bindgen::JsValue> {
    real_from_js(value)
        .ok_or_else(|| wasm_bindgen::JsValue::from_str(&format!("{name} must be finite")))
}

/// Returns a lossy finite binary64 view, or NaN when projection cannot supply one.
/// Rounding/underflow carry no source-error or topology certificate; native data is unchanged.
pub(crate) fn real_to_js(value: &Real) -> f64 {
    hreal_to_f64(value).unwrap_or(f64::NAN)
}

/// Promotes X, then Y, then Z into a native point, returning the first invalid-axis string.
pub(crate) fn point3_from_js(x: f64, y: f64, z: f64) -> Result<Point3, wasm_bindgen::JsValue> {
    Ok(Point3::new(
        real_from_js_named(x, "x")?,
        real_from_js_named(y, "y")?,
        real_from_js_named(z, "z")?,
    ))
}

/// Promotes X, then Y, then Z into a native vector, returning the first invalid-axis string.
pub(crate) fn vector3_from_js(
    x: f64,
    y: f64,
    z: f64,
) -> Result<Vector3, wasm_bindgen::JsValue> {
    Ok(Vector3::from_xyz(
        real_from_js_named(x, "x")?,
        real_from_js_named(y, "y")?,
        real_from_js_named(z, "z")?,
    ))
}

/// Exact binary64 `1e-9` fixture threshold for finite-view tests, not a geometry tolerance.
#[cfg(test)]
pub(crate) fn tolerance() -> Real {
    real_from_js(1.0e-9).expect("finite wasm test tolerance")
}
