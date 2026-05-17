//! Public scalar type, tolerance configuration, and physics backend re-exports.

use hyperreal::RealSign;
use nalgebra::{Point3, Vector3};

pub use parry3d_f64 as parry3d;
pub use rapier3d_f64 as rapier3d;

/// Public API scalar type.
///
/// This is the legacy f64 boundary scalar retained while the crate is ported
/// module-by-module to [`HReal`]. New internal geometry should use `HReal` and
/// convert primitive floats only at API and IO boundaries.
pub type Real = f64;

/// Primary internal scalar type for exact-aware geometry.
pub type HReal = hyperreal::Real;

/// Explicit f64 boundary scalar alias.
pub type F64 = f64;

/// Explicit f32 boundary scalar alias.
pub type F32 = f32;

/// Promote an f64 boundary value to the internal hyperreal scalar.
pub fn hreal_from_f64(value: F64) -> Result<HReal, hyperreal::Problem> {
    HReal::try_from(value)
}

/// Promote an f32 boundary value to the internal hyperreal scalar.
pub fn hreal_from_f32(value: F32) -> Result<HReal, hyperreal::Problem> {
    HReal::try_from(value)
}

/// Lossy f64 export for API, rendering, and IO boundaries.
pub fn hreal_to_f64(value: &HReal) -> Option<F64> {
    value.to_f64_lossy().filter(|value| value.is_finite())
}

/// Promote a finite f64 boundary point to a hyperreal lattice vector.
pub(crate) fn hvector3_from_point3(point: &Point3<Real>) -> Option<hyperlattice::Vector3> {
    Some(hyperlattice::Vector3::new([
        hreal_from_f64(point.x).ok()?,
        hreal_from_f64(point.y).ok()?,
        hreal_from_f64(point.z).ok()?,
    ]))
}

/// Promote a finite f64 boundary vector to a hyperreal lattice vector.
pub(crate) fn hvector3_from_vector3(vector: &Vector3<Real>) -> Option<hyperlattice::Vector3> {
    Some(hyperlattice::Vector3::new([
        hreal_from_f64(vector.x).ok()?,
        hreal_from_f64(vector.y).ok()?,
        hreal_from_f64(vector.z).ok()?,
    ]))
}

/// Compare finite f64 boundary points by squared distance in hyperreal space.
///
/// This keeps tolerance predicates on the exact-aware side of the API boundary:
/// public callers still pass `Point3<f64>`, while topology-sensitive code gets
/// a `hyperlattice::Vector3` predicate that fails closed for non-finite inputs.
/// That mirrors Yap's exact-geometric-computation boundary discipline
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
pub(crate) fn hpoints_within_epsilon(
    lhs: &Point3<Real>,
    rhs: &Point3<Real>,
    epsilon: Real,
) -> bool {
    if !epsilon.is_finite() || epsilon <= 0.0 {
        return false;
    }

    let Some(lhs) = hvector3_from_point3(lhs) else {
        return false;
    };
    let Some(rhs) = hvector3_from_point3(rhs) else {
        return false;
    };

    let delta = lhs - rhs;
    hreal_lt_f64(&delta.dot(&delta), epsilon * epsilon)
}

/// Compare finite f64 boundary vectors by squared distance in hyperreal space.
///
/// This is the vector analogue of [`hpoints_within_epsilon`], used for normals
/// and directions that still cross public API or file-format boundaries as
/// primitive floats while topology-sensitive equality is evaluated with
/// `hyperreal::Real`.
pub(crate) fn hvectors_within_epsilon(
    lhs: &Vector3<Real>,
    rhs: &Vector3<Real>,
    epsilon: Real,
) -> bool {
    if !epsilon.is_finite() || epsilon <= 0.0 {
        return false;
    }

    let Some(lhs) = hvector3_from_vector3(lhs) else {
        return false;
    };
    let Some(rhs) = hvector3_from_vector3(rhs) else {
        return false;
    };

    let delta = lhs - rhs;
    hreal_lt_f64(&delta.dot(&delta), epsilon * epsilon)
}

/// Refine the sign of a hyperreal expression for topology decisions.
pub(crate) fn hreal_sign(value: &HReal) -> Option<RealSign> {
    value.refine_sign_until(128)
}

/// Returns true when `value` is strictly greater than the finite f64 threshold.
pub(crate) fn hreal_gt_f64(value: &HReal, threshold: F64) -> bool {
    let Ok(threshold) = hreal_from_f64(threshold) else {
        return false;
    };
    matches!(
        hreal_sign(&(value.clone() - threshold)),
        Some(RealSign::Positive)
    )
}

/// Returns true when `value` is strictly less than the finite f64 threshold.
pub(crate) fn hreal_lt_f64(value: &HReal, threshold: F64) -> bool {
    let Ok(threshold) = hreal_from_f64(threshold) else {
        return false;
    };
    matches!(
        hreal_sign(&(value.clone() - threshold)),
        Some(RealSign::Negative)
    )
}

use core::str::FromStr;
use std::sync::OnceLock;

/// Lazily-initialized tolerance used across the crate.
/// Defaults to `1e-6`, but can be overridden:
///  1) **Build-time**: set env var `CSGRS_TOLERANCE` (e.g. `CSGRS_TOLERANCE=1e-6 cargo build`)
///  2) **Runtime**: call [`set_tolerance`] once before using the library
static TOLERANCE_CELL: OnceLock<Real> = OnceLock::new();

#[inline]
const fn default_tolerance() -> Real {
    1e-6
}

/// Returns the current epsilon value.
/// If not set yet, it tries `CSGRS_EPSILON` (parsed as the active `Real`) and
/// falls back to a sensible default.
pub fn tolerance() -> Real {
    *TOLERANCE_CELL.get_or_init(|| {
        // Compile-time env if provided, inherited by dependencies
        if let Some(environment_variable) = option_env!("CSGRS_TOLERANCE") {
            if let Ok(value) = Real::from_str(environment_variable) {
                return value.max(Real::EPSILON);
            }
        }
        default_tolerance()
    })
}

/// Set epsilon programmatically once (subsequent calls are ignored).
/// Call near program start: `csgrs::float_types::set_epsilon(1e-6);`
pub fn set_tolerance(value: Real) {
    let _ = TOLERANCE_CELL.set(value.max(Real::EPSILON));
}

/// Archimedes' constant (π)
pub const PI: Real = core::f64::consts::PI;

/// π/2
pub const FRAC_PI_2: Real = core::f64::consts::FRAC_PI_2;

/// The full circle constant (τ)
pub const TAU: Real = core::f64::consts::TAU;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Unit conversion
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
pub const INCH: Real = 25.4;
pub const FOOT: Real = 25.4 * 12.0;
pub const YARD: Real = 25.4 * 36.0;
pub const MM: Real = 1.0;
pub const CM: Real = 10.0;
pub const METER: Real = 1000.0;

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Point3;

    #[test]
    fn hpoints_within_epsilon_accepts_nearby_finite_points() {
        let epsilon = 1e-6;
        let lhs = Point3::new(1.0, 2.0, 3.0);
        let rhs = Point3::new(1.0 + epsilon * 0.25, 2.0, 3.0);

        assert!(hpoints_within_epsilon(&lhs, &rhs, epsilon));
    }

    #[test]
    fn hpoints_within_epsilon_rejects_distant_or_nonfinite_points() {
        let epsilon = 1e-6;
        let lhs = Point3::new(1.0, 2.0, 3.0);
        let distant = Point3::new(1.0 + epsilon * 2.0, 2.0, 3.0);
        let nonfinite = Point3::new(f64::NAN, 2.0, 3.0);

        assert!(!hpoints_within_epsilon(&lhs, &distant, epsilon));
        assert!(!hpoints_within_epsilon(&lhs, &nonfinite, epsilon));
        assert!(!hpoints_within_epsilon(&lhs, &lhs, 0.0));
    }

    #[test]
    fn hvectors_within_epsilon_uses_hyperreal_distance() {
        let epsilon = 1e-6;
        let lhs = Vector3::new(0.0, 1.0, 0.0);
        let near = Vector3::new(0.0, 1.0 + epsilon * 0.25, 0.0);
        let distant = Vector3::new(0.0, 1.0 + epsilon * 2.0, 0.0);
        let nonfinite = Vector3::new(0.0, f64::INFINITY, 0.0);

        assert!(hvectors_within_epsilon(&lhs, &near, epsilon));
        assert!(!hvectors_within_epsilon(&lhs, &distant, epsilon));
        assert!(!hvectors_within_epsilon(&lhs, &nonfinite, epsilon));
    }
}
