// Re-export parry and rapier for the appropriate float size
#[cfg(feature = "f64")]
pub use parry3d_f64 as parry3d;
#[cfg(feature = "f64")]
pub use rapier3d_f64 as rapier3d;

#[cfg(feature = "f32")]
pub use parry3d;
#[cfg(feature = "f32")]
pub use rapier3d;

// Our Real scalar type:
#[cfg(feature = "f32")]
pub type Real = f32;
#[cfg(feature = "f64")]
pub type Real = f64;

use core::str::FromStr;
use std::sync::OnceLock;

/// Lazily-initialized tolerance used across the crate.
/// Defaults depend on precision (`f32` vs `f64`), but can be overridden:
///  1) **Build-time**: set env var `CSGRS_TOLERANCE` (e.g. `CSGRS_TOLERANCE=1e-6 cargo build`)
///  2) **Runtime**: call [`set_tolerance`] once before using the library
static TOLERANCE_CELL: OnceLock<Real> = OnceLock::new();

#[inline]
fn default_tolerance() -> Real {
    #[cfg(feature = "f32")]
    {
        1e-4
    }
    #[cfg(feature = "f64")]
    {
        1e-6
    }
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

// Pi
/// Archimedes' constant (π)
#[cfg(feature = "f32")]
pub const PI: Real = core::f32::consts::PI;
/// Archimedes' constant (π)
#[cfg(feature = "f64")]
pub const PI: Real = core::f64::consts::PI;

// Frac Pi 2
/// π/2
#[cfg(feature = "f32")]
pub const FRAC_PI_2: Real = core::f32::consts::FRAC_PI_2;
/// π/2
#[cfg(feature = "f64")]
pub const FRAC_PI_2: Real = core::f64::consts::FRAC_PI_2;

// Tau
/// The full circle constant (τ)
#[cfg(feature = "f32")]
pub const TAU: Real = core::f32::consts::TAU;
/// The full circle constant (τ)
#[cfg(feature = "f64")]
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
