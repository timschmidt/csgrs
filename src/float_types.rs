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

/// A small epsilon for geometric comparisons, adjusted per precision.
#[cfg(feature = "f32")]
pub const EPSILON: Real = 1e-5;
/// A small epsilon for geometric comparisons, adjusted per precision.
#[cfg(feature = "f64")]
pub const EPSILON: Real = 1e-10;

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
