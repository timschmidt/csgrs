// Re-export parry and rapier for the apropriate float size
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
#[cfg(feature = "f64")]
pub const EPSILON: Real = 1e-12;

// Pi
#[cfg(feature = "f32")]
pub const PI: Real = std::f32::consts::PI;
#[cfg(feature = "f64")]
pub const PI: Real = std::f64::consts::PI;

// Frac Pi 2
#[cfg(feature = "f32")]
pub const FRAC_PI_2: Real = std::f32::consts::FRAC_PI_2;
#[cfg(feature = "f64")]
pub const FRAC_PI_2: Real = std::f64::consts::FRAC_PI_2;

// Tau
#[cfg(feature = "f32")]
pub const TAU: Real = std::f32::consts::TAU;
#[cfg(feature = "f64")]
pub const TAU: Real = std::f64::consts::TAU;
