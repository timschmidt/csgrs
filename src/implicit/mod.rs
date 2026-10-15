//! Native triangle generation from sampled implicit fields.

#[cfg(feature = "metaballs")]
pub(crate) mod metaballs;
#[cfg(any(feature = "sdf", feature = "metaballs"))]
pub(crate) mod sdf;
#[cfg(feature = "sdf")]
pub(crate) mod tpms;
