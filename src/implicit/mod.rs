//! Native triangle generation from sampled implicit fields.

#[cfg(feature = "metaballs")]
pub(crate) mod metaballs;
#[cfg(feature = "sdf")]
pub(crate) mod sdf;
#[cfg(feature = "sdf")]
pub(crate) mod tpms;
