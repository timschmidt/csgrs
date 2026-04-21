#[cfg(all(feature = "realistic-backend", feature = "approx-backend"))]
compile_error!("realistic-backend and approx-backend are mutually exclusive");

#[cfg(feature = "realistic-backend")]
mod realistic;

#[cfg(all(not(feature = "realistic-backend"), feature = "approx-backend"))]
mod approx;

#[cfg(feature = "realistic-backend")]
pub(crate) use realistic::BackendScalar;

#[cfg(all(not(feature = "realistic-backend"), feature = "approx-backend"))]
pub(crate) use approx::BackendScalar;

#[cfg(not(any(feature = "realistic-backend", feature = "approx-backend")))]
compile_error!("enable either realistic-backend or approx-backend");
