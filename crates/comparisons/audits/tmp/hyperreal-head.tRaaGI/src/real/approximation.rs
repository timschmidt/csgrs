//! Approximation/`to_f64` and refinement helpers for [`Real`].
//!
//! The logic currently lives in [`super::arithmetic`], close to structural
//! facts and symbolic folding. That placement lets approximation stay deferred
//! until a caller explicitly asks for it, which is the central performance
//! contract for `Real`.
