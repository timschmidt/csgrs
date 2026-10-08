//! Constant-value constructors and cached nodes for [`Computable`].
//!
//! Constants remain implemented in [`super::node`] because the constructors
//! share representation caches with the core expression enum. Keeping those
//! constructors adjacent avoids extra module boundaries in cold approximation
//! setup while still documenting this semantic area.
