//! Geospatial and astronomical coordinate transformations for the Hyper stack.
//!
//! This initial scaffold has no transformation APIs. Development starts with
//! Earth geodesy, map projections, and local frames, then extends to celestial
//! frames, observers, and ephemerides. The crate's `DEVELOPMENT.md` records the
//! proposed architecture, implementation order, and acceptance requirements.
//!
//! Planned native operations reuse Hyperreal scalars, Hyperlattice carriers,
//! and Hyperlimit predicates. Numerical certainty, model accuracy, and data
//! provenance must remain distinguishable through supported compositions.
//! All production numerical quantities will use `hyperreal::Real` or types
//! backed by it. External crates are development-only oracles and algorithm
//! references, not production transformation backends.

#![forbid(unsafe_code)]
#![warn(missing_docs)]
