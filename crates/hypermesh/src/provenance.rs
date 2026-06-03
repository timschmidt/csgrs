//! Shared provenance records for exact mesh construction.
//!
//! The provenance atoms live in `hyperlimit` so every exact-geometry crate can
//! retain the same predicate certificates, source policies, and approximation
//! boundary checks. This module keeps the historical `hypermesh`
//! boundary from `hyperlimit`.

pub use hyperlimit::{
    ApproximationPolicy, ConstructionProvenance, ConstructionProvenanceValidationError, MeshSource,
    PredicateUse, SourceProvenance,
};
