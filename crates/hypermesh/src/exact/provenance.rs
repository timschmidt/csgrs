//! Shared provenance records for exact mesh construction.
//!
//! The provenance atoms live in `hyperlimit` so every exact-geometry crate can
//! retain the same predicate certificates, source policies, and approximation
//! boundary checks. This module keeps the historical `hypermesh::exact`
//! surface as re-exports while relying on the shared Yap-style exact-object
//! boundary from `hyperlimit`.

pub use hyperlimit::{
    ApproximationPolicy, ConstructionProvenance, ConstructionProvenanceValidationError, MeshSource,
    PredicateUse, SourceProvenance,
};
