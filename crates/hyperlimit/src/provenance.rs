//! Shared provenance records for exact geometric artifacts.
//!
//! These records deliberately describe source, approximation, and predicate
//! evidence at the same boundary as the predicate certificates they retain.
//! The exact geometric computation model separates exact objects, approximate
//! views, and certified combinatorial decisions. Keeping the small provenance
//! atoms in `hyperlimit` lets downstream
//! geometry crates share that boundary instead of reimplementing local
//! certificate summaries.

use crate::predicate::{PredicateApiSemantics, PredicateUse};

/// Source category for geometry data entering an exact artifact.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MeshSource {
    /// Exact coordinates were supplied by the caller.
    Exact,
    /// Primitive `f64` coordinates were checked and imported as exact dyadics.
    LossyF64,
    /// Data came from a hypermesh-derived adapter.
    HypermeshAdapter,
    /// Data came from an external edge adapter such as OBJ, display, or runtime
    /// preview code.
    ExternalAdapter,
}

/// How approximate values may be used at a boundary.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ApproximationPolicy {
    /// Approximation is refused for topology decisions.
    ExactOnly,
    /// Approximation may be exported for IO, display, broad phase, or logs.
    EdgeOnly,
    /// Caller explicitly accepts an approximate decision.
    ExplicitApproximateDecision,
}

/// Provenance for an input coordinate or index stream.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SourceProvenance {
    /// Source category.
    pub source: MeshSource,
    /// Human-readable label supplied by the caller or adapter.
    pub label: String,
    /// Approximation policy at this source boundary.
    pub approximation: ApproximationPolicy,
}

impl SourceProvenance {
    /// Build provenance for a checked `f64` import boundary.
    pub fn lossy_f64(label: impl Into<String>) -> Self {
        Self {
            source: MeshSource::LossyF64,
            label: label.into(),
            approximation: ApproximationPolicy::EdgeOnly,
        }
    }

    /// Build provenance for exact caller-owned coordinates.
    pub fn exact(label: impl Into<String>) -> Self {
        Self {
            source: MeshSource::Exact,
            label: label.into(),
            approximation: ApproximationPolicy::ExactOnly,
        }
    }

    /// Build provenance for a retained hypermesh-derived adapter edge.
    ///
    /// Hypermesh-derived topology can be retained for compatibility reports,
    /// but it must never enter an exact topology boundary as if it were exact
    /// or merely a display view. This keeps approximate topology decisions
    /// outside exact object identity.
    pub fn hypermesh_adapter(label: impl Into<String>) -> Self {
        Self {
            source: MeshSource::HypermeshAdapter,
            label: label.into(),
            approximation: ApproximationPolicy::ExplicitApproximateDecision,
        }
    }

    /// Build provenance for an external edge adapter such as OBJ, display, or
    /// runtime preview code.
    pub fn external_adapter(label: impl Into<String>) -> Self {
        Self {
            source: MeshSource::ExternalAdapter,
            label: label.into(),
            approximation: ApproximationPolicy::EdgeOnly,
        }
    }

    /// Validate that a source label and approximation policy agree.
    ///
    /// Source provenance is the smallest public boundary between exact
    /// geometry objects and edge adapters. Validating the source atom directly
    /// keeps adapters from marking lossy or external data as exact-only before
    /// topology is constructed.
    pub fn validate(&self) -> Result<(), ConstructionProvenanceValidationError> {
        if self.label.trim().is_empty() {
            return Err(ConstructionProvenanceValidationError::EmptySourceLabel);
        }
        match (self.source, self.approximation) {
            (MeshSource::Exact, ApproximationPolicy::ExactOnly) => Ok(()),
            (MeshSource::Exact, _) | (_, ApproximationPolicy::ExactOnly) => {
                Err(ConstructionProvenanceValidationError::SourceApproximationMismatch)
            }
            (MeshSource::LossyF64, ApproximationPolicy::EdgeOnly) => Ok(()),
            (MeshSource::LossyF64, _) => {
                Err(ConstructionProvenanceValidationError::LossySourcePolicyMismatch)
            }
            (MeshSource::HypermeshAdapter, ApproximationPolicy::ExplicitApproximateDecision) => {
                Ok(())
            }
            (MeshSource::HypermeshAdapter, _) => {
                Err(ConstructionProvenanceValidationError::HypermeshAdapterPolicyMismatch)
            }
            (MeshSource::ExternalAdapter, ApproximationPolicy::EdgeOnly) => Ok(()),
            (MeshSource::ExternalAdapter, _) => {
                Err(ConstructionProvenanceValidationError::ExternalAdapterPolicyMismatch)
            }
        }
    }
}

/// Provenance retained by constructed geometry facts.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ConstructionProvenance {
    /// Source stream that created the artifact.
    pub source: SourceProvenance,
    /// Monotonic construction version for retained facts derived from
    /// `source`.
    pub construction_version: u64,
    /// Predicate reports consulted while deriving facts.
    pub predicates: Vec<PredicateUse>,
}

/// Error returned when retained construction provenance contradicts its
/// declared exactness boundary.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConstructionProvenanceValidationError {
    /// The human-readable source label is empty.
    EmptySourceLabel,
    /// An exact source was not marked exact-only, or an exact-only policy was
    /// attached to a non-exact source.
    SourceApproximationMismatch,
    /// A lossy primitive-float source was not marked as an edge-only
    /// approximation boundary.
    LossySourcePolicyMismatch,
    /// A hypermesh adapter source was not marked as an explicit
    /// approximate topology decision.
    HypermeshAdapterPolicyMismatch,
    /// An external display/import adapter was not marked as an edge-only
    /// approximation boundary.
    ExternalAdapterPolicyMismatch,
    /// A retained predicate use did not produce an exact-preserving proof.
    NonProofProducingPredicate,
    /// The cached predicate stage or semantic label does not match the
    /// retained certificate.
    PredicateMetadataMismatch,
    /// The construction version is zero, which cannot identify a live retained
    /// artifact.
    InvalidConstructionVersion,
}

impl PredicateUse {
    /// Validate that this predicate route produced exact-preserving evidence.
    ///
    /// Predicate summaries cross many report boundaries. This direct validator
    /// mirrors the embedded construction-provenance check so fuzzing and
    /// downstream policy code can reject an undecided or approximate predicate
    /// atom before it is copied into a larger exact artifact. The cached stage
    /// and API semantic label are checked against the certificate for the same
    /// reason: the exact-object model keeps the certificate as the
    /// proof-bearing object, while derived scheduling and diagnostic labels are
    /// only valid when they faithfully replay that proof route.
    pub fn validate(&self) -> Result<(), ConstructionProvenanceValidationError> {
        if !self.is_proof_producing() {
            return Err(ConstructionProvenanceValidationError::NonProofProducingPredicate);
        }
        if self.stage != self.certificate.precision_stage()
            || self.semantics != self.certificate.api_semantics()
        {
            return Err(ConstructionProvenanceValidationError::PredicateMetadataMismatch);
        }
        Ok(())
    }

    /// Return whether this predicate use is an explicitly approximate route.
    pub const fn is_approximation_forcing(self) -> bool {
        matches!(self.semantics, PredicateApiSemantics::ApproximationForcing)
    }
}

impl ConstructionProvenance {
    /// Create an empty construction provenance record.
    pub fn new(source: SourceProvenance) -> Self {
        Self {
            source,
            construction_version: 1,
            predicates: Vec::new(),
        }
    }

    /// Create an empty construction provenance record with an explicit
    /// version.
    pub fn with_version(source: SourceProvenance, construction_version: u64) -> Self {
        Self {
            source,
            construction_version,
            predicates: Vec::new(),
        }
    }

    /// Append a predicate-use record.
    pub fn push_predicate(&mut self, predicate: PredicateUse) {
        self.predicates.push(predicate);
    }

    /// Validate source policy and retained predicate certificates.
    ///
    /// The check deliberately allows legacy and external adapter sources only
    /// when they do not masquerade as exact-only sources. Runtime topology
    /// should consume exact facts and proof-producing predicates, while
    /// approximate or adapter provenance remains explicit.
    pub fn validate(&self) -> Result<(), ConstructionProvenanceValidationError> {
        self.source.validate()?;
        if self.construction_version == 0 {
            return Err(ConstructionProvenanceValidationError::InvalidConstructionVersion);
        }
        for predicate in &self.predicates {
            predicate.validate()?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::predicate::PredicateCertificate;

    #[test]
    fn source_provenance_rejects_ambiguous_approximation_boundaries() {
        SourceProvenance::exact("caller").validate().unwrap();
        SourceProvenance::lossy_f64("import").validate().unwrap();
        SourceProvenance::hypermesh_adapter("hypermesh")
            .validate()
            .unwrap();
        SourceProvenance::external_adapter("obj")
            .validate()
            .unwrap();

        assert_eq!(
            SourceProvenance {
                source: MeshSource::LossyF64,
                label: "bad".to_string(),
                approximation: ApproximationPolicy::ExactOnly,
            }
            .validate()
            .unwrap_err(),
            ConstructionProvenanceValidationError::SourceApproximationMismatch
        );
    }

    #[test]
    fn predicate_use_validation_checks_certificate_metadata() {
        let exact = PredicateUse::from_certificate(PredicateCertificate::ExactRealFact);
        exact.validate().unwrap();

        let mut relabeled = exact;
        relabeled.semantics = PredicateApiSemantics::ApproximationDeferring;
        assert_eq!(
            relabeled.validate().unwrap_err(),
            ConstructionProvenanceValidationError::PredicateMetadataMismatch
        );

        assert_eq!(
            PredicateUse::from_certificate(PredicateCertificate::Unknown)
                .validate()
                .unwrap_err(),
            ConstructionProvenanceValidationError::NonProofProducingPredicate
        );
    }
}
