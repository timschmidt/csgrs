//! Provenance records for exact mesh construction.
//!
//! Provenance is deliberately explicit rather than inferred from scalar type.
//! Yap's exact-computation package split treats approximate views, cached
//! facts, and exact predicates as different artifacts; these records let
//! callers audit where mesh facts came from.

use hyperlimit::{PredicateApiSemantics, PredicateCertificate, PredicatePrecisionStage};

/// Source category for mesh data entering hypermesh.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MeshSource {
    /// Exact coordinates were supplied by the caller.
    Exact,
    /// Primitive `f64` coordinates were checked and imported as exact dyadics.
    LossyF64,
    /// Data came from the legacy boolmesh-derived adapter.
    LegacyBoolmeshAdapter,
    /// Data came from an external edge adapter such as OBJ, glam, or Bevy.
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

    /// Build provenance for a retained legacy boolmesh-derived adapter edge.
    ///
    /// Legacy boolmesh topology is primitive-float and epsilon-bearing. It can
    /// be retained for compatibility reports, but it must never enter the exact
    /// mesh boundary as if it were exact or merely a display view. This mirrors
    /// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7.1-2 (1997), by keeping approximate topology decisions outside exact
    /// object identity.
    pub fn legacy_boolmesh_adapter(label: impl Into<String>) -> Self {
        Self {
            source: MeshSource::LegacyBoolmeshAdapter,
            label: label.into(),
            approximation: ApproximationPolicy::ExplicitApproximateDecision,
        }
    }

    /// Build provenance for an external edge adapter such as OBJ, glam, or Bevy.
    pub fn external_adapter(label: impl Into<String>) -> Self {
        Self {
            source: MeshSource::ExternalAdapter,
            label: label.into(),
            approximation: ApproximationPolicy::EdgeOnly,
        }
    }
}

/// Compact record of a predicate used while deriving mesh facts.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PredicateUse {
    /// Certificate returned by `hyperlimit`.
    pub certificate: PredicateCertificate,
    /// Coarse precision stage for diagnostics and benchmarks.
    pub stage: PredicatePrecisionStage,
    /// API semantic class implied by the certificate.
    pub semantics: PredicateApiSemantics,
}

impl PredicateUse {
    /// Record one predicate certificate.
    pub fn from_certificate(certificate: PredicateCertificate) -> Self {
        Self {
            certificate,
            stage: certificate.precision_stage(),
            semantics: certificate.api_semantics(),
        }
    }

    /// Return whether this predicate route produced an exact-preserving proof.
    pub const fn is_proof_producing(self) -> bool {
        self.certificate.is_proof_producing()
    }
}

/// Provenance retained by constructed mesh facts.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ConstructionProvenance {
    /// Source stream that created the mesh.
    pub source: SourceProvenance,
    /// Monotonic construction version for retained facts derived from `source`.
    pub construction_version: u64,
    /// Predicate reports consulted while deriving facts.
    pub predicates: Vec<PredicateUse>,
}

/// Error returned when retained construction provenance contradicts its
/// declared exactness boundary.
///
/// Provenance is part of the exact object, not a comment attached after the
/// fact. Yap, "Towards Exact Geometric Computation," *Computational Geometry*
/// 7.1-2 (1997), separates exact objects, approximate views, and certified
/// predicates; these errors make that separation auditable at the hypermesh
/// API boundary.
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
    /// A legacy boolmesh adapter source was not marked as an explicit
    /// approximate topology decision.
    LegacyAdapterPolicyMismatch,
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

impl SourceProvenance {
    /// Validate that a source label and approximation policy agree.
    ///
    /// Source provenance is the smallest public boundary between exact mesh
    /// objects and edge adapters. Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997), separates exact computation from
    /// approximate views; validating the source atom directly keeps adapters
    /// from marking lossy or external data as exact-only before a mesh is even
    /// constructed.
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
            (
                MeshSource::LegacyBoolmeshAdapter,
                ApproximationPolicy::ExplicitApproximateDecision,
            ) => Ok(()),
            (MeshSource::LegacyBoolmeshAdapter, _) => {
                Err(ConstructionProvenanceValidationError::LegacyAdapterPolicyMismatch)
            }
            (MeshSource::ExternalAdapter, ApproximationPolicy::EdgeOnly) => Ok(()),
            (MeshSource::ExternalAdapter, _) => {
                Err(ConstructionProvenanceValidationError::ExternalAdapterPolicyMismatch)
            }
        }
    }
}

impl PredicateUse {
    /// Validate that this predicate route produced exact-preserving evidence.
    ///
    /// Predicate summaries cross many report boundaries. This direct validator
    /// mirrors the embedded construction-provenance check so fuzzing and
    /// downstream policy code can reject an undecided or approximate predicate
    /// atom before it is copied into a larger exact artifact. The cached stage
    /// and API semantic label are checked against the certificate for the same
    /// reason: Yap's exact-object model keeps the certificate as the
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

    /// Create an empty construction provenance record with an explicit version.
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
