//! Sketch construction certificate manifests.
//!
//! Downstream geometry and simulation crates should not have to infer whether a
//! solved sketch is trustworthy from primitive coordinates alone. This module
//! bundles the current exact proof surfaces for a retained sketch: semantic
//! preflight checks, residual lowering provenance, exact residual replay, and
//! round-trip metadata/proposal traces. The design follows Yap's
//! construction/proof boundary: a construction record is useful only when the
//! attached certificates say what was proved and what remains unsupported. See
//! Yap, "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
//! (1997).

use crate::certification::{CandidateCertificationReport, certify_candidate};
use crate::eval::context_from_problem;
use crate::prepared::PreparedProblem;
use crate::sketch::{
    SketchConstraintHandle, SketchEntityHandle, SketchGeneratedRowStatus, SketchLoweringReport,
    SketchParameterHandle, SketchRoundTripMetadata, SketchSolveProblem,
};
use crate::sketch_degeneracy::{SketchDegeneracyReport, preflight_sketch_degeneracies};
use crate::sketch_domains::{SketchParameterDomainReport, preflight_sketch_parameter_domains};
use crate::sketch_entity_domains::{SketchEntityDomainReport, preflight_sketch_entity_domains};

/// Overall proof status for a sketch construction certificate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SketchConstructionCertificateStatus {
    /// Every retained preflight, lowering, and exact residual replay check
    /// succeeded.
    Certified,
    /// Semantic preflight found invalid retained input before replay.
    InvalidPreflight,
    /// Lowering left at least one non-reference source row ungenerated.
    LoweringIncomplete,
    /// Exact residual replay ran, but the lowered candidate is not certified.
    ReplayRejected,
}

/// Round-trip/provenance row for one retained sketch object.
#[derive(Clone, Debug, PartialEq)]
pub struct SketchConstructionProvenance {
    /// Parameter handle, when this row describes a parameter.
    pub parameter: Option<SketchParameterHandle>,
    /// Entity handle, when this row describes an entity.
    pub entity: Option<SketchEntityHandle>,
    /// Constraint handle, when this row describes a constraint.
    pub constraint: Option<SketchConstraintHandle>,
    /// Source name copied from the retained sketch object.
    pub name: String,
    /// Retained metadata for UI/import/export round-tripping.
    pub metadata: SketchRoundTripMetadata,
}

/// Summary counts for proposal-only and future certificate surfaces.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct SketchConstructionTraceCounts {
    /// Number of retained objects that name a lossy proposal adapter.
    pub lossy_adapter_metadata_rows: usize,
    /// Number of exact replay rows that explicitly carry lossy-adapter-only
    /// non-certificates.
    pub lossy_adapter_only_replay_rows: usize,
    /// Predicate replay rows currently attached to this construction.
    pub predicate_replay_rows: usize,
    /// Interval/Krawczyk uniqueness reports currently attached.
    pub interval_uniqueness_reports: usize,
    /// Algebraic-root object references currently attached.
    pub algebraic_root_references: usize,
}

/// Certificate manifest for one retained sketch construction.
#[derive(Clone, Debug)]
pub struct SketchConstructionCertificate {
    /// Overall status computed from the attached proof reports.
    pub status: SketchConstructionCertificateStatus,
    /// Semantic sketch lowering report and source-row provenance.
    pub lowering: SketchLoweringReport,
    /// Exact scalar parameter-domain preflight.
    pub parameter_domains: SketchParameterDomainReport,
    /// Exact retained-entity degeneracy preflight.
    pub degeneracies: SketchDegeneracyReport,
    /// Exact retained-entity domain preflight.
    pub entity_domains: SketchEntityDomainReport,
    /// Exact residual replay for the lowered initial candidate.
    pub residual_replay: CandidateCertificationReport,
    /// Round-trip/provenance manifest for retained sketch objects.
    pub provenance: Vec<SketchConstructionProvenance>,
    /// Proposal-only and future-proof-surface summary counts.
    pub traces: SketchConstructionTraceCounts,
}

impl SketchConstructionCertificate {
    /// Returns true when all attached proof surfaces certify the construction.
    pub fn is_certified(&self) -> bool {
        self.status == SketchConstructionCertificateStatus::Certified
    }
}

/// Build a construction certificate for a retained sketch's current exact
/// parameter values.
///
/// The lowered [`crate::Problem`] initial values are treated as the candidate.
/// Proposal engines can later attach their own candidate contexts, but this
/// manifest already records whether the retained construction is internally
/// certifiable without relying on primitive-float coordinates.
pub fn certify_sketch_construction(sketch: &SketchSolveProblem) -> SketchConstructionCertificate {
    let parameter_domains = preflight_sketch_parameter_domains(sketch);
    let degeneracies = preflight_sketch_degeneracies(sketch);
    let entity_domains = preflight_sketch_entity_domains(sketch);
    let lowering = sketch.lower_to_problem();
    let context = context_from_problem(&lowering.problem);
    let prepared = PreparedProblem::new(&lowering.problem);
    let residual_replay = certify_candidate(&prepared, &context);
    let provenance = collect_provenance(sketch);
    let traces = SketchConstructionTraceCounts {
        lossy_adapter_metadata_rows: provenance
            .iter()
            .filter(|row| row.metadata.lossy_adapter_label.is_some())
            .count(),
        lossy_adapter_only_replay_rows: residual_replay.lossy_adapter_only_rows,
        predicate_replay_rows: 0,
        interval_uniqueness_reports: 0,
        algebraic_root_references: 0,
    };
    let status = classify_certificate_status(
        &parameter_domains,
        &degeneracies,
        &entity_domains,
        &lowering,
        &residual_replay,
    );

    SketchConstructionCertificate {
        status,
        lowering,
        parameter_domains,
        degeneracies,
        entity_domains,
        residual_replay,
        provenance,
        traces,
    }
}

fn classify_certificate_status(
    parameter_domains: &SketchParameterDomainReport,
    degeneracies: &SketchDegeneracyReport,
    entity_domains: &SketchEntityDomainReport,
    lowering: &SketchLoweringReport,
    residual_replay: &CandidateCertificationReport,
) -> SketchConstructionCertificateStatus {
    if parameter_domains.has_certified_invalid_domain()
        || parameter_domains.unknown_checks > 0
        || degeneracies.has_certified_degeneracy()
        || degeneracies.unknown_checks > 0
        || degeneracies.invalid_reference_checks > 0
        || entity_domains.has_certified_invalid_domain()
        || entity_domains.unknown_checks > 0
        || entity_domains.invalid_reference_checks > 0
    {
        return SketchConstructionCertificateStatus::InvalidPreflight;
    }
    if lowering
        .rows
        .iter()
        .any(|row| row.status != SketchGeneratedRowStatus::Generated)
    {
        return SketchConstructionCertificateStatus::LoweringIncomplete;
    }
    if !residual_replay.all_satisfied() {
        return SketchConstructionCertificateStatus::ReplayRejected;
    }
    SketchConstructionCertificateStatus::Certified
}

fn collect_provenance(sketch: &SketchSolveProblem) -> Vec<SketchConstructionProvenance> {
    let mut provenance = Vec::new();
    provenance.extend(
        sketch
            .parameters()
            .iter()
            .map(|parameter| SketchConstructionProvenance {
                parameter: Some(parameter.handle),
                entity: None,
                constraint: None,
                name: parameter.name.clone(),
                metadata: parameter.metadata.clone(),
            }),
    );
    provenance.extend(
        sketch
            .entities()
            .iter()
            .map(|entity| SketchConstructionProvenance {
                parameter: None,
                entity: Some(entity.handle),
                constraint: None,
                name: entity.name.clone(),
                metadata: entity.metadata.clone(),
            }),
    );
    provenance.extend(
        sketch
            .constraints()
            .iter()
            .map(|constraint| SketchConstructionProvenance {
                parameter: None,
                entity: None,
                constraint: Some(constraint.handle),
                name: constraint.name.clone(),
                metadata: constraint.metadata.clone(),
            }),
    );
    provenance
}
