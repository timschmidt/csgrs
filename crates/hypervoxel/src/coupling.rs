//! Field-grid coupling reports for physics and circuit consumers.
//!
//! Voxel field samples are often used to cache thermal, EM, optical,
//! mechanical, or process state, but downstream residual equations live in
//! `hyperphysics` and `hypercircuit`. This module records whether a voxel field
//! grid has exact residual replay, certified interval evidence, explicit
//! adapter error bounds, or unresolved uncertainty. The boundary follows Yap,
//! "Towards Exact Geometric Computation," *Computational Geometry* 7(1-2),
//! 1997: sampled objects carry provenance and certification status instead of
//! becoming unqualified scalar truth.

use hyperreal::{Real, RealSign};

use crate::{AggregateCertainty, FreshnessStatus, VoxelAggregateFacts};

/// Physical/coupled field family represented by voxel samples.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum VoxelFieldCouplingKind {
    /// Thermal temperature/flux/conductivity sample grid.
    Thermal,
    /// Electromagnetic field, conductivity, permittivity, or permeability grid.
    Electromagnetic,
    /// Optical dose, absorption, scattering, or refractive-index grid.
    Optical,
    /// Photochemical exposure/conversion/gel-state grid.
    Photochemical,
    /// Mechanical stress/strain/displacement/modulus grid.
    Mechanical,
    /// Fluid pressure/velocity/phase/porosity grid.
    Fluid,
}

/// Manifest for handing voxel field samples to a residual-owning crate.
#[derive(Clone, Debug, PartialEq)]
pub struct VoxelFieldCouplingManifest {
    /// Field family.
    pub kind: VoxelFieldCouplingKind,
    /// Freshness of the sampled grid relative to its source.
    pub freshness: FreshnessStatus,
    /// Conservative aggregate facts for the field/sample grid.
    pub aggregate: VoxelAggregateFacts,
    /// Whether the owning physics/circuit crate can replay residual equations exactly.
    pub residual_replay_available: bool,
    /// Optional certified absolute adapter error bound for sampled values.
    pub adapter_error_bound: Option<Real>,
    /// Number of missing field/sample side-table records.
    pub missing_sample_records: usize,
}

/// Report consumed by physics/circuit coupling code.
#[derive(Clone, Debug, PartialEq)]
pub struct VoxelFieldCouplingReport {
    /// Field family.
    pub kind: VoxelFieldCouplingKind,
    /// Freshness of the sampled grid relative to its source.
    pub freshness: FreshnessStatus,
    /// Aggregate certainty for voxel facts.
    pub aggregate_certainty: AggregateCertainty,
    /// Whether exact residual replay is available outside `hypervoxel`.
    pub residual_replay_available: bool,
    /// Optional certified absolute adapter error bound for sampled values.
    pub adapter_error_bound: Option<Real>,
    /// Whether an adapter error bound was supplied.
    pub has_adapter_error_bound: bool,
    /// Whether the supplied adapter error bound is structurally non-negative.
    ///
    /// A negative or sign-unknown "absolute error" is not a certificate. This
    /// keeps interval/error-bounded coupling aligned with Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7(1-2), 1997: adapter
    /// evidence must carry a valid object-level bound before downstream
    /// residual code can consume it as certified evidence.
    pub adapter_error_bound_non_negative: bool,
    /// Whether the adapter route carries a certified usable error bound.
    pub certified_adapter_error_bound_ready: bool,
    /// Number of missing field/sample side-table records.
    pub missing_sample_records: usize,
    /// Whether a consumer may use the grid as exact residual evidence.
    pub usable_as_exact_residual_evidence: bool,
    /// Whether a consumer must treat the grid as interval/error-bounded evidence.
    pub requires_error_bounded_adapter: bool,
}

impl VoxelFieldCouplingManifest {
    /// Builds a coupling report from provenance and aggregate facts.
    pub fn report(&self) -> VoxelFieldCouplingReport {
        let has_adapter_error_bound = self.adapter_error_bound.is_some();
        let adapter_error_bound_non_negative =
            self.adapter_error_bound.as_ref().is_some_and(|bound| {
                matches!(
                    bound.structural_facts().sign,
                    Some(RealSign::Zero | RealSign::Positive)
                )
            });
        let certified_adapter_error_bound_ready =
            has_adapter_error_bound && adapter_error_bound_non_negative;
        let usable_as_exact_residual_evidence = self.freshness == FreshnessStatus::Current
            && self.aggregate.certainty == AggregateCertainty::Exact
            && self.residual_replay_available
            && self.adapter_error_bound.is_none()
            && self.missing_sample_records == 0;
        VoxelFieldCouplingReport {
            kind: self.kind,
            freshness: self.freshness,
            aggregate_certainty: self.aggregate.certainty,
            residual_replay_available: self.residual_replay_available,
            adapter_error_bound: self.adapter_error_bound.clone(),
            has_adapter_error_bound,
            adapter_error_bound_non_negative,
            certified_adapter_error_bound_ready,
            missing_sample_records: self.missing_sample_records,
            usable_as_exact_residual_evidence,
            requires_error_bounded_adapter: !usable_as_exact_residual_evidence
                && certified_adapter_error_bound_ready,
        }
    }
}
