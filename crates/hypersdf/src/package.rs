//! Typed downstream handoff packages.
//!
//! A handoff package is an evidence envelope, not an inference engine. It keeps
//! continuous-field facts beside optional adapter reports and forces consumers
//! to ask for a named domain before using any payload. This mirrors Yap's
//! geometric-computation separation: exact decisions live in predicate reports,
//! while adapter payloads remain explicitly scoped evidence. See Yap, "Towards
//! Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997).

use crate::facts::SdfFacts;
use crate::handoff::SdfVoxelHandoffReport;
use crate::mesh::SdfMeshPreviewReport;
use crate::sampling::SdfGridSamplingReport;
use crate::shader::SdfShaderExportReport;
use crate::solver::SdfProjectionReplayReport;
use crate::status::{SdfDomainStatus, SdfFreshness, SdfMetricStatus};
use crate::voxel::SdfHypervoxelHandoffReport;

/// Downstream evidence domain requested from an [`SdfHandoffPackage`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfHandoffDomain {
    /// Retained continuous-field facts and exact predicate replay readiness.
    ContinuousField,
    /// Lossy regular-grid scalar preview samples.
    SampledGridPreview,
    /// Conservative cell reports for voxel/grid consumers.
    VoxelCells,
    /// Exact frame-aware handoff for `hypervoxel` grid materialization.
    HypervoxelGrid,
    /// Primitive-float preview mesh payload.
    MeshPreview,
    /// Preview shader/source export.
    ShaderPreview,
    /// Exact replay of an external projection/intersection proposal.
    ProjectionReplay,
}

/// Readiness state for a requested handoff domain.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfHandoffReadiness {
    /// The requested domain has internally consistent evidence.
    Ready,
    /// The requested domain is represented but has explicit blockers.
    Blocked,
    /// The requested domain was not supplied in the package.
    Missing,
}

/// Explicit reason a handoff domain is not ready.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfHandoffBlocker {
    /// The required optional report is absent.
    MissingReport,
    /// Prepared evidence is known stale relative to the source construction.
    StaleSource,
    /// Retained expression domain checks failed.
    InvalidDomain,
    /// Retained expression domain checks could not certify validity.
    UnknownDomain,
    /// A contained report failed its structural self-validation.
    InconsistentReport,
    /// Preview sampling or meshing had non-finite primitive-float output.
    NonFinitePreview,
    /// Conservative voxel/cell handoff still contains unknown cells.
    UnknownVoxelCells,
    /// The requested grid frame is not ready for `hypervoxel`'s octree address model.
    HypervoxelFrameNotReady,
    /// Shader source was not emitted completely.
    IncompleteShader,
    /// Projection replay did not certify a boundary candidate.
    ProjectionNotAccepted,
}

/// Readiness report returned by [`SdfHandoffPackage::require_domain`].
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SdfHandoffRequirementReport {
    /// Requested domain.
    pub domain: SdfHandoffDomain,
    /// Readiness result.
    pub readiness: SdfHandoffReadiness,
    /// Explicit blockers that explain `Blocked` or `Missing`.
    pub blockers: Vec<SdfHandoffBlocker>,
}

impl SdfHandoffRequirementReport {
    /// Returns whether this requirement is ready for the requested domain.
    pub const fn is_ready(&self) -> bool {
        matches!(self.readiness, SdfHandoffReadiness::Ready)
    }
}

/// Compact evidence envelope for downstream `hypersdf` consumers.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfHandoffPackage {
    /// Structural facts for the retained continuous expression.
    pub facts: SdfFacts,
    /// Metric claim of the source expression.
    pub metric_status: SdfMetricStatus,
    /// Prepared-source freshness shared by package-level facts.
    pub freshness: SdfFreshness,
    /// Optional regular-grid preview sampling report.
    pub grid_samples: Option<SdfGridSamplingReport>,
    /// Optional conservative voxel/cell handoff report.
    pub voxel_cells: Option<SdfVoxelHandoffReport>,
    /// Optional exact frame-aware `hypervoxel` handoff report.
    pub hypervoxel_grid: Option<SdfHypervoxelHandoffReport>,
    /// Optional primitive-float mesh preview report.
    pub mesh_preview: Option<SdfMeshPreviewReport>,
    /// Optional preview shader export.
    pub shader_preview: Option<SdfShaderExportReport>,
    /// Optional projection proposal replay.
    pub projection_replay: Option<SdfProjectionReplayReport>,
}

impl SdfHandoffPackage {
    /// Start a package from retained continuous-field facts.
    pub fn new(facts: SdfFacts, metric_status: SdfMetricStatus, freshness: SdfFreshness) -> Self {
        Self {
            facts,
            metric_status,
            freshness,
            grid_samples: None,
            voxel_cells: None,
            hypervoxel_grid: None,
            mesh_preview: None,
            shader_preview: None,
            projection_replay: None,
        }
    }

    /// Attach a regular-grid preview sampling report.
    pub fn with_grid_samples(mut self, report: SdfGridSamplingReport) -> Self {
        self.grid_samples = Some(report);
        self
    }

    /// Attach a conservative voxel/cell handoff report.
    pub fn with_voxel_cells(mut self, report: SdfVoxelHandoffReport) -> Self {
        self.voxel_cells = Some(report);
        self
    }

    /// Attach an exact frame-aware `hypervoxel` handoff report.
    pub fn with_hypervoxel_grid(mut self, report: SdfHypervoxelHandoffReport) -> Self {
        self.hypervoxel_grid = Some(report);
        self
    }

    /// Attach a mesh-preview report.
    pub fn with_mesh_preview(mut self, report: SdfMeshPreviewReport) -> Self {
        self.mesh_preview = Some(report);
        self
    }

    /// Attach a shader-preview export report.
    pub fn with_shader_preview(mut self, report: SdfShaderExportReport) -> Self {
        self.shader_preview = Some(report);
        self
    }

    /// Attach a projection replay report.
    pub fn with_projection_replay(mut self, report: SdfProjectionReplayReport) -> Self {
        self.projection_replay = Some(report);
        self
    }

    /// Return a typed readiness report for a downstream consumer domain.
    pub fn require_domain(&self, domain: SdfHandoffDomain) -> SdfHandoffRequirementReport {
        let mut blockers = self.common_blockers();
        match domain {
            SdfHandoffDomain::ContinuousField => {}
            SdfHandoffDomain::SampledGridPreview => match &self.grid_samples {
                Some(report) if !report.is_self_consistent() => {
                    blockers.push(SdfHandoffBlocker::InconsistentReport);
                }
                Some(report) if report.samples.non_finite_count > 0 => {
                    blockers.push(SdfHandoffBlocker::NonFinitePreview);
                }
                Some(_) => {}
                None => blockers.push(SdfHandoffBlocker::MissingReport),
            },
            SdfHandoffDomain::VoxelCells => match &self.voxel_cells {
                Some(report) if !report.is_self_consistent() => {
                    blockers.push(SdfHandoffBlocker::InconsistentReport);
                }
                Some(report) if !report.all_cells_classified() => {
                    blockers.push(SdfHandoffBlocker::UnknownVoxelCells);
                }
                Some(_) => {}
                None => blockers.push(SdfHandoffBlocker::MissingReport),
            },
            SdfHandoffDomain::HypervoxelGrid => match &self.hypervoxel_grid {
                Some(report) if !report.is_self_consistent() => {
                    blockers.push(SdfHandoffBlocker::InconsistentReport);
                }
                Some(report) if !report.frame.hypervoxel_frame_ready => {
                    blockers.push(SdfHandoffBlocker::HypervoxelFrameNotReady);
                }
                Some(report) if report.unknown_count > 0 => {
                    blockers.push(SdfHandoffBlocker::UnknownVoxelCells);
                }
                Some(_) => {}
                None => blockers.push(SdfHandoffBlocker::MissingReport),
            },
            SdfHandoffDomain::MeshPreview => match &self.mesh_preview {
                Some(report) if !report.is_self_consistent() => {
                    blockers.push(SdfHandoffBlocker::InconsistentReport);
                }
                Some(report) if report.non_finite_output_count > 0 => {
                    blockers.push(SdfHandoffBlocker::NonFinitePreview);
                }
                Some(_) => {}
                None => blockers.push(SdfHandoffBlocker::MissingReport),
            },
            SdfHandoffDomain::ShaderPreview => match &self.shader_preview {
                Some(report) if !report.is_complete() => {
                    blockers.push(SdfHandoffBlocker::IncompleteShader);
                }
                Some(_) => {}
                None => blockers.push(SdfHandoffBlocker::MissingReport),
            },
            SdfHandoffDomain::ProjectionReplay => match &self.projection_replay {
                Some(report) if !report.is_self_consistent() => {
                    blockers.push(SdfHandoffBlocker::InconsistentReport);
                }
                Some(report)
                    if !matches!(
                        report.status,
                        crate::solver::SdfProjectionReplayStatus::BoundaryCertified
                    ) =>
                {
                    blockers.push(SdfHandoffBlocker::ProjectionNotAccepted);
                }
                Some(_) => {}
                None => blockers.push(SdfHandoffBlocker::MissingReport),
            },
        }
        let readiness = if blockers.is_empty() {
            SdfHandoffReadiness::Ready
        } else if blockers.contains(&SdfHandoffBlocker::MissingReport) {
            SdfHandoffReadiness::Missing
        } else {
            SdfHandoffReadiness::Blocked
        };
        SdfHandoffRequirementReport {
            domain,
            readiness,
            blockers,
        }
    }

    /// Validate all contained optional reports without implying readiness for
    /// any domain the caller did not request.
    pub fn is_self_consistent(&self) -> bool {
        self.metric_status == self.facts.metric_status
            && self
                .grid_samples
                .as_ref()
                .is_none_or(SdfGridSamplingReport::is_self_consistent)
            && self
                .voxel_cells
                .as_ref()
                .is_none_or(SdfVoxelHandoffReport::is_self_consistent)
            && self
                .hypervoxel_grid
                .as_ref()
                .is_none_or(SdfHypervoxelHandoffReport::is_self_consistent)
            && self
                .mesh_preview
                .as_ref()
                .is_none_or(SdfMeshPreviewReport::is_self_consistent)
            && self
                .projection_replay
                .as_ref()
                .is_none_or(SdfProjectionReplayReport::is_self_consistent)
    }

    fn common_blockers(&self) -> Vec<SdfHandoffBlocker> {
        let mut blockers = Vec::new();
        match self.freshness {
            SdfFreshness::Stale => blockers.push(SdfHandoffBlocker::StaleSource),
            SdfFreshness::Unversioned | SdfFreshness::Current => {}
        }
        match self.facts.domain_status {
            SdfDomainStatus::Invalid => blockers.push(SdfHandoffBlocker::InvalidDomain),
            SdfDomainStatus::Unknown => blockers.push(SdfHandoffBlocker::UnknownDomain),
            SdfDomainStatus::Valid => {}
        }
        blockers
    }
}
