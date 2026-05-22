//! Dual-contouring proposal reports over regular signed grids.
//!
//! This module is deliberately a report layer, not an accepted mesh builder.
//! Dual Contouring, after Ju, Losasso, Schaefer, and Warren, "Dual Contouring
//! of Hermite Data" (SIGGRAPH 2002), places one vertex per active grid cell by
//! solving a quadratic error function (QEF) built from edge intersections and
//! normals. In Hyper terms those edge roots, normals, and QEF rows are proposal
//! evidence until they replay against retained exact objects. That boundary is
//! the EGC split described by Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997): primitive samples can guide an
//! adapter, but topology is accepted only by exact or proof-bearing replay.

use core::cmp::Ordering;
use std::collections::BTreeMap;

use hyperlattice::Vector3;
use hyperlimit::{Point3, PredicateOutcome, compare_reals};
use hyperreal::Real;

use crate::expr::SdfExpr;
use crate::gradient::{SdfGradientReport, gradient_expr_point, normal_from_gradient_report};
use crate::primitive::SdfPrimitive;
use crate::sampling::{SdfGridSamplingReport, SdfPreviewSample};
use crate::status::{
    SdfEvidenceStatus, SdfFreshness, SdfGradientStatus, SdfMetricStatus, SdfNormalStatus,
};
use crate::transform::SdfTransform;

/// Source route used to build a dual-contouring proposal report.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfDualContouringSource {
    /// Endpoint signs and Hermite data were replayed against a retained SDF.
    ExactSdfReplay,
    /// Endpoint signs came only from primitive sampled grid values.
    SignedGridSamples,
}

/// Certified or reported sign of one signed-grid sample.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfGridSampleSign {
    /// The retained or sampled scalar is strictly negative.
    Negative,
    /// The retained or sampled scalar is exactly zero.
    Zero,
    /// The retained or sampled scalar is strictly positive.
    Positive,
    /// The sign was not available or could not be certified.
    Unknown,
}

impl SdfGridSampleSign {
    /// Returns whether the sign is known.
    pub const fn is_known(self) -> bool {
        !matches!(self, Self::Unknown)
    }

    /// Returns whether the sample lies on the reported zero level set.
    pub const fn is_zero(self) -> bool {
        matches!(self, Self::Zero)
    }

    fn from_exact(value: Option<&Real>) -> Self {
        let Some(value) = value else {
            return Self::Unknown;
        };
        match compare_reals(value, &Real::zero()) {
            PredicateOutcome::Decided {
                value: Ordering::Less,
                ..
            } => Self::Negative,
            PredicateOutcome::Decided {
                value: Ordering::Equal,
                ..
            } => Self::Zero,
            PredicateOutcome::Decided {
                value: Ordering::Greater,
                ..
            } => Self::Positive,
            PredicateOutcome::Unknown { .. } => Self::Unknown,
        }
    }

    fn from_primitive(value: Option<f64>) -> Self {
        let Some(value) = value.filter(|value| value.is_finite()) else {
            return Self::Unknown;
        };
        if value < 0.0 {
            Self::Negative
        } else if value > 0.0 {
            Self::Positive
        } else {
            Self::Zero
        }
    }
}

/// Positive axis direction of a regular-grid edge.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum SdfDualGridEdgeAxis {
    /// Edge from `(x, y, z)` to `(x + 1, y, z)`.
    X,
    /// Edge from `(x, y, z)` to `(x, y + 1, z)`.
    Y,
    /// Edge from `(x, y, z)` to `(x, y, z + 1)`.
    Z,
}

/// Kind of active edge crossing reported by the signed grid.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfDualEdgeCrossingKind {
    /// Endpoints have opposite known signs.
    SignChange,
    /// Exactly one endpoint is reported on the zero level set.
    EndpointTouch,
    /// Both endpoints are reported on the zero level set.
    CoincidentZeroEdge,
}

/// Evidence available for a dual-contouring edge root.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfDualEdgeRootEvidence {
    /// The exact root is one endpoint of the grid edge.
    ExactEndpoint,
    /// The retained field is affine on this edge, so linear interpolation is exact.
    ExactAffineEdgeRoot,
    /// A primitive signed grid reported a crossing, but no exact root replay exists.
    SampledSignChangeOnly,
    /// The entire edge is reported as zero; no unique root exists.
    CoincidentZeroEdge,
    /// The report could not construct a root acceptable to exact replay.
    Unknown,
}

impl SdfDualEdgeRootEvidence {
    /// Returns whether this edge carries a unique exact root point.
    pub const fn has_exact_unique_root(self) -> bool {
        matches!(self, Self::ExactEndpoint | Self::ExactAffineEdgeRoot)
    }
}

/// Evidence available for a Hermite normal.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfDualNormalEvidence {
    /// Normal came from a certified exact symbolic gradient.
    ExactSymbolic,
    /// Normal is deliberately absent because exact replay did not certify it.
    Missing,
}

/// Active-cell topology status before a downstream mesh owner validates it.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfDualCellTopologyStatus {
    /// No active edge crosses this cell.
    Inactive,
    /// The cell has known signs and nondegenerate active edges.
    Active,
    /// A zero endpoint or zero edge makes local ownership degenerate.
    DegenerateZeroTouch,
    /// The local sign pattern is an ambiguous dual-contouring case.
    Ambiguous,
    /// At least one relevant sign is unknown.
    Unknown,
}

/// Per-cell QEF/vertex placement readiness.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfDualVertexPlacementStatus {
    /// Inactive cells do not request a vertex.
    NotActive,
    /// Exact Hermite rows are present and a zero-residual affine candidate exists.
    ExactAffineQefCandidate,
    /// The cell has proposal evidence only, not an exact replayable QEF.
    ProposalOnly,
    /// The cell is active but missing required edge roots, normals, or topology facts.
    Blocked,
}

/// Blockers that keep a dual-contouring report out of validation handoff.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfDualContouringBlocker {
    /// Grid dimensions or report rows disagree with the declared point count.
    InvalidSampleCount,
    /// At least one grid dimension is too small to form a cell.
    GridTooSmallForCells,
    /// A sample sign was unavailable or uncertified.
    UnknownSampleSign,
    /// Primitive sample lowering failed or produced a non-finite value.
    NonFinitePrimitiveSample,
    /// The report was built from primitive samples without retained SDF replay.
    LossyPrimitiveSamples,
    /// Prepared SDF evidence is stale relative to its source version.
    StaleSource,
    /// A zero endpoint or zero edge requires tie handling before topology use.
    DegenerateZeroTouch,
    /// The local sign pattern has more than one plausible connectivity.
    AmbiguousCellTopology,
    /// An active edge lacks a unique exact edge root.
    UnsupportedEdgeRoot,
    /// An active exact edge root lacks a certified exact normal.
    MissingGradientEvidence,
}

/// One point of the signed regular grid consumed by the report.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfDualGridSampleRecord {
    /// Linear z-major, y-then-x-fast index.
    pub linear_index: usize,
    /// Integer grid coordinate `[x, y, z]`.
    pub grid_index: [u32; 3],
    /// Exact query point for the grid sample.
    pub point: Point3,
    /// Exact scalar value when the source was replayed through an SDF.
    pub exact_value: Option<Real>,
    /// Primitive lowered value retained from the sampling report.
    pub primitive_value: Option<f64>,
    /// Reported sign used by crossing discovery.
    pub sign: SdfGridSampleSign,
}

/// Active edge crossing retained for a dual-contouring proposal.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfDualEdgeCrossingRecord {
    /// Crossing-row index in the report.
    pub index: usize,
    /// Lower grid coordinate of the positive-axis edge.
    pub lower_grid_index: [u32; 3],
    /// Upper grid coordinate of the positive-axis edge.
    pub upper_grid_index: [u32; 3],
    /// Axis direction of the edge.
    pub axis: SdfDualGridEdgeAxis,
    /// Lower endpoint sample index.
    pub lower_sample_index: usize,
    /// Upper endpoint sample index.
    pub upper_sample_index: usize,
    /// Lower endpoint sign.
    pub lower_sign: SdfGridSampleSign,
    /// Upper endpoint sign.
    pub upper_sign: SdfGridSampleSign,
    /// Active-edge relation.
    pub crossing_kind: SdfDualEdgeCrossingKind,
    /// Root construction evidence.
    pub root_evidence: SdfDualEdgeRootEvidence,
    /// Exact edge parameter when a unique exact root exists.
    pub root_t: Option<Real>,
    /// Exact root point when a unique exact root exists.
    pub point: Option<Point3>,
    /// Exact unnormalized Hermite normal when certified.
    pub normal: Option<Vector3>,
    /// Normal evidence state.
    pub normal_evidence: SdfDualNormalEvidence,
}

impl SdfDualEdgeCrossingRecord {
    /// Returns whether this crossing has a unique exact root and exact normal.
    pub fn exact_hermite_ready(&self) -> bool {
        self.root_evidence.has_exact_unique_root()
            && self.point.is_some()
            && self.normal.is_some()
            && matches!(self.normal_evidence, SdfDualNormalEvidence::ExactSymbolic)
    }
}

/// One exact Hermite plane term used by a cell-local QEF.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfDualQefTerm {
    /// Source edge-crossing row.
    pub crossing_edge_index: usize,
    /// Exact point on the level set.
    pub point: Point3,
    /// Exact unnormalized normal direction.
    pub normal: Vector3,
}

/// Per-cell active topology and QEF readiness.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfDualCellReport {
    /// Integer cell coordinate `[x, y, z]`.
    pub cell_index: [u32; 3],
    /// Active-cell topology status.
    pub topology_status: SdfDualCellTopologyStatus,
    /// Active edge rows incident to this cell.
    pub crossing_edge_indices: Vec<usize>,
    /// Exact Hermite QEF rows retained for this cell.
    pub qef_terms: Vec<SdfDualQefTerm>,
    /// Vertex placement readiness for this cell.
    pub placement_status: SdfDualVertexPlacementStatus,
    /// Exact zero-residual affine candidate when one can be constructed.
    ///
    /// For affine fields, every exact Hermite plane is the same plane. The
    /// centroid of exact edge roots therefore has zero QEF residual, but it is
    /// still only a proposal vertex until a mesh owner validates cell topology.
    pub proposal_vertex: Option<Point3>,
    /// Cell-local blockers.
    pub blockers: Vec<SdfDualContouringBlocker>,
}

impl SdfDualCellReport {
    /// Returns whether this active cell has exact Hermite QEF input.
    pub fn exact_qef_ready(&self) -> bool {
        matches!(
            self.placement_status,
            SdfDualVertexPlacementStatus::ExactAffineQefCandidate
        ) && self.blockers.is_empty()
    }
}

/// Signed-grid dual-contouring proposal report.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfDualContouringReport {
    /// Source route used to build this report.
    pub source: SdfDualContouringSource,
    /// Metric claim of the retained expression or sampled grid.
    pub metric_status: SdfMetricStatus,
    /// Prepared-source freshness.
    pub freshness: SdfFreshness,
    /// Original regular-grid sample report.
    pub grid_samples: SdfGridSamplingReport,
    /// Point sample records with sign evidence.
    pub samples: Vec<SdfDualGridSampleRecord>,
    /// Active edge-crossing records.
    pub crossings: Vec<SdfDualEdgeCrossingRecord>,
    /// Per-cell topology and QEF reports.
    pub cells: Vec<SdfDualCellReport>,
    /// Declared sample count after grid validation.
    pub sample_count: usize,
    /// Number of unknown sample signs.
    pub unknown_sample_count: usize,
    /// Number of primitive sample rows without finite values.
    pub non_finite_sample_count: usize,
    /// Number of active grid edges.
    pub crossing_edge_count: usize,
    /// Number of cells with active crossings.
    pub active_cell_count: usize,
    /// Number of active edges with unique exact roots.
    pub exact_edge_root_count: usize,
    /// Number of active edges backed only by primitive signed samples.
    pub sampled_edge_root_count: usize,
    /// Number of active edges touching a zero endpoint or zero edge.
    pub zero_touch_edge_count: usize,
    /// Global blockers for validation handoff.
    pub blockers: Vec<SdfDualContouringBlocker>,
    /// Whether a mesh owner may consume this as exact validation handoff input.
    pub validation_handoff_ready: bool,
}

impl SdfDualContouringReport {
    /// Build a proposal report from primitive signed-grid samples only.
    ///
    /// This route is useful for imported signed volumes and sampled SDF
    /// previews. It never silently promotes primitive values to topology
    /// evidence: [`SdfDualContouringBlocker::LossyPrimitiveSamples`] remains in
    /// the report until a retained SDF replay supplies exact signs, roots, and
    /// normals.
    pub fn from_signed_grid_samples(grid_samples: SdfGridSamplingReport) -> Self {
        build_dual_contouring_report(None, grid_samples)
    }

    /// Validate summary fields against retained rows.
    pub fn is_self_consistent(&self) -> bool {
        self.grid_samples.is_self_consistent()
            && self.sample_count == self.samples.len()
            && self.unknown_sample_count
                == self
                    .samples
                    .iter()
                    .filter(|sample| sample.sign == SdfGridSampleSign::Unknown)
                    .count()
            && self.non_finite_sample_count
                == self
                    .samples
                    .iter()
                    .filter(|sample| sample.primitive_value.is_none())
                    .count()
            && self.crossing_edge_count == self.crossings.len()
            && self.exact_edge_root_count
                == self
                    .crossings
                    .iter()
                    .filter(|crossing| crossing.root_evidence.has_exact_unique_root())
                    .count()
            && self.sampled_edge_root_count
                == self
                    .crossings
                    .iter()
                    .filter(|crossing| {
                        crossing.root_evidence == SdfDualEdgeRootEvidence::SampledSignChangeOnly
                    })
                    .count()
            && self.zero_touch_edge_count
                == self
                    .crossings
                    .iter()
                    .filter(|crossing| {
                        matches!(
                            crossing.crossing_kind,
                            SdfDualEdgeCrossingKind::EndpointTouch
                                | SdfDualEdgeCrossingKind::CoincidentZeroEdge
                        )
                    })
                    .count()
            && self.active_cell_count
                == self
                    .cells
                    .iter()
                    .filter(|cell| {
                        !matches!(cell.topology_status, SdfDualCellTopologyStatus::Inactive)
                    })
                    .count()
            && self.cells.iter().all(|cell| {
                cell.crossing_edge_indices
                    .iter()
                    .all(|index| *index < self.crossings.len())
                    && cell
                        .qef_terms
                        .iter()
                        .all(|term| term.crossing_edge_index < self.crossings.len())
            })
            && self.validation_handoff_ready == self.blockers.is_empty()
    }
}

pub(crate) fn dual_contouring_report_from_exact_sdf(
    expr: &SdfExpr,
    grid_samples: SdfGridSamplingReport,
) -> SdfDualContouringReport {
    build_dual_contouring_report(Some(expr), grid_samples)
}

fn build_dual_contouring_report(
    expr: Option<&SdfExpr>,
    grid_samples: SdfGridSamplingReport,
) -> SdfDualContouringReport {
    let source = if expr.is_some() {
        SdfDualContouringSource::ExactSdfReplay
    } else {
        SdfDualContouringSource::SignedGridSamples
    };
    let metric_status = if expr.is_some() {
        grid_samples.samples.metric_status
    } else {
        SdfMetricStatus::SampledApproximation
    };
    let freshness = grid_samples.samples.freshness;
    let expected_count = grid_samples.grid.point_count().ok();
    let invalid_sample_count = expected_count != Some(grid_samples.samples.samples.len());
    let grid_too_small = grid_samples
        .grid
        .dimensions
        .iter()
        .any(|dimension| *dimension < 2);

    let samples = build_sample_records(expr, &grid_samples, expected_count);
    let mut blockers = Vec::new();
    if invalid_sample_count {
        push_blocker(&mut blockers, SdfDualContouringBlocker::InvalidSampleCount);
    }
    if grid_too_small {
        push_blocker(
            &mut blockers,
            SdfDualContouringBlocker::GridTooSmallForCells,
        );
    }
    if matches!(source, SdfDualContouringSource::SignedGridSamples) {
        push_blocker(
            &mut blockers,
            SdfDualContouringBlocker::LossyPrimitiveSamples,
        );
    }
    if matches!(freshness, SdfFreshness::Stale) {
        push_blocker(&mut blockers, SdfDualContouringBlocker::StaleSource);
    }

    let unknown_sample_count = samples
        .iter()
        .filter(|sample| sample.sign == SdfGridSampleSign::Unknown)
        .count();
    if unknown_sample_count > 0 {
        push_blocker(&mut blockers, SdfDualContouringBlocker::UnknownSampleSign);
    }
    let non_finite_sample_count = samples
        .iter()
        .filter(|sample| sample.primitive_value.is_none())
        .count();
    if non_finite_sample_count > 0 {
        push_blocker(
            &mut blockers,
            SdfDualContouringBlocker::NonFinitePrimitiveSample,
        );
    }

    let mut edge_index = BTreeMap::new();
    let crossings = if invalid_sample_count || grid_too_small {
        Vec::new()
    } else {
        build_crossings(expr, &grid_samples, &samples, &mut edge_index)
    };
    let cells = if invalid_sample_count || grid_too_small {
        Vec::new()
    } else {
        build_cell_reports(&grid_samples, &samples, &crossings, &edge_index)
    };

    for crossing in &crossings {
        if crossing.root_evidence == SdfDualEdgeRootEvidence::Unknown
            || crossing.root_evidence == SdfDualEdgeRootEvidence::CoincidentZeroEdge
        {
            push_blocker(&mut blockers, SdfDualContouringBlocker::UnsupportedEdgeRoot);
        }
        if crossing.point.is_some()
            && !matches!(
                crossing.normal_evidence,
                SdfDualNormalEvidence::ExactSymbolic
            )
        {
            push_blocker(
                &mut blockers,
                SdfDualContouringBlocker::MissingGradientEvidence,
            );
        }
    }
    for cell in &cells {
        for blocker in &cell.blockers {
            push_blocker(&mut blockers, *blocker);
        }
    }

    let exact_edge_root_count = crossings
        .iter()
        .filter(|crossing| crossing.root_evidence.has_exact_unique_root())
        .count();
    let sampled_edge_root_count = crossings
        .iter()
        .filter(|crossing| crossing.root_evidence == SdfDualEdgeRootEvidence::SampledSignChangeOnly)
        .count();
    let zero_touch_edge_count = crossings
        .iter()
        .filter(|crossing| {
            matches!(
                crossing.crossing_kind,
                SdfDualEdgeCrossingKind::EndpointTouch
                    | SdfDualEdgeCrossingKind::CoincidentZeroEdge
            )
        })
        .count();
    let active_cell_count = cells
        .iter()
        .filter(|cell| !matches!(cell.topology_status, SdfDualCellTopologyStatus::Inactive))
        .count();
    let validation_handoff_ready = blockers.is_empty();
    SdfDualContouringReport {
        source,
        metric_status,
        freshness,
        grid_samples,
        samples,
        crossing_edge_count: crossings.len(),
        crossings,
        active_cell_count,
        cells,
        sample_count: expected_count.unwrap_or(0),
        unknown_sample_count,
        non_finite_sample_count,
        exact_edge_root_count,
        sampled_edge_root_count,
        zero_touch_edge_count,
        blockers,
        validation_handoff_ready,
    }
}

fn build_sample_records(
    expr: Option<&SdfExpr>,
    grid_samples: &SdfGridSamplingReport,
    expected_count: Option<usize>,
) -> Vec<SdfDualGridSampleRecord> {
    let Some(expected_count) = expected_count else {
        return Vec::new();
    };
    let [nx, ny, _] = grid_samples.grid.dimensions;
    grid_samples
        .samples
        .samples
        .iter()
        .take(expected_count)
        .enumerate()
        .map(|(linear_index, sample)| {
            let grid_index = delinearize(linear_index, nx as usize, ny as usize);
            let exact_value =
                expr.and_then(|expr| crate::sampling::scalar_expr_point(expr, &sample.point));
            let sign = if expr.is_some() {
                SdfGridSampleSign::from_exact(exact_value.as_ref())
            } else {
                SdfGridSampleSign::from_primitive(sample.value)
            };
            SdfDualGridSampleRecord {
                linear_index,
                grid_index,
                point: sample.point.clone(),
                exact_value,
                primitive_value: finite_sample_value(sample),
                sign,
            }
        })
        .collect()
}

fn finite_sample_value(sample: &SdfPreviewSample) -> Option<f64> {
    sample.value.filter(|value| value.is_finite())
}

fn build_crossings(
    expr: Option<&SdfExpr>,
    grid_samples: &SdfGridSamplingReport,
    samples: &[SdfDualGridSampleRecord],
    edge_index: &mut BTreeMap<GridEdgeKey, usize>,
) -> Vec<SdfDualEdgeCrossingRecord> {
    let [nx, ny, nz] = grid_samples.grid.dimensions;
    let (nx, ny, nz) = (nx as usize, ny as usize, nz as usize);
    let affine_edges = expr.is_some_and(expr_affine_on_grid_edges);
    let mut crossings = Vec::new();
    for z in 0..nz {
        for y in 0..ny {
            for x in 0..nx {
                for axis in active_axes_from_point(x, y, z, nx, ny, nz) {
                    let key = GridEdgeKey::new(x, y, z, axis);
                    let lower_sample_index = grid_index(x, y, z, nx, ny);
                    let [ux, uy, uz] = key.upper();
                    let upper_sample_index = grid_index(ux, uy, uz, nx, ny);
                    let Some(lower) = samples.get(lower_sample_index) else {
                        continue;
                    };
                    let Some(upper) = samples.get(upper_sample_index) else {
                        continue;
                    };
                    let Some(crossing_kind) = crossing_kind(lower.sign, upper.sign) else {
                        continue;
                    };
                    let (root_evidence, root_t, point) = edge_root(
                        expr,
                        affine_edges,
                        crossing_kind,
                        &lower.point,
                        lower.exact_value.as_ref(),
                        &upper.point,
                        upper.exact_value.as_ref(),
                    );
                    let (normal, normal_evidence) = point
                        .as_ref()
                        .and_then(|point| expr.map(|expr| exact_normal(expr, point)))
                        .unwrap_or((None, SdfDualNormalEvidence::Missing));
                    let index = crossings.len();
                    edge_index.insert(key, index);
                    crossings.push(SdfDualEdgeCrossingRecord {
                        index,
                        lower_grid_index: lower.grid_index,
                        upper_grid_index: upper.grid_index,
                        axis,
                        lower_sample_index,
                        upper_sample_index,
                        lower_sign: lower.sign,
                        upper_sign: upper.sign,
                        crossing_kind,
                        root_evidence,
                        root_t,
                        point,
                        normal,
                        normal_evidence,
                    });
                }
            }
        }
    }
    crossings
}

fn build_cell_reports(
    grid_samples: &SdfGridSamplingReport,
    samples: &[SdfDualGridSampleRecord],
    crossings: &[SdfDualEdgeCrossingRecord],
    edge_index: &BTreeMap<GridEdgeKey, usize>,
) -> Vec<SdfDualCellReport> {
    let [nx, ny, nz] = grid_samples.grid.dimensions;
    let (nx, ny, nz) = (nx as usize, ny as usize, nz as usize);
    let mut cells = Vec::new();
    for z in 0..(nz - 1) {
        for y in 0..(ny - 1) {
            for x in 0..(nx - 1) {
                let cell_index = [x as u32, y as u32, z as u32];
                let crossing_edge_indices = cell_edge_keys(x, y, z)
                    .iter()
                    .filter_map(|key| edge_index.get(key).copied())
                    .collect::<Vec<_>>();
                let corner_signs = cell_corner_signs(x, y, z, nx, ny, samples);
                let topology_status =
                    cell_topology_status(&crossing_edge_indices, &corner_signs, crossings);
                let qef_terms = crossing_edge_indices
                    .iter()
                    .filter_map(|index| {
                        let crossing = &crossings[*index];
                        Some(SdfDualQefTerm {
                            crossing_edge_index: *index,
                            point: crossing.point.clone()?,
                            normal: crossing.normal.clone()?,
                        })
                    })
                    .collect::<Vec<_>>();
                let mut blockers = Vec::new();
                match topology_status {
                    SdfDualCellTopologyStatus::Inactive => {}
                    SdfDualCellTopologyStatus::DegenerateZeroTouch => {
                        push_blocker(&mut blockers, SdfDualContouringBlocker::DegenerateZeroTouch);
                    }
                    SdfDualCellTopologyStatus::Ambiguous => {
                        push_blocker(
                            &mut blockers,
                            SdfDualContouringBlocker::AmbiguousCellTopology,
                        );
                    }
                    SdfDualCellTopologyStatus::Unknown => {
                        push_blocker(&mut blockers, SdfDualContouringBlocker::UnknownSampleSign);
                    }
                    SdfDualCellTopologyStatus::Active => {}
                }
                if matches!(topology_status, SdfDualCellTopologyStatus::Active) {
                    let missing_exact_root_or_normal = crossing_edge_indices
                        .iter()
                        .any(|index| !crossings[*index].exact_hermite_ready());
                    if missing_exact_root_or_normal {
                        let sampled_only = crossing_edge_indices.iter().any(|index| {
                            crossings[*index].root_evidence
                                == SdfDualEdgeRootEvidence::SampledSignChangeOnly
                        });
                        if !sampled_only {
                            push_blocker(
                                &mut blockers,
                                SdfDualContouringBlocker::UnsupportedEdgeRoot,
                            );
                        }
                    }
                }
                let proposal_vertex =
                    if matches!(topology_status, SdfDualCellTopologyStatus::Active)
                        && qef_terms.len() == crossing_edge_indices.len()
                        && !qef_terms.is_empty()
                    {
                        let points = qef_terms
                            .iter()
                            .map(|term| term.point.clone())
                            .collect::<Vec<_>>();
                        Point3::centroid(&points)
                    } else {
                        None
                    };
                let placement_status = match topology_status {
                    SdfDualCellTopologyStatus::Inactive => SdfDualVertexPlacementStatus::NotActive,
                    SdfDualCellTopologyStatus::Active
                        if proposal_vertex.is_some() && blockers.is_empty() =>
                    {
                        SdfDualVertexPlacementStatus::ExactAffineQefCandidate
                    }
                    SdfDualCellTopologyStatus::Active
                        if crossing_edge_indices.iter().any(|index| {
                            crossings[*index].root_evidence
                                == SdfDualEdgeRootEvidence::SampledSignChangeOnly
                        }) =>
                    {
                        SdfDualVertexPlacementStatus::ProposalOnly
                    }
                    _ => SdfDualVertexPlacementStatus::Blocked,
                };
                cells.push(SdfDualCellReport {
                    cell_index,
                    topology_status,
                    crossing_edge_indices,
                    qef_terms,
                    placement_status,
                    proposal_vertex,
                    blockers,
                });
            }
        }
    }
    cells
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
struct GridEdgeKey {
    x: usize,
    y: usize,
    z: usize,
    axis: SdfDualGridEdgeAxis,
}

impl GridEdgeKey {
    const fn new(x: usize, y: usize, z: usize, axis: SdfDualGridEdgeAxis) -> Self {
        Self { x, y, z, axis }
    }

    const fn upper(self) -> [usize; 3] {
        match self.axis {
            SdfDualGridEdgeAxis::X => [self.x + 1, self.y, self.z],
            SdfDualGridEdgeAxis::Y => [self.x, self.y + 1, self.z],
            SdfDualGridEdgeAxis::Z => [self.x, self.y, self.z + 1],
        }
    }
}

fn active_axes_from_point(
    x: usize,
    y: usize,
    z: usize,
    nx: usize,
    ny: usize,
    nz: usize,
) -> impl Iterator<Item = SdfDualGridEdgeAxis> {
    [
        (x + 1 < nx).then_some(SdfDualGridEdgeAxis::X),
        (y + 1 < ny).then_some(SdfDualGridEdgeAxis::Y),
        (z + 1 < nz).then_some(SdfDualGridEdgeAxis::Z),
    ]
    .into_iter()
    .flatten()
}

fn cell_edge_keys(x: usize, y: usize, z: usize) -> [GridEdgeKey; 12] {
    [
        GridEdgeKey::new(x, y, z, SdfDualGridEdgeAxis::X),
        GridEdgeKey::new(x, y + 1, z, SdfDualGridEdgeAxis::X),
        GridEdgeKey::new(x, y, z + 1, SdfDualGridEdgeAxis::X),
        GridEdgeKey::new(x, y + 1, z + 1, SdfDualGridEdgeAxis::X),
        GridEdgeKey::new(x, y, z, SdfDualGridEdgeAxis::Y),
        GridEdgeKey::new(x + 1, y, z, SdfDualGridEdgeAxis::Y),
        GridEdgeKey::new(x, y, z + 1, SdfDualGridEdgeAxis::Y),
        GridEdgeKey::new(x + 1, y, z + 1, SdfDualGridEdgeAxis::Y),
        GridEdgeKey::new(x, y, z, SdfDualGridEdgeAxis::Z),
        GridEdgeKey::new(x + 1, y, z, SdfDualGridEdgeAxis::Z),
        GridEdgeKey::new(x, y + 1, z, SdfDualGridEdgeAxis::Z),
        GridEdgeKey::new(x + 1, y + 1, z, SdfDualGridEdgeAxis::Z),
    ]
}

fn cell_corner_signs(
    x: usize,
    y: usize,
    z: usize,
    nx: usize,
    ny: usize,
    samples: &[SdfDualGridSampleRecord],
) -> [SdfGridSampleSign; 8] {
    [
        samples[grid_index(x, y, z, nx, ny)].sign,
        samples[grid_index(x + 1, y, z, nx, ny)].sign,
        samples[grid_index(x, y + 1, z, nx, ny)].sign,
        samples[grid_index(x + 1, y + 1, z, nx, ny)].sign,
        samples[grid_index(x, y, z + 1, nx, ny)].sign,
        samples[grid_index(x + 1, y, z + 1, nx, ny)].sign,
        samples[grid_index(x, y + 1, z + 1, nx, ny)].sign,
        samples[grid_index(x + 1, y + 1, z + 1, nx, ny)].sign,
    ]
}

fn cell_topology_status(
    crossing_edge_indices: &[usize],
    corner_signs: &[SdfGridSampleSign; 8],
    crossings: &[SdfDualEdgeCrossingRecord],
) -> SdfDualCellTopologyStatus {
    if crossing_edge_indices.is_empty() {
        return SdfDualCellTopologyStatus::Inactive;
    }
    if corner_signs.iter().any(|sign| !sign.is_known()) {
        return SdfDualCellTopologyStatus::Unknown;
    }
    if crossings_for_cell_touch_zero(crossing_edge_indices, crossings) {
        return SdfDualCellTopologyStatus::DegenerateZeroTouch;
    }
    if crossing_edge_indices.len() > 6 || face_checkerboard(corner_signs) {
        return SdfDualCellTopologyStatus::Ambiguous;
    }
    SdfDualCellTopologyStatus::Active
}

fn crossings_for_cell_touch_zero(
    crossing_edge_indices: &[usize],
    crossings: &[SdfDualEdgeCrossingRecord],
) -> bool {
    crossing_edge_indices.iter().any(|index| {
        matches!(
            crossings[*index].crossing_kind,
            SdfDualEdgeCrossingKind::EndpointTouch | SdfDualEdgeCrossingKind::CoincidentZeroEdge
        )
    })
}

fn face_checkerboard(signs: &[SdfGridSampleSign; 8]) -> bool {
    const FACES: [[usize; 4]; 6] = [
        [0, 1, 3, 2],
        [4, 5, 7, 6],
        [0, 1, 5, 4],
        [2, 3, 7, 6],
        [0, 2, 6, 4],
        [1, 3, 7, 5],
    ];
    FACES.iter().any(|face| {
        let [a, b, c, d] = *face;
        signs[a] == signs[c]
            && signs[b] == signs[d]
            && signs[a] != signs[b]
            && signs[a].is_known()
            && signs[b].is_known()
    })
}

fn crossing_kind(
    lower: SdfGridSampleSign,
    upper: SdfGridSampleSign,
) -> Option<SdfDualEdgeCrossingKind> {
    match (lower, upper) {
        (SdfGridSampleSign::Unknown, _) | (_, SdfGridSampleSign::Unknown) => None,
        (SdfGridSampleSign::Zero, SdfGridSampleSign::Zero) => {
            Some(SdfDualEdgeCrossingKind::CoincidentZeroEdge)
        }
        (SdfGridSampleSign::Zero, _) | (_, SdfGridSampleSign::Zero) => {
            Some(SdfDualEdgeCrossingKind::EndpointTouch)
        }
        (SdfGridSampleSign::Negative, SdfGridSampleSign::Positive)
        | (SdfGridSampleSign::Positive, SdfGridSampleSign::Negative) => {
            Some(SdfDualEdgeCrossingKind::SignChange)
        }
        _ => None,
    }
}

fn edge_root(
    expr: Option<&SdfExpr>,
    affine_edges: bool,
    crossing_kind: SdfDualEdgeCrossingKind,
    lower_point: &Point3,
    lower_value: Option<&Real>,
    upper_point: &Point3,
    upper_value: Option<&Real>,
) -> (SdfDualEdgeRootEvidence, Option<Real>, Option<Point3>) {
    let Some(_expr) = expr else {
        return (SdfDualEdgeRootEvidence::SampledSignChangeOnly, None, None);
    };
    match crossing_kind {
        SdfDualEdgeCrossingKind::CoincidentZeroEdge => {
            return (SdfDualEdgeRootEvidence::CoincidentZeroEdge, None, None);
        }
        SdfDualEdgeCrossingKind::EndpointTouch => {
            let Some(lower_value) = lower_value else {
                return (SdfDualEdgeRootEvidence::Unknown, None, None);
            };
            let Some(upper_value) = upper_value else {
                return (SdfDualEdgeRootEvidence::Unknown, None, None);
            };
            if SdfGridSampleSign::from_exact(Some(lower_value)).is_zero() {
                return (
                    SdfDualEdgeRootEvidence::ExactEndpoint,
                    Some(Real::zero()),
                    Some(lower_point.clone()),
                );
            }
            if SdfGridSampleSign::from_exact(Some(upper_value)).is_zero() {
                return (
                    SdfDualEdgeRootEvidence::ExactEndpoint,
                    Some(Real::one()),
                    Some(upper_point.clone()),
                );
            }
            (SdfDualEdgeRootEvidence::Unknown, None, None)
        }
        SdfDualEdgeCrossingKind::SignChange if affine_edges => {
            let Some(lower_value) = lower_value else {
                return (SdfDualEdgeRootEvidence::Unknown, None, None);
            };
            let Some(upper_value) = upper_value else {
                return (SdfDualEdgeRootEvidence::Unknown, None, None);
            };
            let denominator = upper_value - lower_value;
            match -lower_value / &denominator {
                Ok(t) => {
                    let point = lower_point.lerp(upper_point, &t);
                    (
                        SdfDualEdgeRootEvidence::ExactAffineEdgeRoot,
                        Some(t),
                        Some(point),
                    )
                }
                Err(_) => (SdfDualEdgeRootEvidence::Unknown, None, None),
            }
        }
        SdfDualEdgeCrossingKind::SignChange => (SdfDualEdgeRootEvidence::Unknown, None, None),
    }
}

fn exact_normal(expr: &SdfExpr, point: &Point3) -> (Option<Vector3>, SdfDualNormalEvidence) {
    let outcome = gradient_expr_point(expr, point);
    let report = SdfGradientReport {
        point: point.clone(),
        gradient: outcome.clone().value(),
        gradient_status: SdfGradientStatus::ExactSymbolic,
        evidence: SdfEvidenceStatus::from_outcome(&outcome),
        freshness: SdfFreshness::Unversioned,
    };
    let normal = normal_from_gradient_report(report);
    if normal.normal_status == SdfNormalStatus::ExactDirection {
        (normal.normal, SdfDualNormalEvidence::ExactSymbolic)
    } else {
        (None, SdfDualNormalEvidence::Missing)
    }
}

fn expr_affine_on_grid_edges(expr: &SdfExpr) -> bool {
    match expr {
        SdfExpr::Constant(_) | SdfExpr::Coordinate(_) | SdfExpr::Linear { .. } => true,
        SdfExpr::Primitive(SdfPrimitive::Plane { .. }) => true,
        SdfExpr::Add(left, right) | SdfExpr::Sub(left, right) => {
            expr_affine_on_grid_edges(left) && expr_affine_on_grid_edges(right)
        }
        SdfExpr::Complement(inner) => expr_affine_on_grid_edges(inner),
        SdfExpr::Offset { child, .. } => expr_affine_on_grid_edges(child),
        SdfExpr::Transform { child, transform } => match transform {
            SdfTransform::Translation { .. } => expr_affine_on_grid_edges(child),
            SdfTransform::Affine { .. } => false,
        },
        SdfExpr::Primitive(_)
        | SdfExpr::Union(_, _)
        | SdfExpr::Intersection(_, _)
        | SdfExpr::Mul(_, _)
        | SdfExpr::Abs(_)
        | SdfExpr::Sqrt(_)
        | SdfExpr::Sin(_)
        | SdfExpr::Cos(_)
        | SdfExpr::Tan(_) => false,
    }
}

fn push_blocker(blockers: &mut Vec<SdfDualContouringBlocker>, blocker: SdfDualContouringBlocker) {
    if !blockers.contains(&blocker) {
        blockers.push(blocker);
    }
}

fn grid_index(x: usize, y: usize, z: usize, nx: usize, ny: usize) -> usize {
    x + nx * (y + ny * z)
}

fn delinearize(index: usize, nx: usize, ny: usize) -> [u32; 3] {
    let x = index % nx;
    let yz = index / nx;
    let y = yz % ny;
    let z = yz / ny;
    [x as u32, y as u32, z as u32]
}
