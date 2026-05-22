//! Approximated-gradient contouring proposal reports.
//!
//! This module records a classic sampled-field contouring envelope: primitive
//! signed samples, finite-difference gradients, projected surface candidates,
//! simple filters, and face-adjacent connectivity proposals. It deliberately
//! does **not** accept topology. Central/one-sided finite differences are the
//! standard sampled-gradient tool used by volume contouring pipelines; the
//! Hermite-data role follows Ju, Losasso, Schaefer, and Warren, "Dual
//! Contouring of Hermite Data" (SIGGRAPH 2002), while the sampled connectivity
//! lineage is the same family as Lorensen and Cline, "Marching Cubes" (1987).
//! Under Yap's "Towards Exact Geometric Computation" (1997), these primitive
//! gradients and projected points remain proposal evidence until replayed by
//! exact or proof-producing geometry.

use std::collections::VecDeque;

use hyperlimit::Point3;

use crate::sampling::{SdfGridSamplingReport, SdfPreviewSample};
use crate::status::{SdfFreshness, SdfMetricStatus};

/// Source route used to build an approximated-gradient contouring report.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfGradientContourSource {
    /// Samples were generated from a retained prepared SDF expression.
    PreparedSdfGrid,
    /// Samples came from an external or caller-supplied signed grid.
    ExternalSignedGrid,
}

/// Primitive sign used by sampled-gradient contouring.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfGradientContourSampleSign {
    /// Finite sample value is strictly negative.
    Negative,
    /// Finite sample value is exactly zero.
    Zero,
    /// Finite sample value is strictly positive.
    Positive,
    /// Sample value is missing or non-finite.
    Unknown,
}

impl SdfGradientContourSampleSign {
    /// Returns whether this sign is known.
    pub const fn is_known(self) -> bool {
        !matches!(self, Self::Unknown)
    }

    /// Returns whether this sign is zero.
    pub const fn is_zero(self) -> bool {
        matches!(self, Self::Zero)
    }

    fn from_value(value: Option<f64>) -> Self {
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

/// Finite-difference stencil used for one gradient component.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfFiniteDifferenceStencil {
    /// Central difference from neighboring samples on both sides.
    Central,
    /// Forward one-sided difference at the lower boundary.
    Forward,
    /// Backward one-sided difference at the upper boundary.
    Backward,
    /// No finite stencil was available.
    Missing,
}

/// Filter status for a projected contour candidate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfContourProjectionFilterStatus {
    /// The cell does not straddle or touch the zero level set.
    InactiveCell,
    /// One or more corner signs are unknown.
    UnknownSigns,
    /// A corner lies exactly on zero, so ownership needs exact tie handling.
    DegenerateZeroTouch,
    /// A finite-difference gradient was unavailable.
    MissingGradient,
    /// The averaged gradient has zero squared norm.
    ZeroGradient,
    /// Projection produced a non-finite primitive coordinate.
    NonFiniteProjection,
    /// Projection escaped the source cell.
    OutsideCell,
    /// The projected point passed primitive filters as a proposal.
    KeptProposal,
}

impl SdfContourProjectionFilterStatus {
    /// Returns whether the projected point survived primitive filters.
    pub const fn is_kept(self) -> bool {
        matches!(self, Self::KeptProposal)
    }
}

/// Connectivity status for two adjacent projected cells.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfContourConnectivityStatus {
    /// Face adjacency is proposed from sampled signs only.
    SampledFaceProposal,
}

/// Blockers that keep approximated-gradient contouring out of validation handoff.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfGradientContourBlocker {
    /// Grid dimensions or sample rows disagree.
    InvalidSampleCount,
    /// At least one grid dimension is too small to form cells.
    GridTooSmallForCells,
    /// A grid step could not be lowered to a finite nonzero primitive value.
    NonFiniteOrZeroStep,
    /// At least one sample value is missing or non-finite.
    NonFinitePrimitiveSample,
    /// The prepared source was stale when samples were taken.
    StaleSource,
    /// Primitive finite-difference gradients are lossy proposal evidence.
    LossyGradientApproximation,
    /// At least one active cell has unknown corner signs.
    UnknownSampleSign,
    /// At least one active cell touches the zero level set at a sample corner.
    DegenerateZeroTouch,
    /// At least one active cell lacks a usable finite-difference gradient.
    MissingGradient,
    /// At least one active cell has a zero averaged gradient.
    ZeroGradient,
    /// At least one projected point was non-finite.
    NonFiniteProjection,
    /// At least one projected point escaped the source cell.
    ProjectionOutsideCell,
    /// Connectivity came from sampled face signs, not exact topology replay.
    ConnectivityProposalOnly,
}

/// One primitive sample row consumed by the contouring report.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfGradientContourSampleRecord {
    /// Linear z-major, y-then-x-fast index.
    pub linear_index: usize,
    /// Integer grid coordinate `[x, y, z]`.
    pub grid_index: [u32; 3],
    /// Exact query point carried by the source sample report.
    pub point: Point3,
    /// Finite primitive scalar value, if available.
    pub value: Option<f64>,
    /// Primitive sign bucket used by filters and connectivity.
    pub sign: SdfGradientContourSampleSign,
}

/// Finite-difference gradient attached to one sample row.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfApproxGradientRecord {
    /// Source sample linear index.
    pub sample_index: usize,
    /// Grid coordinate of the source sample.
    pub grid_index: [u32; 3],
    /// Approximate primitive gradient.
    pub gradient: Option<[f64; 3]>,
    /// Stencil used for `[x, y, z]` components.
    pub stencils: [SdfFiniteDifferenceStencil; 3],
}

impl SdfApproxGradientRecord {
    /// Returns whether all gradient components are finite.
    pub fn is_finite(&self) -> bool {
        self.gradient
            .is_some_and(|gradient| gradient.iter().all(|value| value.is_finite()))
    }
}

/// Projected surface-point proposal for one active cell.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfProjectedSurfacePointRecord {
    /// Cell-row index in z-major, y-then-x-fast order.
    pub cell_linear_index: usize,
    /// Integer cell coordinate `[x, y, z]`.
    pub cell_index: [u32; 3],
    /// Corner sample rows in the fixed cube order used by the report.
    pub corner_sample_indices: [usize; 8],
    /// Corner primitive signs in the same order as `corner_sample_indices`.
    pub corner_signs: [SdfGradientContourSampleSign; 8],
    /// Primitive cell bounds as `[min, max]` in world coordinates.
    pub cell_bounds: [[f64; 3]; 2],
    /// Primitive cell center.
    pub center: [f64; 3],
    /// Averaged primitive scalar value at the cell center.
    pub averaged_value: Option<f64>,
    /// Averaged finite-difference gradient.
    pub averaged_gradient: Option<[f64; 3]>,
    /// Candidate point after one gradient projection step.
    pub projected_point: Option<[f64; 3]>,
    /// Filter result for the candidate.
    pub filter_status: SdfContourProjectionFilterStatus,
    /// Cell-local blockers.
    pub blockers: Vec<SdfGradientContourBlocker>,
}

impl SdfProjectedSurfacePointRecord {
    /// Returns whether this record carries a kept primitive proposal point.
    pub fn is_kept(&self) -> bool {
        self.filter_status.is_kept() && self.projected_point.is_some()
    }
}

/// Connectivity proposal between two face-adjacent projected cells.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfContourConnectivityEdge {
    /// Edge-row index.
    pub index: usize,
    /// Lower cell projection record.
    pub lower_projection_index: usize,
    /// Upper cell projection record.
    pub upper_projection_index: usize,
    /// Axis normal to the shared face, encoded as `0 = x`, `1 = y`, `2 = z`.
    pub axis: u8,
    /// Number of sampled sign crossings found on the shared face boundary.
    pub face_crossing_count: usize,
    /// Connectivity evidence status.
    pub status: SdfContourConnectivityStatus,
}

/// Approximated-gradient contouring proposal report.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfGradientContourReport {
    /// Source route used to build this report.
    pub source: SdfGradientContourSource,
    /// Metric claim inherited from the sample report.
    pub metric_status: SdfMetricStatus,
    /// Prepared-source freshness inherited from the sample report.
    pub freshness: SdfFreshness,
    /// Original sampled grid report.
    pub grid_samples: SdfGridSamplingReport,
    /// Primitive sample rows.
    pub samples: Vec<SdfGradientContourSampleRecord>,
    /// Approximate sample gradients.
    pub gradients: Vec<SdfApproxGradientRecord>,
    /// Per-cell projected surface proposals and filters.
    pub projections: Vec<SdfProjectedSurfacePointRecord>,
    /// Face-adjacent connectivity proposals between kept projected cells.
    pub connectivity: Vec<SdfContourConnectivityEdge>,
    /// Number of grid sample rows.
    pub sample_count: usize,
    /// Number of samples with unknown sign.
    pub unknown_sample_count: usize,
    /// Number of samples with missing or non-finite primitive values.
    pub non_finite_sample_count: usize,
    /// Number of gradients with all finite components.
    pub finite_gradient_count: usize,
    /// Number of active cells before filtering.
    pub active_cell_count: usize,
    /// Number of kept projected surface proposals.
    pub kept_projection_count: usize,
    /// Number of active cells rejected by filters.
    pub rejected_projection_count: usize,
    /// Number of sampled connectivity components among kept projections.
    pub connectivity_component_count: usize,
    /// Global blockers for validation handoff.
    pub blockers: Vec<SdfGradientContourBlocker>,
    /// Whether this report can be consumed as exact validation handoff.
    pub validation_handoff_ready: bool,
}

impl SdfGradientContourReport {
    /// Build an approximated-gradient report from caller-supplied signed samples.
    pub fn from_signed_grid_samples(grid_samples: SdfGridSamplingReport) -> Self {
        build_gradient_contour_report(SdfGradientContourSource::ExternalSignedGrid, grid_samples)
    }

    /// Validate summary fields against retained rows.
    pub fn is_self_consistent(&self) -> bool {
        self.grid_samples.is_self_consistent()
            && self.sample_count == self.samples.len()
            && self.unknown_sample_count
                == self
                    .samples
                    .iter()
                    .filter(|sample| sample.sign == SdfGradientContourSampleSign::Unknown)
                    .count()
            && self.non_finite_sample_count
                == self
                    .samples
                    .iter()
                    .filter(|sample| sample.value.is_none())
                    .count()
            && self.finite_gradient_count
                == self
                    .gradients
                    .iter()
                    .filter(|gradient| gradient.is_finite())
                    .count()
            && self.active_cell_count
                == self
                    .projections
                    .iter()
                    .filter(|projection| {
                        !matches!(
                            projection.filter_status,
                            SdfContourProjectionFilterStatus::InactiveCell
                        )
                    })
                    .count()
            && self.kept_projection_count
                == self
                    .projections
                    .iter()
                    .filter(|projection| projection.is_kept())
                    .count()
            && self.rejected_projection_count
                == self
                    .active_cell_count
                    .saturating_sub(self.kept_projection_count)
            && self.connectivity.iter().all(|edge| {
                edge.lower_projection_index < self.projections.len()
                    && edge.upper_projection_index < self.projections.len()
                    && self.projections[edge.lower_projection_index].is_kept()
                    && self.projections[edge.upper_projection_index].is_kept()
            })
            && self.validation_handoff_ready == self.blockers.is_empty()
    }
}

pub(crate) fn gradient_contour_report_from_prepared_grid(
    grid_samples: SdfGridSamplingReport,
) -> SdfGradientContourReport {
    build_gradient_contour_report(SdfGradientContourSource::PreparedSdfGrid, grid_samples)
}

fn build_gradient_contour_report(
    source: SdfGradientContourSource,
    grid_samples: SdfGridSamplingReport,
) -> SdfGradientContourReport {
    let metric_status = grid_samples.samples.metric_status;
    let freshness = grid_samples.samples.freshness;
    let expected_count = grid_samples.grid.point_count().ok();
    let actual_count = grid_samples.samples.samples.len();
    let invalid_sample_count = expected_count != Some(actual_count);
    let grid_too_small = grid_samples
        .grid
        .dimensions
        .iter()
        .any(|dimension| *dimension < 2);
    let primitive_step = primitive_step(&grid_samples);
    let bad_step = primitive_step.is_none();

    let samples = if invalid_sample_count {
        Vec::new()
    } else {
        build_sample_records(&grid_samples)
    };
    let gradients = if invalid_sample_count || bad_step {
        Vec::new()
    } else {
        build_gradients(
            &grid_samples,
            primitive_step.expect("checked primitive step"),
        )
    };
    let projections = if invalid_sample_count || grid_too_small || bad_step {
        Vec::new()
    } else {
        build_projections(&grid_samples, &samples, &gradients)
    };
    let connectivity = build_connectivity(&grid_samples, &projections);

    let mut blockers = Vec::new();
    push_blocker(
        &mut blockers,
        SdfGradientContourBlocker::LossyGradientApproximation,
    );
    if invalid_sample_count {
        push_blocker(&mut blockers, SdfGradientContourBlocker::InvalidSampleCount);
    }
    if grid_too_small {
        push_blocker(
            &mut blockers,
            SdfGradientContourBlocker::GridTooSmallForCells,
        );
    }
    if bad_step {
        push_blocker(
            &mut blockers,
            SdfGradientContourBlocker::NonFiniteOrZeroStep,
        );
    }
    if matches!(freshness, SdfFreshness::Stale) {
        push_blocker(&mut blockers, SdfGradientContourBlocker::StaleSource);
    }

    let unknown_sample_count = samples
        .iter()
        .filter(|sample| sample.sign == SdfGradientContourSampleSign::Unknown)
        .count();
    if unknown_sample_count > 0 {
        push_blocker(&mut blockers, SdfGradientContourBlocker::UnknownSampleSign);
    }
    let non_finite_sample_count = samples
        .iter()
        .filter(|sample| sample.value.is_none())
        .count();
    if non_finite_sample_count > 0 {
        push_blocker(
            &mut blockers,
            SdfGradientContourBlocker::NonFinitePrimitiveSample,
        );
    }
    for projection in &projections {
        for blocker in &projection.blockers {
            push_blocker(&mut blockers, *blocker);
        }
    }
    if !connectivity.is_empty() {
        push_blocker(
            &mut blockers,
            SdfGradientContourBlocker::ConnectivityProposalOnly,
        );
    }

    let active_cell_count = projections
        .iter()
        .filter(|projection| {
            !matches!(
                projection.filter_status,
                SdfContourProjectionFilterStatus::InactiveCell
            )
        })
        .count();
    let kept_projection_count = projections
        .iter()
        .filter(|projection| projection.is_kept())
        .count();
    let finite_gradient_count = gradients
        .iter()
        .filter(|gradient| gradient.is_finite())
        .count();
    let rejected_projection_count = active_cell_count.saturating_sub(kept_projection_count);
    let connectivity_component_count =
        connectivity_components(projections.len(), &connectivity, &projections);
    let validation_handoff_ready = blockers.is_empty();

    SdfGradientContourReport {
        source,
        metric_status,
        freshness,
        grid_samples,
        samples,
        gradients,
        projections,
        connectivity,
        sample_count: expected_count.unwrap_or(0),
        unknown_sample_count,
        non_finite_sample_count,
        finite_gradient_count,
        active_cell_count,
        kept_projection_count,
        rejected_projection_count,
        connectivity_component_count,
        blockers,
        validation_handoff_ready,
    }
}

fn build_sample_records(
    grid_samples: &SdfGridSamplingReport,
) -> Vec<SdfGradientContourSampleRecord> {
    let [nx, ny, _] = grid_samples.grid.dimensions;
    grid_samples
        .samples
        .samples
        .iter()
        .enumerate()
        .map(|(linear_index, sample)| {
            let value = finite_sample_value(sample);
            SdfGradientContourSampleRecord {
                linear_index,
                grid_index: delinearize(linear_index, nx as usize, ny as usize),
                point: sample.point.clone(),
                value,
                sign: SdfGradientContourSampleSign::from_value(value),
            }
        })
        .collect()
}

fn build_gradients(
    grid_samples: &SdfGridSamplingReport,
    step: [f64; 3],
) -> Vec<SdfApproxGradientRecord> {
    let [nx, ny, nz] = grid_samples.grid.dimensions;
    let (nx, ny, nz) = (nx as usize, ny as usize, nz as usize);
    let mut gradients = Vec::with_capacity(grid_samples.samples.samples.len());
    for z in 0..nz {
        for y in 0..ny {
            for x in 0..nx {
                let sample_index = grid_index(x, y, z, nx, ny);
                let (gx, sx) = finite_difference_component(grid_samples, x, y, z, 0, step[0]);
                let (gy, sy) = finite_difference_component(grid_samples, x, y, z, 1, step[1]);
                let (gz, sz) = finite_difference_component(grid_samples, x, y, z, 2, step[2]);
                let gradient = match (gx, gy, gz) {
                    (Some(gx), Some(gy), Some(gz))
                        if gx.is_finite() && gy.is_finite() && gz.is_finite() =>
                    {
                        Some([gx, gy, gz])
                    }
                    _ => None,
                };
                gradients.push(SdfApproxGradientRecord {
                    sample_index,
                    grid_index: [x as u32, y as u32, z as u32],
                    gradient,
                    stencils: [sx, sy, sz],
                });
            }
        }
    }
    gradients
}

fn finite_difference_component(
    grid_samples: &SdfGridSamplingReport,
    x: usize,
    y: usize,
    z: usize,
    axis: usize,
    step: f64,
) -> (Option<f64>, SdfFiniteDifferenceStencil) {
    if step == 0.0 || !step.is_finite() {
        return (None, SdfFiniteDifferenceStencil::Missing);
    }
    let [nx, ny, nz] = grid_samples.grid.dimensions;
    let dims = [nx as usize, ny as usize, nz as usize];
    let coord = [x, y, z][axis];
    if dims[axis] < 2 {
        return (None, SdfFiniteDifferenceStencil::Missing);
    }
    let mut minus = [x, y, z];
    let mut plus = [x, y, z];
    if coord > 0 && coord + 1 < dims[axis] {
        minus[axis] -= 1;
        plus[axis] += 1;
        let (Some(plus_value), Some(minus_value)) = (
            sample_value(grid_samples, plus),
            sample_value(grid_samples, minus),
        ) else {
            return (None, SdfFiniteDifferenceStencil::Central);
        };
        let value = (plus_value - minus_value) / (2.0 * step);
        return (Some(value), SdfFiniteDifferenceStencil::Central);
    }
    if coord + 1 < dims[axis] {
        plus[axis] += 1;
        let (Some(plus_value), Some(center_value)) = (
            sample_value(grid_samples, plus),
            sample_value(grid_samples, [x, y, z]),
        ) else {
            return (None, SdfFiniteDifferenceStencil::Forward);
        };
        let value = (plus_value - center_value) / step;
        return (Some(value), SdfFiniteDifferenceStencil::Forward);
    }
    if coord > 0 {
        minus[axis] -= 1;
        let (Some(center_value), Some(minus_value)) = (
            sample_value(grid_samples, [x, y, z]),
            sample_value(grid_samples, minus),
        ) else {
            return (None, SdfFiniteDifferenceStencil::Backward);
        };
        let value = (center_value - minus_value) / step;
        return (Some(value), SdfFiniteDifferenceStencil::Backward);
    }
    (None, SdfFiniteDifferenceStencil::Missing)
}

fn build_projections(
    grid_samples: &SdfGridSamplingReport,
    samples: &[SdfGradientContourSampleRecord],
    gradients: &[SdfApproxGradientRecord],
) -> Vec<SdfProjectedSurfacePointRecord> {
    let [nx, ny, nz] = grid_samples.grid.dimensions;
    let (nx, ny, nz) = (nx as usize, ny as usize, nz as usize);
    let mut projections = Vec::new();
    for z in 0..(nz - 1) {
        for y in 0..(ny - 1) {
            for x in 0..(nx - 1) {
                let cell_linear_index = grid_index(x, y, z, nx - 1, ny - 1);
                let corner_sample_indices = cell_corner_indices(x, y, z, nx, ny);
                let cell_bounds = cell_bounds(grid_samples, corner_sample_indices);
                let center = midpoint(cell_bounds);
                let mut blockers = Vec::new();
                let signs = corner_sample_indices.map(|index| samples[index].sign);
                let active = cell_active(&signs);
                let zero_touch = signs.iter().any(|sign| sign.is_zero());
                let unknown_sign = signs.iter().any(|sign| !sign.is_known());

                let (averaged_value, averaged_gradient, projected_point, filter_status) = if !active
                {
                    (
                        None,
                        None,
                        None,
                        SdfContourProjectionFilterStatus::InactiveCell,
                    )
                } else if unknown_sign {
                    push_blocker(&mut blockers, SdfGradientContourBlocker::UnknownSampleSign);
                    (
                        None,
                        None,
                        None,
                        SdfContourProjectionFilterStatus::UnknownSigns,
                    )
                } else if zero_touch {
                    push_blocker(
                        &mut blockers,
                        SdfGradientContourBlocker::DegenerateZeroTouch,
                    );
                    (
                        average_values(samples, corner_sample_indices),
                        average_gradients(gradients, corner_sample_indices),
                        None,
                        SdfContourProjectionFilterStatus::DegenerateZeroTouch,
                    )
                } else {
                    let averaged_value = average_values(samples, corner_sample_indices);
                    let averaged_gradient = average_gradients(gradients, corner_sample_indices);
                    let Some(value) = averaged_value else {
                        push_blocker(
                            &mut blockers,
                            SdfGradientContourBlocker::NonFinitePrimitiveSample,
                        );
                        projections.push(SdfProjectedSurfacePointRecord {
                            cell_linear_index,
                            cell_index: [x as u32, y as u32, z as u32],
                            corner_sample_indices,
                            corner_signs: signs,
                            cell_bounds,
                            center,
                            averaged_value,
                            averaged_gradient,
                            projected_point: None,
                            filter_status: SdfContourProjectionFilterStatus::UnknownSigns,
                            blockers,
                        });
                        continue;
                    };
                    match averaged_gradient {
                        None => {
                            push_blocker(&mut blockers, SdfGradientContourBlocker::MissingGradient);
                            (
                                Some(value),
                                None,
                                None,
                                SdfContourProjectionFilterStatus::MissingGradient,
                            )
                        }
                        Some(gradient) => {
                            let norm_squared = dot(gradient, gradient);
                            if norm_squared == 0.0 {
                                push_blocker(
                                    &mut blockers,
                                    SdfGradientContourBlocker::ZeroGradient,
                                );
                                (
                                    Some(value),
                                    Some(gradient),
                                    None,
                                    SdfContourProjectionFilterStatus::ZeroGradient,
                                )
                            } else {
                                let scale = value / norm_squared;
                                let point = [
                                    center[0] - scale * gradient[0],
                                    center[1] - scale * gradient[1],
                                    center[2] - scale * gradient[2],
                                ];
                                if !point.iter().all(|value| value.is_finite()) {
                                    push_blocker(
                                        &mut blockers,
                                        SdfGradientContourBlocker::NonFiniteProjection,
                                    );
                                    (
                                        Some(value),
                                        Some(gradient),
                                        None,
                                        SdfContourProjectionFilterStatus::NonFiniteProjection,
                                    )
                                } else if !point_in_bounds(point, cell_bounds) {
                                    push_blocker(
                                        &mut blockers,
                                        SdfGradientContourBlocker::ProjectionOutsideCell,
                                    );
                                    (
                                        Some(value),
                                        Some(gradient),
                                        Some(point),
                                        SdfContourProjectionFilterStatus::OutsideCell,
                                    )
                                } else {
                                    (
                                        Some(value),
                                        Some(gradient),
                                        Some(point),
                                        SdfContourProjectionFilterStatus::KeptProposal,
                                    )
                                }
                            }
                        }
                    }
                };

                projections.push(SdfProjectedSurfacePointRecord {
                    cell_linear_index,
                    cell_index: [x as u32, y as u32, z as u32],
                    corner_sample_indices,
                    corner_signs: signs,
                    cell_bounds,
                    center,
                    averaged_value,
                    averaged_gradient,
                    projected_point,
                    filter_status,
                    blockers,
                });
            }
        }
    }
    projections
}

fn build_connectivity(
    grid_samples: &SdfGridSamplingReport,
    projections: &[SdfProjectedSurfacePointRecord],
) -> Vec<SdfContourConnectivityEdge> {
    let [nx, ny, nz] = grid_samples.grid.dimensions;
    if nx < 2 || ny < 2 || nz < 2 {
        return Vec::new();
    }
    let cell_dims = [nx as usize - 1, ny as usize - 1, nz as usize - 1];
    let expected_projection_count = cell_dims[0] * cell_dims[1] * cell_dims[2];
    if projections.len() != expected_projection_count {
        return Vec::new();
    }
    let mut edges = Vec::new();
    for z in 0..cell_dims[2] {
        for y in 0..cell_dims[1] {
            for x in 0..cell_dims[0] {
                let lower = grid_index(x, y, z, cell_dims[0], cell_dims[1]);
                if !projections[lower].is_kept() {
                    continue;
                }
                for axis in 0..3 {
                    let mut neighbor = [x, y, z];
                    if neighbor[axis] + 1 >= cell_dims[axis] {
                        continue;
                    }
                    neighbor[axis] += 1;
                    let upper = grid_index(
                        neighbor[0],
                        neighbor[1],
                        neighbor[2],
                        cell_dims[0],
                        cell_dims[1],
                    );
                    if !projections[upper].is_kept() {
                        continue;
                    }
                    let face_crossing_count =
                        shared_face_crossing_count(axis, &projections[lower], &projections[upper]);
                    if face_crossing_count == 0 {
                        continue;
                    }
                    edges.push(SdfContourConnectivityEdge {
                        index: edges.len(),
                        lower_projection_index: lower,
                        upper_projection_index: upper,
                        axis: axis as u8,
                        face_crossing_count,
                        status: SdfContourConnectivityStatus::SampledFaceProposal,
                    });
                }
            }
        }
    }
    edges
}

fn shared_face_crossing_count(
    axis: usize,
    lower: &SdfProjectedSurfacePointRecord,
    upper: &SdfProjectedSurfacePointRecord,
) -> usize {
    let mut count = 0_usize;
    for a in lower.corner_sample_indices {
        if !upper.corner_sample_indices.contains(&a) {
            continue;
        }
        for b in lower.corner_sample_indices {
            if !upper.corner_sample_indices.contains(&b) || a >= b {
                continue;
            }
            if sample_indices_form_face_edge(axis, a, b, lower, upper) {
                let Some(local_a) = lower
                    .corner_sample_indices
                    .iter()
                    .position(|index| *index == a)
                else {
                    continue;
                };
                let Some(local_b) = lower
                    .corner_sample_indices
                    .iter()
                    .position(|index| *index == b)
                else {
                    continue;
                };
                if signs_cross_or_touch(lower.corner_signs[local_a], lower.corner_signs[local_b]) {
                    count += 1;
                }
            }
        }
    }
    count
}

fn signs_cross_or_touch(a: SdfGradientContourSampleSign, b: SdfGradientContourSampleSign) -> bool {
    matches!(
        (a, b),
        (SdfGradientContourSampleSign::Zero, _)
            | (_, SdfGradientContourSampleSign::Zero)
            | (
                SdfGradientContourSampleSign::Negative,
                SdfGradientContourSampleSign::Positive
            )
            | (
                SdfGradientContourSampleSign::Positive,
                SdfGradientContourSampleSign::Negative
            )
    )
}

fn sample_indices_form_face_edge(
    axis: usize,
    a: usize,
    b: usize,
    lower: &SdfProjectedSurfacePointRecord,
    upper: &SdfProjectedSurfacePointRecord,
) -> bool {
    let Some(local_a) = lower
        .corner_sample_indices
        .iter()
        .position(|index| *index == a)
    else {
        return false;
    };
    let Some(local_b) = lower
        .corner_sample_indices
        .iter()
        .position(|index| *index == b)
    else {
        return false;
    };
    let ca = LOCAL_CORNERS[local_a];
    let cb = LOCAL_CORNERS[local_b];
    if ca[axis] == 0 || cb[axis] == 0 {
        return false;
    }
    let differing_axes = (0..3).filter(|axis| ca[*axis] != cb[*axis]).count();
    if differing_axes != 1 {
        return false;
    }
    let lower_cell = lower.cell_index;
    let upper_cell = upper.cell_index;
    lower_cell
        .iter()
        .zip(upper_cell.iter())
        .enumerate()
        .all(|(component, (lo, hi))| {
            if component == axis {
                *hi == *lo + 1
            } else {
                hi == lo
            }
        })
}

fn connectivity_components(
    projection_count: usize,
    connectivity: &[SdfContourConnectivityEdge],
    projections: &[SdfProjectedSurfacePointRecord],
) -> usize {
    let mut adjacency = vec![Vec::new(); projection_count];
    for edge in connectivity {
        adjacency[edge.lower_projection_index].push(edge.upper_projection_index);
        adjacency[edge.upper_projection_index].push(edge.lower_projection_index);
    }
    let mut visited = vec![false; projection_count];
    let mut components = 0_usize;
    for index in 0..projection_count {
        if visited[index] || !projections[index].is_kept() {
            continue;
        }
        components += 1;
        let mut queue = VecDeque::from([index]);
        visited[index] = true;
        while let Some(current) = queue.pop_front() {
            for next in &adjacency[current] {
                if !visited[*next] {
                    visited[*next] = true;
                    queue.push_back(*next);
                }
            }
        }
    }
    components
}

const LOCAL_CORNERS: [[u8; 3]; 8] = [
    [0, 0, 0],
    [1, 0, 0],
    [0, 1, 0],
    [1, 1, 0],
    [0, 0, 1],
    [1, 0, 1],
    [0, 1, 1],
    [1, 1, 1],
];

fn cell_corner_indices(x: usize, y: usize, z: usize, nx: usize, ny: usize) -> [usize; 8] {
    [
        grid_index(x, y, z, nx, ny),
        grid_index(x + 1, y, z, nx, ny),
        grid_index(x, y + 1, z, nx, ny),
        grid_index(x + 1, y + 1, z, nx, ny),
        grid_index(x, y, z + 1, nx, ny),
        grid_index(x + 1, y, z + 1, nx, ny),
        grid_index(x, y + 1, z + 1, nx, ny),
        grid_index(x + 1, y + 1, z + 1, nx, ny),
    ]
}

fn cell_active(signs: &[SdfGradientContourSampleSign; 8]) -> bool {
    let has_negative = signs
        .iter()
        .any(|sign| matches!(sign, SdfGradientContourSampleSign::Negative));
    let has_positive = signs
        .iter()
        .any(|sign| matches!(sign, SdfGradientContourSampleSign::Positive));
    let has_zero = signs.iter().any(|sign| sign.is_zero());
    has_zero || (has_negative && has_positive)
}

fn average_values(
    samples: &[SdfGradientContourSampleRecord],
    corner_indices: [usize; 8],
) -> Option<f64> {
    let mut sum = 0.0;
    for index in corner_indices {
        sum += samples[index].value?;
    }
    Some(sum / 8.0)
}

fn average_gradients(
    gradients: &[SdfApproxGradientRecord],
    corner_indices: [usize; 8],
) -> Option<[f64; 3]> {
    let mut sum = [0.0; 3];
    for index in corner_indices {
        let gradient = gradients[index].gradient?;
        sum[0] += gradient[0];
        sum[1] += gradient[1];
        sum[2] += gradient[2];
    }
    Some([sum[0] / 8.0, sum[1] / 8.0, sum[2] / 8.0])
}

fn primitive_step(grid_samples: &SdfGridSamplingReport) -> Option<[f64; 3]> {
    let step = [
        grid_samples.grid.step.x.to_f64_lossy()?,
        grid_samples.grid.step.y.to_f64_lossy()?,
        grid_samples.grid.step.z.to_f64_lossy()?,
    ];
    if step.iter().all(|value| value.is_finite() && *value != 0.0) {
        Some(step)
    } else {
        None
    }
}

fn cell_bounds(grid_samples: &SdfGridSamplingReport, corner_indices: [usize; 8]) -> [[f64; 3]; 2] {
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    for index in corner_indices {
        if let Some(point) = grid_samples.samples.samples[index]
            .point
            .to_f64_array_lossy()
        {
            for axis in 0..3 {
                min[axis] = min[axis].min(point[axis]);
                max[axis] = max[axis].max(point[axis]);
            }
        }
    }
    [min, max]
}

fn midpoint(bounds: [[f64; 3]; 2]) -> [f64; 3] {
    [
        (bounds[0][0] + bounds[1][0]) * 0.5,
        (bounds[0][1] + bounds[1][1]) * 0.5,
        (bounds[0][2] + bounds[1][2]) * 0.5,
    ]
}

fn point_in_bounds(point: [f64; 3], bounds: [[f64; 3]; 2]) -> bool {
    (0..3).all(|axis| point[axis] >= bounds[0][axis] && point[axis] <= bounds[1][axis])
}

fn dot(left: [f64; 3], right: [f64; 3]) -> f64 {
    left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
}

fn sample_value(grid_samples: &SdfGridSamplingReport, index: [usize; 3]) -> Option<f64> {
    let [nx, ny, _] = grid_samples.grid.dimensions;
    finite_sample_value(
        &grid_samples.samples.samples
            [grid_index(index[0], index[1], index[2], nx as usize, ny as usize)],
    )
}

fn finite_sample_value(sample: &SdfPreviewSample) -> Option<f64> {
    sample.value.filter(|value| value.is_finite())
}

fn push_blocker(blockers: &mut Vec<SdfGradientContourBlocker>, blocker: SdfGradientContourBlocker) {
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
