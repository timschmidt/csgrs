//! Exact axis-aligned box voxelization fixtures.
//!
//! This module is intentionally modest: it does not claim to solve general
//! triangle-mesh voxelization. It provides the first proof-carrying
//! geometry-to-grid path for exact axis-aligned boxes, using certified
//! comparisons over [`hyperreal::Real`]. The design follows Yap's exact
//! geometric computation principle that combinatorial classification must be
//! derived from exact/proof-producing predicates rather than primitive-float
//! epsilon tests. See Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry*, 1997, Sections 2 and 6.

use core::cmp::Ordering;

use hyperlimit::{Aabb3Intersection, PredicateOutcome};
use hyperreal::Real;

use crate::{
    BoundaryPolicy, GridFrame, GridSource, HypervoxelError, HypervoxelResult, MaterialRegionId,
    OccupancyState, QuantizationPolicy, SparseVoxelGrid, VoxelAddress, VoxelAggregateFacts,
    VoxelCell, VoxelPayload, VoxelPredicateCertificateReport, VoxelizationPolicy,
    VoxelizationReport,
};

/// Exact axis-aligned box in the same coordinates as a [`GridFrame`].
#[derive(Clone, Debug, PartialEq)]
pub struct ExactBox {
    /// Minimum exact corner.
    pub min: [Real; 3],
    /// Maximum exact corner.
    pub max: [Real; 3],
    /// Optional source provenance.
    pub source: Option<GridSource>,
}

/// Preflight report for an [`ExactBox`].
///
/// Exact voxelization may continue through undecided predicate comparisons as
/// explicit unknown cells, but source geometry with a certified inverted axis
/// is not a valid box. The report keeps that distinction visible, following
/// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
/// 7(1-2), 1997: invalid object structure is rejected, while undecided
/// comparisons remain explicit uncertainty instead of an epsilon repair.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExactBoxReport {
    /// Axes where `min <= max` was certified.
    pub ordered_axes: Vec<usize>,
    /// Axes where `min == max` was certified.
    ///
    /// A zero-thickness axis is ordered, but it is not a valid 3D box volume
    /// for source-geometry voxelization. Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7(1-2), 1997, distinguishes
    /// exact object structure from predicate accidents; degenerate solids are
    /// rejected rather than being promoted into boundary-only topology.
    pub zero_extent_axes: Vec<usize>,
    /// Axes where `min > max` was certified.
    pub invalid_axes: Vec<usize>,
    /// Axes whose endpoint ordering could not be certified.
    pub unknown_axes: Vec<usize>,
    /// Whether the box is ready for exact source-geometry use.
    pub exact_box_ready: bool,
}

impl ExactBox {
    /// Creates an exact box without proving min/max order up front.
    ///
    /// Ordering is certified during voxel classification so unresolved symbolic
    /// bounds become explicit unknown cells instead of construction-time
    /// panics.
    pub fn new(min: [Real; 3], max: [Real; 3], source: Option<GridSource>) -> Self {
        Self { min, max, source }
    }

    /// Reports whether the box endpoints form a valid exact AABB.
    pub fn report(&self) -> ExactBoxReport {
        let mut ordered_axes = Vec::new();
        let mut zero_extent_axes = Vec::new();
        let mut invalid_axes = Vec::new();
        let mut unknown_axes = Vec::new();
        for axis in 0..3 {
            match certified_cmp(&self.min[axis], &self.max[axis], axis) {
                Ok(Ordering::Less) => ordered_axes.push(axis),
                Ok(Ordering::Equal) => {
                    ordered_axes.push(axis);
                    zero_extent_axes.push(axis);
                }
                Ok(Ordering::Greater) => invalid_axes.push(axis),
                Err(HypervoxelError::UnknownOrdering { .. }) => unknown_axes.push(axis),
                Err(_) => unknown_axes.push(axis),
            }
        }
        let exact_box_ready =
            invalid_axes.is_empty() && unknown_axes.is_empty() && zero_extent_axes.is_empty();
        ExactBoxReport {
            ordered_axes,
            zero_extent_axes,
            invalid_axes,
            unknown_axes,
            exact_box_ready,
        }
    }
}

/// Classification of one cell against an exact box.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VoxelBoxClassifier {
    /// Cell is outside the box.
    Outside,
    /// Cell is fully inside the box.
    Inside,
    /// Cell intersects the box boundary or partially overlaps.
    Boundary,
    /// Exact ordering was not available.
    Unknown,
}

fn certified_cmp(left: &Real, right: &Real, axis: usize) -> HypervoxelResult<Ordering> {
    match hyperlimit::compare_reals(left, right).value() {
        Some(ordering) => Ok(ordering),
        None => Err(HypervoxelError::UnknownOrdering { axis }),
    }
}

/// Classifies one cell against an exact axis-aligned box.
pub fn classify_cell_against_box(
    address: VoxelAddress,
    frame: &GridFrame,
    exact_box: &ExactBox,
) -> HypervoxelResult<VoxelBoxClassifier> {
    let bounds = address.bounds(frame)?;
    let relation = decide(
        hyperlimit::classify_aabb3_intersection(
            &point3(&bounds.min),
            &point3(&bounds.max),
            &point3(&exact_box.min),
            &point3(&exact_box.max),
        ),
        0,
    )?;
    if relation == Aabb3Intersection::Disjoint {
        return Ok(VoxelBoxClassifier::Outside);
    }

    let min_inside = decide(
        hyperlimit::classify_point_aabb3(
            &point3(&exact_box.min),
            &point3(&exact_box.max),
            &point3(&bounds.min),
        ),
        0,
    )?;
    let max_inside = decide(
        hyperlimit::classify_point_aabb3(
            &point3(&exact_box.min),
            &point3(&exact_box.max),
            &point3(&bounds.max),
        ),
        0,
    )?;
    if min_inside.is_inside_or_boundary() && max_inside.is_inside_or_boundary() {
        Ok(VoxelBoxClassifier::Inside)
    } else {
        Ok(VoxelBoxClassifier::Boundary)
    }
}

fn decide<T>(outcome: PredicateOutcome<T>, axis: usize) -> HypervoxelResult<T> {
    outcome
        .value()
        .ok_or(HypervoxelError::UnknownOrdering { axis })
}

fn point3(values: &[Real; 3]) -> hyperlimit::Point3 {
    hyperlimit::Point3::new(values[0].clone(), values[1].clone(), values[2].clone())
}

/// Voxelizes an exact axis-aligned box into a semantic sparse grid.
pub fn voxelize_exact_box(
    frame: GridFrame,
    exact_box: &ExactBox,
    material: MaterialRegionId,
    policy: VoxelizationPolicy,
) -> HypervoxelResult<(SparseVoxelGrid, VoxelizationReport)> {
    let source_report = exact_box.report();
    if !source_report.invalid_axes.is_empty() {
        return Err(HypervoxelError::InvalidSourceGeometry {
            reason: "box minimum exceeds maximum",
        });
    }
    if !source_report.zero_extent_axes.is_empty() {
        return Err(HypervoxelError::InvalidSourceGeometry {
            reason: "box has zero extent",
        });
    }

    let mut grid = SparseVoxelGrid::new(frame.clone());
    let mut boundary_cells = 0;
    let mut unknown_cells = 0;
    let mut inside_cells = 0;
    let mut outside_cells = 0;
    let mut predicate_boundary_cells = 0;
    let mut predicate_unknown_cells = 0;
    let cells_per_axis = frame.cells_per_axis();

    for z in 0..cells_per_axis {
        for y in 0..cells_per_axis {
            for x in 0..cells_per_axis {
                let address = VoxelAddress::new(frame.depth(), [x, y, z])?;
                let classifier = match classify_cell_against_box(address, &frame, exact_box) {
                    Ok(classifier) => classifier,
                    Err(HypervoxelError::UnknownOrdering { .. }) => VoxelBoxClassifier::Unknown,
                    Err(err) => return Err(err),
                };
                match classifier {
                    VoxelBoxClassifier::Inside => inside_cells += 1,
                    VoxelBoxClassifier::Outside => outside_cells += 1,
                    VoxelBoxClassifier::Boundary => predicate_boundary_cells += 1,
                    VoxelBoxClassifier::Unknown => predicate_unknown_cells += 1,
                }

                let cell = match (policy.quantization, policy.boundary, classifier) {
                    (_, _, VoxelBoxClassifier::Outside) => VoxelCell::empty(),
                    (_, _, VoxelBoxClassifier::Unknown) => {
                        unknown_cells += 1;
                        VoxelCell::unknown()
                    }
                    (QuantizationPolicy::ConservativeInterior, _, VoxelBoxClassifier::Boundary) => {
                        boundary_cells += 1;
                        match policy.boundary {
                            BoundaryPolicy::BoundaryAsUnknown => {
                                unknown_cells += 1;
                                VoxelCell::unknown()
                            }
                            _ => VoxelCell::empty(),
                        }
                    }
                    (_, _, VoxelBoxClassifier::Inside) => VoxelCell::material(material),
                    (_, BoundaryPolicy::BoundaryAsUnknown, VoxelBoxClassifier::Boundary) => {
                        boundary_cells += 1;
                        unknown_cells += 1;
                        VoxelCell::unknown()
                    }
                    (_, BoundaryPolicy::LossySideChoice, VoxelBoxClassifier::Boundary) => {
                        boundary_cells += 1;
                        VoxelCell {
                            occupancy: OccupancyState::LossyAdapterValue,
                            payload: VoxelPayload::LossyAdapterValue(material.0),
                        }
                    }
                    (_, BoundaryPolicy::KeepBoundary, VoxelBoxClassifier::Boundary) => {
                        boundary_cells += 1;
                        VoxelCell::boundary(VoxelPayload::MaterialRegion(material))
                    }
                };

                if cell.occupancy != OccupancyState::Empty {
                    grid.set(address, cell)?;
                }
            }
        }
    }

    let aggregate = VoxelAggregateFacts::from_explicit_cells_in_frame(
        usize::try_from(cells_per_axis.pow(3)).map_err(|_| HypervoxelError::AddressOverflow)?,
        grid.iter().map(|(_, cell)| cell),
    )?;
    let report = VoxelizationReport {
        source: exact_box.source.clone(),
        frame,
        policy,
        aggregate,
        unknown_cells,
        boundary_cells,
        predicate_certificates: VoxelPredicateCertificateReport::from_counts(
            inside_cells,
            outside_cells,
            predicate_boundary_cells,
            predicate_unknown_cells,
        ),
        legacy_adapter: None,
    };
    Ok((grid, report))
}
