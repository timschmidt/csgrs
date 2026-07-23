//! Exact axis-aligned box voxelization fixtures.
//!
//! This module is intentionally modest: it does not claim to solve general
//! triangle-mesh voxelization. It provides a proof-carrying
//! geometry-to-grid path for exact axis-aligned boxes, using certified
//! comparisons over [`hyperreal::Real`]. Combinatorial classification comes
//! from proof-producing predicates rather than primitive-float epsilon tests.

use core::cmp::Ordering;

use hyperreal::Real;

use crate::{
    BoundaryPolicy, GridFrame, HypervoxelError, HypervoxelResult, MaterialRegionId, OccupancyState,
    QuantizationPolicy, SparseVoxelGrid, VoxelAddress, VoxelAggregateFacts, VoxelCell,
    VoxelPayload, VoxelPredicateCertificateReport, VoxelizationPolicy, VoxelizationReport,
};

/// Exact axis-aligned box in the same coordinates as a [`GridFrame`].
#[derive(Clone, Debug, PartialEq)]
pub struct ExactBox {
    /// Minimum exact corner.
    pub min: [Real; 3],
    /// Maximum exact corner.
    pub max: [Real; 3],
}

/// Preflight report for an [`ExactBox`].
///
/// Exact voxelization may continue through undecided predicate comparisons as
/// explicit unknown cells, but source geometry with a certified inverted axis
/// is not a valid box. The report rejects invalid object structure while
/// keeping undecided comparisons explicit instead of applying an epsilon
/// repair.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExactBoxReport {
    /// Axes where `min <= max` was certified.
    pub ordered_axes: Vec<usize>,
    /// Axes where `min == max` was certified.
    ///
    /// A zero-thickness axis is ordered, but it is not a valid 3D box volume
    /// for source-geometry voxelization. Degenerate solids are rejected rather
    /// than promoted into boundary-only topology.
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
    pub fn new(min: [Real; 3], max: [Real; 3]) -> Self {
        Self { min, max }
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
///
/// Voxel cells are treated as half-open finite volumes
/// `[min, max) x [min, max) x [min, max)`. This avoids promoting a zero-volume
/// touch on an integer grid plane into stored boundary topology for exact box
/// fixtures: a box `[1, 3)^3` covers exactly the cells with coordinates `1`
/// and `2` on each axis, while a fractional box such as `[1/2, 5/2)^3` still
/// produces conservative partial-overlap boundary cells. This is an exact
/// object-level convention, not an epsilon shift; all comparisons remain
/// proof-producing.
pub fn classify_cell_against_box(
    address: VoxelAddress,
    frame: &GridFrame,
    exact_box: &ExactBox,
) -> HypervoxelResult<VoxelBoxClassifier> {
    let bounds = address.bounds(frame)?;
    let mut fully_inside = true;
    for axis in 0..3 {
        if certified_cmp(&bounds.max[axis], &exact_box.min[axis], axis)? != Ordering::Greater {
            return Ok(VoxelBoxClassifier::Outside);
        }
        if certified_cmp(&bounds.min[axis], &exact_box.max[axis], axis)? != Ordering::Less {
            return Ok(VoxelBoxClassifier::Outside);
        }
        if certified_cmp(&bounds.min[axis], &exact_box.min[axis], axis)? == Ordering::Less
            || certified_cmp(&bounds.max[axis], &exact_box.max[axis], axis)? == Ordering::Greater
        {
            fully_inside = false;
        }
    }

    if fully_inside {
        Ok(VoxelBoxClassifier::Inside)
    } else {
        Ok(VoxelBoxClassifier::Boundary)
    }
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
