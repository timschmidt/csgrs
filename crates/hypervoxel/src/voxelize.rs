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

use hyperreal::{CertifiedRealOrdering, Real};

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

impl ExactBox {
    /// Creates an exact box without proving min/max order up front.
    ///
    /// Ordering is certified during voxel classification so unresolved symbolic
    /// bounds become explicit unknown cells instead of construction-time
    /// panics.
    pub fn new(min: [Real; 3], max: [Real; 3], source: Option<GridSource>) -> Self {
        Self { min, max, source }
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
    match left.certified_cmp_until(right, -128) {
        CertifiedRealOrdering::Known { ordering, .. } => Ok(ordering),
        CertifiedRealOrdering::Unknown { .. } => Err(HypervoxelError::UnknownOrdering { axis }),
    }
}

/// Classifies one cell against an exact axis-aligned box.
pub fn classify_cell_against_box(
    address: VoxelAddress,
    frame: &GridFrame,
    exact_box: &ExactBox,
) -> HypervoxelResult<VoxelBoxClassifier> {
    let bounds = address.bounds(frame)?;
    let mut fully_inside = true;

    for axis in 0..3 {
        let cell_max_before_box_min =
            certified_cmp(&bounds.max[axis], &exact_box.min[axis], axis)? != Ordering::Greater;
        let cell_min_after_box_max =
            certified_cmp(&bounds.min[axis], &exact_box.max[axis], axis)? != Ordering::Less;
        if cell_max_before_box_min || cell_min_after_box_max {
            return Ok(VoxelBoxClassifier::Outside);
        }

        let min_inside =
            certified_cmp(&bounds.min[axis], &exact_box.min[axis], axis)? != Ordering::Less;
        let max_inside =
            certified_cmp(&bounds.max[axis], &exact_box.max[axis], axis)? != Ordering::Greater;
        fully_inside &= min_inside && max_inside;
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

    let aggregate = VoxelAggregateFacts::from_cells(grid.iter().map(|(_, cell)| cell));
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
