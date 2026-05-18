//! Exact convex solid fixtures built from half-space predicates.
//!
//! This is a deliberately small solid voxelization path: a convex body is the
//! intersection of exact linear half-spaces. It gives tests and downstream
//! callers a proof-producing closed-solid fixture before the future
//! `hypermesh::exact` triangle/solid voxelizer exists. The design follows Yap,
//! "Towards Exact Geometric Computation," *Computational Geometry* 7(1-2),
//! 1997: classification is made from exact predicates over object structure,
//! and any uncertified predicate becomes an explicit unknown instead of an
//! epsilon-derived topology decision.

use crate::{
    BoundaryPolicy, ExactHalfSpace, GridFrame, GridSource, HypervoxelError, HypervoxelResult,
    MaterialRegionId, OccupancyState, QuantizationPolicy, SparseVoxelGrid, VoxelAddress,
    VoxelAggregateFacts, VoxelCell, VoxelHalfSpaceClassifier, VoxelPayload,
    VoxelPredicateCertificateReport, VoxelizationPolicy, VoxelizationReport,
    classify_cell_against_halfspace,
};

/// Convex solid represented as an intersection of exact half-spaces.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactConvexHalfSpaceSet {
    /// Half-spaces whose intersection defines the solid.
    pub halfspaces: Vec<ExactHalfSpace>,
    /// Optional source provenance for the solid as a whole.
    pub source: Option<GridSource>,
}

impl ExactConvexHalfSpaceSet {
    /// Creates an exact convex half-space set.
    pub fn new(halfspaces: Vec<ExactHalfSpace>, source: Option<GridSource>) -> Self {
        Self { halfspaces, source }
    }

    /// Returns `true` when the set contains at least one boundary predicate.
    pub fn has_predicates(&self) -> bool {
        !self.halfspaces.is_empty()
    }
}

/// Classification of one cell against an exact convex half-space set.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VoxelConvexClassifier {
    /// Cell is entirely outside at least one half-space.
    Outside,
    /// Cell is entirely inside every half-space.
    Inside,
    /// Cell may cross one or more half-space boundaries.
    Boundary,
    /// One or more scalar predicate orderings were not certified.
    Unknown,
}

/// Classifies one cell against a convex intersection of exact half-spaces.
pub fn classify_cell_against_convex_halfspace_set(
    address: VoxelAddress,
    frame: &GridFrame,
    solid: &ExactConvexHalfSpaceSet,
) -> HypervoxelResult<VoxelConvexClassifier> {
    let mut touched_boundary = false;
    for halfspace in &solid.halfspaces {
        match classify_cell_against_halfspace(address, frame, halfspace)? {
            VoxelHalfSpaceClassifier::Outside => return Ok(VoxelConvexClassifier::Outside),
            VoxelHalfSpaceClassifier::Inside => {}
            VoxelHalfSpaceClassifier::Boundary => touched_boundary = true,
            VoxelHalfSpaceClassifier::Unknown => return Ok(VoxelConvexClassifier::Unknown),
        }
    }
    Ok(if touched_boundary {
        VoxelConvexClassifier::Boundary
    } else {
        VoxelConvexClassifier::Inside
    })
}

/// Voxelizes an exact convex half-space set over a finite grid frame.
pub fn voxelize_exact_convex_halfspace_set(
    frame: GridFrame,
    solid: &ExactConvexHalfSpaceSet,
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
                let classifier =
                    match classify_cell_against_convex_halfspace_set(address, &frame, solid) {
                        Ok(classifier) => classifier,
                        Err(HypervoxelError::UnknownOrdering { .. })
                        | Err(HypervoxelError::UnknownScalarOrdering { .. }) => {
                            VoxelConvexClassifier::Unknown
                        }
                        Err(err) => return Err(err),
                    };
                match classifier {
                    VoxelConvexClassifier::Inside => inside_cells += 1,
                    VoxelConvexClassifier::Outside => outside_cells += 1,
                    VoxelConvexClassifier::Boundary => predicate_boundary_cells += 1,
                    VoxelConvexClassifier::Unknown => predicate_unknown_cells += 1,
                }

                let cell = match (policy.quantization, policy.boundary, classifier) {
                    (_, _, VoxelConvexClassifier::Outside) => VoxelCell::empty(),
                    (_, _, VoxelConvexClassifier::Unknown) => {
                        unknown_cells += 1;
                        VoxelCell::unknown()
                    }
                    (
                        QuantizationPolicy::ConservativeInterior,
                        _,
                        VoxelConvexClassifier::Boundary,
                    ) => {
                        boundary_cells += 1;
                        match policy.boundary {
                            BoundaryPolicy::BoundaryAsUnknown => {
                                unknown_cells += 1;
                                VoxelCell::unknown()
                            }
                            _ => VoxelCell::empty(),
                        }
                    }
                    (_, _, VoxelConvexClassifier::Inside) => VoxelCell::material(material),
                    (_, BoundaryPolicy::BoundaryAsUnknown, VoxelConvexClassifier::Boundary) => {
                        boundary_cells += 1;
                        unknown_cells += 1;
                        VoxelCell::unknown()
                    }
                    (_, BoundaryPolicy::LossySideChoice, VoxelConvexClassifier::Boundary) => {
                        boundary_cells += 1;
                        VoxelCell {
                            occupancy: OccupancyState::LossyAdapterValue,
                            payload: VoxelPayload::LossyAdapterValue(material.0),
                        }
                    }
                    (_, BoundaryPolicy::KeepBoundary, VoxelConvexClassifier::Boundary) => {
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
        source: solid.source.clone(),
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
