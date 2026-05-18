//! Exact half-space voxelization fixtures.
//!
//! This module extends the initial exact-box path with a simple but useful
//! exact geometric predicate: classifying voxel cells against one linear
//! half-space. It is still not a full `hypermesh` solid voxelizer. The point is
//! to keep the predicate proof-producing and explicit, following Yap,
//! "Towards Exact Geometric Computation," *Computational Geometry* 7(1-2),
//! 1997, rather than importing triangle epsilon tests as truth.

use std::cmp::Ordering;

use hyperreal::{CertifiedRealOrdering, Real};

use crate::{
    BoundaryPolicy, GridFrame, GridSource, HypervoxelError, HypervoxelResult, MaterialRegionId,
    OccupancyState, QuantizationPolicy, SparseVoxelGrid, VoxelAddress, VoxelAggregateFacts,
    VoxelCell, VoxelPayload, VoxelPredicateCertificateReport, VoxelizationPolicy,
    VoxelizationReport,
};

/// Exact half-space `normal . point <= offset`.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactHalfSpace {
    /// Exact normal coefficients.
    pub normal: [Real; 3],
    /// Exact offset.
    pub offset: Real,
    /// Optional source provenance.
    pub source: Option<GridSource>,
}

impl ExactHalfSpace {
    /// Creates an exact half-space.
    pub fn new(normal: [Real; 3], offset: Real, source: Option<GridSource>) -> Self {
        Self {
            normal,
            offset,
            source,
        }
    }
}

/// Classification of one cell against an exact half-space.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VoxelHalfSpaceClassifier {
    /// Cell is entirely outside the half-space.
    Outside,
    /// Cell is entirely inside the half-space.
    Inside,
    /// Cell crosses the half-space boundary.
    Boundary,
    /// Exact ordering was not available.
    Unknown,
}

/// Classifies one cell against an exact half-space.
pub fn classify_cell_against_halfspace(
    address: VoxelAddress,
    frame: &GridFrame,
    halfspace: &ExactHalfSpace,
) -> HypervoxelResult<VoxelHalfSpaceClassifier> {
    let bounds = address.bounds(frame)?;
    let mut inside_count = 0_usize;
    let mut outside_count = 0_usize;
    for corner in bounds.corners() {
        let value = dot(&halfspace.normal, &corner);
        match certified_cmp(&value, &halfspace.offset)? {
            Ordering::Less | Ordering::Equal => inside_count += 1,
            Ordering::Greater => outside_count += 1,
        }
    }
    Ok(match (inside_count, outside_count) {
        (8, 0) => VoxelHalfSpaceClassifier::Inside,
        (0, 8) => VoxelHalfSpaceClassifier::Outside,
        _ => VoxelHalfSpaceClassifier::Boundary,
    })
}

/// Voxelizes an exact half-space over a finite grid frame.
pub fn voxelize_exact_halfspace(
    frame: GridFrame,
    halfspace: &ExactHalfSpace,
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
                let classifier = match classify_cell_against_halfspace(address, &frame, halfspace) {
                    Ok(classifier) => classifier,
                    Err(HypervoxelError::UnknownOrdering { .. })
                    | Err(HypervoxelError::UnknownScalarOrdering { .. }) => {
                        VoxelHalfSpaceClassifier::Unknown
                    }
                    Err(err) => return Err(err),
                };
                match classifier {
                    VoxelHalfSpaceClassifier::Inside => inside_cells += 1,
                    VoxelHalfSpaceClassifier::Outside => outside_cells += 1,
                    VoxelHalfSpaceClassifier::Boundary => predicate_boundary_cells += 1,
                    VoxelHalfSpaceClassifier::Unknown => predicate_unknown_cells += 1,
                }

                let cell = match (policy.quantization, policy.boundary, classifier) {
                    (_, _, VoxelHalfSpaceClassifier::Outside) => VoxelCell::empty(),
                    (_, _, VoxelHalfSpaceClassifier::Unknown) => {
                        unknown_cells += 1;
                        VoxelCell::unknown()
                    }
                    (
                        QuantizationPolicy::ConservativeInterior,
                        _,
                        VoxelHalfSpaceClassifier::Boundary,
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
                    (_, _, VoxelHalfSpaceClassifier::Inside) => VoxelCell::material(material),
                    (_, BoundaryPolicy::BoundaryAsUnknown, VoxelHalfSpaceClassifier::Boundary) => {
                        boundary_cells += 1;
                        unknown_cells += 1;
                        VoxelCell::unknown()
                    }
                    (_, BoundaryPolicy::LossySideChoice, VoxelHalfSpaceClassifier::Boundary) => {
                        boundary_cells += 1;
                        VoxelCell {
                            occupancy: OccupancyState::LossyAdapterValue,
                            payload: VoxelPayload::LossyAdapterValue(material.0),
                        }
                    }
                    (_, BoundaryPolicy::KeepBoundary, VoxelHalfSpaceClassifier::Boundary) => {
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
        source: halfspace.source.clone(),
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

fn dot(normal: &[Real; 3], point: &[Real; 3]) -> Real {
    normal[0].clone() * point[0].clone()
        + normal[1].clone() * point[1].clone()
        + normal[2].clone() * point[2].clone()
}

fn certified_cmp(left: &Real, right: &Real) -> HypervoxelResult<Ordering> {
    match left.certified_cmp_until(right, -128) {
        CertifiedRealOrdering::Known { ordering, .. } => Ok(ordering),
        CertifiedRealOrdering::Unknown { .. } => Err(HypervoxelError::UnknownScalarOrdering {
            field: "half-space",
        }),
    }
}
