//! Exact half-space voxelization fixtures.
//!
//! This module complements exact-box classification with a useful
//! exact geometric predicate: classifying voxel cells against one linear
//! half-space. It is still not a full `hypermesh` solid voxelizer. The
//! predicate remains proof-producing and explicit rather than treating
//! triangle epsilon tests as truth.

use hyperlimit::{PredicateOutcome, Sign};
use hyperreal::{Real, RealSign};

use crate::{
    BoundaryPolicy, GridFrame, HypervoxelError, HypervoxelResult, MaterialRegionId, OccupancyState,
    QuantizationPolicy, SparseVoxelGrid, VoxelAddress, VoxelAggregateFacts, VoxelCell,
    VoxelPayload, VoxelPredicateCertificateReport, VoxelizationPolicy, VoxelizationReport,
};

/// Exact half-space `normal . point <= offset`.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactHalfSpace {
    /// Exact normal coefficients.
    pub normal: [Real; 3],
    /// Exact offset.
    pub offset: Real,
}

/// Preflight report for an [`ExactHalfSpace`].
///
/// A linear half-space needs at least one nonzero normal component. This report
/// distinguishes a certified zero normal from an undecided one rather than
/// inventing an epsilon threshold.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExactHalfSpaceReport {
    /// Normal axes structurally known to be nonzero.
    pub known_nonzero_normal_axes: Vec<usize>,
    /// Normal axes structurally known to be exactly zero.
    pub known_zero_normal_axes: Vec<usize>,
    /// Normal axes whose zero status is not structurally known.
    pub unknown_normal_axes: Vec<usize>,
    /// Whether the normal is structurally certified as the zero vector.
    pub zero_normal_rejected: bool,
    /// Whether this half-space is ready as exact source predicate geometry.
    ///
    /// Readiness requires a certified nonzero normal and no structurally
    /// unknown normal component. Even if one component is known nonzero, an
    /// unknown second component still changes the predicate being voxelized.
    /// The represented object must be explicit before its predicates can
    /// certify topology.
    pub exact_halfspace_ready: bool,
}

impl ExactHalfSpace {
    /// Creates an exact half-space.
    pub fn new(normal: [Real; 3], offset: Real) -> Self {
        Self { normal, offset }
    }

    /// Reports structural normal validity without lowering to floats.
    pub fn report(&self) -> ExactHalfSpaceReport {
        let mut known_nonzero_normal_axes = Vec::new();
        let mut known_zero_normal_axes = Vec::new();
        let mut unknown_normal_axes = Vec::new();
        for (axis, component) in self.normal.iter().enumerate() {
            match component.structural_facts().sign {
                Some(RealSign::Positive | RealSign::Negative) => {
                    known_nonzero_normal_axes.push(axis);
                }
                Some(RealSign::Zero) => known_zero_normal_axes.push(axis),
                None => unknown_normal_axes.push(axis),
            }
        }
        let zero_normal_rejected =
            known_nonzero_normal_axes.is_empty() && unknown_normal_axes.is_empty();
        let exact_halfspace_ready = !zero_normal_rejected
            && !known_nonzero_normal_axes.is_empty()
            && unknown_normal_axes.is_empty();
        ExactHalfSpaceReport {
            known_nonzero_normal_axes,
            known_zero_normal_axes,
            unknown_normal_axes,
            zero_normal_rejected,
            exact_halfspace_ready,
        }
    }
}

/// Classification of one cell against an exact half-space.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VoxelHalfSpaceClassification {
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
) -> HypervoxelResult<VoxelHalfSpaceClassification> {
    let bounds = address.bounds(frame)?;
    let report = decide(hyperlimit::classify_plane_aabb3_report(
        &halfspace.predicate_plane(),
        &point3(&bounds.min),
        &point3(&bounds.max),
    ))?;
    Ok(match (report.lower_sign, report.upper_sign) {
        (_, Sign::Negative | Sign::Zero) => VoxelHalfSpaceClassification::Inside,
        (Sign::Positive, _) => VoxelHalfSpaceClassification::Outside,
        _ => VoxelHalfSpaceClassification::Boundary,
    })
}

/// Voxelizes an exact half-space over a finite grid frame.
pub fn voxelize_exact_halfspace(
    frame: GridFrame,
    halfspace: &ExactHalfSpace,
    material: MaterialRegionId,
    policy: VoxelizationPolicy,
) -> HypervoxelResult<(SparseVoxelGrid, VoxelizationReport)> {
    if halfspace.report().zero_normal_rejected {
        return Err(HypervoxelError::InvalidSourceGeometry {
            reason: "half-space normal is zero",
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
                let classifier = match classify_cell_against_halfspace(address, &frame, halfspace) {
                    Ok(classifier) => classifier,
                    Err(HypervoxelError::UnknownOrdering { .. })
                    | Err(HypervoxelError::UnknownScalarOrdering { .. }) => {
                        VoxelHalfSpaceClassification::Unknown
                    }
                    Err(err) => return Err(err),
                };
                match classifier {
                    VoxelHalfSpaceClassification::Inside => inside_cells += 1,
                    VoxelHalfSpaceClassification::Outside => outside_cells += 1,
                    VoxelHalfSpaceClassification::Boundary => predicate_boundary_cells += 1,
                    VoxelHalfSpaceClassification::Unknown => predicate_unknown_cells += 1,
                }

                let cell = match (policy.quantization, policy.boundary, classifier) {
                    (_, _, VoxelHalfSpaceClassification::Outside) => VoxelCell::empty(),
                    (_, _, VoxelHalfSpaceClassification::Unknown) => {
                        unknown_cells += 1;
                        VoxelCell::unknown()
                    }
                    (
                        QuantizationPolicy::ConservativeInterior,
                        _,
                        VoxelHalfSpaceClassification::Boundary,
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
                    (_, _, VoxelHalfSpaceClassification::Inside) => VoxelCell::material(material),
                    (
                        _,
                        BoundaryPolicy::BoundaryAsUnknown,
                        VoxelHalfSpaceClassification::Boundary,
                    ) => {
                        boundary_cells += 1;
                        unknown_cells += 1;
                        VoxelCell::unknown()
                    }
                    (
                        _,
                        BoundaryPolicy::LossySideChoice,
                        VoxelHalfSpaceClassification::Boundary,
                    ) => {
                        boundary_cells += 1;
                        VoxelCell {
                            occupancy: OccupancyState::LossyAdapterValue,
                            payload: VoxelPayload::LossyAdapterValue(material.0),
                        }
                    }
                    (_, BoundaryPolicy::KeepBoundary, VoxelHalfSpaceClassification::Boundary) => {
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

impl ExactHalfSpace {
    fn predicate_plane(&self) -> hyperlimit::Plane3 {
        hyperlimit::Plane3::new(point3(&self.normal), -self.offset.clone())
    }
}

fn decide<T>(outcome: PredicateOutcome<T>) -> HypervoxelResult<T> {
    outcome
        .value()
        .ok_or(HypervoxelError::UnknownScalarOrdering {
            field: "half-space",
        })
}

fn point3(values: &[Real; 3]) -> hyperlimit::Point3 {
    hyperlimit::Point3::new(values[0].clone(), values[1].clone(), values[2].clone())
}
