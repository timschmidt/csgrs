//! Prepared exact triangle-solid voxelization.
//!
//! This module is the scheduled counterpart to [`crate::triangle_mesh`]. It
//! keeps retained source triangles as the owning evidence, but prepares exact
//! triangle AABBs once and reports how many triangle predicates each cell
//! actually consumed. The design follows Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7(1-2), 1997: the prepared data are
//! replayable object facts, not approximate acceleration hints that may change
//! topology silently.

use std::collections::VecDeque;

use core::cmp::Ordering;
use hyperlimit::{
    Aabb3Intersection, RayTriangleIntersection, classify_aabb3_intersection,
    classify_ray_triangle3_intersection_report,
};

use hyperreal::Real;

use crate::ray_schedule::{RayAabbIntersection, classify_ray_aabb_intersection};
use crate::triangle_mesh::{
    ExactTriangle3, ExactTriangleSolidMesh, TriangleCellIntersection, VoxelTriangleSolidClassifier,
    insert_unique_parameter, point3, ray_parity_directions, triangle_bounds,
    triangle_intersects_cell,
};
use crate::{
    BoundaryPolicy, GridFrame, HypervoxelError, HypervoxelResult, MaterialRegionId, OccupancyState,
    QuantizationPolicy, SparseVoxelGrid, VoxelAddress, VoxelAggregateFacts, VoxelCell,
    VoxelPayload, VoxelPredicateCertificateReport, VoxelizationPolicy, VoxelizationReport,
};

/// Prepared retained triangle for exact solid classification.
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedExactTriangle {
    /// Retained source triangle.
    pub triangle: ExactTriangle3,
    /// Exact AABB of the retained triangle.
    pub bounds: crate::ExactAabb3,
    points: [hyperlimit::Point3; 3],
}

/// Prepared closed triangle-solid handoff.
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedExactTriangleSolidMesh {
    solid: ExactTriangleSolidMesh,
    triangles: Vec<PreparedExactTriangle>,
    report: PreparedExactTriangleSolidMeshReport,
}

impl PreparedExactTriangleSolidMesh {
    /// Prepare exact triangle bounds for a retained closed solid.
    ///
    /// Preparation replays the same source readiness gate as ordinary solid
    /// voxelization and then stores exact triangle AABBs. The AABB stage is a
    /// broad-phase scheduling application of the exact separating-box test; it
    /// does not replace the later triangle/cell predicates. This mirrors the
    /// broad/narrow phase split described for robust triangle overlap by
    /// Guigue and Devillers, "Fast and Robust Triangle-Triangle Overlap Test
    /// Using Orientation Predicates," *Journal of Graphics Tools* 8(1), 2003,
    /// with Yap's requirement that the broad phase be exact and reportable.
    pub fn prepare(solid: ExactTriangleSolidMesh) -> HypervoxelResult<Self> {
        let source = solid.report();
        if source.surface.empty_triangle_set {
            return Err(HypervoxelError::InvalidSourceGeometry {
                reason: "triangle surface mesh has no triangles",
            });
        }
        if source.surface.degenerate_triangle_count > 0 {
            return Err(HypervoxelError::InvalidSourceGeometry {
                reason: "triangle surface mesh contains degenerate triangles",
            });
        }
        if source.surface.unknown_triangle_count > 0 {
            return Err(HypervoxelError::InvalidSourceGeometry {
                reason: "triangle surface mesh has uncertified triangle predicates",
            });
        }
        if !source.surface.exact_source_replay_available {
            return Err(HypervoxelError::InvalidSourceGeometry {
                reason: "triangle surface mesh lacks exact source replay",
            });
        }
        if !source.exact_closed_solid_replay_available {
            return Err(HypervoxelError::InvalidSourceGeometry {
                reason: "triangle solid mesh lacks exact closed-solid replay",
            });
        }

        let mut triangles = Vec::with_capacity(solid.surface.triangles.len());
        let mut unknown_bound_count = 0_usize;
        for triangle in &solid.surface.triangles {
            let bounds = match triangle_bounds(triangle) {
                Ok(bounds) => bounds,
                Err(HypervoxelError::UnknownOrdering { .. })
                | Err(HypervoxelError::UnknownScalarOrdering { .. }) => {
                    unknown_bound_count += 1;
                    continue;
                }
                Err(err) => return Err(err),
            };
            triangles.push(PreparedExactTriangle {
                triangle: triangle.clone(),
                bounds,
                points: triangle.points(),
            });
        }

        let exact_prepared_solid_ready =
            unknown_bound_count == 0 && triangles.len() == source.surface.triangle_count;
        let report = PreparedExactTriangleSolidMeshReport {
            source,
            prepared_triangle_count: triangles.len(),
            unknown_bound_count,
            exact_prepared_solid_ready,
        };
        Ok(Self {
            solid,
            triangles,
            report,
        })
    }

    /// Return the retained source solid.
    pub const fn solid(&self) -> &ExactTriangleSolidMesh {
        &self.solid
    }

    /// Return the prepared readiness report.
    pub const fn report(&self) -> &PreparedExactTriangleSolidMeshReport {
        &self.report
    }

    /// Return prepared triangles with exact bounds.
    pub fn triangles(&self) -> &[PreparedExactTriangle] {
        &self.triangles
    }
}

/// Readiness report for [`PreparedExactTriangleSolidMesh`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PreparedExactTriangleSolidMeshReport {
    /// Source retained solid readiness.
    pub source: crate::ExactTriangleSolidMeshReport,
    /// Number of triangles retained in the prepared schedule.
    pub prepared_triangle_count: usize,
    /// Number of triangle bounds whose exact ordering could not be replayed.
    pub unknown_bound_count: usize,
    /// Whether the prepared schedule is exact-ready.
    pub exact_prepared_solid_ready: bool,
}

/// Per-cell exact scheduled solid-classification evidence.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PreparedTriangleSolidCellReport {
    /// Final cell classifier.
    pub classifier: VoxelTriangleSolidClassifier,
    /// Triangles rejected by exact cell-AABB/triangle-AABB disjointness.
    pub boundary_aabb_rejections: usize,
    /// Triangles reaching the exact triangle/cell narrow phase.
    pub boundary_triangle_tests: usize,
    /// Whether any boundary predicate returned unknown.
    pub boundary_unknown: bool,
    /// Exact ray parity attempts used after boundary rejection.
    pub ray_attempts: Vec<PreparedRayParityAttemptReport>,
}

impl PreparedTriangleSolidCellReport {
    /// Whether all predicates needed for this cell were proof-producing.
    pub fn is_fully_certified(&self) -> bool {
        self.classifier != VoxelTriangleSolidClassifier::Unknown && !self.boundary_unknown
    }

    /// Number of exact ray/triangle predicates evaluated across all attempted
    /// directions.
    pub fn ray_triangle_tests(&self) -> usize {
        self.ray_attempts
            .iter()
            .map(|attempt| attempt.triangle_tests)
            .sum()
    }
}

/// One exact ray-parity attempt from a cell center.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PreparedRayParityAttemptReport {
    /// Direction index in the deterministic exact rational retry set.
    pub direction_index: usize,
    /// Triangle AABBs rejected by exact ray/slab broad-phase scheduling.
    pub ray_aabb_rejections: usize,
    /// Number of exact ray/triangle predicates evaluated.
    pub triangle_tests: usize,
    /// Number of proper ray/triangle intersections before unique-parameter
    /// collapse.
    pub proper_intersections: usize,
    /// Number of unique exact ray parameters counted for parity.
    pub unique_parameters: usize,
    /// Number of boundary-touch events that made this ray ambiguous.
    pub boundary_touches: usize,
    /// Number of coplanar events that made this ray ambiguous.
    pub coplanar_events: usize,
    /// Whether this ray produced a usable inside/outside parity decision.
    pub certified: bool,
}

/// Classify one cell with prepared exact triangle scheduling.
pub fn classify_cell_against_prepared_triangle_solid_mesh(
    address: VoxelAddress,
    frame: &GridFrame,
    prepared: &PreparedExactTriangleSolidMesh,
) -> HypervoxelResult<PreparedTriangleSolidCellReport> {
    if !prepared.report.exact_prepared_solid_ready {
        return Err(HypervoxelError::InvalidSourceGeometry {
            reason: "prepared triangle solid mesh is not exact-ready",
        });
    }

    let bounds = address.bounds(frame)?;
    let boundary = classify_cell_boundary_against_prepared_triangle_solid(&bounds, prepared)?;

    if boundary.classifier != VoxelTriangleSolidClassifier::Outside {
        return Ok(boundary);
    }

    let (classifier, ray_attempts) =
        classify_point_against_prepared_triangle_solid_by_ray(&bounds.center(), prepared)?;
    Ok(PreparedTriangleSolidCellReport {
        classifier,
        ray_attempts,
        ..boundary
    })
}

fn classify_cell_boundary_against_prepared_triangle_solid(
    bounds: &crate::CellBounds,
    prepared: &PreparedExactTriangleSolidMesh,
) -> HypervoxelResult<PreparedTriangleSolidCellReport> {
    let cell_min = point3(&bounds.min);
    let cell_max = point3(&bounds.max);
    let mut boundary_aabb_rejections = 0_usize;
    let mut boundary_triangle_tests = 0_usize;
    let mut boundary_unknown = false;

    for triangle in &prepared.triangles {
        let relation = classify_aabb3_intersection(
            &point3(&triangle.bounds.min),
            &point3(&triangle.bounds.max),
            &cell_min,
            &cell_max,
        )
        .value()
        .ok_or(HypervoxelError::UnknownScalarOrdering {
            field: "prepared-triangle-aabb",
        })?;
        if relation == Aabb3Intersection::Disjoint {
            boundary_aabb_rejections += 1;
            continue;
        }

        boundary_triangle_tests += 1;
        match triangle_intersects_cell(&triangle.triangle, bounds)? {
            TriangleCellIntersection::Intersects => {
                return Ok(PreparedTriangleSolidCellReport {
                    classifier: VoxelTriangleSolidClassifier::Boundary,
                    boundary_aabb_rejections,
                    boundary_triangle_tests,
                    boundary_unknown,
                    ray_attempts: Vec::new(),
                });
            }
            TriangleCellIntersection::Disjoint => {}
            TriangleCellIntersection::Unknown => boundary_unknown = true,
        }
    }

    if boundary_unknown {
        return Ok(PreparedTriangleSolidCellReport {
            classifier: VoxelTriangleSolidClassifier::Unknown,
            boundary_aabb_rejections,
            boundary_triangle_tests,
            boundary_unknown,
            ray_attempts: Vec::new(),
        });
    }

    Ok(PreparedTriangleSolidCellReport {
        classifier: VoxelTriangleSolidClassifier::Outside,
        boundary_aabb_rejections,
        boundary_triangle_tests,
        boundary_unknown,
        ray_attempts: Vec::new(),
    })
}

/// Voxelize a prepared exact closed triangle solid.
pub fn voxelize_prepared_exact_triangle_solid_mesh(
    frame: GridFrame,
    prepared: &PreparedExactTriangleSolidMesh,
    material: MaterialRegionId,
    policy: VoxelizationPolicy,
) -> HypervoxelResult<(
    SparseVoxelGrid,
    VoxelizationReport,
    PreparedTriangleSolidVoxelizationReport,
)> {
    if !prepared.report.exact_prepared_solid_ready {
        return Err(HypervoxelError::InvalidSourceGeometry {
            reason: "prepared triangle solid mesh is not exact-ready",
        });
    }

    let mut grid = SparseVoxelGrid::new(frame.clone());
    let mut inside_cells = 0_usize;
    let mut outside_cells = 0_usize;
    let mut boundary_cells = 0_usize;
    let mut unknown_cells = 0_usize;
    let mut predicate_boundary_cells = 0_usize;
    let mut predicate_unknown_cells = 0_usize;
    let mut prepared_report = PreparedTriangleSolidVoxelizationReport::default();
    let cells_per_axis = frame.cells_per_axis();

    for z in 0..cells_per_axis {
        for y in 0..cells_per_axis {
            for x in 0..cells_per_axis {
                let address = VoxelAddress::new(frame.depth(), [x, y, z])?;
                let cell_report =
                    classify_cell_against_prepared_triangle_solid_mesh(address, &frame, prepared)?;
                prepared_report.accumulate(&cell_report);

                match cell_report.classifier {
                    VoxelTriangleSolidClassifier::Inside => inside_cells += 1,
                    VoxelTriangleSolidClassifier::Outside => outside_cells += 1,
                    VoxelTriangleSolidClassifier::Boundary => predicate_boundary_cells += 1,
                    VoxelTriangleSolidClassifier::Unknown => predicate_unknown_cells += 1,
                }

                let cell = match (policy.quantization, policy.boundary, cell_report.classifier) {
                    (_, _, VoxelTriangleSolidClassifier::Outside) => VoxelCell::empty(),
                    (_, _, VoxelTriangleSolidClassifier::Unknown) => {
                        unknown_cells += 1;
                        VoxelCell::unknown()
                    }
                    (_, _, VoxelTriangleSolidClassifier::Inside) => VoxelCell::material(material),
                    (
                        QuantizationPolicy::ConservativeInterior,
                        _,
                        VoxelTriangleSolidClassifier::Boundary,
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
                    (
                        _,
                        BoundaryPolicy::BoundaryAsUnknown,
                        VoxelTriangleSolidClassifier::Boundary,
                    ) => {
                        boundary_cells += 1;
                        unknown_cells += 1;
                        VoxelCell::unknown()
                    }
                    (
                        _,
                        BoundaryPolicy::LossySideChoice,
                        VoxelTriangleSolidClassifier::Boundary,
                    ) => {
                        boundary_cells += 1;
                        VoxelCell {
                            occupancy: OccupancyState::LossyAdapterValue,
                            payload: VoxelPayload::LossyAdapterValue(material.0),
                        }
                    }
                    (_, BoundaryPolicy::KeepBoundary, VoxelTriangleSolidClassifier::Boundary) => {
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
        source: prepared.solid.surface.source.clone(),
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
    Ok((grid, report, prepared_report))
}

/// Voxelize a prepared exact closed triangle solid by connected non-boundary
/// components.
///
/// This is the first arrangement-style accelerator above the per-cell parity
/// path. It performs the exact boundary classification for every cell, labels
/// connected components of cells proven disjoint from the boundary, marks
/// components touching the grid boundary as exterior, and ray-classifies only
/// one representative cell for every remaining component. The component
/// labeling follows the 6-neighbor digital topology model used by Rosenfeld
/// and Pfaltz, "Sequential Operations in Digital Picture Processing," *JACM*
/// 13(4), 1966. The topology-changing decisions remain gated by Yap's exact
/// computation discipline: boundary predicates and representative parity
/// queries must be proof-producing, otherwise the whole component is reported
/// as unknown.
pub fn voxelize_prepared_exact_triangle_solid_mesh_by_components(
    frame: GridFrame,
    prepared: &PreparedExactTriangleSolidMesh,
    material: MaterialRegionId,
    policy: VoxelizationPolicy,
) -> HypervoxelResult<(
    SparseVoxelGrid,
    VoxelizationReport,
    PreparedTriangleSolidComponentVoxelizationReport,
)> {
    voxelize_prepared_exact_triangle_solid_mesh_by_components_impl(
        frame, prepared, material, policy, false,
    )
}

/// Voxelize a prepared exact closed triangle solid by connected non-boundary
/// components with full component-arrangement replay.
///
/// The ordinary component scheduler relies on the exact boundary pass plus one
/// representative parity query per enclosed open component. This stricter path
/// keeps the same 6-neighbor component model, but then replays parity for every
/// cell in each enclosed component and reports conflicts before materializing
/// the component. The audit is a discrete arrangement consistency check: a
/// component of cells proven disjoint from the boundary should have constant
/// winding/parity. If any cell disagrees, becomes unknown, or unexpectedly
/// reclassifies as boundary, the whole component is materialized as unknown.
///
/// This follows Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7(1-2), 1997: the accelerated representative decision is accepted
/// only after exact replay validates the combinatorial invariant it depends
/// on. The connected-cell model is the same Rosenfeld and Pfaltz 6-neighbor
/// digital topology used by [`voxelize_prepared_exact_triangle_solid_mesh_by_components`].
pub fn voxelize_prepared_exact_triangle_solid_mesh_by_verified_components(
    frame: GridFrame,
    prepared: &PreparedExactTriangleSolidMesh,
    material: MaterialRegionId,
    policy: VoxelizationPolicy,
) -> HypervoxelResult<(
    SparseVoxelGrid,
    VoxelizationReport,
    PreparedTriangleSolidComponentVoxelizationReport,
)> {
    voxelize_prepared_exact_triangle_solid_mesh_by_components_impl(
        frame, prepared, material, policy, true,
    )
}

/// Voxelize a prepared exact closed triangle solid by exact row-parity sweeps.
///
/// This is an arrangement/winding accelerator beyond the connected-cell
/// scheduler. It still performs the exact triangle/cell boundary pass for
/// every cell. For cells proven disjoint from the retained boundary, it then
/// classifies each `(y,z)` row by one exact `+X` ray/triangle sweep and reuses
/// the sorted exact intersection parameters for all open cell centers on that
/// row. If a row ray hits an edge/vertex/coplanar case, the row is not trusted:
/// every open cell on that row falls back to the existing multi-direction
/// per-cell parity classifier, and the fallback work is reported.
///
/// The row sweep is the same parity/winding idea used by point-in-polyhedron
/// tests, but lifted to a report-bearing arrangement pass. Its sorting and
/// unique-parameter replay follow Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7(1-2), 1997: the accelerated row decision is exact
/// only when the combinatorial crossing sequence is certified. The sweep-line
/// batching is in the spirit of Bentley and Ottmann, "Algorithms for Reporting
/// and Counting Geometric Intersections," *IEEE Transactions on Computers*
/// C-28(9), 1979, but all acceptance still comes from exact ray/triangle
/// predicates.
pub fn voxelize_prepared_exact_triangle_solid_mesh_by_axis_sweeps(
    frame: GridFrame,
    prepared: &PreparedExactTriangleSolidMesh,
    material: MaterialRegionId,
    policy: VoxelizationPolicy,
) -> HypervoxelResult<(
    SparseVoxelGrid,
    VoxelizationReport,
    PreparedTriangleSolidAxisSweepVoxelizationReport,
)> {
    if !prepared.report.exact_prepared_solid_ready {
        return Err(HypervoxelError::InvalidSourceGeometry {
            reason: "prepared triangle solid mesh is not exact-ready",
        });
    }

    let cells_per_axis = frame.cells_per_axis();
    let total_cells =
        usize::try_from(cells_per_axis.pow(3)).map_err(|_| HypervoxelError::AddressOverflow)?;
    let mut classifiers = vec![VoxelTriangleSolidClassifier::Unknown; total_cells];
    let mut open = vec![false; total_cells];
    let mut sweep_report = PreparedTriangleSolidAxisSweepVoxelizationReport {
        classified_cells: total_cells,
        ..PreparedTriangleSolidAxisSweepVoxelizationReport::default()
    };

    for z in 0..cells_per_axis {
        for y in 0..cells_per_axis {
            for x in 0..cells_per_axis {
                let address = VoxelAddress::new(frame.depth(), [x, y, z])?;
                let bounds = address.bounds(&frame)?;
                let boundary =
                    classify_cell_boundary_against_prepared_triangle_solid(&bounds, prepared)?;
                let index = cell_index(cells_per_axis, [x, y, z])?;
                sweep_report.boundary_aabb_rejections += boundary.boundary_aabb_rejections;
                sweep_report.boundary_triangle_tests += boundary.boundary_triangle_tests;
                match boundary.classifier {
                    VoxelTriangleSolidClassifier::Boundary => {
                        classifiers[index] = VoxelTriangleSolidClassifier::Boundary;
                        sweep_report.boundary_cells += 1;
                    }
                    VoxelTriangleSolidClassifier::Unknown => {
                        classifiers[index] = VoxelTriangleSolidClassifier::Unknown;
                        sweep_report.boundary_unknown_cells += 1;
                    }
                    VoxelTriangleSolidClassifier::Outside => {
                        open[index] = true;
                        sweep_report.open_cells += 1;
                    }
                    VoxelTriangleSolidClassifier::Inside => unreachable!(
                        "boundary-only prepared classification never emits inside cells"
                    ),
                }
            }
        }
    }

    for z in 0..cells_per_axis {
        for y in 0..cells_per_axis {
            let mut open_x = Vec::new();
            for x in 0..cells_per_axis {
                if open[cell_index(cells_per_axis, [x, y, z])?] {
                    open_x.push(x);
                }
            }
            if open_x.is_empty() {
                sweep_report.empty_sweep_rows += 1;
                continue;
            }
            sweep_report.sweep_rows += 1;

            let row_origin = VoxelAddress::new(frame.depth(), [0, y, z])?
                .bounds(&frame)?
                .center();
            let row = classify_axis_row_against_prepared_triangle_solid(
                &row_origin,
                prepared,
                &mut sweep_report,
            )?;

            match row {
                AxisRowParity::Certified { parameters } => {
                    sweep_report.certified_sweep_rows += 1;
                    for x in open_x {
                        let address = VoxelAddress::new(frame.depth(), [x, y, z])?;
                        let center = address.bounds(&frame)?.center();
                        let threshold = &center[0] - &row_origin[0];
                        let Some(classifier) = classify_axis_sweep_center(&parameters, &threshold)?
                        else {
                            sweep_report.row_parameter_order_unknowns += 1;
                            classify_axis_sweep_fallback_cell(
                                [x, y, z],
                                &frame,
                                prepared,
                                &mut classifiers,
                                cells_per_axis,
                                &mut sweep_report,
                            )?;
                            continue;
                        };
                        let index = cell_index(cells_per_axis, [x, y, z])?;
                        classifiers[index] = classifier;
                        sweep_report.sweep_classified_cells += 1;
                    }
                }
                AxisRowParity::Ambiguous => {
                    sweep_report.ambiguous_sweep_rows += 1;
                    for x in open_x {
                        classify_axis_sweep_fallback_cell(
                            [x, y, z],
                            &frame,
                            prepared,
                            &mut classifiers,
                            cells_per_axis,
                            &mut sweep_report,
                        )?;
                    }
                }
            }
        }
    }

    let exact_axis_sweep_ready = sweep_report.boundary_unknown_cells == 0
        && sweep_report.fallback_unknown_cells == 0
        && sweep_report.fallback_boundary_regression_cells == 0
        && sweep_report.row_parameter_order_unknowns == 0
        && sweep_report.classified_cells > 0;
    sweep_report.exact_axis_sweep_ready = exact_axis_sweep_ready;

    let (grid, report) = materialize_prepared_classifiers(
        frame,
        prepared.solid.surface.source.clone(),
        policy,
        material,
        &classifiers,
    )?;
    Ok((grid, report, sweep_report))
}

fn voxelize_prepared_exact_triangle_solid_mesh_by_components_impl(
    frame: GridFrame,
    prepared: &PreparedExactTriangleSolidMesh,
    material: MaterialRegionId,
    policy: VoxelizationPolicy,
    verify_component_arrangement: bool,
) -> HypervoxelResult<(
    SparseVoxelGrid,
    VoxelizationReport,
    PreparedTriangleSolidComponentVoxelizationReport,
)> {
    if !prepared.report.exact_prepared_solid_ready {
        return Err(HypervoxelError::InvalidSourceGeometry {
            reason: "prepared triangle solid mesh is not exact-ready",
        });
    }

    let cells_per_axis = frame.cells_per_axis();
    let total_cells =
        usize::try_from(cells_per_axis.pow(3)).map_err(|_| HypervoxelError::AddressOverflow)?;
    let mut classifiers = vec![VoxelTriangleSolidClassifier::Unknown; total_cells];
    let mut open = vec![false; total_cells];
    let mut visited = vec![false; total_cells];
    let mut component_report = PreparedTriangleSolidComponentVoxelizationReport {
        classified_cells: total_cells,
        ..PreparedTriangleSolidComponentVoxelizationReport::default()
    };

    for z in 0..cells_per_axis {
        for y in 0..cells_per_axis {
            for x in 0..cells_per_axis {
                let address = VoxelAddress::new(frame.depth(), [x, y, z])?;
                let bounds = address.bounds(&frame)?;
                let boundary =
                    classify_cell_boundary_against_prepared_triangle_solid(&bounds, prepared)?;
                let index = cell_index(cells_per_axis, [x, y, z])?;
                component_report.boundary_aabb_rejections += boundary.boundary_aabb_rejections;
                component_report.boundary_triangle_tests += boundary.boundary_triangle_tests;
                match boundary.classifier {
                    VoxelTriangleSolidClassifier::Boundary => {
                        classifiers[index] = VoxelTriangleSolidClassifier::Boundary;
                        component_report.boundary_cells += 1;
                    }
                    VoxelTriangleSolidClassifier::Unknown => {
                        classifiers[index] = VoxelTriangleSolidClassifier::Unknown;
                        component_report.boundary_unknown_cells += 1;
                    }
                    VoxelTriangleSolidClassifier::Outside => {
                        open[index] = true;
                        component_report.open_cells += 1;
                    }
                    VoxelTriangleSolidClassifier::Inside => unreachable!(
                        "boundary-only prepared classification never emits inside cells"
                    ),
                }
            }
        }
    }

    for z in 0..cells_per_axis {
        for y in 0..cells_per_axis {
            for x in 0..cells_per_axis {
                let index = cell_index(cells_per_axis, [x, y, z])?;
                if !open[index] || visited[index] {
                    continue;
                }

                let mut queue = VecDeque::new();
                let mut component = Vec::new();
                let mut touches_frame_boundary = false;
                visited[index] = true;
                queue.push_back([x, y, z]);
                while let Some(coords) = queue.pop_front() {
                    component.push(coords);
                    touches_frame_boundary |= coords
                        .iter()
                        .any(|&axis| axis == 0 || axis + 1 == cells_per_axis);

                    for neighbor in component_neighbors(cells_per_axis, coords) {
                        let neighbor_index = cell_index(cells_per_axis, neighbor)?;
                        if open[neighbor_index] && !visited[neighbor_index] {
                            visited[neighbor_index] = true;
                            queue.push_back(neighbor);
                        }
                    }
                }

                component_report.components += 1;
                let classifier = if touches_frame_boundary {
                    component_report.exterior_components += 1;
                    component_report.outside_components += 1;
                    VoxelTriangleSolidClassifier::Outside
                } else {
                    component_report.ray_classified_components += 1;
                    let representative = component[0];
                    let address = VoxelAddress::new(frame.depth(), representative)?;
                    let cell = classify_cell_against_prepared_triangle_solid_mesh(
                        address, &frame, prepared,
                    )?;
                    component_report.component_ray_attempts += cell.ray_attempts.len();
                    component_report.component_ray_aabb_rejections += cell
                        .ray_attempts
                        .iter()
                        .map(|attempt| attempt.ray_aabb_rejections)
                        .sum::<usize>();
                    component_report.component_ray_triangle_tests += cell.ray_triangle_tests();
                    component_report.ambiguous_component_ray_attempts += cell
                        .ray_attempts
                        .iter()
                        .filter(|attempt| !attempt.certified)
                        .count();
                    let mut classifier = match cell.classifier {
                        VoxelTriangleSolidClassifier::Inside => {
                            component_report.inside_components += 1;
                            VoxelTriangleSolidClassifier::Inside
                        }
                        VoxelTriangleSolidClassifier::Outside => {
                            component_report.outside_components += 1;
                            VoxelTriangleSolidClassifier::Outside
                        }
                        VoxelTriangleSolidClassifier::Unknown
                        | VoxelTriangleSolidClassifier::Boundary => {
                            component_report.unknown_components += 1;
                            VoxelTriangleSolidClassifier::Unknown
                        }
                    };
                    if verify_component_arrangement
                        && matches!(
                            classifier,
                            VoxelTriangleSolidClassifier::Inside
                                | VoxelTriangleSolidClassifier::Outside
                        )
                    {
                        component_report.arrangement_verified_components += 1;
                        let audit = verify_component_arrangement_classification(
                            &component,
                            classifier,
                            &frame,
                            prepared,
                            &mut component_report,
                        )?;
                        if !audit {
                            match classifier {
                                VoxelTriangleSolidClassifier::Inside => {
                                    component_report.inside_components -= 1;
                                }
                                VoxelTriangleSolidClassifier::Outside => {
                                    component_report.outside_components -= 1;
                                }
                                _ => {}
                            }
                            component_report.unknown_components += 1;
                            classifier = VoxelTriangleSolidClassifier::Unknown;
                        }
                    }
                    classifier
                };

                for coords in component {
                    let index = cell_index(cells_per_axis, coords)?;
                    classifiers[index] = classifier;
                }
            }
        }
    }

    let (grid, report) = materialize_prepared_classifiers(
        frame,
        prepared.solid.surface.source.clone(),
        policy,
        material,
        &classifiers,
    )?;
    Ok((grid, report, component_report))
}

/// Aggregate prepared-schedule evidence over a voxelization pass.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct PreparedTriangleSolidVoxelizationReport {
    /// Number of cells classified.
    pub classified_cells: usize,
    /// Total exact AABB broad-phase rejections.
    pub boundary_aabb_rejections: usize,
    /// Total exact triangle/cell narrow-phase tests.
    pub boundary_triangle_tests: usize,
    /// Total exact ray-parity attempts.
    pub ray_attempts: usize,
    /// Total triangle AABBs rejected by exact ray/slab broad-phase scheduling.
    pub ray_aabb_rejections: usize,
    /// Total exact ray/triangle predicates.
    pub ray_triangle_tests: usize,
    /// Number of ambiguous ray attempts.
    pub ambiguous_ray_attempts: usize,
}

/// Aggregate connected-component schedule evidence over a prepared
/// triangle-solid voxelization pass.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct PreparedTriangleSolidComponentVoxelizationReport {
    /// Number of cells classified.
    pub classified_cells: usize,
    /// Number of cells proven to intersect the retained triangle boundary.
    pub boundary_cells: usize,
    /// Number of cells whose boundary relation was undecided.
    pub boundary_unknown_cells: usize,
    /// Number of cells proven disjoint from the retained triangle boundary.
    pub open_cells: usize,
    /// Total exact AABB broad-phase rejections in the boundary pass.
    pub boundary_aabb_rejections: usize,
    /// Total exact triangle/cell narrow-phase tests in the boundary pass.
    pub boundary_triangle_tests: usize,
    /// Number of connected open-cell components.
    pub components: usize,
    /// Number of open components touching the frame boundary.
    pub exterior_components: usize,
    /// Number of non-exterior components classified by representative ray.
    pub ray_classified_components: usize,
    /// Number of components classified as inside.
    pub inside_components: usize,
    /// Number of components classified as outside.
    pub outside_components: usize,
    /// Number of components that remained unknown.
    pub unknown_components: usize,
    /// Total exact ray-parity attempts used for representative cells.
    pub component_ray_attempts: usize,
    /// Total triangle AABBs rejected by exact ray/slab broad-phase scheduling
    /// for representative cells.
    pub component_ray_aabb_rejections: usize,
    /// Total exact ray/triangle predicates used for representative cells.
    pub component_ray_triangle_tests: usize,
    /// Number of ambiguous representative ray attempts skipped before a
    /// certified parity decision or component unknown.
    pub ambiguous_component_ray_attempts: usize,
    /// Number of enclosed components whose every open cell was replayed for
    /// arrangement consistency.
    pub arrangement_verified_components: usize,
    /// Number of non-representative open cells replayed during arrangement
    /// verification.
    pub arrangement_verified_cells: usize,
    /// Number of replayed open cells whose inside/outside parity disagreed
    /// with the component representative.
    pub arrangement_conflicting_cells: usize,
    /// Number of replayed open cells whose parity remained unknown.
    pub arrangement_unknown_cells: usize,
    /// Number of replayed open cells that unexpectedly reclassified as
    /// boundary despite the boundary pass marking them open.
    pub arrangement_boundary_regression_cells: usize,
    /// Exact ray-parity attempts consumed by arrangement verification.
    pub arrangement_ray_attempts: usize,
    /// Exact ray/AABB broad-phase rejections consumed by arrangement
    /// verification.
    pub arrangement_ray_aabb_rejections: usize,
    /// Exact ray/triangle predicates consumed by arrangement verification.
    pub arrangement_ray_triangle_tests: usize,
    /// Ambiguous ray attempts seen during arrangement verification.
    pub ambiguous_arrangement_ray_attempts: usize,
}

/// Aggregate exact row-sweep evidence over a prepared triangle-solid pass.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct PreparedTriangleSolidAxisSweepVoxelizationReport {
    /// Number of cells classified.
    pub classified_cells: usize,
    /// Number of cells proven to intersect the retained triangle boundary.
    pub boundary_cells: usize,
    /// Number of cells whose boundary relation was undecided.
    pub boundary_unknown_cells: usize,
    /// Number of cells proven disjoint from the retained triangle boundary.
    pub open_cells: usize,
    /// Total exact AABB broad-phase rejections in the boundary pass.
    pub boundary_aabb_rejections: usize,
    /// Total exact triangle/cell narrow-phase tests in the boundary pass.
    pub boundary_triangle_tests: usize,
    /// Number of `(y,z)` rows containing at least one open cell.
    pub sweep_rows: usize,
    /// Number of rows with no open cells.
    pub empty_sweep_rows: usize,
    /// Rows classified by a certified exact `+X` crossing sequence.
    pub certified_sweep_rows: usize,
    /// Rows rejected from sweep reuse because of edge/vertex/coplanar
    /// ambiguity.
    pub ambiguous_sweep_rows: usize,
    /// Open cells classified directly by certified row-sweep parity.
    pub sweep_classified_cells: usize,
    /// Open cells classified by the per-cell fallback path.
    pub fallback_cells: usize,
    /// Fallback cells that remained unknown.
    pub fallback_unknown_cells: usize,
    /// Fallback cells that unexpectedly reclassified as boundary after the
    /// boundary pass marked them open.
    pub fallback_boundary_regression_cells: usize,
    /// Exact ray/AABB broad-phase rejections consumed by row sweeps.
    pub row_ray_aabb_rejections: usize,
    /// Exact ray/triangle predicates consumed by row sweeps.
    pub row_ray_triangle_tests: usize,
    /// Proper row-ray/triangle intersections before unique-parameter collapse.
    pub row_proper_intersections: usize,
    /// Sum of unique exact crossing parameters retained by certified rows.
    pub row_unique_parameters: usize,
    /// Boundary-touch events that made row sweeps ambiguous.
    pub row_boundary_touches: usize,
    /// Coplanar events that made row sweeps ambiguous.
    pub row_coplanar_events: usize,
    /// Certified row parameters that could not be ordered against a cell
    /// center threshold.
    pub row_parameter_order_unknowns: usize,
    /// Exact ray-parity attempts consumed by fallback cells.
    pub fallback_ray_attempts: usize,
    /// Exact ray/AABB broad-phase rejections consumed by fallback cells.
    pub fallback_ray_aabb_rejections: usize,
    /// Exact ray/triangle predicates consumed by fallback cells.
    pub fallback_ray_triangle_tests: usize,
    /// Ambiguous ray attempts seen during fallback classification.
    pub ambiguous_fallback_ray_attempts: usize,
    /// Whether the row-sweep pass produced exact arrangement evidence for all
    /// cells.
    pub exact_axis_sweep_ready: bool,
}

impl PreparedTriangleSolidVoxelizationReport {
    fn accumulate(&mut self, cell: &PreparedTriangleSolidCellReport) {
        self.classified_cells += 1;
        self.boundary_aabb_rejections += cell.boundary_aabb_rejections;
        self.boundary_triangle_tests += cell.boundary_triangle_tests;
        self.ray_attempts += cell.ray_attempts.len();
        self.ray_aabb_rejections += cell
            .ray_attempts
            .iter()
            .map(|attempt| attempt.ray_aabb_rejections)
            .sum::<usize>();
        self.ray_triangle_tests += cell.ray_triangle_tests();
        self.ambiguous_ray_attempts += cell
            .ray_attempts
            .iter()
            .filter(|attempt| !attempt.certified)
            .count();
    }
}

fn verify_component_arrangement_classification(
    component: &[[u64; 3]],
    representative_classifier: VoxelTriangleSolidClassifier,
    frame: &GridFrame,
    prepared: &PreparedExactTriangleSolidMesh,
    component_report: &mut PreparedTriangleSolidComponentVoxelizationReport,
) -> HypervoxelResult<bool> {
    let mut exact_arrangement_ready = true;
    for coords in component.iter().skip(1) {
        let address = VoxelAddress::new(frame.depth(), *coords)?;
        let cell = classify_cell_against_prepared_triangle_solid_mesh(address, frame, prepared)?;
        component_report.arrangement_verified_cells += 1;
        component_report.arrangement_ray_attempts += cell.ray_attempts.len();
        component_report.arrangement_ray_aabb_rejections += cell
            .ray_attempts
            .iter()
            .map(|attempt| attempt.ray_aabb_rejections)
            .sum::<usize>();
        component_report.arrangement_ray_triangle_tests += cell.ray_triangle_tests();
        component_report.ambiguous_arrangement_ray_attempts += cell
            .ray_attempts
            .iter()
            .filter(|attempt| !attempt.certified)
            .count();

        match cell.classifier {
            classifier if classifier == representative_classifier => {}
            VoxelTriangleSolidClassifier::Unknown => {
                component_report.arrangement_unknown_cells += 1;
                exact_arrangement_ready = false;
            }
            VoxelTriangleSolidClassifier::Boundary => {
                component_report.arrangement_boundary_regression_cells += 1;
                exact_arrangement_ready = false;
            }
            VoxelTriangleSolidClassifier::Inside | VoxelTriangleSolidClassifier::Outside => {
                component_report.arrangement_conflicting_cells += 1;
                exact_arrangement_ready = false;
            }
        }
    }
    Ok(exact_arrangement_ready)
}

enum AxisRowParity {
    Certified { parameters: Vec<Real> },
    Ambiguous,
}

fn classify_axis_row_against_prepared_triangle_solid(
    row_origin: &[Real; 3],
    prepared: &PreparedExactTriangleSolidMesh,
    sweep_report: &mut PreparedTriangleSolidAxisSweepVoxelizationReport,
) -> HypervoxelResult<AxisRowParity> {
    let origin = point3(row_origin);
    let direction = hyperlimit::Point3::new(Real::from(1), Real::from(0), Real::from(0));
    let direction_components = point_components(&direction);
    let mut parameters = Vec::<Real>::new();

    for triangle in &prepared.triangles {
        match classify_ray_aabb_intersection(row_origin, &direction_components, &triangle.bounds)? {
            RayAabbIntersection::Disjoint => {
                sweep_report.row_ray_aabb_rejections += 1;
                continue;
            }
            RayAabbIntersection::Intersects => {}
        }

        sweep_report.row_ray_triangle_tests += 1;
        let report = classify_ray_triangle3_intersection_report(
            &origin,
            &direction,
            &triangle.points[0],
            &triangle.points[1],
            &triangle.points[2],
        )
        .value()
        .ok_or(HypervoxelError::UnknownScalarOrdering {
            field: "axis-sweep-triangle-solid-ray",
        })?;
        match report.relation {
            RayTriangleIntersection::Disjoint => {}
            RayTriangleIntersection::Proper => {
                let Some(parameter) = report.parameter else {
                    return Ok(AxisRowParity::Ambiguous);
                };
                sweep_report.row_proper_intersections += 1;
                insert_unique_parameter(&mut parameters, parameter)?;
            }
            RayTriangleIntersection::BoundaryTouch => {
                sweep_report.row_boundary_touches += 1;
                return Ok(AxisRowParity::Ambiguous);
            }
            RayTriangleIntersection::Coplanar => {
                sweep_report.row_coplanar_events += 1;
                return Ok(AxisRowParity::Ambiguous);
            }
        }
    }

    sweep_report.row_unique_parameters += parameters.len();
    Ok(AxisRowParity::Certified { parameters })
}

fn classify_axis_sweep_center(
    parameters: &[Real],
    threshold: &Real,
) -> HypervoxelResult<Option<VoxelTriangleSolidClassifier>> {
    let mut crossings_after_center = 0_usize;
    for parameter in parameters {
        match hyperlimit::compare_reals(parameter, threshold).value() {
            Some(Ordering::Greater) => crossings_after_center += 1,
            Some(Ordering::Less) => {}
            Some(Ordering::Equal) => return Ok(None),
            None => {
                return Err(HypervoxelError::UnknownScalarOrdering {
                    field: "axis-sweep-parameter-order",
                });
            }
        }
    }
    Ok(Some(if crossings_after_center % 2 == 0 {
        VoxelTriangleSolidClassifier::Outside
    } else {
        VoxelTriangleSolidClassifier::Inside
    }))
}

fn classify_axis_sweep_fallback_cell(
    coords: [u64; 3],
    frame: &GridFrame,
    prepared: &PreparedExactTriangleSolidMesh,
    classifiers: &mut [VoxelTriangleSolidClassifier],
    cells_per_axis: u64,
    sweep_report: &mut PreparedTriangleSolidAxisSweepVoxelizationReport,
) -> HypervoxelResult<()> {
    let address = VoxelAddress::new(frame.depth(), coords)?;
    let cell = classify_cell_against_prepared_triangle_solid_mesh(address, frame, prepared)?;
    sweep_report.fallback_cells += 1;
    sweep_report.fallback_ray_attempts += cell.ray_attempts.len();
    sweep_report.fallback_ray_aabb_rejections += cell
        .ray_attempts
        .iter()
        .map(|attempt| attempt.ray_aabb_rejections)
        .sum::<usize>();
    sweep_report.fallback_ray_triangle_tests += cell.ray_triangle_tests();
    sweep_report.ambiguous_fallback_ray_attempts += cell
        .ray_attempts
        .iter()
        .filter(|attempt| !attempt.certified)
        .count();

    match cell.classifier {
        VoxelTriangleSolidClassifier::Inside | VoxelTriangleSolidClassifier::Outside => {}
        VoxelTriangleSolidClassifier::Unknown => sweep_report.fallback_unknown_cells += 1,
        VoxelTriangleSolidClassifier::Boundary => {
            sweep_report.fallback_boundary_regression_cells += 1;
        }
    }
    let index = cell_index(cells_per_axis, coords)?;
    classifiers[index] = cell.classifier;
    Ok(())
}

fn materialize_prepared_classifiers(
    frame: GridFrame,
    source: Option<crate::GridSource>,
    policy: VoxelizationPolicy,
    material: MaterialRegionId,
    classifiers: &[VoxelTriangleSolidClassifier],
) -> HypervoxelResult<(SparseVoxelGrid, VoxelizationReport)> {
    let mut grid = SparseVoxelGrid::new(frame.clone());
    let mut inside_cells = 0_usize;
    let mut outside_cells = 0_usize;
    let mut boundary_cells = 0_usize;
    let mut unknown_cells = 0_usize;
    let mut predicate_boundary_cells = 0_usize;
    let mut predicate_unknown_cells = 0_usize;
    let cells_per_axis = frame.cells_per_axis();

    for z in 0..cells_per_axis {
        for y in 0..cells_per_axis {
            for x in 0..cells_per_axis {
                let index = cell_index(cells_per_axis, [x, y, z])?;
                let classifier = classifiers[index];
                match classifier {
                    VoxelTriangleSolidClassifier::Inside => inside_cells += 1,
                    VoxelTriangleSolidClassifier::Outside => outside_cells += 1,
                    VoxelTriangleSolidClassifier::Boundary => predicate_boundary_cells += 1,
                    VoxelTriangleSolidClassifier::Unknown => predicate_unknown_cells += 1,
                }

                let cell = match (policy.quantization, policy.boundary, classifier) {
                    (_, _, VoxelTriangleSolidClassifier::Outside) => VoxelCell::empty(),
                    (_, _, VoxelTriangleSolidClassifier::Unknown) => {
                        unknown_cells += 1;
                        VoxelCell::unknown()
                    }
                    (_, _, VoxelTriangleSolidClassifier::Inside) => VoxelCell::material(material),
                    (
                        QuantizationPolicy::ConservativeInterior,
                        _,
                        VoxelTriangleSolidClassifier::Boundary,
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
                    (
                        _,
                        BoundaryPolicy::BoundaryAsUnknown,
                        VoxelTriangleSolidClassifier::Boundary,
                    ) => {
                        boundary_cells += 1;
                        unknown_cells += 1;
                        VoxelCell::unknown()
                    }
                    (
                        _,
                        BoundaryPolicy::LossySideChoice,
                        VoxelTriangleSolidClassifier::Boundary,
                    ) => {
                        boundary_cells += 1;
                        VoxelCell {
                            occupancy: OccupancyState::LossyAdapterValue,
                            payload: VoxelPayload::LossyAdapterValue(material.0),
                        }
                    }
                    (_, BoundaryPolicy::KeepBoundary, VoxelTriangleSolidClassifier::Boundary) => {
                        boundary_cells += 1;
                        VoxelCell::boundary(VoxelPayload::MaterialRegion(material))
                    }
                };

                if cell.occupancy != OccupancyState::Empty {
                    grid.set(VoxelAddress::new(frame.depth(), [x, y, z])?, cell)?;
                }
            }
        }
    }

    let aggregate = VoxelAggregateFacts::from_explicit_cells_in_frame(
        classifiers.len(),
        grid.iter().map(|(_, cell)| cell),
    )?;
    let report = VoxelizationReport {
        source,
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

fn cell_index(cells_per_axis: u64, coords: [u64; 3]) -> HypervoxelResult<usize> {
    usize::try_from((coords[2] * cells_per_axis + coords[1]) * cells_per_axis + coords[0])
        .map_err(|_| HypervoxelError::AddressOverflow)
}

fn component_neighbors(cells_per_axis: u64, coords: [u64; 3]) -> Vec<[u64; 3]> {
    let mut neighbors = Vec::with_capacity(6);
    for axis in 0..3 {
        if coords[axis] > 0 {
            let mut neighbor = coords;
            neighbor[axis] -= 1;
            neighbors.push(neighbor);
        }
        if coords[axis] + 1 < cells_per_axis {
            let mut neighbor = coords;
            neighbor[axis] += 1;
            neighbors.push(neighbor);
        }
    }
    neighbors
}

fn classify_point_against_prepared_triangle_solid_by_ray(
    point: &[Real; 3],
    prepared: &PreparedExactTriangleSolidMesh,
) -> HypervoxelResult<(
    VoxelTriangleSolidClassifier,
    Vec<PreparedRayParityAttemptReport>,
)> {
    let mut attempts = Vec::new();
    for (direction_index, direction) in ray_parity_directions().into_iter().enumerate() {
        let (classification, attempt) =
            classify_point_against_prepared_triangle_solid_by_single_ray(
                point,
                prepared,
                &direction,
                direction_index,
            )?;
        let certified = attempt.certified;
        attempts.push(attempt);
        if certified {
            return Ok((classification, attempts));
        }
    }
    Ok((VoxelTriangleSolidClassifier::Unknown, attempts))
}

fn classify_point_against_prepared_triangle_solid_by_single_ray(
    point: &[Real; 3],
    prepared: &PreparedExactTriangleSolidMesh,
    direction: &hyperlimit::Point3,
    direction_index: usize,
) -> HypervoxelResult<(VoxelTriangleSolidClassifier, PreparedRayParityAttemptReport)> {
    let origin = point3(point);
    let direction_components = point_components(direction);
    let mut parameters: Vec<Real> = Vec::new();
    let mut attempt = PreparedRayParityAttemptReport {
        direction_index,
        ray_aabb_rejections: 0,
        triangle_tests: 0,
        proper_intersections: 0,
        unique_parameters: 0,
        boundary_touches: 0,
        coplanar_events: 0,
        certified: false,
    };

    for triangle in &prepared.triangles {
        match classify_ray_aabb_intersection(point, &direction_components, &triangle.bounds)? {
            RayAabbIntersection::Disjoint => {
                attempt.ray_aabb_rejections += 1;
                continue;
            }
            RayAabbIntersection::Intersects => {}
        }

        attempt.triangle_tests += 1;
        let report = classify_ray_triangle3_intersection_report(
            &origin,
            direction,
            &triangle.points[0],
            &triangle.points[1],
            &triangle.points[2],
        )
        .value()
        .ok_or(HypervoxelError::UnknownScalarOrdering {
            field: "prepared-triangle-solid-ray",
        })?;
        match report.relation {
            RayTriangleIntersection::Disjoint => {}
            RayTriangleIntersection::Proper => {
                let Some(parameter) = report.parameter else {
                    return Ok((VoxelTriangleSolidClassifier::Unknown, attempt));
                };
                attempt.proper_intersections += 1;
                insert_unique_parameter(&mut parameters, parameter)?;
                attempt.unique_parameters = parameters.len();
            }
            RayTriangleIntersection::BoundaryTouch => {
                attempt.boundary_touches += 1;
                return Ok((VoxelTriangleSolidClassifier::Unknown, attempt));
            }
            RayTriangleIntersection::Coplanar => {
                attempt.coplanar_events += 1;
                return Ok((VoxelTriangleSolidClassifier::Unknown, attempt));
            }
        }
    }

    attempt.certified = true;
    let classifier = if parameters.len() % 2 == 0 {
        VoxelTriangleSolidClassifier::Outside
    } else {
        VoxelTriangleSolidClassifier::Inside
    };
    Ok((classifier, attempt))
}

fn point_components(point: &hyperlimit::Point3) -> [Real; 3] {
    [point.x.clone(), point.y.clone(), point.z.clone()]
}
