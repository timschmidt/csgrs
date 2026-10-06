//! Exact triangle-solid construction, scheduling, and voxelization.
//!
//! This module is the scheduled counterpart to [`crate::triangle_mesh`]. It
//! keeps retained source triangles as the owning evidence, stores exact
//! triangle AABBs on the solid, and reports how many triangle predicates each
//! cell actually consumed. Scheduled data are replayable object facts, not
//! approximate hints that may silently change topology.

use std::collections::VecDeque;

use core::cmp::Ordering;
use hyperlimit::{
    Aabb3Intersection, RayTriangleIntersection, classify_aabb3_intersection,
    classify_ray_triangle3_intersection_report,
};

use hyperreal::Real;

use crate::component_row_plan::plan_component_axis_rows;
use crate::ray_schedule::{
    RayAabbIntersection, RayAabbWindowIntersection, classify_ray_aabb_intersection,
    classify_ray_aabb_intersection_from_lower,
};
use crate::triangle_mesh::{
    ExactTriangleSolidMesh, TriangleCellIntersection, VoxelTriangleSolidClassification,
    insert_unique_parameter, point3, ray_parity_directions, triangle_bounds,
    triangle_intersects_cell_after_aabb,
};
use crate::triangle_row_cache::{ComponentAxisRowCache, ComponentAxisRowKey};
use crate::{
    BoundaryPolicy, GridFrame, HypervoxelError, HypervoxelResult, MaterialRegionId, OccupancyState,
    QuantizationPolicy, SparseVoxelGrid, VoxelAddress, VoxelAggregateFacts, VoxelCell,
    VoxelPayload, VoxelPredicateCertificateReport, VoxelizationPolicy, VoxelizationReport,
};

/// Retained triangle facts for exact solid classification.
#[derive(Clone, Debug, PartialEq)]
struct ExactTriangleFacet {
    source_index: usize,
    bounds: crate::ExactAabb3,
    points: [hyperlimit::Point3; 3],
}

/// Validated exact triangle solid with retained broad-phase facts.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactTriangleSolid {
    mesh: ExactTriangleSolidMesh,
    triangles: Vec<ExactTriangleFacet>,
}

impl ExactTriangleSolid {
    /// Validates a retained closed mesh and stores its exact triangle bounds.
    ///
    /// Construction replays the source readiness gate before storing exact
    /// AABBs. These broad-phase facts schedule, but never replace, the later
    /// triangle/cell predicates.
    pub fn new(mesh: ExactTriangleSolidMesh) -> HypervoxelResult<Self> {
        if mesh.surface.triangles.is_empty() {
            return Err(HypervoxelError::InvalidSourceGeometry {
                reason: "triangle surface mesh has no triangles",
            });
        }
        if !mesh.exact_closed_solid_replay_available {
            return Err(HypervoxelError::InvalidSourceGeometry {
                reason: "triangle solid mesh lacks exact closed-solid replay",
            });
        }

        let mut triangles = Vec::with_capacity(mesh.surface.triangles.len());
        for (source_index, triangle) in mesh.surface.triangles.iter().enumerate() {
            let (points, report) = triangle.points_and_report();
            if report.degenerate {
                return Err(HypervoxelError::InvalidSourceGeometry {
                    reason: "triangle surface mesh contains degenerate triangles",
                });
            }
            if report.unknown_predicate {
                return Err(HypervoxelError::InvalidSourceGeometry {
                    reason: "triangle surface mesh has uncertified triangle predicates",
                });
            }
            let bounds = triangle_bounds(triangle)?;
            triangles.push(ExactTriangleFacet {
                source_index,
                bounds,
                points,
            });
        }

        Ok(Self { mesh, triangles })
    }

    /// Returns the retained source mesh.
    pub const fn mesh(&self) -> &ExactTriangleSolidMesh {
        &self.mesh
    }

    /// Returns the number of retained boundary triangles.
    pub const fn triangle_count(&self) -> usize {
        self.triangles.len()
    }
}

/// Per-cell exact scheduled solid-classification evidence.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TriangleSolidCellReport {
    /// Final cell classification.
    pub classification: VoxelTriangleSolidClassification,
    /// Triangles rejected by exact cell-AABB/triangle-AABB disjointness.
    pub boundary_aabb_rejections: usize,
    /// Triangles reaching the exact triangle/cell narrow phase.
    pub boundary_triangle_tests: usize,
    /// Whether any boundary predicate returned unknown.
    pub boundary_unknown: bool,
    /// Exact ray parity attempts used after boundary rejection.
    pub ray_attempts: Vec<RayParityAttemptReport>,
}

impl TriangleSolidCellReport {
    /// Whether all predicates needed for this cell were proof-producing.
    pub fn is_fully_certified(&self) -> bool {
        self.classification != VoxelTriangleSolidClassification::Unknown && !self.boundary_unknown
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
pub struct RayParityAttemptReport {
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

/// Classify one cell against an exact triangle solid.
pub fn classify_cell_against_exact_triangle_solid(
    address: VoxelAddress,
    frame: &GridFrame,
    solid: &ExactTriangleSolid,
) -> HypervoxelResult<TriangleSolidCellReport> {
    let bounds = address.bounds(frame)?;
    let boundary = classify_cell_boundary_against_exact_triangle_solid(&bounds, solid)?;

    if boundary.classification != VoxelTriangleSolidClassification::Outside {
        return Ok(boundary);
    }

    let (classifier, ray_attempts) =
        classify_point_against_exact_triangle_solid_by_ray(&bounds.center(), solid)?;
    Ok(TriangleSolidCellReport {
        classification: classifier,
        ray_attempts,
        ..boundary
    })
}

fn classify_cell_boundary_against_exact_triangle_solid(
    bounds: &crate::CellBounds,
    solid: &ExactTriangleSolid,
) -> HypervoxelResult<TriangleSolidCellReport> {
    let cell_min = point3(&bounds.min);
    let cell_max = point3(&bounds.max);
    let mut boundary_aabb_rejections = 0_usize;
    let mut boundary_triangle_tests = 0_usize;
    let mut boundary_unknown = false;

    for triangle in &solid.triangles {
        let relation = classify_aabb3_intersection(
            &point3(&triangle.bounds.min),
            &point3(&triangle.bounds.max),
            &cell_min,
            &cell_max,
            hyperlimit::PredicatePolicy::STRICT,
        )
        .value()
        .ok_or(HypervoxelError::UnknownScalarOrdering {
            field: "triangle-solid-aabb",
        })?;
        if relation == Aabb3Intersection::Disjoint {
            boundary_aabb_rejections += 1;
            continue;
        }

        boundary_triangle_tests += 1;
        match triangle_intersects_cell_after_aabb(
            &solid.mesh.surface.triangles[triangle.source_index],
            bounds,
        )? {
            TriangleCellIntersection::Intersects => {
                return Ok(TriangleSolidCellReport {
                    classification: VoxelTriangleSolidClassification::Boundary,
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
        return Ok(TriangleSolidCellReport {
            classification: VoxelTriangleSolidClassification::Unknown,
            boundary_aabb_rejections,
            boundary_triangle_tests,
            boundary_unknown,
            ray_attempts: Vec::new(),
        });
    }

    Ok(TriangleSolidCellReport {
        classification: VoxelTriangleSolidClassification::Outside,
        boundary_aabb_rejections,
        boundary_triangle_tests,
        boundary_unknown,
        ray_attempts: Vec::new(),
    })
}

/// Voxelize an exact closed triangle solid.
pub fn voxelize_exact_triangle_solid(
    frame: GridFrame,
    solid: &ExactTriangleSolid,
    material: MaterialRegionId,
    policy: VoxelizationPolicy,
) -> HypervoxelResult<(
    SparseVoxelGrid,
    VoxelizationReport,
    TriangleSolidVoxelizationReport,
)> {
    let mut grid = SparseVoxelGrid::new(frame.clone());
    let mut inside_cells = 0_usize;
    let mut outside_cells = 0_usize;
    let mut boundary_cells = 0_usize;
    let mut unknown_cells = 0_usize;
    let mut predicate_boundary_cells = 0_usize;
    let mut predicate_unknown_cells = 0_usize;
    let mut schedule_report = TriangleSolidVoxelizationReport::default();
    let cells_per_axis = frame.cells_per_axis();

    for z in 0..cells_per_axis {
        for y in 0..cells_per_axis {
            for x in 0..cells_per_axis {
                let address = VoxelAddress::new(frame.depth(), [x, y, z])?;
                let cell_report =
                    classify_cell_against_exact_triangle_solid(address, &frame, solid)?;
                schedule_report.accumulate(&cell_report);

                match cell_report.classification {
                    VoxelTriangleSolidClassification::Inside => inside_cells += 1,
                    VoxelTriangleSolidClassification::Outside => outside_cells += 1,
                    VoxelTriangleSolidClassification::Boundary => predicate_boundary_cells += 1,
                    VoxelTriangleSolidClassification::Unknown => predicate_unknown_cells += 1,
                }

                let cell = match (
                    policy.quantization,
                    policy.boundary,
                    cell_report.classification,
                ) {
                    (_, _, VoxelTriangleSolidClassification::Outside) => VoxelCell::empty(),
                    (_, _, VoxelTriangleSolidClassification::Unknown) => {
                        unknown_cells += 1;
                        VoxelCell::unknown()
                    }
                    (_, _, VoxelTriangleSolidClassification::Inside) => {
                        VoxelCell::material(material)
                    }
                    (
                        QuantizationPolicy::ConservativeInterior,
                        _,
                        VoxelTriangleSolidClassification::Boundary,
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
                        VoxelTriangleSolidClassification::Boundary,
                    ) => {
                        boundary_cells += 1;
                        unknown_cells += 1;
                        VoxelCell::unknown()
                    }
                    (
                        _,
                        BoundaryPolicy::LossySideChoice,
                        VoxelTriangleSolidClassification::Boundary,
                    ) => {
                        boundary_cells += 1;
                        VoxelCell {
                            occupancy: OccupancyState::LossyAdapterValue,
                            payload: VoxelPayload::LossyAdapterValue(material.0),
                        }
                    }
                    (
                        _,
                        BoundaryPolicy::KeepBoundary,
                        VoxelTriangleSolidClassification::Boundary,
                    ) => {
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
    Ok((grid, report, schedule_report))
}

/// Voxelize an exact closed triangle solid by connected non-boundary
/// components.
///
/// This is an arrangement-style accelerator above the per-cell parity
/// path. It performs the exact boundary classification for every cell, labels
/// connected components of cells proven disjoint from the boundary, marks
/// components touching the grid boundary as exterior, and ray-classifies only
/// one representative cell for every remaining component. Components use
/// 6-neighbor digital topology. Boundary predicates and representative parity
/// queries must be proof-producing; otherwise the whole component is unknown.
pub fn voxelize_exact_triangle_solid_by_components(
    frame: GridFrame,
    solid: &ExactTriangleSolid,
    material: MaterialRegionId,
    policy: VoxelizationPolicy,
) -> HypervoxelResult<(
    SparseVoxelGrid,
    VoxelizationReport,
    TriangleSolidComponentVoxelizationReport,
)> {
    voxelize_exact_triangle_solid_by_components_impl(frame, solid, material, policy)
}

fn voxelize_exact_triangle_solid_by_components_impl(
    frame: GridFrame,
    solid: &ExactTriangleSolid,
    material: MaterialRegionId,
    policy: VoxelizationPolicy,
) -> HypervoxelResult<(
    SparseVoxelGrid,
    VoxelizationReport,
    TriangleSolidComponentVoxelizationReport,
)> {
    let cells_per_axis = frame.cells_per_axis();
    let total_cells =
        usize::try_from(cells_per_axis.pow(3)).map_err(|_| HypervoxelError::AddressOverflow)?;
    let mut classifiers = vec![VoxelTriangleSolidClassification::Unknown; total_cells];
    let mut open = vec![false; total_cells];
    let mut visited = vec![false; total_cells];
    let mut component_report = TriangleSolidComponentVoxelizationReport {
        classified_cells: total_cells,
        ..TriangleSolidComponentVoxelizationReport::default()
    };

    for z in 0..cells_per_axis {
        for y in 0..cells_per_axis {
            for x in 0..cells_per_axis {
                let address = VoxelAddress::new(frame.depth(), [x, y, z])?;
                let bounds = address.bounds(&frame)?;
                let boundary = classify_cell_boundary_against_exact_triangle_solid(&bounds, solid)?;
                let index = cell_index(cells_per_axis, [x, y, z])?;
                component_report.boundary_aabb_rejections += boundary.boundary_aabb_rejections;
                component_report.boundary_triangle_tests += boundary.boundary_triangle_tests;
                match boundary.classification {
                    VoxelTriangleSolidClassification::Boundary => {
                        classifiers[index] = VoxelTriangleSolidClassification::Boundary;
                        component_report.boundary_cells += 1;
                    }
                    VoxelTriangleSolidClassification::Unknown => {
                        classifiers[index] = VoxelTriangleSolidClassification::Unknown;
                        component_report.boundary_unknown_cells += 1;
                    }
                    VoxelTriangleSolidClassification::Outside => {
                        open[index] = true;
                        component_report.open_cells += 1;
                    }
                    VoxelTriangleSolidClassification::Inside => {
                        unreachable!("boundary-only exact classification never emits inside cells")
                    }
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
                    VoxelTriangleSolidClassification::Outside
                } else {
                    component_report.ray_classified_components += 1;
                    let address = VoxelAddress::new(frame.depth(), component[0])?;
                    let cell = classify_cell_against_exact_triangle_solid(address, &frame, solid)?;
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
                    match cell.classification {
                        VoxelTriangleSolidClassification::Inside => {
                            component_report.inside_components += 1;
                            VoxelTriangleSolidClassification::Inside
                        }
                        VoxelTriangleSolidClassification::Outside => {
                            component_report.outside_components += 1;
                            VoxelTriangleSolidClassification::Outside
                        }
                        VoxelTriangleSolidClassification::Unknown
                        | VoxelTriangleSolidClassification::Boundary => {
                            component_report.unknown_components += 1;
                            VoxelTriangleSolidClassification::Unknown
                        }
                    }
                };

                for coords in component {
                    let index = cell_index(cells_per_axis, coords)?;
                    classifiers[index] = classifier;
                }
            }
        }
    }

    let (grid, report) =
        materialize_triangle_solid_classifiers(frame, policy, material, &classifiers)?;
    Ok((grid, report, component_report))
}

/// Voxelize an exact closed triangle solid by exact row-parity sweeps.
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
/// The row sweep lifts point-in-polyhedron parity to a report-bearing
/// arrangement pass. A row decision is exact only when its sorted crossing
/// sequence is certified; all acceptance comes from exact ray/triangle
/// predicates.
pub fn voxelize_exact_triangle_solid_by_axis_sweeps(
    frame: GridFrame,
    solid: &ExactTriangleSolid,
    material: MaterialRegionId,
    policy: VoxelizationPolicy,
) -> HypervoxelResult<(
    SparseVoxelGrid,
    VoxelizationReport,
    TriangleSolidAxisSweepVoxelizationReport,
)> {
    let cells_per_axis = frame.cells_per_axis();
    let total_cells =
        usize::try_from(cells_per_axis.pow(3)).map_err(|_| HypervoxelError::AddressOverflow)?;
    let mut classifiers = vec![VoxelTriangleSolidClassification::Unknown; total_cells];
    let mut open = vec![false; total_cells];
    let mut sweep_report = TriangleSolidAxisSweepVoxelizationReport {
        classified_cells: total_cells,
        ..TriangleSolidAxisSweepVoxelizationReport::default()
    };

    for z in 0..cells_per_axis {
        for y in 0..cells_per_axis {
            for x in 0..cells_per_axis {
                let address = VoxelAddress::new(frame.depth(), [x, y, z])?;
                let bounds = address.bounds(&frame)?;
                let boundary = classify_cell_boundary_against_exact_triangle_solid(&bounds, solid)?;
                let index = cell_index(cells_per_axis, [x, y, z])?;
                sweep_report.boundary_aabb_rejections += boundary.boundary_aabb_rejections;
                sweep_report.boundary_triangle_tests += boundary.boundary_triangle_tests;
                match boundary.classification {
                    VoxelTriangleSolidClassification::Boundary => {
                        classifiers[index] = VoxelTriangleSolidClassification::Boundary;
                        sweep_report.boundary_cells += 1;
                    }
                    VoxelTriangleSolidClassification::Unknown => {
                        classifiers[index] = VoxelTriangleSolidClassification::Unknown;
                        sweep_report.boundary_unknown_cells += 1;
                    }
                    VoxelTriangleSolidClassification::Outside => {
                        open[index] = true;
                        sweep_report.open_cells += 1;
                    }
                    VoxelTriangleSolidClassification::Inside => {
                        unreachable!("boundary-only exact classification never emits inside cells")
                    }
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
            let row = classify_axis_row_against_exact_triangle_solid(
                0,
                &row_origin,
                solid,
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
                                solid,
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
                            solid,
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

    let (grid, report) =
        materialize_triangle_solid_classifiers(frame, policy, material, &classifiers)?;
    Ok((grid, report, sweep_report))
}

/// Voxelize an exact closed triangle solid by adaptive exact row sweeps.
///
/// This is the multi-axis counterpart to
/// [`voxelize_exact_triangle_solid_by_axis_sweeps`]. It performs
/// the same exact boundary pass, then tries to classify every remaining open
/// cell by certified row parity along `+X`, `+Y`, and `+Z` before falling back
/// to the multi-direction per-cell classifier. A row is reused only when the
/// exact ray/triangle crossing sequence is free of vertex, edge, coplanar, and
/// parameter-order ambiguity.
///
/// The accelerator is an exact arrangement replay with explicit refusal
/// states, not a numerical shortcut. Row batching is a discrete sweep-line
/// arrangement, and the final parity rule uses exact rational ray/triangle
/// predicates.
pub fn voxelize_exact_triangle_solid_by_adaptive_axis_sweeps(
    frame: GridFrame,
    solid: &ExactTriangleSolid,
    material: MaterialRegionId,
    policy: VoxelizationPolicy,
) -> HypervoxelResult<(
    SparseVoxelGrid,
    VoxelizationReport,
    TriangleSolidAdaptiveAxisSweepVoxelizationReport,
)> {
    let cells_per_axis = frame.cells_per_axis();
    let total_cells =
        usize::try_from(cells_per_axis.pow(3)).map_err(|_| HypervoxelError::AddressOverflow)?;
    let mut classifiers = vec![VoxelTriangleSolidClassification::Unknown; total_cells];
    let mut remaining_open = vec![false; total_cells];
    let mut adaptive_report = TriangleSolidAdaptiveAxisSweepVoxelizationReport {
        classified_cells: total_cells,
        ..TriangleSolidAdaptiveAxisSweepVoxelizationReport::default()
    };

    for z in 0..cells_per_axis {
        for y in 0..cells_per_axis {
            for x in 0..cells_per_axis {
                let address = VoxelAddress::new(frame.depth(), [x, y, z])?;
                let bounds = address.bounds(&frame)?;
                let boundary = classify_cell_boundary_against_exact_triangle_solid(&bounds, solid)?;
                let index = cell_index(cells_per_axis, [x, y, z])?;
                adaptive_report.boundary_aabb_rejections += boundary.boundary_aabb_rejections;
                adaptive_report.boundary_triangle_tests += boundary.boundary_triangle_tests;
                match boundary.classification {
                    VoxelTriangleSolidClassification::Boundary => {
                        classifiers[index] = VoxelTriangleSolidClassification::Boundary;
                        adaptive_report.boundary_cells += 1;
                    }
                    VoxelTriangleSolidClassification::Unknown => {
                        classifiers[index] = VoxelTriangleSolidClassification::Unknown;
                        adaptive_report.boundary_unknown_cells += 1;
                    }
                    VoxelTriangleSolidClassification::Outside => {
                        remaining_open[index] = true;
                        adaptive_report.open_cells += 1;
                    }
                    VoxelTriangleSolidClassification::Inside => {
                        unreachable!("boundary-only exact classification never emits inside cells")
                    }
                }
            }
        }
    }

    for axis in 0..3 {
        let [row_axis_a, row_axis_b] = perpendicular_axes(axis);
        for row_b in 0..cells_per_axis {
            for row_a in 0..cells_per_axis {
                let mut row_cells = Vec::new();
                for sweep_coord in 0..cells_per_axis {
                    let mut coords = [0_u64; 3];
                    coords[axis] = sweep_coord;
                    coords[row_axis_a] = row_a;
                    coords[row_axis_b] = row_b;
                    if remaining_open[cell_index(cells_per_axis, coords)?] {
                        row_cells.push(coords);
                    }
                }
                if row_cells.is_empty() {
                    adaptive_report.axis_empty_sweep_rows[axis] += 1;
                    continue;
                }
                adaptive_report.axis_sweep_rows[axis] += 1;

                let mut origin_coords = [0_u64; 3];
                origin_coords[row_axis_a] = row_a;
                origin_coords[row_axis_b] = row_b;
                let row_origin = VoxelAddress::new(frame.depth(), origin_coords)?
                    .bounds(&frame)?
                    .center();
                let row = classify_adaptive_axis_row_against_exact_triangle_solid(
                    axis,
                    &row_origin,
                    solid,
                    &mut adaptive_report,
                )?;

                match row {
                    AxisRowParity::Certified { parameters } => {
                        adaptive_report.axis_certified_sweep_rows[axis] += 1;
                        for coords in row_cells {
                            let address = VoxelAddress::new(frame.depth(), coords)?;
                            let center = address.bounds(&frame)?.center();
                            let threshold = &center[axis] - &row_origin[axis];
                            let Some(classifier) =
                                classify_axis_sweep_center(&parameters, &threshold)?
                            else {
                                adaptive_report.row_parameter_order_unknowns += 1;
                                continue;
                            };
                            let index = cell_index(cells_per_axis, coords)?;
                            classifiers[index] = classifier;
                            remaining_open[index] = false;
                            adaptive_report.sweep_classified_cells += 1;
                        }
                    }
                    AxisRowParity::Ambiguous => {
                        adaptive_report.axis_ambiguous_sweep_rows[axis] += 1;
                        adaptive_report.deferred_ambiguous_cells += row_cells.len();
                    }
                }
            }
        }
    }

    for z in 0..cells_per_axis {
        for y in 0..cells_per_axis {
            for x in 0..cells_per_axis {
                let coords = [x, y, z];
                if remaining_open[cell_index(cells_per_axis, coords)?] {
                    classify_adaptive_axis_sweep_fallback_cell(
                        coords,
                        &frame,
                        solid,
                        &mut classifiers,
                        cells_per_axis,
                        &mut adaptive_report,
                    )?;
                }
            }
        }
    }

    adaptive_report.exact_adaptive_axis_sweep_ready = adaptive_report.boundary_unknown_cells == 0
        && adaptive_report.fallback_unknown_cells == 0
        && adaptive_report.fallback_boundary_regression_cells == 0
        && adaptive_report.row_parameter_order_unknowns == 0
        && adaptive_report.classified_cells > 0;

    let (grid, report) =
        materialize_triangle_solid_classifiers(frame, policy, material, &classifiers)?;
    Ok((grid, report, adaptive_report))
}

/// Voxelize an exact closed triangle solid by multi-axis winding
/// consensus.
///
/// This is stricter than
/// [`voxelize_exact_triangle_solid_by_adaptive_axis_sweeps`]:
/// every open cell is offered certified `+X`, `+Y`, and `+Z` row-arrangement
/// votes, and a row-sweep classification is accepted only when all certified
/// votes for that cell agree. Cells with no certified vote, threshold
/// ambiguity, or contradictory exact row votes fall back to the ordinary
/// multi-direction exact parity classifier, and the report keeps those cases
/// separate.
///
/// Row arrangements retain exact predicates, and acceleration is refused when
/// combinatorial winding evidence is incomplete or inconsistent. The consensus
/// rule is report-bearing ray-crossing parity for polyhedra; no floating
/// arrangement state decides topology.
pub fn voxelize_exact_triangle_solid_by_consensus_axis_sweeps(
    frame: GridFrame,
    solid: &ExactTriangleSolid,
    material: MaterialRegionId,
    policy: VoxelizationPolicy,
) -> HypervoxelResult<(
    SparseVoxelGrid,
    VoxelizationReport,
    TriangleSolidConsensusAxisSweepVoxelizationReport,
)> {
    let cells_per_axis = frame.cells_per_axis();
    let total_cells =
        usize::try_from(cells_per_axis.pow(3)).map_err(|_| HypervoxelError::AddressOverflow)?;
    let mut classifiers = vec![VoxelTriangleSolidClassification::Unknown; total_cells];
    let mut open = vec![false; total_cells];
    let mut votes = vec![None::<VoxelTriangleSolidClassification>; total_cells];
    let mut vote_conflicts = vec![false; total_cells];
    let mut consensus_report = TriangleSolidConsensusAxisSweepVoxelizationReport {
        classified_cells: total_cells,
        ..TriangleSolidConsensusAxisSweepVoxelizationReport::default()
    };

    for z in 0..cells_per_axis {
        for y in 0..cells_per_axis {
            for x in 0..cells_per_axis {
                let address = VoxelAddress::new(frame.depth(), [x, y, z])?;
                let bounds = address.bounds(&frame)?;
                let boundary = classify_cell_boundary_against_exact_triangle_solid(&bounds, solid)?;
                let index = cell_index(cells_per_axis, [x, y, z])?;
                consensus_report.boundary_aabb_rejections += boundary.boundary_aabb_rejections;
                consensus_report.boundary_triangle_tests += boundary.boundary_triangle_tests;
                match boundary.classification {
                    VoxelTriangleSolidClassification::Boundary => {
                        classifiers[index] = VoxelTriangleSolidClassification::Boundary;
                        consensus_report.boundary_cells += 1;
                    }
                    VoxelTriangleSolidClassification::Unknown => {
                        classifiers[index] = VoxelTriangleSolidClassification::Unknown;
                        consensus_report.boundary_unknown_cells += 1;
                    }
                    VoxelTriangleSolidClassification::Outside => {
                        open[index] = true;
                        consensus_report.open_cells += 1;
                    }
                    VoxelTriangleSolidClassification::Inside => {
                        unreachable!("boundary-only exact classification never emits inside cells")
                    }
                }
            }
        }
    }

    for axis in 0..3 {
        let [row_axis_a, row_axis_b] = perpendicular_axes(axis);
        for row_b in 0..cells_per_axis {
            for row_a in 0..cells_per_axis {
                let mut row_cells = Vec::new();
                for sweep_coord in 0..cells_per_axis {
                    let mut coords = [0_u64; 3];
                    coords[axis] = sweep_coord;
                    coords[row_axis_a] = row_a;
                    coords[row_axis_b] = row_b;
                    if open[cell_index(cells_per_axis, coords)?] {
                        row_cells.push(coords);
                    }
                }
                if row_cells.is_empty() {
                    consensus_report.axis_empty_sweep_rows[axis] += 1;
                    continue;
                }
                consensus_report.axis_sweep_rows[axis] += 1;

                let mut origin_coords = [0_u64; 3];
                origin_coords[row_axis_a] = row_a;
                origin_coords[row_axis_b] = row_b;
                let row_origin = VoxelAddress::new(frame.depth(), origin_coords)?
                    .bounds(&frame)?
                    .center();
                let row = classify_consensus_axis_row_against_exact_triangle_solid(
                    axis,
                    &row_origin,
                    solid,
                    &mut consensus_report,
                )?;

                match row {
                    AxisRowParity::Certified { parameters } => {
                        consensus_report.axis_certified_sweep_rows[axis] += 1;
                        for coords in row_cells {
                            let address = VoxelAddress::new(frame.depth(), coords)?;
                            let center = address.bounds(&frame)?.center();
                            let threshold = &center[axis] - &row_origin[axis];
                            let Some(classifier) =
                                classify_axis_sweep_center(&parameters, &threshold)?
                            else {
                                consensus_report.row_parameter_order_unknowns += 1;
                                continue;
                            };
                            let index = cell_index(cells_per_axis, coords)?;
                            consensus_report.consensus_votes += 1;
                            match votes[index] {
                                Some(existing) if existing != classifier => {
                                    vote_conflicts[index] = true;
                                }
                                Some(_) => {}
                                None => votes[index] = Some(classifier),
                            }
                        }
                    }
                    AxisRowParity::Ambiguous => {
                        consensus_report.axis_ambiguous_sweep_rows[axis] += 1;
                        consensus_report.deferred_ambiguous_cells += row_cells.len();
                    }
                }
            }
        }
    }

    for z in 0..cells_per_axis {
        for y in 0..cells_per_axis {
            for x in 0..cells_per_axis {
                let coords = [x, y, z];
                let index = cell_index(cells_per_axis, coords)?;
                if !open[index] {
                    continue;
                }

                match (votes[index], vote_conflicts[index]) {
                    (Some(classifier), false) => {
                        classifiers[index] = classifier;
                        consensus_report.voted_cells += 1;
                        consensus_report.consensus_classified_cells += 1;
                    }
                    (Some(_), true) => {
                        consensus_report.voted_cells += 1;
                        consensus_report.conflicting_vote_cells += 1;
                        classify_consensus_axis_sweep_fallback_cell(
                            coords,
                            &frame,
                            solid,
                            &mut classifiers,
                            cells_per_axis,
                            &mut consensus_report,
                        )?;
                    }
                    (None, _) => {
                        consensus_report.unvoted_cells += 1;
                        classify_consensus_axis_sweep_fallback_cell(
                            coords,
                            &frame,
                            solid,
                            &mut classifiers,
                            cells_per_axis,
                            &mut consensus_report,
                        )?;
                    }
                }
            }
        }
    }

    consensus_report.exact_consensus_axis_sweep_ready = consensus_report.boundary_unknown_cells
        == 0
        && consensus_report.conflicting_vote_cells == 0
        && consensus_report.fallback_unknown_cells == 0
        && consensus_report.fallback_boundary_regression_cells == 0
        && consensus_report.row_parameter_order_unknowns == 0
        && consensus_report.classified_cells > 0;

    let (grid, report) =
        materialize_triangle_solid_classifiers(frame, policy, material, &classifiers)?;
    Ok((grid, report, consensus_report))
}

/// Voxelize an exact closed triangle solid by connected components
/// classified from multi-axis winding consensus.
///
/// This pass moves the exact row-arrangement evidence from individual cells to
/// connected open components. It first performs the same exact boundary pass
/// as the per-cell classifier, then labels 6-neighbor components of cells
/// proven disjoint from the retained triangle boundary. Each component is
/// accepted as a whole only if every non-exterior cell has at least one
/// certified axis-row vote and all certified votes throughout the component
/// agree on the same inside/outside parity. Components with missing votes,
/// contradictory votes, or parameter-order ambiguity fall back to exact
/// per-cell replay.
///
/// Components use 6-neighbor digital topology. Component-level acceleration is
/// accepted only when retained exact predicates prove that parity is constant
/// on a boundary-disjoint component.
pub fn voxelize_exact_triangle_solid_by_component_consensus(
    frame: GridFrame,
    solid: &ExactTriangleSolid,
    material: MaterialRegionId,
    policy: VoxelizationPolicy,
) -> HypervoxelResult<(
    SparseVoxelGrid,
    VoxelizationReport,
    TriangleSolidComponentConsensusVoxelizationReport,
)> {
    let cells_per_axis = frame.cells_per_axis();
    let total_cells =
        usize::try_from(cells_per_axis.pow(3)).map_err(|_| HypervoxelError::AddressOverflow)?;
    let mut classifiers = vec![VoxelTriangleSolidClassification::Unknown; total_cells];
    let mut open = vec![false; total_cells];
    let mut visited = vec![false; total_cells];
    let mut votes = vec![None::<VoxelTriangleSolidClassification>; total_cells];
    let mut vote_conflicts = vec![false; total_cells];
    let mut component_report = TriangleSolidComponentConsensusVoxelizationReport {
        classified_cells: total_cells,
        ..TriangleSolidComponentConsensusVoxelizationReport::default()
    };

    for z in 0..cells_per_axis {
        for y in 0..cells_per_axis {
            for x in 0..cells_per_axis {
                let address = VoxelAddress::new(frame.depth(), [x, y, z])?;
                let bounds = address.bounds(&frame)?;
                let boundary = classify_cell_boundary_against_exact_triangle_solid(&bounds, solid)?;
                let index = cell_index(cells_per_axis, [x, y, z])?;
                component_report.boundary_aabb_rejections += boundary.boundary_aabb_rejections;
                component_report.boundary_triangle_tests += boundary.boundary_triangle_tests;
                match boundary.classification {
                    VoxelTriangleSolidClassification::Boundary => {
                        classifiers[index] = VoxelTriangleSolidClassification::Boundary;
                        component_report.boundary_cells += 1;
                    }
                    VoxelTriangleSolidClassification::Unknown => {
                        classifiers[index] = VoxelTriangleSolidClassification::Unknown;
                        component_report.boundary_unknown_cells += 1;
                    }
                    VoxelTriangleSolidClassification::Outside => {
                        open[index] = true;
                        component_report.open_cells += 1;
                    }
                    VoxelTriangleSolidClassification::Inside => {
                        unreachable!("boundary-only exact classification never emits inside cells")
                    }
                }
            }
        }
    }

    for axis in 0..3 {
        let [row_axis_a, row_axis_b] = perpendicular_axes(axis);
        for row_b in 0..cells_per_axis {
            for row_a in 0..cells_per_axis {
                let mut row_cells = Vec::new();
                for sweep_coord in 0..cells_per_axis {
                    let mut coords = [0_u64; 3];
                    coords[axis] = sweep_coord;
                    coords[row_axis_a] = row_a;
                    coords[row_axis_b] = row_b;
                    if open[cell_index(cells_per_axis, coords)?] {
                        row_cells.push(coords);
                    }
                }
                if row_cells.is_empty() {
                    component_report.axis_empty_sweep_rows[axis] += 1;
                    continue;
                }
                component_report.axis_sweep_rows[axis] += 1;

                let mut origin_coords = [0_u64; 3];
                origin_coords[row_axis_a] = row_a;
                origin_coords[row_axis_b] = row_b;
                let row_origin = VoxelAddress::new(frame.depth(), origin_coords)?
                    .bounds(&frame)?
                    .center();
                let row = classify_component_consensus_axis_row_against_exact_triangle_solid(
                    axis,
                    &row_origin,
                    solid,
                    &mut component_report,
                )?;

                match row {
                    AxisRowParity::Certified { parameters } => {
                        component_report.axis_certified_sweep_rows[axis] += 1;
                        for coords in row_cells {
                            let address = VoxelAddress::new(frame.depth(), coords)?;
                            let center = address.bounds(&frame)?.center();
                            let threshold = &center[axis] - &row_origin[axis];
                            let Some(classifier) =
                                classify_axis_sweep_center(&parameters, &threshold)?
                            else {
                                component_report.row_parameter_order_unknowns += 1;
                                continue;
                            };
                            let index = cell_index(cells_per_axis, coords)?;
                            component_report.row_votes += 1;
                            match votes[index] {
                                Some(existing) if existing != classifier => {
                                    vote_conflicts[index] = true;
                                }
                                Some(_) => {}
                                None => votes[index] = Some(classifier),
                            }
                        }
                    }
                    AxisRowParity::Ambiguous => {
                        component_report.axis_ambiguous_sweep_rows[axis] += 1;
                        component_report.deferred_ambiguous_cells += row_cells.len();
                    }
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
                        .any(|&axis_coord| axis_coord == 0 || axis_coord + 1 == cells_per_axis);

                    for neighbor in component_neighbors(cells_per_axis, coords) {
                        let neighbor_index = cell_index(cells_per_axis, neighbor)?;
                        if open[neighbor_index] && !visited[neighbor_index] {
                            visited[neighbor_index] = true;
                            queue.push_back(neighbor);
                        }
                    }
                }

                component_report.components += 1;
                if touches_frame_boundary {
                    component_report.exterior_components += 1;
                    component_report.exterior_cells += component.len();
                    for coords in component {
                        let index = cell_index(cells_per_axis, coords)?;
                        classifiers[index] = VoxelTriangleSolidClassification::Outside;
                    }
                    continue;
                }

                let mut component_vote = None::<VoxelTriangleSolidClassification>;
                let mut component_unvoted = 0_usize;
                let mut component_conflicts = 0_usize;
                for coords in &component {
                    let index = cell_index(cells_per_axis, *coords)?;
                    if vote_conflicts[index] {
                        component_conflicts += 1;
                    }
                    match votes[index] {
                        Some(vote) => match component_vote {
                            Some(existing) if existing != vote => component_conflicts += 1,
                            Some(_) => {}
                            None => component_vote = Some(vote),
                        },
                        None => component_unvoted += 1,
                    }
                }

                if component_unvoted == 0 && component_conflicts == 0 {
                    let classifier = component_vote.expect("non-empty open component has votes");
                    component_report.consensus_components += 1;
                    component_report.consensus_cells += component.len();
                    for coords in component {
                        let index = cell_index(cells_per_axis, coords)?;
                        classifiers[index] = classifier;
                    }
                } else {
                    component_report.fallback_components += 1;
                    component_report.unvoted_component_cells += component_unvoted;
                    component_report.conflicting_component_cells += component_conflicts;
                    for coords in component {
                        classify_component_consensus_fallback_cell(
                            coords,
                            &frame,
                            solid,
                            &mut classifiers,
                            cells_per_axis,
                            &mut component_report,
                        )?;
                    }
                }
            }
        }
    }

    let fallback_replay_ready = component_report.certified_fallback_ray_attempts
        + component_report.ambiguous_fallback_ray_attempts
        == component_report.fallback_ray_attempts
        && component_report.fallback_cells
            >= component_report.fallback_unknown_cells
                + component_report.fallback_boundary_regression_cells
        && component_report.certified_fallback_ray_attempts
            == component_report.fallback_cells
                - component_report.fallback_unknown_cells
                - component_report.fallback_boundary_regression_cells;
    component_report.exact_component_consensus_ready = component_report.boundary_unknown_cells == 0
        && component_report.unvoted_component_cells == 0
        && component_report.conflicting_component_cells == 0
        && component_report.fallback_unknown_cells == 0
        && component_report.fallback_boundary_regression_cells == 0
        && fallback_replay_ready
        && component_report.row_parameter_order_unknowns == 0
        && component_report.row_plan_duplicate_memberships == 0
        && component_report.row_plan_missing_memberships == 0
        && component_report.row_plan_min_axis_violations == 0
        && component_report.classified_cells > 0;

    let (grid, report) =
        materialize_triangle_solid_classifiers(frame, policy, material, &classifiers)?;
    Ok((grid, report, component_report))
}

/// Voxelize an exact closed triangle solid by component-local winding
/// consensus.
///
/// Unlike [`voxelize_exact_triangle_solid_by_component_consensus`],
/// this scheduler labels exact open-cell components before doing row work.
/// Exterior components are accepted as outside from finite-frame connectivity,
/// and enclosed components schedule only the `+X`, `+Y`, and `+Z` rows that
/// actually contain cells from that component. This avoids treating unrelated
/// frame rows as arrangement evidence while preserving the same exact
/// acceptance rule: every cell in an enclosed component must have certified
/// row votes and all votes in that component must agree.
///
/// Components use 6-neighbor digital topology. Component-local row schedules
/// are performance evidence only; exact predicates and explicit refusal states
/// decide whether a component can be materialized.
pub fn voxelize_exact_triangle_solid_by_local_component_consensus(
    frame: GridFrame,
    solid: &ExactTriangleSolid,
    material: MaterialRegionId,
    policy: VoxelizationPolicy,
) -> HypervoxelResult<(
    SparseVoxelGrid,
    VoxelizationReport,
    TriangleSolidComponentConsensusVoxelizationReport,
)> {
    let cells_per_axis = frame.cells_per_axis();
    let total_cells =
        usize::try_from(cells_per_axis.pow(3)).map_err(|_| HypervoxelError::AddressOverflow)?;
    let mut classifiers = vec![VoxelTriangleSolidClassification::Unknown; total_cells];
    let mut open = vec![false; total_cells];
    let mut visited = vec![false; total_cells];
    let mut row_cache = ComponentAxisRowCache::default();
    let mut component_report = TriangleSolidComponentConsensusVoxelizationReport {
        classified_cells: total_cells,
        ..TriangleSolidComponentConsensusVoxelizationReport::default()
    };

    for z in 0..cells_per_axis {
        for y in 0..cells_per_axis {
            for x in 0..cells_per_axis {
                let address = VoxelAddress::new(frame.depth(), [x, y, z])?;
                let bounds = address.bounds(&frame)?;
                let boundary = classify_cell_boundary_against_exact_triangle_solid(&bounds, solid)?;
                let index = cell_index(cells_per_axis, [x, y, z])?;
                component_report.boundary_aabb_rejections += boundary.boundary_aabb_rejections;
                component_report.boundary_triangle_tests += boundary.boundary_triangle_tests;
                match boundary.classification {
                    VoxelTriangleSolidClassification::Boundary => {
                        classifiers[index] = VoxelTriangleSolidClassification::Boundary;
                        component_report.boundary_cells += 1;
                    }
                    VoxelTriangleSolidClassification::Unknown => {
                        classifiers[index] = VoxelTriangleSolidClassification::Unknown;
                        component_report.boundary_unknown_cells += 1;
                    }
                    VoxelTriangleSolidClassification::Outside => {
                        open[index] = true;
                        component_report.open_cells += 1;
                    }
                    VoxelTriangleSolidClassification::Inside => {
                        unreachable!("boundary-only exact classification never emits inside cells")
                    }
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
                        .any(|&axis_coord| axis_coord == 0 || axis_coord + 1 == cells_per_axis);

                    for neighbor in component_neighbors(cells_per_axis, coords) {
                        let neighbor_index = cell_index(cells_per_axis, neighbor)?;
                        if open[neighbor_index] && !visited[neighbor_index] {
                            visited[neighbor_index] = true;
                            queue.push_back(neighbor);
                        }
                    }
                }

                component_report.components += 1;
                if touches_frame_boundary {
                    component_report.exterior_components += 1;
                    component_report.exterior_cells += component.len();
                    for coords in component {
                        let index = cell_index(cells_per_axis, coords)?;
                        classifiers[index] = VoxelTriangleSolidClassification::Outside;
                    }
                    continue;
                }

                let mut votes = vec![None::<VoxelTriangleSolidClassification>; component.len()];
                let mut vote_conflicts = vec![false; component.len()];
                for axis in 0..3 {
                    let plan = plan_component_axis_rows(axis, &component);
                    component_report.row_plan_axes += 1;
                    component_report.row_plan_rows += plan.report.planned_rows;
                    component_report.row_plan_cell_memberships += plan.report.row_cell_memberships;
                    component_report.row_plan_duplicate_memberships +=
                        plan.report.duplicate_memberships.len();
                    component_report.row_plan_missing_memberships +=
                        plan.report.missing_memberships.len();
                    component_report.row_plan_min_axis_violations +=
                        plan.report.min_axis_coord_violations;

                    if plan.rows.is_empty() {
                        component_report.axis_empty_sweep_rows[axis] += 1;
                        continue;
                    }

                    for row_membership in plan.rows {
                        component_report.axis_sweep_rows[axis] += 1;
                        let [row_axis_a, row_axis_b] = perpendicular_axes(axis);
                        let mut origin_coords = [0_u64; 3];
                        origin_coords[row_axis_a] = row_membership.row[0];
                        origin_coords[row_axis_b] = row_membership.row[1];
                        let row_origin = VoxelAddress::new(frame.depth(), origin_coords)?
                            .bounds(&frame)?
                            .center();
                        let min_axis_coord = row_membership.min_axis_coord;
                        let mut min_axis_coords = component[row_membership.component_indices[0]];
                        min_axis_coords[axis] = min_axis_coord;
                        let min_axis_threshold = VoxelAddress::new(frame.depth(), min_axis_coords)?
                            .bounds(&frame)?
                            .center()[axis]
                            .clone()
                            - &row_origin[axis];
                        component_report.row_cache_lookups += 1;
                        let row_key = ComponentAxisRowKey::new(axis, row_membership.row);
                        let (row, cache_hit, broadened_miss) = row_cache
                            .get_or_insert_window_with(row_key, min_axis_coord, || {
                                classify_component_consensus_axis_row_with_candidate_schedule(
                                    axis,
                                    &row_origin,
                                    &min_axis_threshold,
                                    solid,
                                    &mut component_report,
                                )
                            })?;
                        if cache_hit {
                            component_report.row_cache_hits += 1;
                            match &row {
                                AxisRowParity::Certified { .. } => {
                                    component_report.row_cache_certified_hits += 1;
                                }
                                AxisRowParity::Ambiguous => {
                                    component_report.row_cache_ambiguous_hits += 1;
                                }
                            }
                        } else {
                            component_report.row_cache_misses += 1;
                            if broadened_miss {
                                component_report.row_cache_broadened_misses += 1;
                            }
                        }

                        match row {
                            AxisRowParity::Certified { parameters } => {
                                component_report.axis_certified_sweep_rows[axis] += 1;
                                for component_index in row_membership.component_indices {
                                    let coords = component[component_index];
                                    let address = VoxelAddress::new(frame.depth(), coords)?;
                                    let center = address.bounds(&frame)?.center();
                                    let threshold = &center[axis] - &row_origin[axis];
                                    let Some(classifier) =
                                        classify_axis_sweep_center(&parameters, &threshold)?
                                    else {
                                        component_report.row_parameter_order_unknowns += 1;
                                        continue;
                                    };
                                    component_report.row_votes += 1;
                                    match votes[component_index] {
                                        Some(existing) if existing != classifier => {
                                            vote_conflicts[component_index] = true;
                                        }
                                        Some(_) => {}
                                        None => votes[component_index] = Some(classifier),
                                    }
                                }
                            }
                            AxisRowParity::Ambiguous => {
                                component_report.axis_ambiguous_sweep_rows[axis] += 1;
                                component_report.deferred_ambiguous_cells +=
                                    row_membership.component_indices.len();
                            }
                        }
                    }
                }

                let mut component_vote = None::<VoxelTriangleSolidClassification>;
                let mut component_unvoted = 0_usize;
                let mut component_conflicts = 0_usize;
                for (component_index, vote) in votes.iter().enumerate() {
                    if vote_conflicts[component_index] {
                        component_conflicts += 1;
                    }
                    match vote {
                        Some(vote) => match component_vote {
                            Some(existing) if existing != *vote => component_conflicts += 1,
                            Some(_) => {}
                            None => component_vote = Some(*vote),
                        },
                        None => component_unvoted += 1,
                    }
                }

                if component_unvoted == 0 && component_conflicts == 0 {
                    let classifier = component_vote.expect("non-empty open component has votes");
                    component_report.consensus_components += 1;
                    component_report.consensus_cells += component.len();
                    for coords in component {
                        let index = cell_index(cells_per_axis, coords)?;
                        classifiers[index] = classifier;
                    }
                } else {
                    component_report.fallback_components += 1;
                    component_report.unvoted_component_cells += component_unvoted;
                    component_report.conflicting_component_cells += component_conflicts;
                    for coords in component {
                        classify_component_consensus_fallback_cell(
                            coords,
                            &frame,
                            solid,
                            &mut classifiers,
                            cells_per_axis,
                            &mut component_report,
                        )?;
                    }
                }
            }
        }
    }

    let fallback_replay_ready = component_report.certified_fallback_ray_attempts
        + component_report.ambiguous_fallback_ray_attempts
        == component_report.fallback_ray_attempts
        && component_report.fallback_cells
            >= component_report.fallback_unknown_cells
                + component_report.fallback_boundary_regression_cells
        && component_report.certified_fallback_ray_attempts
            == component_report.fallback_cells
                - component_report.fallback_unknown_cells
                - component_report.fallback_boundary_regression_cells;
    component_report.exact_component_consensus_ready = component_report.boundary_unknown_cells == 0
        && component_report.unvoted_component_cells == 0
        && component_report.conflicting_component_cells == 0
        && component_report.fallback_unknown_cells == 0
        && component_report.fallback_boundary_regression_cells == 0
        && fallback_replay_ready
        && component_report.row_parameter_order_unknowns == 0
        && component_report.row_plan_duplicate_memberships == 0
        && component_report.row_plan_missing_memberships == 0
        && component_report.row_plan_min_axis_violations == 0
        && component_report.classified_cells > 0;

    let (grid, report) =
        materialize_triangle_solid_classifiers(frame, policy, material, &classifiers)?;
    Ok((grid, report, component_report))
}

/// Voxelize an exact closed triangle solid by adaptive component-local
/// winding consensus.
///
/// This is the adaptive counterpart to
/// [`voxelize_exact_triangle_solid_by_local_component_consensus`].
/// It labels exact open-cell components first, accepts exterior components
/// from finite-frame connectivity, then tries `+X`, `+Y`, and `+Z` row
/// arrangements for each enclosed component in order. After each axis, the
/// component is accepted immediately if every cell has certified row evidence
/// and all component votes agree; later axes are skipped because they would be
/// redundant schedule evidence. If no axis prefix proves the component, the
/// component falls back to exact per-cell replay.
///
/// Early acceptance is allowed only after exact predicates prove the component
/// invariant. Skipped rows are avoided work, never implicit topology evidence.
/// Components use the same 6-neighbor topology as the other schedulers.
pub fn voxelize_exact_triangle_solid_by_adaptive_local_component_consensus(
    frame: GridFrame,
    solid: &ExactTriangleSolid,
    material: MaterialRegionId,
    policy: VoxelizationPolicy,
) -> HypervoxelResult<(
    SparseVoxelGrid,
    VoxelizationReport,
    TriangleSolidComponentConsensusVoxelizationReport,
)> {
    let cells_per_axis = frame.cells_per_axis();
    let total_cells =
        usize::try_from(cells_per_axis.pow(3)).map_err(|_| HypervoxelError::AddressOverflow)?;
    let mut classifiers = vec![VoxelTriangleSolidClassification::Unknown; total_cells];
    let mut open = vec![false; total_cells];
    let mut visited = vec![false; total_cells];
    let mut row_cache = ComponentAxisRowCache::default();
    let mut component_report = TriangleSolidComponentConsensusVoxelizationReport {
        classified_cells: total_cells,
        ..TriangleSolidComponentConsensusVoxelizationReport::default()
    };

    for z in 0..cells_per_axis {
        for y in 0..cells_per_axis {
            for x in 0..cells_per_axis {
                let address = VoxelAddress::new(frame.depth(), [x, y, z])?;
                let bounds = address.bounds(&frame)?;
                let boundary = classify_cell_boundary_against_exact_triangle_solid(&bounds, solid)?;
                let index = cell_index(cells_per_axis, [x, y, z])?;
                component_report.boundary_aabb_rejections += boundary.boundary_aabb_rejections;
                component_report.boundary_triangle_tests += boundary.boundary_triangle_tests;
                match boundary.classification {
                    VoxelTriangleSolidClassification::Boundary => {
                        classifiers[index] = VoxelTriangleSolidClassification::Boundary;
                        component_report.boundary_cells += 1;
                    }
                    VoxelTriangleSolidClassification::Unknown => {
                        classifiers[index] = VoxelTriangleSolidClassification::Unknown;
                        component_report.boundary_unknown_cells += 1;
                    }
                    VoxelTriangleSolidClassification::Outside => {
                        open[index] = true;
                        component_report.open_cells += 1;
                    }
                    VoxelTriangleSolidClassification::Inside => {
                        unreachable!("boundary-only exact classification never emits inside cells")
                    }
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
                        .any(|&axis_coord| axis_coord == 0 || axis_coord + 1 == cells_per_axis);

                    for neighbor in component_neighbors(cells_per_axis, coords) {
                        let neighbor_index = cell_index(cells_per_axis, neighbor)?;
                        if open[neighbor_index] && !visited[neighbor_index] {
                            visited[neighbor_index] = true;
                            queue.push_back(neighbor);
                        }
                    }
                }

                component_report.components += 1;
                if touches_frame_boundary {
                    component_report.exterior_components += 1;
                    component_report.exterior_cells += component.len();
                    for coords in component {
                        let index = cell_index(cells_per_axis, coords)?;
                        classifiers[index] = VoxelTriangleSolidClassification::Outside;
                    }
                    continue;
                }

                let mut votes = vec![None::<VoxelTriangleSolidClassification>; component.len()];
                let mut vote_conflicts = vec![false; component.len()];
                let mut accepted = None::<VoxelTriangleSolidClassification>;
                let mut accepted_by_retry = false;
                let mut retry_attempted = false;
                let mut final_unvoted = component.len();
                let mut final_conflicts = 0_usize;

                for axis in 0..3 {
                    let plan = plan_component_axis_rows(axis, &component);
                    component_report.row_plan_axes += 1;
                    component_report.row_plan_rows += plan.report.planned_rows;
                    component_report.row_plan_cell_memberships += plan.report.row_cell_memberships;
                    component_report.row_plan_duplicate_memberships +=
                        plan.report.duplicate_memberships.len();
                    component_report.row_plan_missing_memberships +=
                        plan.report.missing_memberships.len();
                    component_report.row_plan_min_axis_violations +=
                        plan.report.min_axis_coord_violations;

                    if plan.rows.is_empty() {
                        component_report.axis_empty_sweep_rows[axis] += 1;
                        continue;
                    }

                    for row_membership in plan.rows {
                        component_report.axis_sweep_rows[axis] += 1;
                        let [row_axis_a, row_axis_b] = perpendicular_axes(axis);
                        let mut origin_coords = [0_u64; 3];
                        origin_coords[row_axis_a] = row_membership.row[0];
                        origin_coords[row_axis_b] = row_membership.row[1];
                        let row_origin = VoxelAddress::new(frame.depth(), origin_coords)?
                            .bounds(&frame)?
                            .center();
                        let min_axis_coord = row_membership.min_axis_coord;
                        let mut min_axis_coords = component[row_membership.component_indices[0]];
                        min_axis_coords[axis] = min_axis_coord;
                        let min_axis_threshold = VoxelAddress::new(frame.depth(), min_axis_coords)?
                            .bounds(&frame)?
                            .center()[axis]
                            .clone()
                            - &row_origin[axis];
                        component_report.row_cache_lookups += 1;
                        let row_key = ComponentAxisRowKey::new(axis, row_membership.row);
                        let (row, cache_hit, broadened_miss) = row_cache
                            .get_or_insert_window_with(row_key, min_axis_coord, || {
                                classify_component_consensus_axis_row_with_candidate_schedule(
                                    axis,
                                    &row_origin,
                                    &min_axis_threshold,
                                    solid,
                                    &mut component_report,
                                )
                            })?;
                        if cache_hit {
                            component_report.row_cache_hits += 1;
                            match &row {
                                AxisRowParity::Certified { .. } => {
                                    component_report.row_cache_certified_hits += 1;
                                }
                                AxisRowParity::Ambiguous => {
                                    component_report.row_cache_ambiguous_hits += 1;
                                }
                            }
                        } else {
                            component_report.row_cache_misses += 1;
                            if broadened_miss {
                                component_report.row_cache_broadened_misses += 1;
                            }
                        }

                        match row {
                            AxisRowParity::Certified { parameters } => {
                                component_report.axis_certified_sweep_rows[axis] += 1;
                                for component_index in row_membership.component_indices {
                                    let coords = component[component_index];
                                    let address = VoxelAddress::new(frame.depth(), coords)?;
                                    let center = address.bounds(&frame)?.center();
                                    let threshold = &center[axis] - &row_origin[axis];
                                    let Some(classifier) =
                                        classify_axis_sweep_center(&parameters, &threshold)?
                                    else {
                                        component_report.row_parameter_order_unknowns += 1;
                                        continue;
                                    };
                                    component_report.row_votes += 1;
                                    match votes[component_index] {
                                        Some(existing) if existing != classifier => {
                                            vote_conflicts[component_index] = true;
                                        }
                                        Some(_) => {}
                                        None => votes[component_index] = Some(classifier),
                                    }
                                }
                            }
                            AxisRowParity::Ambiguous => {
                                component_report.axis_ambiguous_sweep_rows[axis] += 1;
                                component_report.deferred_ambiguous_cells +=
                                    row_membership.component_indices.len();
                            }
                        }
                    }

                    let mut component_vote = None::<VoxelTriangleSolidClassification>;
                    let mut component_unvoted = 0_usize;
                    let mut component_conflicts = 0_usize;
                    for (component_index, vote) in votes.iter().enumerate() {
                        if vote_conflicts[component_index] {
                            component_conflicts += 1;
                        }
                        match vote {
                            Some(vote) => match component_vote {
                                Some(existing) if existing != *vote => component_conflicts += 1,
                                Some(_) => {}
                                None => component_vote = Some(*vote),
                            },
                            None => component_unvoted += 1,
                        }
                    }
                    final_unvoted = component_unvoted;
                    final_conflicts = component_conflicts;
                    if component_unvoted == 0 && component_conflicts == 0 {
                        accepted = component_vote;
                        break;
                    }
                    if !retry_attempted {
                        retry_attempted = true;
                        accepted = classify_component_by_retry_ray_consensus(
                            &component,
                            &frame,
                            solid,
                            &mut component_report,
                        )?;
                        if accepted.is_some() {
                            accepted_by_retry = true;
                            final_unvoted = 0;
                            final_conflicts = 0;
                            break;
                        }
                    }
                }

                if let Some(classifier) = accepted {
                    component_report.consensus_components += 1;
                    if accepted_by_retry {
                        component_report.retry_consensus_components += 1;
                        component_report.retry_consensus_cells += component.len();
                    } else {
                        component_report.consensus_cells += component.len();
                    }
                    for coords in component {
                        let index = cell_index(cells_per_axis, coords)?;
                        classifiers[index] = classifier;
                    }
                } else {
                    component_report.fallback_components += 1;
                    component_report.unvoted_component_cells += final_unvoted;
                    component_report.conflicting_component_cells += final_conflicts;
                    for coords in component {
                        classify_component_consensus_fallback_cell(
                            coords,
                            &frame,
                            solid,
                            &mut classifiers,
                            cells_per_axis,
                            &mut component_report,
                        )?;
                    }
                }
            }
        }
    }

    component_report.exact_component_consensus_ready = component_report.boundary_unknown_cells == 0
        && component_report.unvoted_component_cells == 0
        && component_report.conflicting_component_cells == 0
        && component_report.fallback_unknown_cells == 0
        && component_report.fallback_boundary_regression_cells == 0
        && component_report.row_parameter_order_unknowns == 0
        && component_report.row_plan_duplicate_memberships == 0
        && component_report.row_plan_missing_memberships == 0
        && component_report.row_plan_min_axis_violations == 0
        && component_report.classified_cells > 0;

    let (grid, report) =
        materialize_triangle_solid_classifiers(frame, policy, material, &classifiers)?;
    Ok((grid, report, component_report))
}

/// Aggregate exact schedule evidence over a voxelization pass.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct TriangleSolidVoxelizationReport {
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

/// Aggregate connected-component schedule evidence over a solid
/// triangle-solid voxelization pass.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct TriangleSolidComponentVoxelizationReport {
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
}

/// Aggregate exact row-sweep evidence over a triangle-solid pass.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct TriangleSolidAxisSweepVoxelizationReport {
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

/// Aggregate exact adaptive multi-axis row-sweep evidence.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct TriangleSolidAdaptiveAxisSweepVoxelizationReport {
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
    /// Per-axis row counts for `+X`, `+Y`, and `+Z` sweeps that still had at
    /// least one unclassified open cell when the axis was attempted.
    pub axis_sweep_rows: [usize; 3],
    /// Per-axis rows with no remaining open cells.
    pub axis_empty_sweep_rows: [usize; 3],
    /// Per-axis rows accepted by certified exact crossing sequences.
    pub axis_certified_sweep_rows: [usize; 3],
    /// Per-axis rows rejected because they hit an ambiguous arrangement event.
    pub axis_ambiguous_sweep_rows: [usize; 3],
    /// Open cells classified directly by a certified axis row.
    pub sweep_classified_cells: usize,
    /// Open-cell row memberships deferred by ambiguous rows. A cell may be
    /// counted more than once if multiple axes could not certify its row.
    pub deferred_ambiguous_cells: usize,
    /// Open cells classified by the per-cell fallback path after all axes.
    pub fallback_cells: usize,
    /// Fallback cells that remained unknown.
    pub fallback_unknown_cells: usize,
    /// Fallback cells that unexpectedly reclassified as boundary after the
    /// boundary pass marked them open.
    pub fallback_boundary_regression_cells: usize,
    /// Exact ray/AABB broad-phase rejections consumed by adaptive row sweeps.
    pub row_ray_aabb_rejections: usize,
    /// Exact ray/triangle predicates consumed by adaptive row sweeps.
    pub row_ray_triangle_tests: usize,
    /// Proper row-ray/triangle intersections before unique-parameter collapse.
    pub row_proper_intersections: usize,
    /// Sum of unique exact crossing parameters retained by certified rows.
    pub row_unique_parameters: usize,
    /// Boundary-touch events that made adaptive row sweeps ambiguous.
    pub row_boundary_touches: usize,
    /// Coplanar events that made adaptive row sweeps ambiguous.
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
    /// Whether the adaptive multi-axis sweep produced exact arrangement
    /// evidence for all cells.
    pub exact_adaptive_axis_sweep_ready: bool,
}

/// Aggregate exact multi-axis winding-consensus evidence.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct TriangleSolidConsensusAxisSweepVoxelizationReport {
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
    /// Per-axis row counts for `+X`, `+Y`, and `+Z` consensus sweeps.
    pub axis_sweep_rows: [usize; 3],
    /// Per-axis rows with no open cells.
    pub axis_empty_sweep_rows: [usize; 3],
    /// Per-axis rows accepted by certified exact crossing sequences.
    pub axis_certified_sweep_rows: [usize; 3],
    /// Per-axis rows rejected because they hit an ambiguous arrangement event.
    pub axis_ambiguous_sweep_rows: [usize; 3],
    /// Open cells receiving at least one certified axis-row vote.
    pub voted_cells: usize,
    /// Open-cell votes cast by certified axis rows.
    pub consensus_votes: usize,
    /// Open cells accepted directly because all certified axis-row votes agreed.
    pub consensus_classified_cells: usize,
    /// Open cells with no certified axis-row vote.
    pub unvoted_cells: usize,
    /// Open cells whose certified axis-row votes disagreed.
    pub conflicting_vote_cells: usize,
    /// Open-cell row memberships deferred by ambiguous rows. A cell may be
    /// counted more than once if multiple axes could not certify its row.
    pub deferred_ambiguous_cells: usize,
    /// Open cells classified by the per-cell fallback path after consensus.
    pub fallback_cells: usize,
    /// Fallback cells that remained unknown.
    pub fallback_unknown_cells: usize,
    /// Fallback cells that unexpectedly reclassified as boundary after the
    /// boundary pass marked them open.
    pub fallback_boundary_regression_cells: usize,
    /// Exact ray/AABB broad-phase rejections consumed by consensus row sweeps.
    pub row_ray_aabb_rejections: usize,
    /// Exact ray/triangle predicates consumed by consensus row sweeps.
    pub row_ray_triangle_tests: usize,
    /// Proper row-ray/triangle intersections before unique-parameter collapse.
    pub row_proper_intersections: usize,
    /// Sum of unique exact crossing parameters retained by certified rows.
    pub row_unique_parameters: usize,
    /// Boundary-touch events that made consensus row sweeps ambiguous.
    pub row_boundary_touches: usize,
    /// Coplanar events that made consensus row sweeps ambiguous.
    pub row_coplanar_events: usize,
    /// Certified row parameters that coincided with, or could not be ordered
    /// against, a cell center threshold.
    pub row_parameter_order_unknowns: usize,
    /// Exact ray-parity attempts consumed by fallback cells.
    pub fallback_ray_attempts: usize,
    /// Exact ray/AABB broad-phase rejections consumed by fallback cells.
    pub fallback_ray_aabb_rejections: usize,
    /// Exact ray/triangle predicates consumed by fallback cells.
    pub fallback_ray_triangle_tests: usize,
    /// Ambiguous ray attempts seen during fallback classification.
    pub ambiguous_fallback_ray_attempts: usize,
    /// Whether the multi-axis winding consensus produced exact arrangement
    /// evidence for all cells without relying on contradictory row votes.
    pub exact_consensus_axis_sweep_ready: bool,
}

/// Aggregate exact component-level winding-consensus evidence.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct TriangleSolidComponentConsensusVoxelizationReport {
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
    /// Components accepted from a single exact winding consensus.
    pub consensus_components: usize,
    /// Open cells materialized through component-level consensus.
    pub consensus_cells: usize,
    /// Components touching the finite frame boundary and therefore accepted as
    /// exterior without ray parity.
    pub exterior_components: usize,
    /// Open cells in exterior components.
    pub exterior_cells: usize,
    /// Components that fell back to per-cell exact replay.
    pub fallback_components: usize,
    /// Open cells classified by fallback exact replay.
    pub fallback_cells: usize,
    /// Components accepted by deterministic exact ray-retry consensus after
    /// row consensus could not prove the component.
    pub retry_consensus_components: usize,
    /// Open cells materialized by retry consensus.
    pub retry_consensus_cells: usize,
    /// Deterministic retry directions attempted for enclosed components.
    pub retry_direction_attempts: usize,
    /// Retry directions that accepted one component with certified consensus.
    pub retry_successful_direction_attempts: usize,
    /// Retry directions that failed because of unknown, conflicting, or absent
    /// certified component evidence.
    pub retry_failed_direction_attempts: usize,
    /// Component-cell memberships scheduled by attempted retry directions.
    ///
    /// This is retained schedule evidence for the deterministic retry witness:
    /// each attempted direction must test every cell in the enclosed component.
    /// This keeps the accelerated combinatorial decision replayable instead of
    /// trusting aggregate predicate counts.
    pub retry_direction_component_cells: usize,
    /// Component-cell memberships scheduled by successful retry directions.
    pub retry_successful_direction_cells: usize,
    /// Component-cell memberships scheduled by failed retry directions.
    pub retry_failed_direction_cells: usize,
    /// Per-cell exact ray attempts consumed by retry consensus probes.
    pub retry_ray_attempts: usize,
    /// Retry-probed cells whose ray produced certified non-unknown evidence.
    pub retry_certified_cells: usize,
    /// Cells in accepted retry-consensus directions.
    pub retry_successful_cells: usize,
    /// Exact ray/AABB broad-phase rejections consumed by retry probes.
    pub retry_ray_aabb_rejections: usize,
    /// Exact ray/triangle predicates consumed by retry probes.
    pub retry_ray_triangle_tests: usize,
    /// Retry-probed cells whose ray remained unknown.
    pub retry_unknown_cells: usize,
    /// Retry-probed cells whose certified classifier disagreed with another
    /// cell in the same component for that direction.
    pub retry_conflicting_cells: usize,
    /// Component cells that had no certified row vote.
    pub unvoted_component_cells: usize,
    /// Component cells whose certified row votes disagreed.
    pub conflicting_component_cells: usize,
    /// Component row memberships deferred because a row had edge, vertex, or
    /// coplanar ambiguity. A cell may contribute more than once.
    pub deferred_ambiguous_cells: usize,
    /// Fallback cells that remained unknown.
    pub fallback_unknown_cells: usize,
    /// Fallback cells that unexpectedly reclassified as boundary after the
    /// boundary pass marked them open.
    pub fallback_boundary_regression_cells: usize,
    /// Per-axis row counts for `+X`, `+Y`, and `+Z` component consensus sweeps.
    pub axis_sweep_rows: [usize; 3],
    /// Per-axis rows with no open cells.
    pub axis_empty_sweep_rows: [usize; 3],
    /// Component-axis row plans retained as exact schedule evidence.
    pub row_plan_axes: usize,
    /// Component-local rows emitted by retained row plans.
    pub row_plan_rows: usize,
    /// Component cell-to-row memberships emitted by retained row plans.
    pub row_plan_cell_memberships: usize,
    /// Duplicate component cell memberships found while planning rows.
    pub row_plan_duplicate_memberships: usize,
    /// Missing component cell memberships found while planning rows.
    pub row_plan_missing_memberships: usize,
    /// Rows whose retained minimum sweep coordinate failed exact replay.
    pub row_plan_min_axis_violations: usize,
    /// Per-axis rows accepted by certified exact crossing sequences.
    pub axis_certified_sweep_rows: [usize; 3],
    /// Per-axis rows rejected because they hit an ambiguous arrangement event.
    pub axis_ambiguous_sweep_rows: [usize; 3],
    /// Exact row votes cast for open cells.
    pub row_votes: usize,
    /// Component-local row certificate lookups by exact integer row key.
    pub row_cache_lookups: usize,
    /// Component-local row certificate lookups satisfied by retained row
    /// evidence from an earlier component in the same voxelization pass.
    pub row_cache_hits: usize,
    /// Cache hits that replayed certified exact crossing-sequence evidence.
    ///
    /// This counter proves that a cache hit reused a certified row rather than
    /// an untyped shortcut.
    pub row_cache_certified_hits: usize,
    /// Cache hits that replayed retained ambiguous arrangement evidence.
    ///
    /// Ambiguous rows are still exact evidence: they prove that this accelerated
    /// path refused to cast a row vote because the row hit an edge, vertex, or
    /// coplanar event. Keeping them separate from certified hits prevents cache
    /// replay from hiding exact refusals behind aggregate hit counts.
    pub row_cache_ambiguous_hits: usize,
    /// Component-local row certificate lookups that had to run the exact row
    /// scheduler and then retain the result.
    pub row_cache_misses: usize,
    /// Cache misses caused by a retained row certificate whose lower row
    /// window started after the current row segment, requiring a broader exact
    /// replay before it could be reused.
    pub row_cache_broadened_misses: usize,
    /// Component-local rows whose triangle candidate set was built by exact
    /// ray/AABB slab replay before narrow ray/triangle predicates.
    pub row_candidate_scheduled_rows: usize,
    /// Component-local rows scheduled with an exact lower-bound window at the
    /// first component cell center on that row.
    pub row_window_scheduled_rows: usize,
    /// Triangle candidates admitted to those exact row schedules.
    pub row_candidate_triangles: usize,
    /// Triangles rejected from row schedules by exact ray/AABB slab replay.
    pub row_candidate_aabb_rejections: usize,
    /// Triangle AABBs rejected specifically because their exact row interval
    /// exits before the first component cell center on the row.
    pub row_window_aabb_rejections: usize,
    /// Exact ray/AABB broad-phase rejections consumed by component-consensus
    /// row sweeps.
    pub row_ray_aabb_rejections: usize,
    /// Exact ray/triangle predicates consumed by component-consensus row
    /// sweeps.
    pub row_ray_triangle_tests: usize,
    /// Proper row-ray/triangle intersections before unique-parameter collapse.
    pub row_proper_intersections: usize,
    /// Sum of unique exact crossing parameters retained by certified rows.
    pub row_unique_parameters: usize,
    /// Boundary-touch events that made component-consensus rows ambiguous.
    pub row_boundary_touches: usize,
    /// Coplanar events that made component-consensus rows ambiguous.
    pub row_coplanar_events: usize,
    /// Certified row parameters that coincided with, or could not be ordered
    /// against, a cell center threshold.
    pub row_parameter_order_unknowns: usize,
    /// Exact ray-parity attempts consumed by fallback cells.
    pub fallback_ray_attempts: usize,
    /// Exact ray/AABB broad-phase rejections consumed by fallback cells.
    pub fallback_ray_aabb_rejections: usize,
    /// Exact ray/triangle predicates consumed by fallback cells.
    pub fallback_ray_triangle_tests: usize,
    /// Ambiguous ray attempts seen during fallback classification.
    pub ambiguous_fallback_ray_attempts: usize,
    /// Certified ray attempts seen during fallback classification.
    ///
    /// Fallback is an exact repair path, not an exemption from arrangement
    /// evidence. This counter records which fallback cells ended with certified
    /// parity rays.
    pub certified_fallback_ray_attempts: usize,
    /// Whether component-level winding consensus produced exact arrangement
    /// evidence for all non-boundary cells without fallback blockers.
    pub exact_component_consensus_ready: bool,
}

impl TriangleSolidVoxelizationReport {
    fn accumulate(&mut self, cell: &TriangleSolidCellReport) {
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

#[derive(Clone, Debug, PartialEq)]
pub(crate) enum AxisRowParity {
    Certified { parameters: Vec<Real> },
    Ambiguous,
}

fn classify_axis_row_against_exact_triangle_solid(
    axis: usize,
    row_origin: &[Real; 3],
    solid: &ExactTriangleSolid,
    sweep_report: &mut TriangleSolidAxisSweepVoxelizationReport,
) -> HypervoxelResult<AxisRowParity> {
    let origin = point3(row_origin);
    let direction = axis_direction(axis);
    let direction_components = point_components(&direction);
    let mut parameters = Vec::<Real>::new();

    for triangle in &solid.triangles {
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
            hyperlimit::PredicatePolicy::STRICT,
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

fn classify_adaptive_axis_row_against_exact_triangle_solid(
    axis: usize,
    row_origin: &[Real; 3],
    solid: &ExactTriangleSolid,
    adaptive_report: &mut TriangleSolidAdaptiveAxisSweepVoxelizationReport,
) -> HypervoxelResult<AxisRowParity> {
    let origin = point3(row_origin);
    let direction = axis_direction(axis);
    let direction_components = point_components(&direction);
    let mut parameters = Vec::<Real>::new();

    for triangle in &solid.triangles {
        match classify_ray_aabb_intersection(row_origin, &direction_components, &triangle.bounds)? {
            RayAabbIntersection::Disjoint => {
                adaptive_report.row_ray_aabb_rejections += 1;
                continue;
            }
            RayAabbIntersection::Intersects => {}
        }

        adaptive_report.row_ray_triangle_tests += 1;
        let report = classify_ray_triangle3_intersection_report(
            &origin,
            &direction,
            &triangle.points[0],
            &triangle.points[1],
            &triangle.points[2],
            hyperlimit::PredicatePolicy::STRICT,
        )
        .value()
        .ok_or(HypervoxelError::UnknownScalarOrdering {
            field: "adaptive-axis-sweep-triangle-solid-ray",
        })?;
        match report.relation {
            RayTriangleIntersection::Disjoint => {}
            RayTriangleIntersection::Proper => {
                let Some(parameter) = report.parameter else {
                    return Ok(AxisRowParity::Ambiguous);
                };
                adaptive_report.row_proper_intersections += 1;
                insert_unique_parameter(&mut parameters, parameter)?;
            }
            RayTriangleIntersection::BoundaryTouch => {
                adaptive_report.row_boundary_touches += 1;
                return Ok(AxisRowParity::Ambiguous);
            }
            RayTriangleIntersection::Coplanar => {
                adaptive_report.row_coplanar_events += 1;
                return Ok(AxisRowParity::Ambiguous);
            }
        }
    }

    adaptive_report.row_unique_parameters += parameters.len();
    Ok(AxisRowParity::Certified { parameters })
}

fn classify_consensus_axis_row_against_exact_triangle_solid(
    axis: usize,
    row_origin: &[Real; 3],
    solid: &ExactTriangleSolid,
    consensus_report: &mut TriangleSolidConsensusAxisSweepVoxelizationReport,
) -> HypervoxelResult<AxisRowParity> {
    let origin = point3(row_origin);
    let direction = axis_direction(axis);
    let direction_components = point_components(&direction);
    let mut parameters = Vec::<Real>::new();

    for triangle in &solid.triangles {
        match classify_ray_aabb_intersection(row_origin, &direction_components, &triangle.bounds)? {
            RayAabbIntersection::Disjoint => {
                consensus_report.row_ray_aabb_rejections += 1;
                continue;
            }
            RayAabbIntersection::Intersects => {}
        }

        consensus_report.row_ray_triangle_tests += 1;
        let report = classify_ray_triangle3_intersection_report(
            &origin,
            &direction,
            &triangle.points[0],
            &triangle.points[1],
            &triangle.points[2],
            hyperlimit::PredicatePolicy::STRICT,
        )
        .value()
        .ok_or(HypervoxelError::UnknownScalarOrdering {
            field: "consensus-axis-sweep-triangle-solid-ray",
        })?;
        match report.relation {
            RayTriangleIntersection::Disjoint => {}
            RayTriangleIntersection::Proper => {
                let Some(parameter) = report.parameter else {
                    return Ok(AxisRowParity::Ambiguous);
                };
                consensus_report.row_proper_intersections += 1;
                insert_unique_parameter(&mut parameters, parameter)?;
            }
            RayTriangleIntersection::BoundaryTouch => {
                consensus_report.row_boundary_touches += 1;
                return Ok(AxisRowParity::Ambiguous);
            }
            RayTriangleIntersection::Coplanar => {
                consensus_report.row_coplanar_events += 1;
                return Ok(AxisRowParity::Ambiguous);
            }
        }
    }

    consensus_report.row_unique_parameters += parameters.len();
    Ok(AxisRowParity::Certified { parameters })
}

fn classify_component_consensus_axis_row_against_exact_triangle_solid(
    axis: usize,
    row_origin: &[Real; 3],
    solid: &ExactTriangleSolid,
    component_report: &mut TriangleSolidComponentConsensusVoxelizationReport,
) -> HypervoxelResult<AxisRowParity> {
    let origin = point3(row_origin);
    let direction = axis_direction(axis);
    let direction_components = point_components(&direction);
    let mut parameters = Vec::<Real>::new();

    for triangle in &solid.triangles {
        match classify_ray_aabb_intersection(row_origin, &direction_components, &triangle.bounds)? {
            RayAabbIntersection::Disjoint => {
                component_report.row_ray_aabb_rejections += 1;
                continue;
            }
            RayAabbIntersection::Intersects => {}
        }

        component_report.row_ray_triangle_tests += 1;
        let report = classify_ray_triangle3_intersection_report(
            &origin,
            &direction,
            &triangle.points[0],
            &triangle.points[1],
            &triangle.points[2],
            hyperlimit::PredicatePolicy::STRICT,
        )
        .value()
        .ok_or(HypervoxelError::UnknownScalarOrdering {
            field: "component-consensus-triangle-solid-ray",
        })?;
        match report.relation {
            RayTriangleIntersection::Disjoint => {}
            RayTriangleIntersection::Proper => {
                let Some(parameter) = report.parameter else {
                    return Ok(AxisRowParity::Ambiguous);
                };
                component_report.row_proper_intersections += 1;
                insert_unique_parameter(&mut parameters, parameter)?;
            }
            RayTriangleIntersection::BoundaryTouch => {
                component_report.row_boundary_touches += 1;
                return Ok(AxisRowParity::Ambiguous);
            }
            RayTriangleIntersection::Coplanar => {
                component_report.row_coplanar_events += 1;
                return Ok(AxisRowParity::Ambiguous);
            }
        }
    }

    component_report.row_unique_parameters += parameters.len();
    Ok(AxisRowParity::Certified { parameters })
}

fn classify_component_consensus_axis_row_with_candidate_schedule(
    axis: usize,
    row_origin: &[Real; 3],
    min_axis_threshold: &Real,
    solid: &ExactTriangleSolid,
    component_report: &mut TriangleSolidComponentConsensusVoxelizationReport,
) -> HypervoxelResult<AxisRowParity> {
    let origin = point3(row_origin);
    let direction = axis_direction(axis);
    let direction_components = point_components(&direction);
    let mut candidates = Vec::new();

    component_report.row_candidate_scheduled_rows += 1;
    component_report.row_window_scheduled_rows += 1;
    for (triangle_index, triangle) in solid.triangles.iter().enumerate() {
        match classify_ray_aabb_intersection_from_lower(
            row_origin,
            &direction_components,
            &triangle.bounds,
            min_axis_threshold,
        )? {
            RayAabbWindowIntersection::Disjoint => {
                component_report.row_candidate_aabb_rejections += 1;
                component_report.row_ray_aabb_rejections += 1;
            }
            RayAabbWindowIntersection::BeforeLower => {
                component_report.row_candidate_aabb_rejections += 1;
                component_report.row_ray_aabb_rejections += 1;
                component_report.row_window_aabb_rejections += 1;
            }
            RayAabbWindowIntersection::Intersects => candidates.push(triangle_index),
        }
    }
    component_report.row_candidate_triangles += candidates.len();

    let mut parameters = Vec::<Real>::new();
    for triangle_index in candidates {
        let triangle = &solid.triangles[triangle_index];
        component_report.row_ray_triangle_tests += 1;
        let report = classify_ray_triangle3_intersection_report(
            &origin,
            &direction,
            &triangle.points[0],
            &triangle.points[1],
            &triangle.points[2],
            hyperlimit::PredicatePolicy::STRICT,
        )
        .value()
        .ok_or(HypervoxelError::UnknownScalarOrdering {
            field: "component-consensus-candidate-triangle-solid-ray",
        })?;
        match report.relation {
            RayTriangleIntersection::Disjoint => {}
            RayTriangleIntersection::Proper => {
                let Some(parameter) = report.parameter else {
                    return Ok(AxisRowParity::Ambiguous);
                };
                component_report.row_proper_intersections += 1;
                insert_unique_parameter(&mut parameters, parameter)?;
            }
            RayTriangleIntersection::BoundaryTouch => {
                component_report.row_boundary_touches += 1;
                return Ok(AxisRowParity::Ambiguous);
            }
            RayTriangleIntersection::Coplanar => {
                component_report.row_coplanar_events += 1;
                return Ok(AxisRowParity::Ambiguous);
            }
        }
    }

    component_report.row_unique_parameters += parameters.len();
    Ok(AxisRowParity::Certified { parameters })
}

/// Try to prove a boundary-disjoint component with deterministic exact retry
/// rays before falling back to the full per-cell classifier.
///
/// Each retry direction is a component-wide parity witness: every cell center
/// must produce a certified ray/triangle classification and all certified
/// cells must agree. Failed or conflicting rays become refusal evidence; only
/// a complete component witness can replace slower fallback.
fn classify_component_by_retry_ray_consensus(
    component: &[[u64; 3]],
    frame: &GridFrame,
    solid: &ExactTriangleSolid,
    component_report: &mut TriangleSolidComponentConsensusVoxelizationReport,
) -> HypervoxelResult<Option<VoxelTriangleSolidClassification>> {
    for (direction_index, direction) in ray_parity_directions().into_iter().enumerate() {
        component_report.retry_direction_attempts += 1;
        component_report.retry_direction_component_cells += component.len();
        let mut component_vote = None::<VoxelTriangleSolidClassification>;
        let mut direction_unknowns = 0_usize;
        let mut direction_conflicts = 0_usize;
        let mut direction_certified = 0_usize;

        for coords in component {
            let point = VoxelAddress::new(frame.depth(), *coords)?
                .bounds(frame)?
                .center();
            let (classifier, attempt) = classify_point_against_exact_triangle_solid_by_single_ray(
                &point,
                solid,
                &direction,
                direction_index,
            )?;
            component_report.retry_ray_attempts += 1;
            component_report.retry_ray_aabb_rejections += attempt.ray_aabb_rejections;
            component_report.retry_ray_triangle_tests += attempt.triangle_tests;

            if !attempt.certified || classifier == VoxelTriangleSolidClassification::Unknown {
                direction_unknowns += 1;
                continue;
            }
            direction_certified += 1;
            match component_vote {
                Some(existing) if existing != classifier => direction_conflicts += 1,
                Some(_) => {}
                None => component_vote = Some(classifier),
            }
        }

        if direction_unknowns == 0
            && direction_conflicts == 0
            && let Some(classifier) = component_vote
        {
            component_report.retry_successful_direction_attempts += 1;
            component_report.retry_successful_direction_cells += component.len();
            component_report.retry_certified_cells += direction_certified;
            component_report.retry_successful_cells += component.len();
            return Ok(Some(classifier));
        }
        component_report.retry_failed_direction_attempts += 1;
        component_report.retry_failed_direction_cells += component.len();
        component_report.retry_certified_cells += direction_certified;
        component_report.retry_unknown_cells += direction_unknowns;
        component_report.retry_conflicting_cells += direction_conflicts;
    }

    Ok(None)
}

fn classify_axis_sweep_center(
    parameters: &[Real],
    threshold: &Real,
) -> HypervoxelResult<Option<VoxelTriangleSolidClassification>> {
    let mut crossings_after_center = 0_usize;
    for parameter in parameters {
        match hyperlimit::compare_reals(parameter, threshold, hyperlimit::PredicatePolicy::STRICT)
            .value()
        {
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
    Ok(Some(if crossings_after_center.is_multiple_of(2) {
        VoxelTriangleSolidClassification::Outside
    } else {
        VoxelTriangleSolidClassification::Inside
    }))
}

fn classify_axis_sweep_fallback_cell(
    coords: [u64; 3],
    frame: &GridFrame,
    solid: &ExactTriangleSolid,
    classifiers: &mut [VoxelTriangleSolidClassification],
    cells_per_axis: u64,
    sweep_report: &mut TriangleSolidAxisSweepVoxelizationReport,
) -> HypervoxelResult<()> {
    let address = VoxelAddress::new(frame.depth(), coords)?;
    let cell = classify_cell_against_exact_triangle_solid(address, frame, solid)?;
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

    match cell.classification {
        VoxelTriangleSolidClassification::Inside | VoxelTriangleSolidClassification::Outside => {}
        VoxelTriangleSolidClassification::Unknown => sweep_report.fallback_unknown_cells += 1,
        VoxelTriangleSolidClassification::Boundary => {
            sweep_report.fallback_boundary_regression_cells += 1;
        }
    }
    let index = cell_index(cells_per_axis, coords)?;
    classifiers[index] = cell.classification;
    Ok(())
}

fn classify_adaptive_axis_sweep_fallback_cell(
    coords: [u64; 3],
    frame: &GridFrame,
    solid: &ExactTriangleSolid,
    classifiers: &mut [VoxelTriangleSolidClassification],
    cells_per_axis: u64,
    adaptive_report: &mut TriangleSolidAdaptiveAxisSweepVoxelizationReport,
) -> HypervoxelResult<()> {
    let address = VoxelAddress::new(frame.depth(), coords)?;
    let cell = classify_cell_against_exact_triangle_solid(address, frame, solid)?;
    adaptive_report.fallback_cells += 1;
    adaptive_report.fallback_ray_attempts += cell.ray_attempts.len();
    adaptive_report.fallback_ray_aabb_rejections += cell
        .ray_attempts
        .iter()
        .map(|attempt| attempt.ray_aabb_rejections)
        .sum::<usize>();
    adaptive_report.fallback_ray_triangle_tests += cell.ray_triangle_tests();
    adaptive_report.ambiguous_fallback_ray_attempts += cell
        .ray_attempts
        .iter()
        .filter(|attempt| !attempt.certified)
        .count();

    match cell.classification {
        VoxelTriangleSolidClassification::Inside | VoxelTriangleSolidClassification::Outside => {}
        VoxelTriangleSolidClassification::Unknown => adaptive_report.fallback_unknown_cells += 1,
        VoxelTriangleSolidClassification::Boundary => {
            adaptive_report.fallback_boundary_regression_cells += 1;
        }
    }
    let index = cell_index(cells_per_axis, coords)?;
    classifiers[index] = cell.classification;
    Ok(())
}

fn classify_consensus_axis_sweep_fallback_cell(
    coords: [u64; 3],
    frame: &GridFrame,
    solid: &ExactTriangleSolid,
    classifiers: &mut [VoxelTriangleSolidClassification],
    cells_per_axis: u64,
    consensus_report: &mut TriangleSolidConsensusAxisSweepVoxelizationReport,
) -> HypervoxelResult<()> {
    let address = VoxelAddress::new(frame.depth(), coords)?;
    let cell = classify_cell_against_exact_triangle_solid(address, frame, solid)?;
    consensus_report.fallback_cells += 1;
    consensus_report.fallback_ray_attempts += cell.ray_attempts.len();
    consensus_report.fallback_ray_aabb_rejections += cell
        .ray_attempts
        .iter()
        .map(|attempt| attempt.ray_aabb_rejections)
        .sum::<usize>();
    consensus_report.fallback_ray_triangle_tests += cell.ray_triangle_tests();
    consensus_report.ambiguous_fallback_ray_attempts += cell
        .ray_attempts
        .iter()
        .filter(|attempt| !attempt.certified)
        .count();

    match cell.classification {
        VoxelTriangleSolidClassification::Inside | VoxelTriangleSolidClassification::Outside => {}
        VoxelTriangleSolidClassification::Unknown => consensus_report.fallback_unknown_cells += 1,
        VoxelTriangleSolidClassification::Boundary => {
            consensus_report.fallback_boundary_regression_cells += 1;
        }
    }
    let index = cell_index(cells_per_axis, coords)?;
    classifiers[index] = cell.classification;
    Ok(())
}

fn classify_component_consensus_fallback_cell(
    coords: [u64; 3],
    frame: &GridFrame,
    solid: &ExactTriangleSolid,
    classifiers: &mut [VoxelTriangleSolidClassification],
    cells_per_axis: u64,
    component_report: &mut TriangleSolidComponentConsensusVoxelizationReport,
) -> HypervoxelResult<()> {
    let address = VoxelAddress::new(frame.depth(), coords)?;
    let cell = classify_cell_against_exact_triangle_solid(address, frame, solid)?;
    component_report.fallback_cells += 1;
    component_report.fallback_ray_attempts += cell.ray_attempts.len();
    component_report.fallback_ray_aabb_rejections += cell
        .ray_attempts
        .iter()
        .map(|attempt| attempt.ray_aabb_rejections)
        .sum::<usize>();
    component_report.fallback_ray_triangle_tests += cell.ray_triangle_tests();
    component_report.ambiguous_fallback_ray_attempts += cell
        .ray_attempts
        .iter()
        .filter(|attempt| !attempt.certified)
        .count();
    component_report.certified_fallback_ray_attempts += cell
        .ray_attempts
        .iter()
        .filter(|attempt| attempt.certified)
        .count();

    match cell.classification {
        VoxelTriangleSolidClassification::Inside | VoxelTriangleSolidClassification::Outside => {}
        VoxelTriangleSolidClassification::Unknown => component_report.fallback_unknown_cells += 1,
        VoxelTriangleSolidClassification::Boundary => {
            component_report.fallback_boundary_regression_cells += 1;
        }
    }
    let index = cell_index(cells_per_axis, coords)?;
    classifiers[index] = cell.classification;
    Ok(())
}

fn perpendicular_axes(axis: usize) -> [usize; 2] {
    match axis {
        0 => [1, 2],
        1 => [0, 2],
        2 => [0, 1],
        _ => unreachable!("axis sweep callers only pass lattice axes"),
    }
}

fn axis_direction(axis: usize) -> hyperlimit::Point3 {
    match axis {
        0 => hyperlimit::Point3::new(Real::from(1), Real::from(0), Real::from(0)),
        1 => hyperlimit::Point3::new(Real::from(0), Real::from(1), Real::from(0)),
        2 => hyperlimit::Point3::new(Real::from(0), Real::from(0), Real::from(1)),
        _ => unreachable!("axis sweep callers only pass lattice axes"),
    }
}

fn materialize_triangle_solid_classifiers(
    frame: GridFrame,
    policy: VoxelizationPolicy,
    material: MaterialRegionId,
    classifiers: &[VoxelTriangleSolidClassification],
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
                    VoxelTriangleSolidClassification::Inside => inside_cells += 1,
                    VoxelTriangleSolidClassification::Outside => outside_cells += 1,
                    VoxelTriangleSolidClassification::Boundary => predicate_boundary_cells += 1,
                    VoxelTriangleSolidClassification::Unknown => predicate_unknown_cells += 1,
                }

                let cell = match (policy.quantization, policy.boundary, classifier) {
                    (_, _, VoxelTriangleSolidClassification::Outside) => VoxelCell::empty(),
                    (_, _, VoxelTriangleSolidClassification::Unknown) => {
                        unknown_cells += 1;
                        VoxelCell::unknown()
                    }
                    (_, _, VoxelTriangleSolidClassification::Inside) => {
                        VoxelCell::material(material)
                    }
                    (
                        QuantizationPolicy::ConservativeInterior,
                        _,
                        VoxelTriangleSolidClassification::Boundary,
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
                        VoxelTriangleSolidClassification::Boundary,
                    ) => {
                        boundary_cells += 1;
                        unknown_cells += 1;
                        VoxelCell::unknown()
                    }
                    (
                        _,
                        BoundaryPolicy::LossySideChoice,
                        VoxelTriangleSolidClassification::Boundary,
                    ) => {
                        boundary_cells += 1;
                        VoxelCell {
                            occupancy: OccupancyState::LossyAdapterValue,
                            payload: VoxelPayload::LossyAdapterValue(material.0),
                        }
                    }
                    (
                        _,
                        BoundaryPolicy::KeepBoundary,
                        VoxelTriangleSolidClassification::Boundary,
                    ) => {
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

fn classify_point_against_exact_triangle_solid_by_ray(
    point: &[Real; 3],
    solid: &ExactTriangleSolid,
) -> HypervoxelResult<(
    VoxelTriangleSolidClassification,
    Vec<RayParityAttemptReport>,
)> {
    let mut attempts = Vec::new();
    for (direction_index, direction) in ray_parity_directions().into_iter().enumerate() {
        let (classification, attempt) = classify_point_against_exact_triangle_solid_by_single_ray(
            point,
            solid,
            &direction,
            direction_index,
        )?;
        let certified = attempt.certified;
        attempts.push(attempt);
        if certified {
            return Ok((classification, attempts));
        }
    }
    Ok((VoxelTriangleSolidClassification::Unknown, attempts))
}

fn classify_point_against_exact_triangle_solid_by_single_ray(
    point: &[Real; 3],
    solid: &ExactTriangleSolid,
    direction: &hyperlimit::Point3,
    direction_index: usize,
) -> HypervoxelResult<(VoxelTriangleSolidClassification, RayParityAttemptReport)> {
    let origin = point3(point);
    let direction_components = point_components(direction);
    let mut parameters: Vec<Real> = Vec::new();
    let mut attempt = RayParityAttemptReport {
        direction_index,
        ray_aabb_rejections: 0,
        triangle_tests: 0,
        proper_intersections: 0,
        unique_parameters: 0,
        boundary_touches: 0,
        coplanar_events: 0,
        certified: false,
    };

    for triangle in &solid.triangles {
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
            hyperlimit::PredicatePolicy::STRICT,
        )
        .value()
        .ok_or(HypervoxelError::UnknownScalarOrdering {
            field: "triangle-solid-ray",
        })?;
        match report.relation {
            RayTriangleIntersection::Disjoint => {}
            RayTriangleIntersection::Proper => {
                let Some(parameter) = report.parameter else {
                    return Ok((VoxelTriangleSolidClassification::Unknown, attempt));
                };
                attempt.proper_intersections += 1;
                insert_unique_parameter(&mut parameters, parameter)?;
                attempt.unique_parameters = parameters.len();
            }
            RayTriangleIntersection::BoundaryTouch => {
                attempt.boundary_touches += 1;
                return Ok((VoxelTriangleSolidClassification::Unknown, attempt));
            }
            RayTriangleIntersection::Coplanar => {
                attempt.coplanar_events += 1;
                return Ok((VoxelTriangleSolidClassification::Unknown, attempt));
            }
        }
    }

    attempt.certified = true;
    let classifier = if parameters.len().is_multiple_of(2) {
        VoxelTriangleSolidClassification::Outside
    } else {
        VoxelTriangleSolidClassification::Inside
    };
    Ok((classifier, attempt))
}

fn point_components(point: &hyperlimit::Point3) -> [Real; 3] {
    [point.x.clone(), point.y.clone(), point.z.clone()]
}
