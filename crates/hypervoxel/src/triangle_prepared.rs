//! Prepared exact triangle-solid voxelization.
//!
//! This module is the scheduled counterpart to [`crate::triangle_mesh`]. It
//! keeps retained source triangles as the owning evidence, but prepares exact
//! triangle AABBs once and reports how many triangle predicates each cell
//! actually consumed. The design follows Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7(1-2), 1997: the prepared data are
//! replayable object facts, not approximate acceleration hints that may change
//! topology silently.

use hyperlimit::{
    Aabb3Intersection, RayTriangleIntersection, classify_aabb3_intersection,
    classify_ray_triangle3_intersection_report,
};
use hyperreal::Real;

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
        match triangle_intersects_cell(&triangle.triangle, &bounds)? {
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

    let (classifier, ray_attempts) =
        classify_point_against_prepared_triangle_solid_by_ray(&bounds.center(), prepared)?;
    Ok(PreparedTriangleSolidCellReport {
        classifier,
        boundary_aabb_rejections,
        boundary_triangle_tests,
        boundary_unknown,
        ray_attempts,
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
    /// Total exact ray/triangle predicates.
    pub ray_triangle_tests: usize,
    /// Number of ambiguous ray attempts.
    pub ambiguous_ray_attempts: usize,
}

impl PreparedTriangleSolidVoxelizationReport {
    fn accumulate(&mut self, cell: &PreparedTriangleSolidCellReport) {
        self.classified_cells += 1;
        self.boundary_aabb_rejections += cell.boundary_aabb_rejections;
        self.boundary_triangle_tests += cell.boundary_triangle_tests;
        self.ray_attempts += cell.ray_attempts.len();
        self.ray_triangle_tests += cell.ray_triangle_tests();
        self.ambiguous_ray_attempts += cell
            .ray_attempts
            .iter()
            .filter(|attempt| !attempt.certified)
            .count();
    }
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
    let mut parameters: Vec<Real> = Vec::new();
    let mut attempt = PreparedRayParityAttemptReport {
        direction_index,
        triangle_tests: 0,
        proper_intersections: 0,
        unique_parameters: 0,
        boundary_touches: 0,
        coplanar_events: 0,
        certified: false,
    };

    for triangle in &prepared.triangles {
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
