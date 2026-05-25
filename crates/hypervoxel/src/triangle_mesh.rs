//! Exact triangle-surface voxelization handoff.
//!
//! This module is the first `hypervoxel` consumer-side path for exact
//! mesh/BREP-style triangle evidence. It materializes two related handoffs:
//! a conservative surface cover, where cells whose exact AABBs intersect
//! retained source triangles become boundary cells, and a closed-solid cover,
//! where strict non-boundary cells are filled by exact ray-parity tests.
//!
//! The cell/triangle test is a composition of exact predicates: AABB broad
//! phase, exact 3D point-in-triangle tests, and exact triangle/triangle tests
//! against the twelve AABB face triangles. This follows Yap, "Towards Exact
//! Geometric Computation," *Computational Geometry* 7(1-2), 1997: topology is
//! changed only after proof-producing predicates, and undecided comparisons
//! stay explicit unknowns. The triangle/triangle decomposition is the same
//! predicate-family boundary used by Moller, "A Fast Triangle-Triangle
//! Intersection Test" (1997), and Guigue and Devillers, "Fast and Robust
//! Triangle-Triangle Overlap Test Using Orientation Predicates" (2003), but
//! here routed through `hyperlimit`'s exact reports instead of primitive
//! floating-point tests.

use hyperlimit::{
    Aabb3Intersection, Aabb3PointLocation, PredicateOutcome, RayTriangleIntersection,
    Triangle3Location, TriangleTriangleIntersection, classify_aabb3_intersection,
    classify_point_aabb3, classify_point_triangle3, classify_ray_triangle3_intersection_report,
    classify_triangle_triangle3_points_with_policy,
};
use hyperreal::{Rational, Real};

use crate::{
    BoundaryPolicy, CellBounds, GridFrame, GridSource, HypervoxelError, HypervoxelResult,
    MaterialRegionId, OccupancyState, QuantizationPolicy, SparseVoxelGrid, VoxelAddress,
    VoxelAggregateFacts, VoxelCell, VoxelPayload, VoxelPredicateCertificateReport,
    VoxelizationPolicy, VoxelizationReport,
};

/// Exact triangle supplied by a mesh/BREP owner.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactTriangle3 {
    /// Triangle vertices in source coordinates.
    pub vertices: [[Real; 3]; 3],
    /// Optional retained source face id.
    pub source_face: Option<u64>,
}

impl ExactTriangle3 {
    /// Creates an exact triangle from retained coordinates.
    pub const fn new(vertices: [[Real; 3]; 3], source_face: Option<u64>) -> Self {
        Self {
            vertices,
            source_face,
        }
    }

    /// Reports whether this triangle is structurally usable for voxelization.
    pub fn report(&self) -> ExactTriangle3Report {
        let points = self.points();
        let location = classify_point_triangle3(&points[0], &points[1], &points[2], &points[0]);
        let (degenerate, unknown_predicate) = match location.value() {
            Some(Triangle3Location::Degenerate) => (true, false),
            Some(_) => (false, false),
            None => (false, true),
        };
        ExactTriangle3Report {
            degenerate,
            unknown_predicate,
            exact_triangle_ready: !degenerate && !unknown_predicate,
        }
    }

    fn points(&self) -> [hyperlimit::Point3; 3] {
        [
            point3(&self.vertices[0]),
            point3(&self.vertices[1]),
            point3(&self.vertices[2]),
        ]
    }
}

/// Preflight report for one [`ExactTriangle3`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ExactTriangle3Report {
    /// The triangle vertices were certified degenerate.
    pub degenerate: bool,
    /// Triangle readiness could not be certified.
    pub unknown_predicate: bool,
    /// The triangle can participate in exact surface voxelization.
    pub exact_triangle_ready: bool,
}

/// Exact triangle-surface handoff from a mesh/BREP owner.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactTriangleSurfaceMesh {
    /// Retained source triangles.
    pub triangles: Vec<ExactTriangle3>,
    /// Optional source/version that produced the triangles.
    pub source: Option<GridSource>,
    /// Whether the producer reported exact source replay for these triangles.
    pub exact_source_replay_available: bool,
}

impl ExactTriangleSurfaceMesh {
    /// Creates a retained exact triangle-surface handoff.
    pub fn new(
        triangles: Vec<ExactTriangle3>,
        source: Option<GridSource>,
        exact_source_replay_available: bool,
    ) -> Self {
        Self {
            triangles,
            source,
            exact_source_replay_available,
        }
    }

    /// Reports whether this retained triangle set is an exact source handoff.
    pub fn report(&self) -> ExactTriangleSurfaceMeshReport {
        let mut degenerate_triangle_count = 0_usize;
        let mut unknown_triangle_count = 0_usize;
        for triangle in &self.triangles {
            let report = triangle.report();
            degenerate_triangle_count += usize::from(report.degenerate);
            unknown_triangle_count += usize::from(report.unknown_predicate);
        }
        let triangle_count = self.triangles.len();
        let empty_triangle_set = triangle_count == 0;
        let exact_triangle_source_ready = self.exact_source_replay_available
            && !empty_triangle_set
            && degenerate_triangle_count == 0
            && unknown_triangle_count == 0;
        ExactTriangleSurfaceMeshReport {
            triangle_count,
            empty_triangle_set,
            degenerate_triangle_count,
            unknown_triangle_count,
            exact_source_replay_available: self.exact_source_replay_available,
            exact_triangle_source_ready,
        }
    }
}

/// Preflight report for an [`ExactTriangleSurfaceMesh`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExactTriangleSurfaceMeshReport {
    /// Number of retained source triangles.
    pub triangle_count: usize,
    /// Whether no source triangles were supplied.
    pub empty_triangle_set: bool,
    /// Number of certified degenerate triangles.
    pub degenerate_triangle_count: usize,
    /// Number of triangles whose predicate readiness was undecided.
    pub unknown_triangle_count: usize,
    /// Whether producer-side exact replay was available.
    pub exact_source_replay_available: bool,
    /// Whether source triangles are ready for exact surface voxelization.
    pub exact_triangle_source_ready: bool,
}

/// Exact closed triangle-solid handoff from a mesh/BREP owner.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactTriangleSolidMesh {
    /// Retained source surface triangles.
    pub surface: ExactTriangleSurfaceMesh,
    /// Whether the producer reported exact closed-solid replay for these
    /// triangles.
    pub exact_closed_solid_replay_available: bool,
}

impl ExactTriangleSolidMesh {
    /// Creates an exact closed triangle-solid handoff.
    pub const fn new(
        surface: ExactTriangleSurfaceMesh,
        exact_closed_solid_replay_available: bool,
    ) -> Self {
        Self {
            surface,
            exact_closed_solid_replay_available,
        }
    }

    /// Reports whether this retained triangle set is ready for exact solid
    /// filling.
    pub fn report(&self) -> ExactTriangleSolidMeshReport {
        let surface = self.surface.report();
        let exact_solid_source_ready =
            surface.exact_triangle_source_ready && self.exact_closed_solid_replay_available;
        ExactTriangleSolidMeshReport {
            surface,
            exact_closed_solid_replay_available: self.exact_closed_solid_replay_available,
            exact_solid_source_ready,
        }
    }
}

/// Preflight report for an [`ExactTriangleSolidMesh`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExactTriangleSolidMeshReport {
    /// Surface-triangle source readiness.
    pub surface: ExactTriangleSurfaceMeshReport,
    /// Whether producer-side closed-solid replay was available.
    pub exact_closed_solid_replay_available: bool,
    /// Whether this handoff can drive exact solid filling.
    pub exact_solid_source_ready: bool,
}

/// Classification of one cell against a retained triangle surface.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VoxelTriangleMeshClassifier {
    /// Cell AABB is disjoint from all retained triangles.
    Outside,
    /// Cell AABB intersects one or more retained triangles.
    Boundary,
    /// At least one predicate needed to prove disjointness was undecided.
    Unknown,
}

/// Classification of one cell against a retained closed triangle solid.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VoxelTriangleSolidClassifier {
    /// Cell AABB is outside the solid and disjoint from its boundary.
    Outside,
    /// Cell center is inside the solid and the cell AABB is disjoint from the
    /// retained triangle boundary.
    Inside,
    /// Cell AABB intersects one or more retained boundary triangles.
    Boundary,
    /// Exact inside/outside or boundary evidence was ambiguous.
    Unknown,
}

/// Classifies one cell against an exact retained triangle surface.
pub fn classify_cell_against_triangle_surface_mesh(
    address: VoxelAddress,
    frame: &GridFrame,
    mesh: &ExactTriangleSurfaceMesh,
) -> HypervoxelResult<VoxelTriangleMeshClassifier> {
    let bounds = address.bounds(frame)?;
    let mut saw_unknown = false;
    for triangle in &mesh.triangles {
        match triangle_intersects_cell(triangle, &bounds)? {
            TriangleCellIntersection::Intersects => {
                return Ok(VoxelTriangleMeshClassifier::Boundary);
            }
            TriangleCellIntersection::Disjoint => {}
            TriangleCellIntersection::Unknown => saw_unknown = true,
        }
    }
    Ok(if saw_unknown {
        VoxelTriangleMeshClassifier::Unknown
    } else {
        VoxelTriangleMeshClassifier::Outside
    })
}

/// Classifies one cell against an exact retained closed triangle solid.
///
/// Boundary cells are detected first by the exact surface-cover classifier.
/// Non-boundary cells are classified by exact ray-parity queries from the exact
/// cell center. Proper ray/triangle intersections are counted by unique exact
/// ray parameter; boundary touches and coplanar ray events cause that ray to be
/// skipped, and the classifier returns [`VoxelTriangleSolidClassifier::Unknown`]
/// only when every configured exact rational ray is ambiguous. This is the
/// standard parity point-in-polyhedron idea, but guarded at Yap's
/// exact-computation boundary: ambiguous combinatorial events remain explicit
/// instead of being repaired with floating epsilons. See Yap (1997) and the
/// ray/triangle decomposition of Moller and Trumbore, "Fast, Minimum Storage
/// Ray/Triangle Intersection," *Journal of Graphics Tools* 2(1), 1997.
pub fn classify_cell_against_triangle_solid_mesh(
    address: VoxelAddress,
    frame: &GridFrame,
    mesh: &ExactTriangleSolidMesh,
) -> HypervoxelResult<VoxelTriangleSolidClassifier> {
    match classify_cell_against_triangle_surface_mesh(address, frame, &mesh.surface)? {
        VoxelTriangleMeshClassifier::Boundary => return Ok(VoxelTriangleSolidClassifier::Boundary),
        VoxelTriangleMeshClassifier::Unknown => return Ok(VoxelTriangleSolidClassifier::Unknown),
        VoxelTriangleMeshClassifier::Outside => {}
    }

    let bounds = address.bounds(frame)?;
    match classify_point_against_triangle_solid_by_ray(&bounds.center(), mesh)? {
        RayParityPointClassification::Outside => Ok(VoxelTriangleSolidClassifier::Outside),
        RayParityPointClassification::Inside => Ok(VoxelTriangleSolidClassifier::Inside),
        RayParityPointClassification::Unknown => Ok(VoxelTriangleSolidClassifier::Unknown),
    }
}

/// Voxelizes a retained triangle surface as exact boundary cells.
pub fn voxelize_exact_triangle_surface_mesh(
    frame: GridFrame,
    mesh: &ExactTriangleSurfaceMesh,
    material: MaterialRegionId,
    policy: VoxelizationPolicy,
) -> HypervoxelResult<(SparseVoxelGrid, VoxelizationReport)> {
    let source_report = mesh.report();
    if source_report.empty_triangle_set {
        return Err(HypervoxelError::InvalidSourceGeometry {
            reason: "triangle surface mesh has no triangles",
        });
    }
    if source_report.degenerate_triangle_count > 0 {
        return Err(HypervoxelError::InvalidSourceGeometry {
            reason: "triangle surface mesh contains degenerate triangles",
        });
    }
    if source_report.unknown_triangle_count > 0 {
        return Err(HypervoxelError::InvalidSourceGeometry {
            reason: "triangle surface mesh has uncertified triangle predicates",
        });
    }
    if !source_report.exact_source_replay_available {
        return Err(HypervoxelError::InvalidSourceGeometry {
            reason: "triangle surface mesh lacks exact source replay",
        });
    }

    let mut grid = SparseVoxelGrid::new(frame.clone());
    let mut boundary_cells = 0_usize;
    let mut unknown_cells = 0_usize;
    let mut outside_cells = 0_usize;
    let mut predicate_boundary_cells = 0_usize;
    let mut predicate_unknown_cells = 0_usize;
    let cells_per_axis = frame.cells_per_axis();

    for z in 0..cells_per_axis {
        for y in 0..cells_per_axis {
            for x in 0..cells_per_axis {
                let address = VoxelAddress::new(frame.depth(), [x, y, z])?;
                let classifier =
                    match classify_cell_against_triangle_surface_mesh(address, &frame, mesh) {
                        Ok(classifier) => classifier,
                        Err(HypervoxelError::UnknownOrdering { .. })
                        | Err(HypervoxelError::UnknownScalarOrdering { .. }) => {
                            VoxelTriangleMeshClassifier::Unknown
                        }
                        Err(err) => return Err(err),
                    };
                match classifier {
                    VoxelTriangleMeshClassifier::Outside => outside_cells += 1,
                    VoxelTriangleMeshClassifier::Boundary => predicate_boundary_cells += 1,
                    VoxelTriangleMeshClassifier::Unknown => predicate_unknown_cells += 1,
                }

                let cell = match (policy.quantization, policy.boundary, classifier) {
                    (_, _, VoxelTriangleMeshClassifier::Outside) => VoxelCell::empty(),
                    (_, _, VoxelTriangleMeshClassifier::Unknown) => {
                        unknown_cells += 1;
                        VoxelCell::unknown()
                    }
                    (
                        QuantizationPolicy::ConservativeInterior,
                        _,
                        VoxelTriangleMeshClassifier::Boundary,
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
                        VoxelTriangleMeshClassifier::Boundary,
                    ) => {
                        boundary_cells += 1;
                        unknown_cells += 1;
                        VoxelCell::unknown()
                    }
                    (_, BoundaryPolicy::LossySideChoice, VoxelTriangleMeshClassifier::Boundary) => {
                        boundary_cells += 1;
                        VoxelCell {
                            occupancy: OccupancyState::LossyAdapterValue,
                            payload: VoxelPayload::LossyAdapterValue(material.0),
                        }
                    }
                    (_, BoundaryPolicy::KeepBoundary, VoxelTriangleMeshClassifier::Boundary) => {
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
        source: mesh.source.clone(),
        frame,
        policy,
        aggregate,
        unknown_cells,
        boundary_cells,
        predicate_certificates: VoxelPredicateCertificateReport::from_counts(
            0,
            outside_cells,
            predicate_boundary_cells,
            predicate_unknown_cells,
        ),
        legacy_adapter: None,
    };
    Ok((grid, report))
}

/// Voxelizes a retained closed triangle solid into exact filled and boundary
/// cells.
pub fn voxelize_exact_triangle_solid_mesh(
    frame: GridFrame,
    mesh: &ExactTriangleSolidMesh,
    material: MaterialRegionId,
    policy: VoxelizationPolicy,
) -> HypervoxelResult<(SparseVoxelGrid, VoxelizationReport)> {
    let source_report = mesh.report();
    validate_surface_source_report(&source_report.surface)?;
    if !source_report.exact_closed_solid_replay_available {
        return Err(HypervoxelError::InvalidSourceGeometry {
            reason: "triangle solid mesh lacks exact closed-solid replay",
        });
    }

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
                let address = VoxelAddress::new(frame.depth(), [x, y, z])?;
                let classifier =
                    match classify_cell_against_triangle_solid_mesh(address, &frame, mesh) {
                        Ok(classifier) => classifier,
                        Err(HypervoxelError::UnknownOrdering { .. })
                        | Err(HypervoxelError::UnknownScalarOrdering { .. }) => {
                            VoxelTriangleSolidClassifier::Unknown
                        }
                        Err(err) => return Err(err),
                    };
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
        source: mesh.surface.source.clone(),
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

fn validate_surface_source_report(
    source_report: &ExactTriangleSurfaceMeshReport,
) -> HypervoxelResult<()> {
    if source_report.empty_triangle_set {
        return Err(HypervoxelError::InvalidSourceGeometry {
            reason: "triangle surface mesh has no triangles",
        });
    }
    if source_report.degenerate_triangle_count > 0 {
        return Err(HypervoxelError::InvalidSourceGeometry {
            reason: "triangle surface mesh contains degenerate triangles",
        });
    }
    if source_report.unknown_triangle_count > 0 {
        return Err(HypervoxelError::InvalidSourceGeometry {
            reason: "triangle surface mesh has uncertified triangle predicates",
        });
    }
    if !source_report.exact_source_replay_available {
        return Err(HypervoxelError::InvalidSourceGeometry {
            reason: "triangle surface mesh lacks exact source replay",
        });
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum TriangleCellIntersection {
    Disjoint,
    Intersects,
    Unknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RayParityPointClassification {
    Outside,
    Inside,
    Unknown,
}

fn classify_point_against_triangle_solid_by_ray(
    point: &[Real; 3],
    mesh: &ExactTriangleSolidMesh,
) -> HypervoxelResult<RayParityPointClassification> {
    // The direction family is an exact finite retry set, not a symbolic
    // perturbation. Each ray is either a proof-producing parity query or an
    // explicit ambiguous event. This follows Yap's (1997) separation between
    // certified predicates and unresolved combinatorics while still avoiding a
    // brittle single-ray dependency for common grid-aligned solids.
    for direction in ray_parity_directions() {
        match classify_point_against_triangle_solid_by_single_ray(point, mesh, &direction)? {
            RayParityPointClassification::Unknown => {}
            decided => return Ok(decided),
        }
    }

    Ok(RayParityPointClassification::Unknown)
}

fn classify_point_against_triangle_solid_by_single_ray(
    point: &[Real; 3],
    mesh: &ExactTriangleSolidMesh,
    direction: &hyperlimit::Point3,
) -> HypervoxelResult<RayParityPointClassification> {
    let origin = point3(point);
    let mut parameters: Vec<Real> = Vec::new();

    for triangle in &mesh.surface.triangles {
        let points = triangle.points();
        let report = classify_ray_triangle3_intersection_report(
            &origin, direction, &points[0], &points[1], &points[2],
        )
        .value()
        .ok_or(HypervoxelError::UnknownScalarOrdering {
            field: "triangle-solid-ray",
        })?;
        match report.relation {
            RayTriangleIntersection::Disjoint => {}
            RayTriangleIntersection::Proper => {
                let Some(parameter) = report.parameter else {
                    return Ok(RayParityPointClassification::Unknown);
                };
                insert_unique_parameter(&mut parameters, parameter)?;
            }
            RayTriangleIntersection::BoundaryTouch | RayTriangleIntersection::Coplanar => {
                return Ok(RayParityPointClassification::Unknown);
            }
        }
    }

    if parameters.len() % 2 == 0 {
        Ok(RayParityPointClassification::Outside)
    } else {
        Ok(RayParityPointClassification::Inside)
    }
}

fn ray_parity_directions() -> [hyperlimit::Point3; 7] {
    [
        rational_direction([1, 2, 3], 1),
        rational_direction([1, 3, 5], 1),
        rational_direction([2, 5, 7], 1),
        rational_direction([3, 7, 11], 1),
        rational_direction([5, 11, 17], 1),
        rational_direction([7, 13, 19], 1),
        rational_direction([11, 17, 23], 1),
    ]
}

fn rational_direction(numerators: [i64; 3], denominator: u64) -> hyperlimit::Point3 {
    hyperlimit::Point3::new(
        Rational::fraction(numerators[0], denominator)
            .expect("positive literal denominator")
            .into(),
        Rational::fraction(numerators[1], denominator)
            .expect("positive literal denominator")
            .into(),
        Rational::fraction(numerators[2], denominator)
            .expect("positive literal denominator")
            .into(),
    )
}

fn insert_unique_parameter(parameters: &mut Vec<Real>, parameter: Real) -> HypervoxelResult<()> {
    for existing in parameters.iter() {
        match hyperlimit::compare_reals(existing, &parameter).value() {
            Some(core::cmp::Ordering::Equal) => return Ok(()),
            Some(_) => {}
            None => {
                return Err(HypervoxelError::UnknownScalarOrdering {
                    field: "triangle-solid-ray-parameter",
                });
            }
        }
    }
    parameters.push(parameter);
    Ok(())
}

fn triangle_intersects_cell(
    triangle: &ExactTriangle3,
    bounds: &CellBounds,
) -> HypervoxelResult<TriangleCellIntersection> {
    let triangle_report = triangle.report();
    if triangle_report.degenerate {
        return Err(HypervoxelError::InvalidSourceGeometry {
            reason: "triangle surface mesh contains degenerate triangles",
        });
    }
    if triangle_report.unknown_predicate {
        return Ok(TriangleCellIntersection::Unknown);
    }

    let triangle_points = triangle.points();
    let triangle_bounds = triangle_bounds(triangle)?;
    let cell_min = point3(&bounds.min);
    let cell_max = point3(&bounds.max);
    match decide_aabb(classify_aabb3_intersection(
        &point3(&triangle_bounds.min),
        &point3(&triangle_bounds.max),
        &cell_min,
        &cell_max,
    ))? {
        Aabb3Intersection::Disjoint => return Ok(TriangleCellIntersection::Disjoint),
        _ => {}
    }

    for point in &triangle_points {
        if point_in_aabb(point, &cell_min, &cell_max)? {
            return Ok(TriangleCellIntersection::Intersects);
        }
    }

    let cell_corners = cell_corner_points(bounds);
    for corner in &cell_corners {
        match decide_triangle_location(classify_point_triangle3(
            &triangle_points[0],
            &triangle_points[1],
            &triangle_points[2],
            corner,
        ))? {
            Triangle3Location::Inside | Triangle3Location::OnEdge | Triangle3Location::OnVertex => {
                return Ok(TriangleCellIntersection::Intersects);
            }
            Triangle3Location::Degenerate => {
                return Err(HypervoxelError::InvalidSourceGeometry {
                    reason: "triangle surface mesh contains degenerate triangles",
                });
            }
            Triangle3Location::OffPlane | Triangle3Location::Outside => {}
        }
    }

    for face_triangle in aabb_face_triangles(&cell_corners) {
        match decide_triangle_triangle(classify_triangle_triangle3_points_with_policy(
            [
                &triangle_points[0],
                &triangle_points[1],
                &triangle_points[2],
            ],
            [face_triangle[0], face_triangle[1], face_triangle[2]],
            hyperlimit::PredicatePolicy::default(),
        ))? {
            TriangleTriangleIntersection::Degenerate => {}
            relation if relation.intersects() => return Ok(TriangleCellIntersection::Intersects),
            _ => {}
        }
    }

    Ok(TriangleCellIntersection::Disjoint)
}

fn point_in_aabb(
    point: &hyperlimit::Point3,
    min: &hyperlimit::Point3,
    max: &hyperlimit::Point3,
) -> HypervoxelResult<bool> {
    Ok(matches!(
        decide_aabb_point(classify_point_aabb3(min, max, point))?,
        Aabb3PointLocation::Inside | Aabb3PointLocation::Boundary
    ))
}

fn triangle_bounds(triangle: &ExactTriangle3) -> HypervoxelResult<crate::ExactAabb3> {
    let mut min = triangle.vertices[0].clone();
    let mut max = triangle.vertices[0].clone();
    for vertex in triangle.vertices.iter().skip(1) {
        for axis in 0..3 {
            if compare(&vertex[axis], &min[axis], axis)? == core::cmp::Ordering::Less {
                min[axis] = vertex[axis].clone();
            }
            if compare(&vertex[axis], &max[axis], axis)? == core::cmp::Ordering::Greater {
                max[axis] = vertex[axis].clone();
            }
        }
    }
    Ok(crate::ExactAabb3 { min, max })
}

fn compare(left: &Real, right: &Real, axis: usize) -> HypervoxelResult<core::cmp::Ordering> {
    hyperlimit::compare_reals(left, right)
        .value()
        .ok_or(HypervoxelError::UnknownOrdering { axis })
}

fn decide_aabb(
    outcome: PredicateOutcome<Aabb3Intersection>,
) -> HypervoxelResult<Aabb3Intersection> {
    outcome.ok_or_unknown("triangle-aabb")
}

fn decide_aabb_point(
    outcome: PredicateOutcome<Aabb3PointLocation>,
) -> HypervoxelResult<Aabb3PointLocation> {
    outcome.ok_or_unknown("triangle-aabb-point")
}

fn decide_triangle_location(
    outcome: PredicateOutcome<Triangle3Location>,
) -> HypervoxelResult<Triangle3Location> {
    outcome.ok_or_unknown("triangle-point")
}

fn decide_triangle_triangle(
    outcome: PredicateOutcome<hyperlimit::TriangleTriangleClassification>,
) -> HypervoxelResult<TriangleTriangleIntersection> {
    outcome
        .value()
        .map(|classification| classification.relation)
        .ok_or(HypervoxelError::UnknownScalarOrdering {
            field: "triangle-triangle",
        })
}

trait PredicateOutcomeExt<T> {
    fn ok_or_unknown(self, field: &'static str) -> HypervoxelResult<T>;
}

impl<T> PredicateOutcomeExt<T> for PredicateOutcome<T> {
    fn ok_or_unknown(self, field: &'static str) -> HypervoxelResult<T> {
        self.value()
            .ok_or(HypervoxelError::UnknownScalarOrdering { field })
    }
}

fn cell_corner_points(bounds: &CellBounds) -> [hyperlimit::Point3; 8] {
    bounds.corners().map(|corner| point3(&corner))
}

fn aabb_face_triangles(corners: &[hyperlimit::Point3; 8]) -> [[&hyperlimit::Point3; 3]; 12] {
    [
        [&corners[0], &corners[1], &corners[2]],
        [&corners[1], &corners[3], &corners[2]],
        [&corners[4], &corners[6], &corners[5]],
        [&corners[5], &corners[6], &corners[7]],
        [&corners[0], &corners[4], &corners[1]],
        [&corners[1], &corners[4], &corners[5]],
        [&corners[2], &corners[3], &corners[6]],
        [&corners[3], &corners[7], &corners[6]],
        [&corners[0], &corners[2], &corners[4]],
        [&corners[2], &corners[6], &corners[4]],
        [&corners[1], &corners[5], &corners[3]],
        [&corners[3], &corners[5], &corners[7]],
    ]
}

fn point3(values: &[Real; 3]) -> hyperlimit::Point3 {
    hyperlimit::Point3::new(values[0].clone(), values[1].clone(), values[2].clone())
}
