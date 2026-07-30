//! Exact triangle-surface voxelization handoff.
//!
//! This module is the `hypervoxel` consumer-side path for exact
//! mesh/BREP-style triangle evidence. It materializes two related handoffs:
//! a conservative surface cover, where cells whose exact AABBs intersect
//! retained source triangles become boundary cells, and a closed-solid cover,
//! where strict non-boundary cells are filled by exact ray-parity tests.
//!
//! The cell/triangle test is the exact separating-axis test of
//! Akenine-Möller: the three box axes, triangle plane normal, and nine
//! edge-cross-axis directions. Doubled cell-centered coordinates avoid
//! division while retaining exact closed-boundary contact. Topology changes
//! only after proof-producing comparisons; undecided comparisons stay
//! explicit unknowns.

use hyperlimit::{
    Aabb3Intersection, PredicateOutcome, RayTriangleIntersection, Triangle3Location,
    classify_aabb3_intersection, classify_point_triangle3,
    classify_ray_triangle3_intersection_report,
};
use hyperreal::{Rational, Real};

use crate::{
    BoundaryPolicy, CellBounds, GridFrame, HypervoxelError, HypervoxelResult, MaterialRegionId,
    OccupancyState, QuantizationPolicy, SparseVoxelGrid, VoxelAddress, VoxelAggregateFacts,
    VoxelCell, VoxelPayload, VoxelPredicateCertificateReport, VoxelizationPolicy,
    VoxelizationReport,
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
        self.points_and_report().1
    }

    pub(crate) fn points_and_report(&self) -> ([hyperlimit::Point3; 3], ExactTriangle3Report) {
        let points = self.points();
        let location = classify_point_triangle3(
            &points[0],
            &points[1],
            &points[2],
            &points[0],
            hyperlimit::PredicatePolicy::STRICT,
        );
        let (degenerate, unknown_predicate) = match location.value() {
            Some(Triangle3Location::Degenerate) => (true, false),
            Some(_) => (false, false),
            None => (false, true),
        };
        (
            points,
            ExactTriangle3Report {
                degenerate,
                unknown_predicate,
                exact_triangle_ready: !degenerate && !unknown_predicate,
            },
        )
    }

    pub(crate) fn points(&self) -> [hyperlimit::Point3; 3] {
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
}

impl ExactTriangleSurfaceMesh {
    /// Creates a retained exact triangle-surface handoff.
    pub fn new(triangles: Vec<ExactTriangle3>) -> Self {
        Self { triangles }
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
        let exact_triangle_source_ready =
            !empty_triangle_set && degenerate_triangle_count == 0 && unknown_triangle_count == 0;
        ExactTriangleSurfaceMeshReport {
            triangle_count,
            empty_triangle_set,
            degenerate_triangle_count,
            unknown_triangle_count,
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
pub enum VoxelTriangleMeshClassification {
    /// Cell AABB is disjoint from all retained triangles.
    Outside,
    /// Cell AABB intersects one or more retained triangles.
    Boundary,
    /// At least one predicate needed to prove disjointness was undecided.
    Unknown,
}

/// Classification of one cell against a retained closed triangle solid.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VoxelTriangleSolidClassification {
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
) -> HypervoxelResult<VoxelTriangleMeshClassification> {
    let bounds = address.bounds(frame)?;
    let mut saw_unknown = false;
    for triangle in &mesh.triangles {
        match triangle_intersects_cell(triangle, &bounds)? {
            TriangleCellIntersection::Intersects => {
                return Ok(VoxelTriangleMeshClassification::Boundary);
            }
            TriangleCellIntersection::Disjoint => {}
            TriangleCellIntersection::Unknown => saw_unknown = true,
        }
    }
    Ok(if saw_unknown {
        VoxelTriangleMeshClassification::Unknown
    } else {
        VoxelTriangleMeshClassification::Outside
    })
}

/// Classifies one cell against an exact retained closed triangle solid.
///
/// Boundary cells are detected first by the exact surface-cover classifier.
/// Non-boundary cells are classified by exact ray-parity queries from the exact
/// cell center. Proper ray/triangle intersections are counted by unique exact
/// ray parameter; boundary touches and coplanar ray events cause that ray to be
/// skipped, and the classifier returns [`VoxelTriangleSolidClassification::Unknown`]
/// only when every configured exact rational ray is ambiguous. This is an exact
/// parity point-in-polyhedron test using the Möller–Trumbore ray/triangle
/// decomposition; ambiguous events remain explicit instead of being repaired
/// with floating epsilons.
pub fn classify_cell_against_triangle_solid_mesh(
    address: VoxelAddress,
    frame: &GridFrame,
    mesh: &ExactTriangleSolidMesh,
) -> HypervoxelResult<VoxelTriangleSolidClassification> {
    match classify_cell_against_triangle_surface_mesh(address, frame, &mesh.surface)? {
        VoxelTriangleMeshClassification::Boundary => {
            return Ok(VoxelTriangleSolidClassification::Boundary);
        }
        VoxelTriangleMeshClassification::Unknown => {
            return Ok(VoxelTriangleSolidClassification::Unknown);
        }
        VoxelTriangleMeshClassification::Outside => {}
    }

    let bounds = address.bounds(frame)?;
    match classify_point_against_triangle_solid_by_ray(&bounds.center(), mesh)? {
        RayParityPointClassification::Outside => Ok(VoxelTriangleSolidClassification::Outside),
        RayParityPointClassification::Inside => Ok(VoxelTriangleSolidClassification::Inside),
        RayParityPointClassification::Unknown => Ok(VoxelTriangleSolidClassification::Unknown),
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
                            VoxelTriangleMeshClassification::Unknown
                        }
                        Err(err) => return Err(err),
                    };
                match classifier {
                    VoxelTriangleMeshClassification::Outside => outside_cells += 1,
                    VoxelTriangleMeshClassification::Boundary => predicate_boundary_cells += 1,
                    VoxelTriangleMeshClassification::Unknown => predicate_unknown_cells += 1,
                }

                let cell = match (policy.quantization, policy.boundary, classifier) {
                    (_, _, VoxelTriangleMeshClassification::Outside) => VoxelCell::empty(),
                    (_, _, VoxelTriangleMeshClassification::Unknown) => {
                        unknown_cells += 1;
                        VoxelCell::unknown()
                    }
                    (
                        QuantizationPolicy::ConservativeInterior,
                        _,
                        VoxelTriangleMeshClassification::Boundary,
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
                        VoxelTriangleMeshClassification::Boundary,
                    ) => {
                        boundary_cells += 1;
                        unknown_cells += 1;
                        VoxelCell::unknown()
                    }
                    (
                        _,
                        BoundaryPolicy::LossySideChoice,
                        VoxelTriangleMeshClassification::Boundary,
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
                        VoxelTriangleMeshClassification::Boundary,
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
                            VoxelTriangleSolidClassification::Unknown
                        }
                        Err(err) => return Err(err),
                    };
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
    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum TriangleCellIntersection {
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
    // explicit ambiguous event, avoiding a brittle single-ray dependency for
    // common grid-aligned solids.
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
            &origin,
            direction,
            &points[0],
            &points[1],
            &points[2],
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
                    return Ok(RayParityPointClassification::Unknown);
                };
                insert_unique_parameter(&mut parameters, parameter)?;
            }
            RayTriangleIntersection::BoundaryTouch | RayTriangleIntersection::Coplanar => {
                return Ok(RayParityPointClassification::Unknown);
            }
        }
    }

    if parameters.len().is_multiple_of(2) {
        Ok(RayParityPointClassification::Outside)
    } else {
        Ok(RayParityPointClassification::Inside)
    }
}

pub(crate) fn ray_parity_directions() -> [hyperlimit::Point3; 7] {
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

pub(crate) fn insert_unique_parameter(
    parameters: &mut Vec<Real>,
    parameter: Real,
) -> HypervoxelResult<()> {
    for existing in parameters.iter() {
        match hyperlimit::compare_reals(existing, &parameter, hyperlimit::PredicatePolicy::STRICT)
            .value()
        {
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

pub(crate) fn triangle_intersects_cell(
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

    let triangle_bounds = triangle_bounds(triangle)?;
    let cell_min = point3(&bounds.min);
    let cell_max = point3(&bounds.max);
    if decide_aabb(classify_aabb3_intersection(
        &point3(&triangle_bounds.min),
        &point3(&triangle_bounds.max),
        &cell_min,
        &cell_max,
        hyperlimit::PredicatePolicy::STRICT,
    ))? == Aabb3Intersection::Disjoint
    {
        return Ok(TriangleCellIntersection::Disjoint);
    }

    triangle_intersects_cell_after_aabb(triangle, bounds)
}

pub(crate) fn triangle_intersects_cell_after_aabb(
    triangle: &ExactTriangle3,
    bounds: &CellBounds,
) -> HypervoxelResult<TriangleCellIntersection> {
    let center_sum: [Real; 3] = core::array::from_fn(|axis| &bounds.min[axis] + &bounds.max[axis]);
    let full_extents: [Real; 3] =
        core::array::from_fn(|axis| &bounds.max[axis] - &bounds.min[axis]);
    let centered: [[Real; 3]; 3] = core::array::from_fn(|vertex| {
        core::array::from_fn(|axis| {
            (&triangle.vertices[vertex][axis] * &Real::from(2_u8)) - &center_sum[axis]
        })
    });
    let edges: [[Real; 3]; 3] = core::array::from_fn(|edge| {
        let next = (edge + 1) % 3;
        core::array::from_fn(|axis| &centered[next][axis] - &centered[edge][axis])
    });
    let normal = cross3(&edges[0], &edges[1]);
    if separates_triangle_cell_axis(&centered, &full_extents, &normal)? {
        return Ok(TriangleCellIntersection::Disjoint);
    }
    for edge in &edges {
        let cross_axes = [
            [Real::zero(), edge[2].clone(), Real::zero() - &edge[1]],
            [Real::zero() - &edge[2], Real::zero(), edge[0].clone()],
            [edge[1].clone(), Real::zero() - &edge[0], Real::zero()],
        ];
        for axis in &cross_axes {
            if separates_triangle_cell_axis(&centered, &full_extents, axis)? {
                return Ok(TriangleCellIntersection::Disjoint);
            }
        }
    }
    Ok(TriangleCellIntersection::Intersects)
}

fn cross3(left: &[Real; 3], right: &[Real; 3]) -> [Real; 3] {
    [
        &left[1] * &right[2] - &left[2] * &right[1],
        &left[2] * &right[0] - &left[0] * &right[2],
        &left[0] * &right[1] - &left[1] * &right[0],
    ]
}

fn separates_triangle_cell_axis(
    triangle: &[[Real; 3]; 3],
    full_extents: &[Real; 3],
    axis: &[Real; 3],
) -> HypervoxelResult<bool> {
    let mut magnitudes: [Real; 3] = core::array::from_fn(|_| Real::zero());
    let mut nonzero = false;
    for component in 0..3 {
        match compare(&axis[component], &Real::zero(), component)? {
            core::cmp::Ordering::Less => {
                magnitudes[component] = Real::zero() - &axis[component];
                nonzero = true;
            }
            core::cmp::Ordering::Equal => {}
            core::cmp::Ordering::Greater => {
                magnitudes[component] = axis[component].clone();
                nonzero = true;
            }
        }
    }
    if !nonzero {
        return Ok(false);
    }
    let projections: [Real; 3] = core::array::from_fn(|vertex| {
        (0..3).fold(Real::zero(), |sum, component| {
            &sum + &triangle[vertex][component] * &axis[component]
        })
    });
    let mut min = projections[0].clone();
    let mut max = projections[0].clone();
    for projection in projections.iter().skip(1) {
        if compare(projection, &min, 0)? == core::cmp::Ordering::Less {
            min = projection.clone();
        }
        if compare(projection, &max, 0)? == core::cmp::Ordering::Greater {
            max = projection.clone();
        }
    }
    let radius = (0..3).fold(Real::zero(), |sum, component| {
        &sum + &full_extents[component] * &magnitudes[component]
    });
    Ok(compare(&min, &radius, 0)? == core::cmp::Ordering::Greater
        || compare(&max, &(Real::zero() - &radius), 0)? == core::cmp::Ordering::Less)
}

pub(crate) fn triangle_bounds(triangle: &ExactTriangle3) -> HypervoxelResult<crate::ExactAabb3> {
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
    hyperlimit::compare_reals(left, right, hyperlimit::PredicatePolicy::STRICT)
        .value()
        .ok_or(HypervoxelError::UnknownOrdering { axis })
}

fn decide_aabb(
    outcome: PredicateOutcome<Aabb3Intersection>,
) -> HypervoxelResult<Aabb3Intersection> {
    outcome.ok_or_unknown("triangle-aabb")
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

pub(crate) fn point3(values: &[Real; 3]) -> hyperlimit::Point3 {
    hyperlimit::Point3::new(values[0].clone(), values[1].clone(), values[2].clone())
}
