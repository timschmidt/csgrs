//! Exact ray-parity winding classification for closed triangle meshes.
//!
//! This module is the general, nonconvex counterpart to the convex halfspace
//! classifier in [`crate::solid`]. It uses a deterministic set of exact rays
//! over `Real` arithmetic, then treats ray/edge and ray/vertex degeneracies as
//! explicit blockers so a selected parity result was obtained without hidden
//! tolerance choices. The parity query is the standard ray-crossing point
//! classification described by Preparata and Shamos, *Computational Geometry:
//! An Introduction* (1985), with exact predicates replacing tolerance tests.

use std::cmp::Ordering;

use hyperlimit::{Point3, compare_reals};
use hyperreal::Real;

use super::mesh::ExactMesh;

/// Deterministic exact ray direction used by an exact ray-parity query.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum WindingRayAxis {
    /// Positive X ray, projected to the YZ plane.
    X,
    /// Positive Y ray, projected to the XZ plane.
    Y,
    /// Positive Z ray, projected to the XY plane.
    Z,
    /// Positive ray in direction `(1, 1, 1)`.
    Xyz,
    /// Positive ray in direction `(1, 1, -1)`.
    XyNegZ,
    /// Positive ray in direction `(1, -1, 1)`.
    XNegYZ,
    /// Positive ray in direction `(-1, 1, 1)`.
    NegXYZ,
    /// Positive ray in direction `(1, 2, 3)`.
    D123,
    /// Positive ray in direction `(2, 3, 5)`.
    D235,
    /// Positive ray in direction `(3, 5, 7)`.
    D357,
}

impl WindingRayAxis {
    const ALL: [Self; 10] = [
        Self::X,
        Self::Y,
        Self::Z,
        Self::Xyz,
        Self::XyNegZ,
        Self::XNegYZ,
        Self::NegXYZ,
        Self::D123,
        Self::D235,
        Self::D357,
    ];

    const fn direction(self) -> [i64; 3] {
        match self {
            Self::X => [1, 0, 0],
            Self::Y => [0, 1, 0],
            Self::Z => [0, 0, 1],
            Self::Xyz => [1, 1, 1],
            Self::XyNegZ => [1, 1, -1],
            Self::XNegYZ => [1, -1, 1],
            Self::NegXYZ => [-1, 1, 1],
            Self::D123 => [1, 2, 3],
            Self::D235 => [2, 3, 5],
            Self::D357 => [3, 5, 7],
        }
    }
}

/// Exact point/closed-mesh winding relation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ClosedMeshWindingRelation {
    /// The target mesh is not a closed two-manifold under exact validation
    /// facts.
    NotClosed,
    /// Odd ray-parity crossing count certified the point as inside.
    Inside,
    /// Even ray-parity crossing count certified the point as outside.
    Outside,
    /// The point lies on a source triangle.
    Boundary,
    /// Every attempted exact ray was degenerate or depended on an undecided
    /// scalar comparison.
    Unknown,
}

/// Exact point/closed-mesh winding report.
#[derive(Clone, Debug, PartialEq)]
pub struct PointMeshWindingReport {
    /// Final certified or unresolved relation.
    pub relation: ClosedMeshWindingRelation,
    /// Ray direction that produced the retained parity decision or boundary hit.
    pub axis: Option<WindingRayAxis>,
    /// Number of rays attempted.
    pub tested_axes: usize,
    /// Number of source triangles scanned by each attempted ray.
    pub triangle_count: usize,
    /// Certified positive ray/triangle crossings on the selected axis.
    pub crossings: usize,
    /// Triangles whose exact projected relation put the query point on the
    /// triangle boundary.
    pub boundary_hits: usize,
    /// Triangles where the ray hit a projected edge/vertex and that axis had
    /// to be rejected.
    pub degenerate_hits: usize,
    /// Triangles parallel to the selected ray axis.
    pub parallel_faces: usize,
    /// Triangles whose comparison state was undecidable for the selected ray.
    pub unknown_hits: usize,
}

impl PointMeshWindingReport {
    /// Return whether this report gives a decided inside/outside/boundary
    /// relation for a closed mesh.
    pub const fn is_decided(&self) -> bool {
        matches!(
            self.relation,
            ClosedMeshWindingRelation::Inside
                | ClosedMeshWindingRelation::Outside
                | ClosedMeshWindingRelation::Boundary
        )
    }

    /// Validate local report consistency.
    ///
    /// This audits the report shape, not the source mesh. Inside/outside
    /// relations must carry an axis and parity-compatible crossing count;
    /// unknown reports must retain evidence that all attempted ray directions were
    /// blocked by degeneracy or undecidable comparisons. The split mirrors
    /// unresolved states are separate public values, not nearby booleans.
    pub fn validate(&self) -> Result<(), WindingReportError> {
        if self.tested_axes > WindingRayAxis::ALL.len() {
            return Err(WindingReportError::InvalidAxisCount);
        }
        match self.relation {
            ClosedMeshWindingRelation::NotClosed => {
                if self.axis.is_some()
                    || self.tested_axes != 0
                    || self.crossings != 0
                    || self.boundary_hits != 0
                    || self.degenerate_hits != 0
                    || self.parallel_faces != 0
                    || self.unknown_hits != 0
                {
                    Err(WindingReportError::StatusEvidenceMismatch)
                } else {
                    Ok(())
                }
            }
            ClosedMeshWindingRelation::Inside => {
                self.axis.ok_or(WindingReportError::MissingAxis)?;
                if self.crossings % 2 == 1 && self.boundary_hits == 0 && self.unknown_hits == 0 {
                    Ok(())
                } else {
                    Err(WindingReportError::StatusEvidenceMismatch)
                }
            }
            ClosedMeshWindingRelation::Outside => {
                self.axis.ok_or(WindingReportError::MissingAxis)?;
                if self.crossings.is_multiple_of(2)
                    && self.boundary_hits == 0
                    && self.unknown_hits == 0
                {
                    Ok(())
                } else {
                    Err(WindingReportError::StatusEvidenceMismatch)
                }
            }
            ClosedMeshWindingRelation::Boundary => {
                self.axis.ok_or(WindingReportError::MissingAxis)?;
                if self.boundary_hits != 0 {
                    Ok(())
                } else {
                    Err(WindingReportError::StatusEvidenceMismatch)
                }
            }
            ClosedMeshWindingRelation::Unknown => {
                if self.axis.is_none()
                    && self.tested_axes == WindingRayAxis::ALL.len()
                    && (self.degenerate_hits != 0 || self.unknown_hits != 0)
                {
                    Ok(())
                } else {
                    Err(WindingReportError::StatusEvidenceMismatch)
                }
            }
        }
    }

    /// Validate this report by recomputing it from the source point and mesh.
    pub fn validate_against_sources(
        &self,
        point: &Point3,
        mesh: &ExactMesh,
    ) -> Result<(), WindingReportError> {
        self.validate()?;
        if self == &classify_point_against_closed_mesh_winding_report(point, mesh) {
            Ok(())
        } else {
            Err(WindingReportError::SourceReplayMismatch)
        }
    }
}

/// Exact relation between every subject vertex and a closed target mesh.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ClosedMeshWindingMeshRelation {
    /// The target mesh is not closed.
    NotClosed,
    /// Every subject vertex is strictly inside the target mesh.
    StrictlyInside,
    /// Every subject vertex is outside the target mesh.
    Outside,
    /// Subject vertices touch the target boundary or mix inside/outside states.
    BoundaryOrMixed,
    /// At least one subject vertex could not be classified by exact parity.
    Unknown,
}

/// Report for classifying a subject mesh's vertices against a closed target.
#[derive(Clone, Debug, PartialEq)]
pub struct ClosedMeshWindingMeshReport {
    /// Summary relation derived from retained per-vertex reports.
    pub relation: ClosedMeshWindingMeshRelation,
    /// Whether the target mesh was a closed two-manifold.
    pub target_closed: bool,
    /// Number of subject vertices checked.
    pub subject_vertex_count: usize,
    /// Per-subject-vertex winding reports.
    pub vertices: Vec<PointMeshWindingReport>,
}

impl ClosedMeshWindingMeshReport {
    /// Validate that the summary relation follows from retained vertex reports.
    pub fn validate(&self) -> Result<(), WindingReportError> {
        if !self.target_closed {
            return if self.relation == ClosedMeshWindingMeshRelation::NotClosed
                && self.vertices.is_empty()
            {
                Ok(())
            } else {
                Err(WindingReportError::StatusEvidenceMismatch)
            };
        }
        if self.vertices.len() != self.subject_vertex_count {
            return Err(WindingReportError::VertexCountMismatch);
        }
        let mut inside = 0_usize;
        let mut outside = 0_usize;
        let mut boundary = 0_usize;
        for vertex in &self.vertices {
            vertex.validate()?;
            match vertex.relation {
                ClosedMeshWindingRelation::Inside => inside += 1,
                ClosedMeshWindingRelation::Outside => outside += 1,
                ClosedMeshWindingRelation::Boundary => boundary += 1,
                ClosedMeshWindingRelation::Unknown => {
                    return if self.relation == ClosedMeshWindingMeshRelation::Unknown {
                        Ok(())
                    } else {
                        Err(WindingReportError::StatusEvidenceMismatch)
                    };
                }
                ClosedMeshWindingRelation::NotClosed => {
                    return Err(WindingReportError::StatusEvidenceMismatch);
                }
            }
        }
        let derived = match (inside, outside, boundary) {
            (_, 0, 0) if inside == self.subject_vertex_count => {
                ClosedMeshWindingMeshRelation::StrictlyInside
            }
            (0, _, 0) if outside == self.subject_vertex_count => {
                ClosedMeshWindingMeshRelation::Outside
            }
            _ => ClosedMeshWindingMeshRelation::BoundaryOrMixed,
        };
        if self.relation == derived {
            Ok(())
        } else {
            Err(WindingReportError::StatusEvidenceMismatch)
        }
    }

    /// Validate this retained report against its subject and target meshes.
    pub fn validate_against_sources(
        &self,
        subject: &ExactMesh,
        target: &ExactMesh,
    ) -> Result<(), WindingReportError> {
        self.validate()?;
        if self == &classify_mesh_vertices_against_closed_mesh_winding_report(subject, target) {
            Ok(())
        } else {
            Err(WindingReportError::SourceReplayMismatch)
        }
    }
}

/// Validation or source-replay failure for winding reports.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum WindingReportError {
    /// A report retained an impossible number of attempted ray directions.
    InvalidAxisCount,
    /// A decided relation did not retain the axis that decided it.
    MissingAxis,
    /// Status and retained crossing/dependency counts disagree.
    StatusEvidenceMismatch,
    /// Mesh summary vertex count does not match retained per-vertex reports.
    VertexCountMismatch,
    /// Recomputed source evidence did not match the retained report.
    SourceReplayMismatch,
}

/// Classify `point` against a closed triangle mesh by exact ray parity.
pub fn classify_point_against_closed_mesh_winding(
    point: &Point3,
    mesh: &ExactMesh,
) -> ClosedMeshWindingRelation {
    classify_point_against_closed_mesh_winding_report(point, mesh).relation
}

/// Classify `point` against a closed triangle mesh and retain parity evidence.
///
/// The classifier tries positive X, Y, and Z rays first, then deterministic
/// primitive non-axis rays. For each ray, triangle tests are carried out in an
/// exact quotient projection whose kernel is the ray direction and the 3D plane
/// intersection sign is compared exactly. If a ray hits a projected
/// edge/vertex, that ray is rejected rather than "nudged" by an epsilon;
/// another exact ray may still decide the relation. This keeps the report
/// stable and replayable: a decided report includes the exact ray, crossing
/// count, and blocker counts used to reach the decision.
pub fn classify_point_against_closed_mesh_winding_report(
    point: &Point3,
    mesh: &ExactMesh,
) -> PointMeshWindingReport {
    if !mesh.facts().mesh.closed_manifold {
        return PointMeshWindingReport {
            relation: ClosedMeshWindingRelation::NotClosed,
            axis: None,
            tested_axes: 0,
            triangle_count: mesh.triangles().len(),
            crossings: 0,
            boundary_hits: 0,
            degenerate_hits: 0,
            parallel_faces: 0,
            unknown_hits: 0,
        };
    }

    let triangles = mesh
        .triangles()
        .iter()
        .map(|triangle| {
            let [a, b, c] = triangle.0;
            [
                mesh.vertices()[a].clone(),
                mesh.vertices()[b].clone(),
                mesh.vertices()[c].clone(),
            ]
        })
        .collect::<Vec<_>>();

    let mut last_degenerate = 0_usize;
    let mut last_unknown = 0_usize;
    for (tested, axis) in WindingRayAxis::ALL.into_iter().enumerate() {
        let axis_report = classify_axis(point, &triangles, axis);
        if axis_report.boundary_hits != 0 {
            return PointMeshWindingReport {
                relation: ClosedMeshWindingRelation::Boundary,
                axis: Some(axis),
                tested_axes: tested + 1,
                triangle_count: triangles.len(),
                crossings: axis_report.crossings,
                boundary_hits: axis_report.boundary_hits,
                degenerate_hits: axis_report.degenerate_hits,
                parallel_faces: axis_report.parallel_faces,
                unknown_hits: axis_report.unknown_hits,
            };
        }
        if axis_report.degenerate_hits == 0 && axis_report.unknown_hits == 0 {
            return PointMeshWindingReport {
                relation: if axis_report.crossings % 2 == 1 {
                    ClosedMeshWindingRelation::Inside
                } else {
                    ClosedMeshWindingRelation::Outside
                },
                axis: Some(axis),
                tested_axes: tested + 1,
                triangle_count: triangles.len(),
                crossings: axis_report.crossings,
                boundary_hits: 0,
                degenerate_hits: 0,
                parallel_faces: axis_report.parallel_faces,
                unknown_hits: 0,
            };
        }
        last_degenerate += axis_report.degenerate_hits;
        last_unknown += axis_report.unknown_hits;
    }

    if let Some(boundary_hits) = exact_boundary_hits(point, &triangles)
        && boundary_hits != 0
    {
        return PointMeshWindingReport {
            relation: ClosedMeshWindingRelation::Boundary,
            axis: Some(WindingRayAxis::X),
            tested_axes: WindingRayAxis::ALL.len(),
            triangle_count: triangles.len(),
            crossings: 0,
            boundary_hits,
            degenerate_hits: last_degenerate,
            parallel_faces: 0,
            unknown_hits: last_unknown,
        };
    }

    PointMeshWindingReport {
        relation: ClosedMeshWindingRelation::Unknown,
        axis: None,
        tested_axes: WindingRayAxis::ALL.len(),
        triangle_count: triangles.len(),
        crossings: 0,
        boundary_hits: 0,
        degenerate_hits: last_degenerate,
        parallel_faces: 0,
        unknown_hits: last_unknown,
    }
}

fn exact_boundary_hits(point: &Point3, triangles: &[[Point3; 3]]) -> Option<usize> {
    let mut hits = 0usize;
    let mut unresolved = false;
    for triangle in triangles {
        match point_on_closed_triangle(point, triangle) {
            Some(true) => hits += 1,
            Some(false) => {}
            None => unresolved = true,
        }
    }
    if hits != 0 {
        Some(hits)
    } else if unresolved {
        None
    } else {
        Some(0)
    }
}

fn point_on_closed_triangle(point: &Point3, triangle: &[Point3; 3]) -> Option<bool> {
    match point_on_triangle_plane(point, triangle) {
        Some(true) => {}
        Some(false) => return Some(false),
        None => return None,
    }

    let mut unresolved = false;
    for axis in WindingRayAxis::ALL {
        match projected_point_relation(point, triangle, axis) {
            ProjectedPointRelation::Inside | ProjectedPointRelation::Boundary => {
                return Some(true);
            }
            ProjectedPointRelation::Outside => return Some(false),
            ProjectedPointRelation::Unknown => unresolved = true,
        }
    }
    if unresolved { None } else { Some(false) }
}

/// Classify every vertex of `subject` against a closed target mesh.
pub fn classify_mesh_vertices_against_closed_mesh_winding(
    subject: &ExactMesh,
    target: &ExactMesh,
) -> ClosedMeshWindingMeshRelation {
    classify_mesh_vertices_against_closed_mesh_winding_report(subject, target).relation
}

/// Classify every vertex of `subject` against a closed target mesh and retain
/// exact ray-parity reports.
pub fn classify_mesh_vertices_against_closed_mesh_winding_report(
    subject: &ExactMesh,
    target: &ExactMesh,
) -> ClosedMeshWindingMeshReport {
    if !target.facts().mesh.closed_manifold {
        return ClosedMeshWindingMeshReport {
            relation: ClosedMeshWindingMeshRelation::NotClosed,
            target_closed: false,
            subject_vertex_count: subject.vertices().len(),
            vertices: Vec::new(),
        };
    }

    let mut inside = 0_usize;
    let mut outside = 0_usize;
    let mut boundary = 0_usize;
    let mut vertices = Vec::with_capacity(subject.vertices().len());
    for vertex in subject.vertices() {
        let report = classify_point_against_closed_mesh_winding_report(&vertex.clone(), target);
        match report.relation {
            ClosedMeshWindingRelation::Inside => inside += 1,
            ClosedMeshWindingRelation::Outside => outside += 1,
            ClosedMeshWindingRelation::Boundary => boundary += 1,
            ClosedMeshWindingRelation::Unknown => {
                vertices.push(report);
                return ClosedMeshWindingMeshReport {
                    relation: ClosedMeshWindingMeshRelation::Unknown,
                    target_closed: true,
                    subject_vertex_count: subject.vertices().len(),
                    vertices,
                };
            }
            ClosedMeshWindingRelation::NotClosed => unreachable!("target closure checked above"),
        }
        vertices.push(report);
    }

    let relation = match (inside, outside, boundary) {
        (_, 0, 0) if inside == subject.vertices().len() => {
            ClosedMeshWindingMeshRelation::StrictlyInside
        }
        (0, _, 0) if outside == subject.vertices().len() => ClosedMeshWindingMeshRelation::Outside,
        _ => ClosedMeshWindingMeshRelation::BoundaryOrMixed,
    };
    ClosedMeshWindingMeshReport {
        relation,
        target_closed: true,
        subject_vertex_count: subject.vertices().len(),
        vertices,
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct AxisParityReport {
    crossings: usize,
    boundary_hits: usize,
    degenerate_hits: usize,
    parallel_faces: usize,
    unknown_hits: usize,
}

fn classify_axis(
    point: &Point3,
    triangles: &[[Point3; 3]],
    axis: WindingRayAxis,
) -> AxisParityReport {
    let mut report = AxisParityReport::default();
    for triangle in triangles {
        match classify_ray_triangle(point, triangle, axis) {
            RayTriangleRelation::Crossing => report.crossings += 1,
            RayTriangleRelation::Boundary => report.boundary_hits += 1,
            RayTriangleRelation::Degenerate => report.degenerate_hits += 1,
            RayTriangleRelation::Parallel => report.parallel_faces += 1,
            RayTriangleRelation::NoHit => {}
            RayTriangleRelation::Unknown => report.unknown_hits += 1,
        }
    }
    report
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum RayTriangleRelation {
    Crossing,
    Boundary,
    Degenerate,
    Parallel,
    NoHit,
    Unknown,
}

fn classify_ray_triangle(
    point: &Point3,
    triangle: &[Point3; 3],
    axis: WindingRayAxis,
) -> RayTriangleRelation {
    let area = projected_orient(&triangle[0], &triangle[1], &triangle[2], axis);
    let Some(area_sign) = sign(&area) else {
        return RayTriangleRelation::Unknown;
    };
    if area_sign == RealSign::Zero {
        return RayTriangleRelation::Parallel;
    }

    let projected = projected_point_relation(point, triangle, axis);
    match projected {
        ProjectedPointRelation::Outside => RayTriangleRelation::NoHit,
        ProjectedPointRelation::Boundary => {
            if point_on_triangle_plane(point, triangle) == Some(true) {
                RayTriangleRelation::Boundary
            } else {
                RayTriangleRelation::Degenerate
            }
        }
        ProjectedPointRelation::Inside => {
            if point_on_triangle_plane(point, triangle) == Some(true) {
                return RayTriangleRelation::Boundary;
            }
            let Some(ray_sign) = ray_parameter_sign(point, triangle, axis) else {
                return RayTriangleRelation::Unknown;
            };
            match ray_sign {
                RealSign::Positive => RayTriangleRelation::Crossing,
                RealSign::Negative => RayTriangleRelation::NoHit,
                RealSign::Zero => RayTriangleRelation::Boundary,
            }
        }
        ProjectedPointRelation::Unknown => RayTriangleRelation::Unknown,
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ProjectedPointRelation {
    Inside,
    Boundary,
    Outside,
    Unknown,
}

fn projected_point_relation(
    point: &Point3,
    triangle: &[Point3; 3],
    axis: WindingRayAxis,
) -> ProjectedPointRelation {
    let area = projected_orient(&triangle[0], &triangle[1], &triangle[2], axis);
    let Some(area_sign) = sign(&area) else {
        return ProjectedPointRelation::Unknown;
    };
    if area_sign == RealSign::Zero {
        return ProjectedPointRelation::Outside;
    }
    let signs = [
        sign(&projected_orient(&triangle[0], &triangle[1], point, axis)),
        sign(&projected_orient(&triangle[1], &triangle[2], point, axis)),
        sign(&projected_orient(&triangle[2], &triangle[0], point, axis)),
    ];
    if signs.iter().any(Option::is_none) {
        return ProjectedPointRelation::Unknown;
    }
    let signs = signs.map(Option::unwrap);
    if signs.contains(&RealSign::Zero) {
        let nonzero_ok = signs
            .iter()
            .copied()
            .filter(|sign| *sign != RealSign::Zero)
            .all(|sign| sign == area_sign);
        return if nonzero_ok {
            ProjectedPointRelation::Boundary
        } else {
            ProjectedPointRelation::Outside
        };
    }
    if signs.into_iter().all(|sign| sign == area_sign) {
        ProjectedPointRelation::Inside
    } else {
        ProjectedPointRelation::Outside
    }
}

fn point_on_triangle_plane(point: &Point3, triangle: &[Point3; 3]) -> Option<bool> {
    let normal = normal(triangle);
    let offset = dot_point(&normal, &triangle[0]);
    let side = dot_point(&normal, point) - offset;
    sign(&side).map(|sign| sign == RealSign::Zero)
}

fn ray_parameter_sign(
    point: &Point3,
    triangle: &[Point3; 3],
    axis: WindingRayAxis,
) -> Option<RealSign> {
    let normal = normal(triangle);
    let numerator = dot_point(&normal, &triangle[0]) - dot_point(&normal, point);
    let denominator = dot_i64(&normal, axis.direction());
    let numerator_sign = sign(&numerator)?;
    let denominator_sign = sign(&denominator)?;
    match (numerator_sign, denominator_sign) {
        (RealSign::Zero, _) => Some(RealSign::Zero),
        (_, RealSign::Zero) => None,
        (RealSign::Positive, RealSign::Positive) | (RealSign::Negative, RealSign::Negative) => {
            Some(RealSign::Positive)
        }
        (RealSign::Positive, RealSign::Negative) | (RealSign::Negative, RealSign::Positive) => {
            Some(RealSign::Negative)
        }
    }
}

fn projected_orient(a: &Point3, b: &Point3, c: &Point3, axis: WindingRayAxis) -> Real {
    let (au, av) = project(a, axis);
    let (bu, bv) = project(b, axis);
    let (cu, cv) = project(c, axis);
    (bu - au.clone()) * (cv - av.clone()) - (bv - av) * (cu - au)
}

fn project(point: &Point3, axis: WindingRayAxis) -> (Real, Real) {
    let [dx, dy, dz] = axis.direction();
    let basis_u = if dx == 0 && dy == 0 {
        [1, 0, 0]
    } else {
        [dy, -dx, 0]
    };
    let basis_v = cross_i64([dx, dy, dz], basis_u);
    (dot_point_i64(point, basis_u), dot_point_i64(point, basis_v))
}

fn normal(triangle: &[Point3; 3]) -> [Real; 3] {
    let ax = triangle[0].x.clone();
    let ay = triangle[0].y.clone();
    let az = triangle[0].z.clone();
    let ux = triangle[1].x.clone() - ax.clone();
    let uy = triangle[1].y.clone() - ay.clone();
    let uz = triangle[1].z.clone() - az.clone();
    let vx = triangle[2].x.clone() - ax;
    let vy = triangle[2].y.clone() - ay;
    let vz = triangle[2].z.clone() - az;
    [
        uy.clone() * vz.clone() - uz.clone() * vy.clone(),
        uz * vx.clone() - ux.clone() * vz,
        ux * vy - uy * vx,
    ]
}

fn dot_point(normal: &[Real; 3], point: &Point3) -> Real {
    normal[0].clone() * point.x.clone()
        + normal[1].clone() * point.y.clone()
        + normal[2].clone() * point.z.clone()
}

fn dot_i64(left: &[Real; 3], right: [i64; 3]) -> Real {
    left[0].clone() * Real::from(right[0])
        + left[1].clone() * Real::from(right[1])
        + left[2].clone() * Real::from(right[2])
}

fn dot_point_i64(point: &Point3, coeffs: [i64; 3]) -> Real {
    point.x.clone() * Real::from(coeffs[0])
        + point.y.clone() * Real::from(coeffs[1])
        + point.z.clone() * Real::from(coeffs[2])
}

const fn cross_i64(left: [i64; 3], right: [i64; 3]) -> [i64; 3] {
    [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum RealSign {
    Negative,
    Zero,
    Positive,
}

fn sign(value: &Real) -> Option<RealSign> {
    match compare_reals(value, &Real::from(0)).value()? {
        Ordering::Less => Some(RealSign::Negative),
        Ordering::Equal => Some(RealSign::Zero),
        Ordering::Greater => Some(RealSign::Positive),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    fn cube_with_centered_positive_face_diagonals() -> ExactMesh {
        ExactMesh::from_i64_triangles(
            &[
                -1, -1, -1, //
                1, -1, -1, //
                1, 1, -1, //
                -1, 1, -1, //
                -1, -1, 1, //
                1, -1, 1, //
                1, 1, 1, //
                -1, 1, 1, //
            ],
            &[
                0, 3, 2, 0, 2, 1, // -Z
                4, 5, 6, 4, 6, 7, // +Z, diagonal through +Z ray hit
                0, 4, 7, 0, 7, 3, // -X
                1, 2, 6, 1, 6, 5, // +X, diagonal through +X ray hit
                0, 1, 5, 0, 5, 4, // -Y
                3, 7, 6, 3, 6, 2, // +Y, diagonal through +Y ray hit
            ],
        )
        .unwrap()
    }

    #[test]
    fn non_axis_exact_ray_classifies_when_coordinate_rays_are_degenerate() {
        let mesh = cube_with_centered_positive_face_diagonals();
        let report = classify_point_against_closed_mesh_winding_report(&p(0, 0, 0), &mesh);

        assert_eq!(report.relation, ClosedMeshWindingRelation::Inside);
        assert!(report.tested_axes > 3);
        assert!(matches!(
            report.axis,
            Some(
                WindingRayAxis::Xyz
                    | WindingRayAxis::XyNegZ
                    | WindingRayAxis::XNegYZ
                    | WindingRayAxis::NegXYZ
                    | WindingRayAxis::D123
                    | WindingRayAxis::D235
                    | WindingRayAxis::D357
            )
        ));
        report.validate_against_sources(&p(0, 0, 0), &mesh).unwrap();
    }
}
