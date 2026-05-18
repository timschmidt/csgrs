//! Certified closed-convex solid facts.
//!
//! The routines in this module are intentionally narrow: they certify a closed
//! triangular mesh as an oriented convex polyhedron and classify points or
//! vertex sets with exact oriented halfspace predicates. They do not implement
//! arbitrary winding. This follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997): application-level topology decisions
//! must be made from certified predicate facts, with unsupported cases kept
//! explicit rather than hidden behind approximate representative points.

use std::cmp::Ordering;

use hyperlimit::{PlaneSide, Point3, compare_reals, orient3d_report};

use super::mesh::ExactMesh;
use super::provenance::PredicateUse;
use super::scalar::ExactReal;

/// Certified orientation of a closed triangular surface.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ClosedMeshOrientation {
    /// The mesh is not a closed two-manifold under exact validation facts.
    NotClosed,
    /// The signed volume was certified positive.
    Positive,
    /// The signed volume was certified negative.
    Negative,
    /// The signed volume sign could not be certified or was zero.
    Unknown,
}

/// Certified convexity state for a closed triangular surface.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConvexSolidClassification {
    /// The mesh is not a closed two-manifold under exact validation facts.
    NotClosed,
    /// Every vertex is on or inside every oriented face halfspace.
    Convex,
    /// At least one vertex was certified outside an oriented face halfspace.
    NonConvex,
    /// A required orientation or halfspace predicate was undecided.
    Unknown,
}

/// Exact facts retained while certifying a closed convex solid.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ConvexSolidFacts {
    /// Certified closed-surface orientation.
    pub orientation: ClosedMeshOrientation,
    /// Certified convexity state.
    pub convexity: ConvexSolidClassification,
    /// Predicate certificates used by face/vertex halfspace tests.
    pub predicates: Vec<PredicateUse>,
}

impl ConvexSolidFacts {
    /// Return whether the mesh is certified as an oriented convex closed solid.
    pub const fn is_certified_convex(&self) -> bool {
        matches!(
            (self.orientation, self.convexity),
            (
                ClosedMeshOrientation::Positive | ClosedMeshOrientation::Negative,
                ConvexSolidClassification::Convex
            )
        )
    }

    /// Return whether every retained predicate route was proof-producing.
    pub fn all_proof_producing(&self) -> bool {
        self.predicates
            .iter()
            .copied()
            .all(PredicateUse::is_proof_producing)
    }
}

/// Certified point/solid classification with retained predicate provenance.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ConvexSolidPointClassification {
    /// Exact point/solid relation.
    pub relation: ConvexSolidPointRelation,
    /// Predicate certificates used by the halfspace tests for this point.
    pub predicates: Vec<PredicateUse>,
}

impl ConvexSolidPointClassification {
    /// Return whether every retained predicate route was proof-producing.
    pub fn all_proof_producing(&self) -> bool {
        self.predicates
            .iter()
            .copied()
            .all(PredicateUse::is_proof_producing)
    }
}

/// Exact relation between a point and a certified convex solid.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConvexSolidPointRelation {
    /// The containing mesh was not certified as a convex closed solid.
    NotCertifiedConvex,
    /// The point is strictly inside all oriented halfspaces.
    Inside,
    /// The point is on at least one boundary plane and outside none.
    Boundary,
    /// The point is certified outside at least one oriented halfspace.
    Outside,
    /// A required predicate was undecided.
    Unknown,
}

/// Exact relation between one mesh's vertices and a certified convex solid.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConvexSolidMeshRelation {
    /// The containing mesh was not certified as a convex closed solid.
    NotCertifiedConvex,
    /// Every subject vertex is strictly inside the solid.
    StrictlyInside,
    /// No subject vertex is inside the solid.
    Outside,
    /// Subject vertices touch the boundary, mix boundary/interior states, or
    /// otherwise require a full winding/surface-overlap policy.
    BoundaryOrMixed,
    /// A required predicate was undecided.
    Unknown,
}

/// Certified mesh/solid vertex classification with retained predicate provenance.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ConvexSolidMeshClassification {
    /// Exact relation between the subject vertices and the convex solid.
    pub relation: ConvexSolidMeshRelation,
    /// Convexity facts certified for the containing solid.
    pub solid_facts: ConvexSolidFacts,
    /// Per-subject-vertex classifications.
    pub vertices: Vec<ConvexSolidPointClassification>,
}

impl ConvexSolidMeshClassification {
    /// Return whether every retained predicate route was proof-producing.
    pub fn all_proof_producing(&self) -> bool {
        self.solid_facts.all_proof_producing()
            && self
                .vertices
                .iter()
                .all(ConvexSolidPointClassification::all_proof_producing)
    }
}

/// Certify whether a closed triangular mesh is an oriented convex solid.
///
/// Convexity is tested by replaying every mesh vertex against every oriented
/// face plane with `hyperlimit::orient3d_report`. `hyperlimit` uses the
/// translated determinant convention where the canonical tetrahedron
/// `(0,0,0),(1,0,0),(0,1,0),(0,0,1)` has negative orientation; with the signed
/// volume convention used here, interior points of a positively oriented
/// closed surface therefore lie on the above side of every face. The signed
/// volume orientation is exact `Real` arithmetic and is compared through
/// `hyperlimit::compare_reals`, keeping Yap's exact predicate boundary intact.
pub fn certify_convex_solid(mesh: &ExactMesh) -> ConvexSolidFacts {
    if !mesh.facts().mesh.closed_manifold {
        return ConvexSolidFacts {
            orientation: ClosedMeshOrientation::NotClosed,
            convexity: ConvexSolidClassification::NotClosed,
            predicates: Vec::new(),
        };
    }

    let orientation = mesh_orientation(mesh);
    if !matches!(
        orientation,
        ClosedMeshOrientation::Positive | ClosedMeshOrientation::Negative
    ) {
        return ConvexSolidFacts {
            orientation,
            convexity: ConvexSolidClassification::Unknown,
            predicates: Vec::new(),
        };
    }

    let mut predicates = Vec::new();
    let mut saw_unknown = false;
    for triangle in mesh.triangles() {
        let tri = triangle.0;
        let a = mesh.vertices()[tri[0]].to_hyperlimit_point();
        let b = mesh.vertices()[tri[1]].to_hyperlimit_point();
        let c = mesh.vertices()[tri[2]].to_hyperlimit_point();

        for (vertex, point) in mesh.vertices().iter().enumerate() {
            if tri.contains(&vertex) {
                continue;
            }
            let report = orient3d_report(&a, &b, &c, &point.to_hyperlimit_point());
            predicates.push(PredicateUse::from_certificate(report.certificate));
            let Some(side) = report.value().map(PlaneSide::from) else {
                saw_unknown = true;
                continue;
            };
            if side_is_outside(orientation, side) {
                return ConvexSolidFacts {
                    orientation,
                    convexity: ConvexSolidClassification::NonConvex,
                    predicates,
                };
            }
        }
    }

    ConvexSolidFacts {
        orientation,
        convexity: if saw_unknown {
            ConvexSolidClassification::Unknown
        } else {
            ConvexSolidClassification::Convex
        },
        predicates,
    }
}

/// Classify one point against a certified convex solid.
pub fn classify_point_against_convex_solid(
    point: &Point3,
    solid: &ExactMesh,
) -> ConvexSolidPointRelation {
    classify_point_against_convex_solid_report(point, solid).relation
}

/// Classify one point against a certified convex solid and retain predicates.
///
/// This is the auditable form of [`classify_point_against_convex_solid`].
/// Each face halfspace query records the `hyperlimit::orient3d_report`
/// certificate that drove the relation. Keeping those certificates near the
/// point/solid decision follows Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997): topological shortcuts should expose
/// the exact predicates they consumed rather than returning only a collapsed
/// boolean-like answer.
pub fn classify_point_against_convex_solid_report(
    point: &Point3,
    solid: &ExactMesh,
) -> ConvexSolidPointClassification {
    let facts = certify_convex_solid(solid);
    classify_point_with_convex_facts_report(point, solid, &facts)
}

/// Classify every vertex of `subject` against a certified convex solid.
///
/// This is a containment precondition for simple named-boolean shortcuts. It
/// is not a substitute for general winding: boundary and mixed relations remain
/// explicit so coplanar overlaps, shared faces, and partial intersections are
/// rejected by higher-level boolean policy.
pub fn classify_mesh_vertices_against_convex_solid(
    subject: &ExactMesh,
    solid: &ExactMesh,
) -> ConvexSolidMeshRelation {
    classify_mesh_vertices_against_convex_solid_report(subject, solid).relation
}

/// Classify every vertex of `subject` against a convex solid and retain predicates.
///
/// This report-returning API is the boolean-shortcut audit artifact. It
/// separates the solid certification predicates from the per-vertex halfspace
/// predicates so callers can inspect whether a containment/disjoint shortcut
/// was entirely proof-producing. This is the Yap-style exact computation
/// contract used throughout the port: predicates and uncertainty stay explicit
/// at API boundaries.
pub fn classify_mesh_vertices_against_convex_solid_report(
    subject: &ExactMesh,
    solid: &ExactMesh,
) -> ConvexSolidMeshClassification {
    let facts = certify_convex_solid(solid);
    if !facts.is_certified_convex() {
        return ConvexSolidMeshClassification {
            relation: ConvexSolidMeshRelation::NotCertifiedConvex,
            solid_facts: facts,
            vertices: Vec::new(),
        };
    }

    let mut inside = 0_usize;
    let mut boundary = 0_usize;
    let mut outside = 0_usize;
    let mut vertices = Vec::with_capacity(subject.vertices().len());
    for vertex in subject.vertices() {
        let classification =
            classify_point_with_convex_facts_report(&vertex.to_hyperlimit_point(), solid, &facts);
        match classification.relation {
            ConvexSolidPointRelation::Inside => inside += 1,
            ConvexSolidPointRelation::Boundary => boundary += 1,
            ConvexSolidPointRelation::Outside => outside += 1,
            ConvexSolidPointRelation::Unknown => {
                vertices.push(classification);
                return ConvexSolidMeshClassification {
                    relation: ConvexSolidMeshRelation::Unknown,
                    solid_facts: facts,
                    vertices,
                };
            }
            ConvexSolidPointRelation::NotCertifiedConvex => {
                vertices.push(classification);
                return ConvexSolidMeshClassification {
                    relation: ConvexSolidMeshRelation::NotCertifiedConvex,
                    solid_facts: facts,
                    vertices,
                };
            }
        }
        vertices.push(classification);
    }

    let relation = match (inside, boundary, outside) {
        (_, 0, 0) if inside == subject.vertices().len() => ConvexSolidMeshRelation::StrictlyInside,
        (0, 0, _) => ConvexSolidMeshRelation::Outside,
        _ => ConvexSolidMeshRelation::BoundaryOrMixed,
    };
    ConvexSolidMeshClassification {
        relation,
        solid_facts: facts,
        vertices,
    }
}

fn classify_point_with_convex_facts_report(
    point: &Point3,
    solid: &ExactMesh,
    facts: &ConvexSolidFacts,
) -> ConvexSolidPointClassification {
    if !facts.is_certified_convex() {
        return ConvexSolidPointClassification {
            relation: ConvexSolidPointRelation::NotCertifiedConvex,
            predicates: Vec::new(),
        };
    }

    let mut touches_boundary = false;
    let mut predicates = Vec::with_capacity(solid.triangles().len());
    for triangle in solid.triangles() {
        let tri = triangle.0;
        let a = solid.vertices()[tri[0]].to_hyperlimit_point();
        let b = solid.vertices()[tri[1]].to_hyperlimit_point();
        let c = solid.vertices()[tri[2]].to_hyperlimit_point();
        let report = orient3d_report(&a, &b, &c, point);
        predicates.push(PredicateUse::from_certificate(report.certificate));
        let Some(side) = report.value().map(PlaneSide::from) else {
            return ConvexSolidPointClassification {
                relation: ConvexSolidPointRelation::Unknown,
                predicates,
            };
        };
        if side_is_outside(facts.orientation, side) {
            return ConvexSolidPointClassification {
                relation: ConvexSolidPointRelation::Outside,
                predicates,
            };
        }
        touches_boundary |= side == PlaneSide::On;
    }

    let relation = if touches_boundary {
        ConvexSolidPointRelation::Boundary
    } else {
        ConvexSolidPointRelation::Inside
    };
    ConvexSolidPointClassification {
        relation,
        predicates,
    }
}

fn mesh_orientation(mesh: &ExactMesh) -> ClosedMeshOrientation {
    let signed_volume = mesh
        .triangles()
        .iter()
        .map(|triangle| {
            let tri = triangle.0;
            determinant_from_origin(
                &mesh.vertices()[tri[0]].to_hyperlimit_point(),
                &mesh.vertices()[tri[1]].to_hyperlimit_point(),
                &mesh.vertices()[tri[2]].to_hyperlimit_point(),
            )
        })
        .fold(ExactReal::from(0), |sum, det| &sum + &det);

    match compare_reals(&signed_volume, &ExactReal::from(0)).value() {
        Some(Ordering::Greater) => ClosedMeshOrientation::Positive,
        Some(Ordering::Less) => ClosedMeshOrientation::Negative,
        _ => ClosedMeshOrientation::Unknown,
    }
}

fn determinant_from_origin(a: &Point3, b: &Point3, c: &Point3) -> ExactReal {
    let by_cz = &b.y * &c.z;
    let bz_cy = &b.z * &c.y;
    let bx_cz = &b.x * &c.z;
    let bz_cx = &b.z * &c.x;
    let bx_cy = &b.x * &c.y;
    let by_cx = &b.y * &c.x;

    let x_minor = &by_cz - &bz_cy;
    let y_minor = &bx_cz - &bz_cx;
    let z_minor = &bx_cy - &by_cx;

    let x_term = &a.x * &x_minor;
    let y_term = &a.y * &y_minor;
    let z_term = &a.z * &z_minor;

    &(&x_term - &y_term) + &z_term
}

fn side_is_outside(orientation: ClosedMeshOrientation, side: PlaneSide) -> bool {
    matches!(
        (orientation, side),
        (ClosedMeshOrientation::Positive, PlaneSide::Below)
            | (ClosedMeshOrientation::Negative, PlaneSide::Above)
    )
}
