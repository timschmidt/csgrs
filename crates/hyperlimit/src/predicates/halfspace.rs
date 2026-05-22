//! Exact feasibility certificates for small 3D halfspace systems.
//!
//! A system is represented by oriented planes with the convention
//! `plane.normal . point + plane.offset <= 0`. This module intentionally
//! returns a witness-bearing report instead of a bare boolean: downstream mesh,
//! voxel, packing, and solver crates can replay the candidate through
//! point-plane predicates before trusting a combinatorial decision.
//!
//! The implementation is a deterministic low-dimensional active-set search. If
//! a nonempty closed convex polyhedron exists, the Euclidean projection of the
//! origin onto it is characterized by active constraints, and in 3D a basis of
//! at most three active planes is enough to recover a candidate. This is the
//! same fixed-dimension LP viewpoint used by Seidel, "Small-Dimensional Linear
//! Programming and Convex Hulls Made Easy," *Discrete & Computational
//! Geometry* 6 (1991), while preserving Yap's exact-geometric-computation
//! boundary: every candidate is built over `Real` and then certified by exact
//! predicates; see Yap, "Towards Exact Geometric Computation," *Computational
//! Geometry* 7.1-2 (1997).

use hyperreal::Real;

use crate::classify::{HalfspaceFeasibility, PlaneSide};
use crate::geometry::{Plane3, Point3, intersect_three_planes};
use crate::plane::classify_point_plane_with_policy;
use crate::predicate::{
    Certainty, Escalation, PredicateOutcome, PredicatePolicy, RefinementNeed, Sign,
};
use crate::real::{add_ref, mul_ref, sub_ref};
use crate::resolve::resolve_real_sign;

/// Feasibility report for a closed 3D halfspace system.
#[derive(Clone, Debug, PartialEq)]
pub struct HalfspaceFeasibilityReport {
    /// Feasible or infeasible status.
    pub status: HalfspaceFeasibility,
    /// Exact point satisfying every halfspace when feasible.
    pub witness: Option<Point3>,
    /// Active plane indices used to construct [`Self::witness`].
    ///
    /// Empty entries mean the witness came from a lower-dimensional active set:
    /// no active planes for the origin, one active plane for a plane
    /// projection, two for a line projection, or three for a vertex.
    pub active_planes: [Option<usize>; 3],
}

impl HalfspaceFeasibilityReport {
    /// Construct a feasible report.
    pub fn feasible(witness: Point3, active_planes: [Option<usize>; 3]) -> Self {
        Self {
            status: HalfspaceFeasibility::Feasible,
            witness: Some(witness),
            active_planes,
        }
    }

    /// Construct an infeasible report.
    pub const fn infeasible() -> Self {
        Self {
            status: HalfspaceFeasibility::Infeasible,
            witness: None,
            active_planes: [None, None, None],
        }
    }

    /// Return whether the report found a feasible witness.
    pub const fn is_feasible(&self) -> bool {
        matches!(self.status, HalfspaceFeasibility::Feasible)
    }

    /// Replay the witness against the source planes.
    ///
    /// Infeasible reports have no compact Farkas certificate yet, so this
    /// method validates only structural consistency for infeasible outcomes.
    pub fn validate_against_planes(
        &self,
        planes: &[Plane3],
        policy: PredicatePolicy,
    ) -> PredicateOutcome<bool> {
        match (&self.status, &self.witness) {
            (HalfspaceFeasibility::Feasible, Some(witness)) => {
                point_satisfies_halfspaces(witness, planes, policy)
            }
            (HalfspaceFeasibility::Infeasible, None) => {
                PredicateOutcome::decided(true, Certainty::Exact, Escalation::Structural)
            }
            _ => PredicateOutcome::decided(false, Certainty::Exact, Escalation::Structural),
        }
    }
}

/// Decide feasibility of `normal . point + offset <= 0` halfspaces.
pub fn classify_halfspace_feasibility3(
    planes: &[Plane3],
) -> PredicateOutcome<HalfspaceFeasibilityReport> {
    classify_halfspace_feasibility3_with_policy(planes, PredicatePolicy::default())
}

/// Decide feasibility of 3D halfspaces with an explicit predicate policy.
///
/// The search checks the origin, closest points on every plane, closest points
/// on every nonparallel plane pair, and every finite triple-plane vertex. Each
/// candidate is accepted only after all halfspace predicates certify
/// `Below | On`. Infeasibility is reported only when every active-set candidate
/// and every replay comparison is exactly decided under the supplied policy.
pub fn classify_halfspace_feasibility3_with_policy(
    planes: &[Plane3],
    policy: PredicatePolicy,
) -> PredicateOutcome<HalfspaceFeasibilityReport> {
    crate::trace_dispatch!("hyperlimit", "halfspace_feasibility3", "active-set-search");

    let origin = Point3::new(Real::from(0), Real::from(0), Real::from(0));
    if let Some(outcome) = accept_candidate(&origin, [None, None, None], planes, policy) {
        return outcome;
    }

    for (index, plane) in planes.iter().enumerate() {
        let candidate = match closest_point_on_plane(plane, policy) {
            CandidateConstruction::Point(point) => point,
            CandidateConstruction::Skip => continue,
            CandidateConstruction::Unknown { needed, stage } => {
                return PredicateOutcome::unknown(needed, stage);
            }
        };
        if let Some(outcome) =
            accept_candidate(&candidate, [Some(index), None, None], planes, policy)
        {
            return outcome;
        }
    }

    for first in 0..planes.len() {
        for second in first + 1..planes.len() {
            let candidate =
                match closest_point_on_plane_pair(&planes[first], &planes[second], policy) {
                    CandidateConstruction::Point(point) => point,
                    CandidateConstruction::Skip => continue,
                    CandidateConstruction::Unknown { needed, stage } => {
                        return PredicateOutcome::unknown(needed, stage);
                    }
                };
            if let Some(outcome) = accept_candidate(
                &candidate,
                [Some(first), Some(second), None],
                planes,
                policy,
            ) {
                return outcome;
            }
        }
    }

    for first in 0..planes.len() {
        for second in first + 1..planes.len() {
            for third in second + 1..planes.len() {
                let homogeneous =
                    intersect_three_planes(&planes[first], &planes[second], &planes[third]);
                let candidate = match homogeneous.to_affine_point() {
                    Ok(point) => point,
                    Err(_) => continue,
                };
                if let Some(outcome) = accept_candidate(
                    &candidate,
                    [Some(first), Some(second), Some(third)],
                    planes,
                    policy,
                ) {
                    return outcome;
                }
            }
        }
    }

    PredicateOutcome::decided(
        HalfspaceFeasibilityReport::infeasible(),
        Certainty::Exact,
        Escalation::Exact,
    )
}

enum CandidateConstruction {
    Point(Point3),
    Skip,
    Unknown {
        needed: RefinementNeed,
        stage: Escalation,
    },
}

fn accept_candidate(
    candidate: &Point3,
    active_planes: [Option<usize>; 3],
    planes: &[Plane3],
    policy: PredicatePolicy,
) -> Option<PredicateOutcome<HalfspaceFeasibilityReport>> {
    match point_satisfies_halfspaces(candidate, planes, policy) {
        PredicateOutcome::Decided {
            value: true,
            certainty,
            stage,
        } => Some(PredicateOutcome::decided(
            HalfspaceFeasibilityReport::feasible(candidate.clone(), active_planes),
            certainty,
            stage,
        )),
        PredicateOutcome::Decided { value: false, .. } => None,
        PredicateOutcome::Unknown { needed, stage } => {
            Some(PredicateOutcome::unknown(needed, stage))
        }
    }
}

fn point_satisfies_halfspaces(
    point: &Point3,
    planes: &[Plane3],
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    let mut certainty = Certainty::Exact;
    let mut stage = Escalation::Structural;
    for plane in planes {
        match classify_point_plane_with_policy(point, plane, policy) {
            PredicateOutcome::Decided {
                value,
                certainty: value_certainty,
                stage: value_stage,
            } => {
                absorb_trace(&mut certainty, &mut stage, value_certainty, value_stage);
                if value == PlaneSide::Above {
                    return PredicateOutcome::decided(false, certainty, stage);
                }
            }
            PredicateOutcome::Unknown { needed, stage } => {
                return PredicateOutcome::unknown(needed, stage);
            }
        }
    }
    PredicateOutcome::decided(true, certainty, stage)
}

fn closest_point_on_plane(plane: &Plane3, policy: PredicatePolicy) -> CandidateConstruction {
    let norm2 = dot(&plane.normal, &plane.normal);
    match sign_of(&norm2, policy) {
        PredicateOutcome::Decided {
            value: Sign::Zero, ..
        } => return CandidateConstruction::Skip,
        PredicateOutcome::Decided { .. } => {}
        PredicateOutcome::Unknown { needed, stage } => {
            return CandidateConstruction::Unknown { needed, stage };
        }
    }

    let scale = match div(&neg(&plane.offset), &norm2) {
        Some(value) => value,
        None => return CandidateConstruction::Skip,
    };
    CandidateConstruction::Point(scale_point(&plane.normal, &scale))
}

fn closest_point_on_plane_pair(
    first: &Plane3,
    second: &Plane3,
    policy: PredicatePolicy,
) -> CandidateConstruction {
    let a = dot(&first.normal, &first.normal);
    let b = dot(&first.normal, &second.normal);
    let c = dot(&second.normal, &second.normal);
    let det = sub(&mul(&a, &c), &mul(&b, &b));
    match sign_of(&det, policy) {
        PredicateOutcome::Decided {
            value: Sign::Zero, ..
        } => return CandidateConstruction::Skip,
        PredicateOutcome::Decided { .. } => {}
        PredicateOutcome::Unknown { needed, stage } => {
            return CandidateConstruction::Unknown { needed, stage };
        }
    }

    let rhs_first = neg(&first.offset);
    let rhs_second = neg(&second.offset);
    let lambda_first_num = sub(&mul(&rhs_first, &c), &mul(&b, &rhs_second));
    let lambda_second_num = sub(&mul(&a, &rhs_second), &mul(&b, &rhs_first));
    let lambda_first = match div(&lambda_first_num, &det) {
        Some(value) => value,
        None => return CandidateConstruction::Skip,
    };
    let lambda_second = match div(&lambda_second_num, &det) {
        Some(value) => value,
        None => return CandidateConstruction::Skip,
    };

    CandidateConstruction::Point(add_points(
        &scale_point(&first.normal, &lambda_first),
        &scale_point(&second.normal, &lambda_second),
    ))
}

fn sign_of(value: &Real, policy: PredicatePolicy) -> PredicateOutcome<Sign> {
    resolve_real_sign(
        value,
        policy,
        || None,
        || None,
        RefinementNeed::RealRefinement,
    )
}

fn dot(left: &Point3, right: &Point3) -> Real {
    add(
        &add(&mul(&left.x, &right.x), &mul(&left.y, &right.y)),
        &mul(&left.z, &right.z),
    )
}

fn scale_point(point: &Point3, scale: &Real) -> Point3 {
    Point3::new(
        mul(&point.x, scale),
        mul(&point.y, scale),
        mul(&point.z, scale),
    )
}

fn add_points(left: &Point3, right: &Point3) -> Point3 {
    Point3::new(
        add(&left.x, &right.x),
        add(&left.y, &right.y),
        add(&left.z, &right.z),
    )
}

fn div(numerator: &Real, denominator: &Real) -> Option<Real> {
    (numerator / denominator).ok()
}

fn neg(value: &Real) -> Real {
    sub(&Real::from(0), value)
}

fn add(left: &Real, right: &Real) -> Real {
    add_ref(left, right)
}

fn mul(left: &Real, right: &Real) -> Real {
    mul_ref(left, right)
}

fn sub(left: &Real, right: &Real) -> Real {
    sub_ref(left, right)
}

fn absorb_trace(
    certainty: &mut Certainty,
    stage: &mut Escalation,
    value_certainty: Certainty,
    value_stage: Escalation,
) {
    *certainty = max_certainty(*certainty, value_certainty);
    *stage = max_stage(*stage, value_stage);
}

fn max_certainty(left: Certainty, right: Certainty) -> Certainty {
    match (left, right) {
        (Certainty::Filtered, _) | (_, Certainty::Filtered) => Certainty::Filtered,
        _ => Certainty::Exact,
    }
}

fn max_stage(left: Escalation, right: Escalation) -> Escalation {
    if stage_rank(left) >= stage_rank(right) {
        left
    } else {
        right
    }
}

fn stage_rank(stage: Escalation) -> u8 {
    match stage {
        Escalation::Structural => 0,
        Escalation::Filter => 1,
        Escalation::Exact => 2,
        Escalation::Refined => 3,
        Escalation::Undecided => 4,
    }
}
