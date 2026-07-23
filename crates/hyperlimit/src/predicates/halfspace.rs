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
//! fixed-dimension LP viewpoint. Infeasible outcomes can carry a Farkas
//! certificate: a
//! nonnegative linear combination of halfspace inequalities whose normal sum is
//! zero and whose offset sum is strictly positive. Candidates and certificates
//! are built over `Real` and replayed by exact predicates.

use crate::predicate::PredicatePolicy;
use hyperreal::Real;

use crate::classify::{HalfspaceFeasibility, PlaneSide};
use crate::geometry::{Plane3, Plane3Facts, Point3, intersect_three_planes};
use crate::plane::classify_point_plane_without_filter_with_policy;
use crate::predicate::{Certainty, Escalation, PredicateOutcome, RefinementNeed, Sign};
use crate::real::{add_ref, mul_ref, sub_ref};
use crate::resolve::resolve_real_sign;

/// Reusable borrowed 3D halfspace system.
///
/// A prepared halfspace system stores the source plane slice and cached
/// [`Plane3Facts`] for diagnostics and scheduling. Feasibility still replays
/// through exact predicates; cached facts are not themselves feasibility
/// certificates. They retain object structure and preparation metadata while
/// exact predicates decide combinatorial truth.
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedHalfspaceSystem3<'a> {
    planes: &'a [Plane3],
    plane_facts: Vec<Plane3Facts>,
}

impl<'a> PreparedHalfspaceSystem3<'a> {
    /// Prepare a borrowed plane-halfspace system.
    pub fn new(planes: &'a [Plane3]) -> Self {
        crate::trace_dispatch!("hyperlimit", "prepared_halfspace_system3", "new");
        let plane_facts = planes.iter().map(Plane3::structural_facts).collect();
        Self {
            planes,
            plane_facts,
        }
    }

    /// Return the borrowed source planes.
    pub fn planes(&self) -> &'a [Plane3] {
        self.planes
    }

    /// Return cached per-plane structural facts.
    pub fn plane_facts(&self) -> &[Plane3Facts] {
        &self.plane_facts
    }

    /// Classify feasibility using the default predicate policy.
    pub fn classify_feasibility(&self) -> PredicateOutcome<HalfspaceFeasibilityReport> {
        self.classify_feasibility_with_policy(PredicatePolicy)
    }

    /// Classify feasibility using an explicit predicate policy.
    pub(crate) fn classify_feasibility_with_policy(
        &self,
        policy: PredicatePolicy,
    ) -> PredicateOutcome<HalfspaceFeasibilityReport> {
        classify_halfspace_feasibility3_with_policy(self.planes, policy)
    }
}

/// Feasibility report for a closed 3D halfspace system.
#[derive(Clone, Debug, PartialEq)]
pub struct HalfspaceFeasibilityReport {
    /// Feasible or infeasible status.
    pub status: HalfspaceFeasibility,
    /// Exact point satisfying every halfspace when feasible.
    pub witness: Option<Point3>,
    /// Exact Farkas certificate for infeasible systems, when found.
    pub infeasibility_certificate: Option<HalfspaceInfeasibilityCertificate>,
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
            infeasibility_certificate: None,
            active_planes,
        }
    }

    /// Construct an infeasible report.
    pub const fn infeasible(
        infeasibility_certificate: Option<HalfspaceInfeasibilityCertificate>,
    ) -> Self {
        Self {
            status: HalfspaceFeasibility::Infeasible,
            witness: None,
            infeasibility_certificate,
            active_planes: [None, None, None],
        }
    }

    /// Return whether the report found a feasible witness.
    pub const fn is_feasible(&self) -> bool {
        matches!(self.status, HalfspaceFeasibility::Feasible)
    }

    /// Replay the witness against the source planes.
    ///
    /// Feasible reports replay their witness against every halfspace.
    /// Infeasible reports replay their Farkas certificate when present; an
    /// older or deliberately compact infeasible report without a certificate is
    /// structurally valid but not proof-producing.
    pub fn validate_against_planes(&self, planes: &[Plane3]) -> PredicateOutcome<bool> {
        let policy = PredicatePolicy;
        match (&self.status, &self.witness) {
            (HalfspaceFeasibility::Feasible, Some(witness)) => {
                point_satisfies_halfspaces(witness, planes, policy)
            }
            (HalfspaceFeasibility::Infeasible, None) => match &self.infeasibility_certificate {
                Some(certificate) => certificate.validate_against_planes(planes),
                None => PredicateOutcome::decided(true, Certainty::Exact, Escalation::Structural),
            },
            _ => PredicateOutcome::decided(false, Certainty::Exact, Escalation::Structural),
        }
    }
}

/// Exact Farkas certificate for an infeasible 3D halfspace system.
///
/// For halfspaces `n_i . x + d_i <= 0`, nonnegative multipliers with
/// `sum(lambda_i n_i) = 0` and `sum(lambda_i d_i) > 0` prove infeasibility:
/// multiplying all inequalities by `lambda_i` and summing gives
/// `0 . x + positive <= 0`, a contradiction. This certificate keeps that proof
/// as exact `Real` data so callers can replay it without depending on a solver
/// implementation. The theorem is Farkas' lemma.
#[derive(Clone, Debug, PartialEq)]
pub struct HalfspaceInfeasibilityCertificate {
    /// Plane indices participating in the certificate.
    pub active_planes: [Option<usize>; 4],
    /// Nonnegative multipliers corresponding to [`Self::active_planes`].
    pub multipliers: [Real; 4],
    /// Exact positive offset sum `sum(lambda_i * plane_i.offset)`.
    pub offset_sum: Real,
}

impl HalfspaceInfeasibilityCertificate {
    /// Replay the Farkas certificate against a source plane list.
    pub fn validate_against_planes(&self, planes: &[Plane3]) -> PredicateOutcome<bool> {
        let policy = PredicatePolicy;
        let mut normal_sum = Point3::new(Real::from(0), Real::from(0), Real::from(0));
        let mut offset_sum = Real::from(0);
        let mut saw_positive_multiplier = false;

        for slot in 0..4 {
            let multiplier = &self.multipliers[slot];
            let sign = match sign_of(multiplier, policy) {
                PredicateOutcome::Decided { value, .. } => value,
                PredicateOutcome::Unknown { needed, stage } => {
                    return PredicateOutcome::unknown(needed, stage);
                }
            };
            if sign == Sign::Negative {
                return PredicateOutcome::decided(false, Certainty::Exact, Escalation::Exact);
            }
            saw_positive_multiplier |= sign == Sign::Positive;

            match self.active_planes[slot] {
                Some(index) if index < planes.len() => {
                    let plane = &planes[index];
                    normal_sum = add_points(&normal_sum, &scale_point(&plane.normal, multiplier));
                    offset_sum = add(&offset_sum, &mul(multiplier, &plane.offset));
                }
                Some(_) => {
                    return PredicateOutcome::decided(false, Certainty::Exact, Escalation::Exact);
                }
                None => {
                    if sign != Sign::Zero {
                        return PredicateOutcome::decided(
                            false,
                            Certainty::Exact,
                            Escalation::Exact,
                        );
                    }
                }
            }
        }

        if !saw_positive_multiplier {
            return PredicateOutcome::decided(false, Certainty::Exact, Escalation::Exact);
        }
        if offset_sum != self.offset_sum {
            return PredicateOutcome::decided(false, Certainty::Exact, Escalation::Exact);
        }
        match point_zero(&normal_sum, policy) {
            PredicateOutcome::Decided { value: true, .. } => {}
            PredicateOutcome::Decided { value: false, .. } => {
                return PredicateOutcome::decided(false, Certainty::Exact, Escalation::Exact);
            }
            PredicateOutcome::Unknown { needed, stage } => {
                return PredicateOutcome::unknown(needed, stage);
            }
        }
        match sign_of(&offset_sum, policy) {
            PredicateOutcome::Decided {
                value: Sign::Positive,
                ..
            } => PredicateOutcome::decided(true, Certainty::Exact, Escalation::Exact),
            PredicateOutcome::Decided { .. } => {
                PredicateOutcome::decided(false, Certainty::Exact, Escalation::Exact)
            }
            PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
        }
    }
}

/// Decide feasibility of `normal . point + offset <= 0` halfspaces.
pub fn classify_halfspace_feasibility3(
    planes: &[Plane3],
) -> PredicateOutcome<HalfspaceFeasibilityReport> {
    classify_halfspace_feasibility3_with_policy(planes, PredicatePolicy)
}

/// Decide feasibility of 3D halfspaces with an explicit predicate policy.
///
/// The search checks the origin, closest points on every plane, closest points
/// on every nonparallel plane pair, and every finite triple-plane vertex. Each
/// candidate is accepted only after all halfspace predicates certify
/// `Below | On`. Infeasibility is reported only when every active-set candidate
/// and every replay comparison is exactly decided under the supplied policy.
pub(crate) fn classify_halfspace_feasibility3_with_policy(
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

    let certificate = match find_farkas_certificate(planes, policy) {
        CertificateSearch::Found(certificate) => Some(*certificate),
        CertificateSearch::NotFound => None,
        CertificateSearch::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    };

    PredicateOutcome::decided(
        HalfspaceFeasibilityReport::infeasible(certificate),
        Certainty::Exact,
        Escalation::Exact,
    )
}

enum CertificateSearch {
    Found(Box<HalfspaceInfeasibilityCertificate>),
    NotFound,
    Unknown {
        needed: RefinementNeed,
        stage: Escalation,
    },
}

fn find_farkas_certificate(planes: &[Plane3], policy: PredicatePolicy) -> CertificateSearch {
    crate::trace_dispatch!("hyperlimit", "halfspace_feasibility3", "farkas-certificate");

    for (index, plane) in planes.iter().enumerate() {
        match point_zero(&plane.normal, policy) {
            PredicateOutcome::Decided { value: true, .. } => {
                match accept_farkas_dependency(
                    planes,
                    [Some(index), None, None, None],
                    [Real::from(1), Real::from(0), Real::from(0), Real::from(0)],
                    policy,
                ) {
                    CertificateSearch::Found(certificate) => {
                        return CertificateSearch::Found(certificate);
                    }
                    CertificateSearch::Unknown { needed, stage } => {
                        return CertificateSearch::Unknown { needed, stage };
                    }
                    CertificateSearch::NotFound => {}
                }
            }
            PredicateOutcome::Decided { value: false, .. } => {}
            PredicateOutcome::Unknown { needed, stage } => {
                return CertificateSearch::Unknown { needed, stage };
            }
        }
    }

    for first in 0..planes.len() {
        for second in first + 1..planes.len() {
            match pair_dependency(&planes[first].normal, &planes[second].normal, policy) {
                DependencySearch::Found(multipliers) => {
                    match accept_farkas_dependency(
                        planes,
                        [Some(first), Some(second), None, None],
                        multipliers,
                        policy,
                    ) {
                        CertificateSearch::Found(certificate) => {
                            return CertificateSearch::Found(certificate);
                        }
                        CertificateSearch::Unknown { needed, stage } => {
                            return CertificateSearch::Unknown { needed, stage };
                        }
                        CertificateSearch::NotFound => {}
                    }
                }
                DependencySearch::NotFound => {}
                DependencySearch::Unknown { needed, stage } => {
                    return CertificateSearch::Unknown { needed, stage };
                }
            }
        }
    }

    for first in 0..planes.len() {
        for second in first + 1..planes.len() {
            for third in second + 1..planes.len() {
                match triple_dependency(
                    &planes[first].normal,
                    &planes[second].normal,
                    &planes[third].normal,
                    policy,
                ) {
                    DependencySearch::Found(multipliers) => {
                        match accept_farkas_dependency(
                            planes,
                            [Some(first), Some(second), Some(third), None],
                            multipliers,
                            policy,
                        ) {
                            CertificateSearch::Found(certificate) => {
                                return CertificateSearch::Found(certificate);
                            }
                            CertificateSearch::Unknown { needed, stage } => {
                                return CertificateSearch::Unknown { needed, stage };
                            }
                            CertificateSearch::NotFound => {}
                        }
                    }
                    DependencySearch::NotFound => {}
                    DependencySearch::Unknown { needed, stage } => {
                        return CertificateSearch::Unknown { needed, stage };
                    }
                }
            }
        }
    }

    for first in 0..planes.len() {
        for second in first + 1..planes.len() {
            for third in second + 1..planes.len() {
                for fourth in third + 1..planes.len() {
                    let multipliers = four_plane_dependency(
                        &planes[first].normal,
                        &planes[second].normal,
                        &planes[third].normal,
                        &planes[fourth].normal,
                    );
                    match accept_farkas_dependency(
                        planes,
                        [Some(first), Some(second), Some(third), Some(fourth)],
                        multipliers,
                        policy,
                    ) {
                        CertificateSearch::Found(certificate) => {
                            return CertificateSearch::Found(certificate);
                        }
                        CertificateSearch::Unknown { needed, stage } => {
                            return CertificateSearch::Unknown { needed, stage };
                        }
                        CertificateSearch::NotFound => {}
                    }
                }
            }
        }
    }

    CertificateSearch::NotFound
}

enum DependencySearch {
    Found([Real; 4]),
    NotFound,
    Unknown {
        needed: RefinementNeed,
        stage: Escalation,
    },
}

fn pair_dependency(first: &Point3, second: &Point3, policy: PredicatePolicy) -> DependencySearch {
    for (a, b) in [
        (&first.x, &second.x),
        (&first.y, &second.y),
        (&first.z, &second.z),
    ] {
        let a_sign = match sign_of(a, policy) {
            PredicateOutcome::Decided { value, .. } => value,
            PredicateOutcome::Unknown { needed, stage } => {
                return DependencySearch::Unknown { needed, stage };
            }
        };
        let b_sign = match sign_of(b, policy) {
            PredicateOutcome::Decided { value, .. } => value,
            PredicateOutcome::Unknown { needed, stage } => {
                return DependencySearch::Unknown { needed, stage };
            }
        };
        if a_sign == Sign::Zero || b_sign == Sign::Zero {
            continue;
        }
        let multipliers = [b.clone(), neg(a), Real::from(0), Real::from(0)];
        return DependencySearch::Found(multipliers);
    }
    DependencySearch::NotFound
}

fn triple_dependency(
    first: &Point3,
    second: &Point3,
    third: &Point3,
    policy: PredicatePolicy,
) -> DependencySearch {
    for projection in 0..3 {
        let multipliers = match projection {
            0 => [
                det2(&second.y, &second.z, &third.y, &third.z),
                det2(&third.y, &third.z, &first.y, &first.z),
                det2(&first.y, &first.z, &second.y, &second.z),
                Real::from(0),
            ],
            1 => [
                det2(&second.x, &second.z, &third.x, &third.z),
                det2(&third.x, &third.z, &first.x, &first.z),
                det2(&first.x, &first.z, &second.x, &second.z),
                Real::from(0),
            ],
            _ => [
                det2(&second.x, &second.y, &third.x, &third.y),
                det2(&third.x, &third.y, &first.x, &first.y),
                det2(&first.x, &first.y, &second.x, &second.y),
                Real::from(0),
            ],
        };
        match orient_nonnegative_multipliers(multipliers, policy) {
            MultiplierOrientation::Oriented(oriented) => {
                let zero = Point3::new(0.into(), 0.into(), 0.into());
                let normal_sum = weighted_normal_sum(&[first, second, third, &zero], &oriented);
                match point_zero(&normal_sum, policy) {
                    PredicateOutcome::Decided { value: true, .. } => {
                        return DependencySearch::Found(oriented);
                    }
                    PredicateOutcome::Decided { value: false, .. } => {}
                    PredicateOutcome::Unknown { needed, stage } => {
                        return DependencySearch::Unknown { needed, stage };
                    }
                }
            }
            MultiplierOrientation::NotConePositive => {}
            MultiplierOrientation::Unknown { needed, stage } => {
                return DependencySearch::Unknown { needed, stage };
            }
        }
    }
    DependencySearch::NotFound
}

fn four_plane_dependency(
    first: &Point3,
    second: &Point3,
    third: &Point3,
    fourth: &Point3,
) -> [Real; 4] {
    [
        det3(second, third, fourth),
        neg(&det3(first, third, fourth)),
        det3(first, second, fourth),
        neg(&det3(first, second, third)),
    ]
}

fn accept_farkas_dependency(
    planes: &[Plane3],
    active_planes: [Option<usize>; 4],
    multipliers: [Real; 4],
    policy: PredicatePolicy,
) -> CertificateSearch {
    let multipliers = match orient_nonnegative_multipliers(multipliers, policy) {
        MultiplierOrientation::Oriented(multipliers) => multipliers,
        MultiplierOrientation::NotConePositive => return CertificateSearch::NotFound,
        MultiplierOrientation::Unknown { needed, stage } => {
            return CertificateSearch::Unknown { needed, stage };
        }
    };

    let mut offset_sum = Real::from(0);
    let zero = Point3::new(0.into(), 0.into(), 0.into());
    let mut normals = [&zero, &zero, &zero, &zero];
    for slot in 0..4 {
        match active_planes[slot] {
            Some(index) if index < planes.len() => {
                normals[slot] = &planes[index].normal;
                offset_sum = add(&offset_sum, &mul(&multipliers[slot], &planes[index].offset));
            }
            Some(_) => return CertificateSearch::NotFound,
            None => {}
        }
    }

    let normal_sum = weighted_normal_sum(&normals, &multipliers);
    match point_zero(&normal_sum, policy) {
        PredicateOutcome::Decided { value: true, .. } => {}
        PredicateOutcome::Decided { value: false, .. } => return CertificateSearch::NotFound,
        PredicateOutcome::Unknown { needed, stage } => {
            return CertificateSearch::Unknown { needed, stage };
        }
    }
    match sign_of(&offset_sum, policy) {
        PredicateOutcome::Decided {
            value: Sign::Positive,
            ..
        } => CertificateSearch::Found(Box::new(HalfspaceInfeasibilityCertificate {
            active_planes,
            multipliers,
            offset_sum,
        })),
        PredicateOutcome::Decided { .. } => CertificateSearch::NotFound,
        PredicateOutcome::Unknown { needed, stage } => CertificateSearch::Unknown { needed, stage },
    }
}

enum MultiplierOrientation {
    Oriented([Real; 4]),
    NotConePositive,
    Unknown {
        needed: RefinementNeed,
        stage: Escalation,
    },
}

fn orient_nonnegative_multipliers(
    multipliers: [Real; 4],
    policy: PredicatePolicy,
) -> MultiplierOrientation {
    let mut positive = false;
    let mut negative = false;
    for multiplier in &multipliers {
        match sign_of(multiplier, policy) {
            PredicateOutcome::Decided {
                value: Sign::Positive,
                ..
            } => positive = true,
            PredicateOutcome::Decided {
                value: Sign::Negative,
                ..
            } => negative = true,
            PredicateOutcome::Decided {
                value: Sign::Zero, ..
            } => {}
            PredicateOutcome::Unknown { needed, stage } => {
                return MultiplierOrientation::Unknown { needed, stage };
            }
        }
    }

    match (positive, negative) {
        (true, false) => MultiplierOrientation::Oriented(multipliers),
        (false, true) => MultiplierOrientation::Oriented([
            neg(&multipliers[0]),
            neg(&multipliers[1]),
            neg(&multipliers[2]),
            neg(&multipliers[3]),
        ]),
        _ => MultiplierOrientation::NotConePositive,
    }
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
        match classify_point_plane_without_filter_with_policy(point, plane, policy) {
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

fn weighted_normal_sum(normals: &[&Point3; 4], multipliers: &[Real; 4]) -> Point3 {
    let mut sum = Point3::new(Real::from(0), Real::from(0), Real::from(0));
    for index in 0..4 {
        sum = add_points(&sum, &scale_point(normals[index], &multipliers[index]));
    }
    sum
}

fn point_zero(point: &Point3, policy: PredicatePolicy) -> PredicateOutcome<bool> {
    for coordinate in [&point.x, &point.y, &point.z] {
        match sign_of(coordinate, policy) {
            PredicateOutcome::Decided {
                value: Sign::Zero, ..
            } => {}
            PredicateOutcome::Decided { .. } => {
                return PredicateOutcome::decided(false, Certainty::Exact, Escalation::Exact);
            }
            PredicateOutcome::Unknown { needed, stage } => {
                return PredicateOutcome::unknown(needed, stage);
            }
        }
    }
    PredicateOutcome::decided(true, Certainty::Exact, Escalation::Exact)
}

fn det2(ax: &Real, ay: &Real, bx: &Real, by: &Real) -> Real {
    sub(&mul(ax, by), &mul(ay, bx))
}

fn det3(a: &Point3, b: &Point3, c: &Point3) -> Real {
    let by_cz = mul(&b.y, &c.z);
    let bz_cy = mul(&b.z, &c.y);
    let bz_cx = mul(&b.z, &c.x);
    let bx_cz = mul(&b.x, &c.z);
    let bx_cy = mul(&b.x, &c.y);
    let by_cx = mul(&b.y, &c.x);
    let x_cofactor = sub(&by_cz, &bz_cy);
    let y_cofactor = sub(&bz_cx, &bx_cz);
    let z_cofactor = sub(&bx_cy, &by_cx);
    add(
        &add(&mul(&a.x, &x_cofactor), &mul(&a.y, &y_cofactor)),
        &mul(&a.z, &z_cofactor),
    )
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
