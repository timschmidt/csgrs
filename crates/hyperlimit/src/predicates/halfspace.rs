//! Active-set feasibility search for closed 3D inequalities `n . x + d <= 0`.
//!
//! With exact decisions, a nonempty closed polyhedron has a point closest to
//! the origin; in three dimensions a basis of at most three active planes
//! suffices to recover such a candidate. This motivates testing the origin,
//! plane projections, pair-line projections, and finite triple intersections.
//! The implementation returns the first feasible candidate in its search order,
//! not necessarily the globally closest point. Constant inequalities with zero
//! normal are included rather than rejected as invalid planes.
//!
//! After candidate rejection, the search attempts dependencies involving up to
//! four planes. Nonnegative multipliers with zero weighted normal and positive
//! weighted offset give a Farkas contradiction. Exact Real expressions retain
//! the data, but their decision qualification belongs to the public outcome.
//! The classifier uses strict-first composite replay; direct report/certificate
//! validation has different receipt and proof-presence behavior, documented on
//! those methods. A certificate-free infeasible report is accepted structurally
//! without proving the supplied system infeasible.
//!
//! This is finite subset enumeration, not a configured total-work or memory
//! bound. Scalar uncertainty can stop the search before later candidates, and
//! arithmetic cost depends on the represented Real data. No practical uniform
//! completion bound follows from the fixed ambient dimension.

use crate::predicate::PredicatePolicy;
use crate::predicates::decision::max_stage;
use hyperreal::Real;

use crate::classify::{HalfspaceFeasibility, PlaneSide};
use crate::geometry::{HomogeneousPoint3, Plane3, Point3, intersect_three_planes};
use crate::plane::classify_point_plane_without_filter_with_policy;
use crate::predicate::{Certainty, Escalation, PredicateOutcome, RefinementNeed, Sign};
use crate::predicates::order::reciprocal_real_with_policy;
use crate::real::{add_ref, mul_ref, sub_ref};
use crate::resolve::{resolve_composite_policy, resolve_real_sign_direct};

/// Feasibility label with optional construction data, qualified by its outer outcome.
///
/// Public fields do not retain certainty, source identity, or the rejected active
/// sets used by search. Constructors arrange fields without proving their claims.
/// Infeasible reports may lack a Farkas certificate; their validation then accepts
/// only a field shape, not a geometric proof.
#[derive(Clone, Debug, PartialEq)]
pub struct HalfspaceFeasibilityReport {
    /// Feasible or infeasible label, without its enclosing outcome receipt.
    pub status: HalfspaceFeasibility,
    /// Candidate coordinates retained for Feasible; the constructor does not
    /// verify membership, and qualification belongs to the producing outcome.
    pub witness: Option<Point3>,
    /// Optional contradiction data for Infeasible; absence is permitted even
    /// though no standalone infeasibility proof is then retained.
    pub infeasibility_certificate: Option<HalfspaceInfeasibilityCertificate>,
    /// Active plane indices used to construct [`Self::witness`].
    ///
    /// Empty entries mean the witness came from a lower-dimensional active set:
    /// no active planes for the origin, one active plane for a plane
    /// projection, two for a line projection, or three for a vertex.
    pub active_planes: [Option<usize>; 3],
}

impl HalfspaceFeasibilityReport {
    /// Package a witness and active indices as Feasible without checking any planes.
    ///
    /// No bounds, distinctness, activity, or witness-incidence checks occur here.
    pub fn feasible(witness: Point3, active_planes: [Option<usize>; 3]) -> Self {
        Self {
            status: HalfspaceFeasibility::Feasible,
            witness: Some(witness),
            infeasibility_certificate: None,
            active_planes,
        }
    }

    /// Package Infeasible with an optional certificate and no witness or active set.
    ///
    /// `None` is permitted and does not independently establish infeasibility.
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

    /// Inspect only the status label, without requiring a present or valid witness.
    pub const fn is_feasible(&self) -> bool {
        matches!(self.status, HalfspaceFeasibility::Feasible)
    }

    /// Check a feasible witness, an available certificate, or the certificate-free shape.
    ///
    /// Feasible with a witness replays all halfspace memberships and retains that
    /// query's receipt; active indices and any attached infeasibility certificate are
    /// ignored. Infeasible without a witness delegates to the certificate when present.
    /// Without a certificate it returns true/Exact/Structural without inspecting the
    /// planes, even for a feasible system. Other status/witness shapes return false.
    /// Active-set provenance and source identity are not validated. Direct certificate
    /// replay has the receipt limitations of
    /// [`HalfspaceInfeasibilityCertificate::validate_against_planes`].
    pub fn validate_against_planes(
        &self,
        planes: &[Plane3],
        policy: PredicatePolicy,
    ) -> PredicateOutcome<bool> {
        match (&self.status, &self.witness) {
            (HalfspaceFeasibility::Feasible, Some(witness)) => {
                point_satisfies_halfspaces(witness, planes, policy)
            }
            (HalfspaceFeasibility::Infeasible, None) => match &self.infeasibility_certificate {
                Some(certificate) => certificate.validate_against_planes(planes, policy),
                None => PredicateOutcome::decided(true, Certainty::Exact, Escalation::Structural),
            },
            _ => PredicateOutcome::decided(false, Certainty::Exact, Escalation::Structural),
        }
    }
}

/// Retained Farkas data intended to witness an inconsistent halfspace system.
///
/// For `n_i . x + d_i <= 0`, nonnegative multipliers with
/// `sum(lambda_i * n_i) = 0` and `sum(lambda_i * d_i) > 0` yield a contradiction
/// when the inequalities are summed. Four slots retain such a dependency in 3D.
/// Fields are mutable Real expressions, not self-validating proof objects; neither
/// child receipts nor source identity are stored. See the validation method's
/// policy and structural-equality limitations before treating replay as proof.
#[derive(Clone, Debug, PartialEq)]
pub struct HalfspaceInfeasibilityCertificate {
    /// Up to four source-plane indices; absent slots should have zero multipliers.
    /// Repeated indices are permitted by validation.
    pub active_planes: [Option<usize>; 4],
    /// Intended nonnegative multipliers corresponding to [`Self::active_planes`].
    /// Public mutation can violate their signs or the absent-slot condition.
    pub multipliers: [Real; 4],
    /// Stored weighted offset expression, intended to be positive. Validation
    /// compares it structurally with a freshly accumulated expression.
    pub offset_sum: Real,
}

impl HalfspaceInfeasibilityCertificate {
    /// Replay multiplier signs, weighted sums, and positive offset under `policy`.
    ///
    /// Visit four slots in order. Reject negative multipliers, out-of-range indices,
    /// nonzero multipliers in absent slots, and the absence of a positive multiplier.
    /// Repeated indices are allowed. Recompute weighted sums, require structural Real
    /// equality with the stored offset sum, then decide the normal is zero and offset
    /// is positive. Structural inequality can reject mathematically equal expressions.
    /// Unknown propagates its need/stage at the first unresolved decision.
    ///
    /// All decided results are labeled Exact/Exact while child sign certainty and
    /// stages are discarded. This method has no strict-first composite wrapper;
    /// under an approximate policy its Exact label is therefore not independently a
    /// certificate of exact validation. It also cannot certify the qualification or
    /// source identity of an earlier construction.
    pub fn validate_against_planes(
        &self,
        planes: &[Plane3],
        policy: PredicatePolicy,
    ) -> PredicateOutcome<bool> {
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

/// Search a finite list of closed halfspaces for a witness or infeasible report.
///
/// Test origin, single-plane projections, pair-line projections, then triple-plane
/// intersections in input/lexicographic subset order. Recheck each candidate against
/// all planes in input order; a decided violation advances search, but the first
/// Unknown aborts that pass. Zero denominators skip constructions. The empty system
/// accepts the origin with Exact/Structural certainty/stage. Zero-normal planes
/// represent constant inequalities; normals need not be normalized.
///
/// After all candidates are rejected or skipped, attempt one-, two-, three-, and
/// four-plane Farkas dependencies. A completed search with no certificate still
/// returns Infeasible with no certificate, relying on the preceding candidate
/// search rather than publishing replayable rejected-candidate evidence. Unknown
/// in certificate search returns Unknown instead of an infeasible report.
///
/// APPROXIMATE_512 first runs the full composition under STRICT. A decided strict
/// outcome is retained; only strict uncertainty triggers replay, whose decided
/// result is conservatively Approximate/Refined. Other policies evaluate directly.
/// Keep the enclosing receipt: the report and construction helpers discard parts
/// of the child evidence. Subset enumeration and nested scalar work have no shared
/// total-work budget or cancellation contract here.
pub fn classify_halfspace_feasibility3_with_policy(
    planes: &[Plane3],
    policy: PredicatePolicy,
) -> PredicateOutcome<HalfspaceFeasibilityReport> {
    resolve_composite_policy(policy, |policy| {
        classify_halfspace_feasibility3_impl(planes, policy)
    })
}

/// Run one active-set pass, preserving candidate and constraint order.
///
/// Candidate Unknown stops immediately rather than searching for a later witness.
/// Only after all origin/plane/pair/triple candidates fail does certificate search
/// run. Construction receipts are not aggregated here; the public wrapper owns
/// strict-first replay and aggregate approximation qualification.
fn classify_halfspace_feasibility3_impl(
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
                let candidate = match affine_point_with_policy(&homogeneous, policy) {
                    CandidateConstruction::Point(point) => point,
                    CandidateConstruction::Skip => continue,
                    CandidateConstruction::Unknown { needed, stage } => {
                        return PredicateOutcome::unknown(needed, stage);
                    }
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

    infeasible_outcome_from_certificate_search(find_farkas_certificate(planes, policy))
}

/// Convert completed certificate search to an Exact/Exact infeasible report.
///
/// NotFound also yields Infeasible, with no certificate; callers must already have
/// completed active-set rejection. Unknown preserves its need/stage. This helper
/// neither proves candidate completeness nor validates the supplied certificate.
fn infeasible_outcome_from_certificate_search(
    search: CertificateSearch,
) -> PredicateOutcome<HalfspaceFeasibilityReport> {
    let certificate = match search {
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

/// Internal certificate-search result, distinct from feasibility of the system.
enum CertificateSearch {
    /// A dependency passed the local contradiction checks under the supplied policy.
    Found(
        /// Retained certificate data; no child receipt is attached.
        Box<HalfspaceInfeasibilityCertificate>,
    ),
    /// No candidate passed; this is not itself a feasibility decision.
    NotFound,
    /// A required decision remained unresolved; no completed result is supplied.
    Unknown {
        /// Retained child request or an Unsupported construction blocker.
        needed: RefinementNeed,
        /// Escalation stage accompanying the unresolved decision.
        stage: Escalation,
    },
}

/// Enumerate constant, pair, triple, then quadruple contradiction candidates.
///
/// Visit subsets in increasing input-index order, returning on the first accepted
/// dependency or Unknown. Zero-normal constant candidates use multiplier one.
/// Pair/triple helpers nominate dependencies; quadruples use signed 3x3 minors.
/// Every candidate passes normal-sum and positive-offset checks. NotFound means
/// only that this search completed without an accepted certificate; this helper
/// alone does not assert feasibility or infeasibility.
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

/// Candidate normal multipliers or an unresolved/unsuccessful dependency search.
///
/// Found is not yet an infeasibility certificate: offset positivity is separate,
/// and the pair helper has not even checked its full normal sum.
enum DependencySearch {
    /// A multiplier candidate was found, with validation depending on its producer.
    Found(
        /// Four coefficients in the selected normals' order.
        [Real; 4],
    ),
    /// The helper found no usable dependency in its search.
    NotFound,
    /// A required decision remained unresolved; no completed result is supplied.
    Unknown {
        /// Retained child request or an Unsupported construction blocker.
        needed: RefinementNeed,
        /// Escalation stage accompanying the unresolved decision.
        stage: Escalation,
    },
}

/// Nominate `[b, -a, 0, 0]` from the first axis where both normals decide nonzero.
///
/// Query first then second normal on x, y, z, stopping at the first Unknown.
/// Axes with a decided zero are skipped. This cancels one coordinate only;
/// nonnegative orientation and the full weighted normal must be checked later.
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

/// Try YZ, XZ, then XY 2x2 minors as a three-normal dependency.
///
/// For each projection, orient multipliers to a nonnegative ray and check the full
/// 3D weighted sum. The fourth multiplier is zero. Unknown stops immediately;
/// a rejected orientation or nonzero sum advances to the next projection.
/// Offset positivity is not checked here, and successful child receipts are lost.
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

/// Construct alternating 3x3 minors of four normal columns.
///
/// The signs are `+,-,+,-`, omitting each normal in turn. These cofactor expressions
/// annihilate the 3x4 normal matrix algebraically, but can all vanish at lower rank.
/// No sign, nontriviality, offset, or predicate qualification is established here.
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

/// Orient a candidate ray, then check zero normal sum and positive offset sum.
///
/// Out-of-range selected indices reject the candidate. Absent slots contribute
/// zero normals and offsets, without requiring their multipliers to be zero;
/// search callers supply that extra condition, while public certificate validation
/// checks it. Successful child receipts are discarded. Unknown stops at the first
/// undecided orientation/sum/offset check, preserving its need/stage.
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

/// Outcome of selecting a nonnegative, nontrivial multiplier ray under policy.
enum MultiplierOrientation {
    /// The sign decisions selected a nonnegative, nontrivial ray.
    Oriented(
        /// Original or jointly negated coefficients, without sign receipts.
        [Real; 4],
    ),
    /// Signs were mixed or all were decided zero.
    NotConePositive,
    /// A required decision remained unresolved; no completed result is supplied.
    Unknown {
        /// Retained child request or an Unsupported construction blocker.
        needed: RefinementNeed,
        /// Escalation stage accompanying the unresolved decision.
        stage: Escalation,
    },
}

/// Keep a nonnegative nonzero ray or negate a nonpositive nonzero ray.
///
/// Scan all four signs in order; even after seeing mixed signs, a later Unknown
/// wins over rejection. Mixed signs and all-zero decisions reject the ray.
/// Successful sign receipts are discarded, so approximate zeros remain qualified
/// only through the enclosing public classifier's composite replay.
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

/// Candidate coordinates, a skipped singular construction, or unresolved arithmetic.
enum CandidateConstruction {
    /// Coordinates were constructed but full-system membership is not established.
    Point(
        /// Candidate Cartesian point, without the construction receipt.
        Point3,
    ),
    /// A singular or policy-decided zero denominator omitted this construction.
    Skip,
    /// A required decision remained unresolved; no completed result is supplied.
    Unknown {
        /// Retained child request or an Unsupported construction blocker.
        needed: RefinementNeed,
        /// Escalation stage accompanying the unresolved decision.
        stage: Escalation,
    },
}

/// Replay a candidate against all constraints and package an accepted witness.
///
/// A decided violation returns None so search continues. Both acceptance and
/// Unknown return Some, causing the caller to stop. The feasibility receipt is
/// retained, but candidate-construction receipts and active-index validity are not.
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

/// Test `n . point + d <= 0` in source order using the unfiltered plane predicate.
///
/// Join the weakest certainty and latest stage of decided memberships, returning
/// false at the first Above. Unknown returns its own need/stage and stops before
/// later constraints. Empty input returns true/Exact/Structural.
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

/// Construct the origin projection `-d * n / (n . n)` when division is admitted.
///
/// DivideByZero skips the plane, including a policy-decided terminal zero.
/// Other arithmetic errors become Unsupported/Undecided; Unknown retains its
/// need/stage. A successful reciprocal receipt is discarded, and the resulting
/// point still needs replay against the full constraint system.
fn closest_point_on_plane(plane: &Plane3, policy: PredicatePolicy) -> CandidateConstruction {
    let norm2 = dot(&plane.normal, &plane.normal);
    let reciprocal = match reciprocal_real_with_policy(&norm2, policy) {
        Err(hyperreal::Problem::DivideByZero) => return CandidateConstruction::Skip,
        Err(_) => {
            return CandidateConstruction::Unknown {
                needed: RefinementNeed::Unsupported,
                stage: Escalation::Undecided,
            };
        }
        Ok(PredicateOutcome::Decided { value, .. }) => value,
        Ok(PredicateOutcome::Unknown { needed, stage }) => {
            return CandidateConstruction::Unknown { needed, stage };
        }
    };

    let scale = mul(&neg(&plane.offset), &reciprocal);
    CandidateConstruction::Point(scale_point(&plane.normal, &scale))
}

/// Solve the 2x2 normal Gram system for the origin projection onto a plane pair.
///
/// With `a=n1.n1`, `b=n1.n2`, `c=n2.n2`, divide by `a*c-b*b` and combine the two
/// normal vectors with the resulting coefficients. A decided zero determinant
/// skips the construction; other errors become Unsupported/Undecided. Unknown
/// propagates, while successful reciprocal qualification is discarded. The point
/// is only a candidate until all halfspaces accept it.
fn closest_point_on_plane_pair(
    first: &Plane3,
    second: &Plane3,
    policy: PredicatePolicy,
) -> CandidateConstruction {
    let a = dot(&first.normal, &first.normal);
    let b = dot(&first.normal, &second.normal);
    let c = dot(&second.normal, &second.normal);
    let det = sub(&mul(&a, &c), &mul(&b, &b));
    let reciprocal = match reciprocal_real_with_policy(&det, policy) {
        Err(hyperreal::Problem::DivideByZero) => return CandidateConstruction::Skip,
        Err(_) => {
            return CandidateConstruction::Unknown {
                needed: RefinementNeed::Unsupported,
                stage: Escalation::Undecided,
            };
        }
        Ok(PredicateOutcome::Decided { value, .. }) => value,
        Ok(PredicateOutcome::Unknown { needed, stage }) => {
            return CandidateConstruction::Unknown { needed, stage };
        }
    };

    let rhs_first = neg(&first.offset);
    let rhs_second = neg(&second.offset);
    let lambda_first_num = sub(&mul(&rhs_first, &c), &mul(&b, &rhs_second));
    let lambda_second_num = sub(&mul(&a, &rhs_second), &mul(&b, &rhs_first));
    let lambda_first = mul(&lambda_first_num, &reciprocal);
    let lambda_second = mul(&lambda_second_num, &reciprocal);

    CandidateConstruction::Point(add_points(
        &scale_point(&first.normal, &lambda_first),
        &scale_point(&second.normal, &lambda_second),
    ))
}

/// Recover candidate Cartesian coordinates from a homogeneous triple intersection.
///
/// Known zero weight skips; known nonzero weight tries direct affine conversion.
/// Unknown weight first uses a policy-controlled reciprocal. DivideByZero skips,
/// other errors become Unsupported/Undecided, and success scales x/y/z without
/// retaining its receipt. If reciprocal resolution is Unknown, retry direct affine
/// conversion: success yields a point, DivideByZero skips, and other failure keeps
/// the original Unknown. No full-system feasibility is established here.
fn affine_point_with_policy(
    point: &HomogeneousPoint3,
    policy: PredicatePolicy,
) -> CandidateConstruction {
    match point.w.zero_status() {
        hyperreal::ZeroKnowledge::Zero => return CandidateConstruction::Skip,
        hyperreal::ZeroKnowledge::NonZero => {
            return match point.to_affine_point() {
                Ok(point) => CandidateConstruction::Point(point),
                Err(_) => CandidateConstruction::Unknown {
                    needed: RefinementNeed::Unsupported,
                    stage: Escalation::Undecided,
                },
            };
        }
        hyperreal::ZeroKnowledge::Unknown => {}
    }

    match reciprocal_real_with_policy(&point.w, policy) {
        Err(hyperreal::Problem::DivideByZero) => CandidateConstruction::Skip,
        Err(_) => CandidateConstruction::Unknown {
            needed: RefinementNeed::Unsupported,
            stage: Escalation::Undecided,
        },
        Ok(PredicateOutcome::Decided {
            value: reciprocal, ..
        }) => CandidateConstruction::Point(Point3::new(
            mul(&point.x, &reciprocal),
            mul(&point.y, &reciprocal),
            mul(&point.z, &reciprocal),
        )),
        Ok(PredicateOutcome::Unknown { needed, stage }) => match point.to_affine_point() {
            Ok(point) => CandidateConstruction::Point(point),
            Err(hyperreal::Problem::DivideByZero) => CandidateConstruction::Skip,
            Err(_) => CandidateConstruction::Unknown { needed, stage },
        },
    }
}

/// Accumulate four scaled normal vectors in slot order as exact Real expressions.
fn weighted_normal_sum(normals: &[&Point3; 4], multipliers: &[Real; 4]) -> Point3 {
    let mut sum = Point3::new(Real::from(0), Real::from(0), Real::from(0));
    for index in 0..4 {
        sum = add_points(&sum, &scale_point(normals[index], &multipliers[index]));
    }
    sum
}

/// Query x, y, z signs in order, stopping at the first nonzero or Unknown.
///
/// Decided booleans are labeled Exact/Exact without joining child receipts. Under
/// approximate policy a decided zero need not be a proof of mathematical zero.
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

/// Construct `ax*by - ay*bx`, the signed determinant of two planar vectors.
fn det2(ax: &Real, ay: &Real, bx: &Real, by: &Real) -> Real {
    sub(&mul(ax, by), &mul(ay, bx))
}

/// Construct `a . (b cross c)` using three 2x2 cofactors, without deciding its sign.
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

/// Resolve a scalar sign directly under policy, using RealRefinement for uncertainty.
fn sign_of(value: &Real, policy: PredicatePolicy) -> PredicateOutcome<Sign> {
    resolve_real_sign_direct(value, policy, RefinementNeed::RealRefinement)
}

/// Construct the exact three-coordinate Euclidean dot-product expression.
fn dot(left: &Point3, right: &Point3) -> Real {
    add(
        &add(&mul(&left.x, &right.x), &mul(&left.y, &right.y)),
        &mul(&left.z, &right.z),
    )
}

/// Multiply all three coordinates by one Real scalar without deciding its sign.
fn scale_point(point: &Point3, scale: &Real) -> Point3 {
    Point3::new(
        mul(&point.x, scale),
        mul(&point.y, scale),
        mul(&point.z, scale),
    )
}

/// Add coordinate vectors componentwise in construction-expression order.
fn add_points(left: &Point3, right: &Point3) -> Point3 {
    Point3::new(
        add(&left.x, &right.x),
        add(&left.y, &right.y),
        add(&left.z, &right.z),
    )
}

/// Construct exact additive negation as zero minus the supplied value.
fn neg(value: &Real) -> Real {
    sub(&Real::from(0), value)
}

/// Delegate borrowed scalar addition to the shared Real construction helper.
fn add(left: &Real, right: &Real) -> Real {
    add_ref(left, right)
}

/// Delegate borrowed scalar multiplication to the shared Real construction helper.
fn mul(left: &Real, right: &Real) -> Real {
    mul_ref(left, right)
}

/// Delegate borrowed scalar subtraction to the shared Real construction helper.
fn sub(left: &Real, right: &Real) -> Real {
    sub_ref(left, right)
}

/// Join a consumed membership receipt into the current certainty and stage.
fn absorb_trace(
    certainty: &mut Certainty,
    stage: &mut Escalation,
    value_certainty: Certainty,
    value_stage: Escalation,
) {
    *certainty = max_certainty(*certainty, value_certainty);
    *stage = max_stage(*stage, value_stage);
}

/// Retain the weakest consumed certainty: Approximate before Filtered before Exact.
fn max_certainty(left: Certainty, right: Certainty) -> Certainty {
    match (left, right) {
        (Certainty::Approximate, _) | (_, Certainty::Approximate) => Certainty::Approximate,
        (Certainty::Filtered, _) | (_, Certainty::Filtered) => Certainty::Filtered,
        _ => Certainty::Exact,
    }
}

/// Finite controls for active sets, dependency construction, certificate replay, and Unknown.
#[cfg(test)]
mod tests {
    use super::*;
    use crate::predicates::decision::stage_rank;
    use crate::test_support::exact_normal_positive;

    /// Approximation-permitted policy for rational controls and selected helper calls.
    const APPROX: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

    /// Construct a spatial fixture point from supplied exact Real coordinates.
    fn point(x: Real, y: Real, z: Real) -> Point3 {
        Point3::new(x, y, z)
    }

    /// Construct a spatial fixture point from exact integer coordinates.
    fn p3(x: i32, y: i32, z: i32) -> Point3 {
        point(x.into(), y.into(), z.into())
    }

    /// Construct an integer-coefficient fixture inequality `n . x + offset <= 0`.
    fn plane(nx: i32, ny: i32, nz: i32, offset: i32) -> Plane3 {
        Plane3::new(p3(nx, ny, nz), offset.into())
    }

    /// Assert the normalized sine/cosine identity and return a harder angle-addition zero.
    fn terminal_zero() -> Real {
        let sine = Real::e().sin();
        let cosine = Real::e().cos();
        let original = &sine * &sine + &cosine * &cosine - Real::one();
        // Preserve the original identity as a newly certified proof case.
        assert_eq!(
            original.exact_rational_normal_form(),
            Some(hyperreal::Rational::zero())
        );
        // A separate angle-addition identity retains the terminal-policy control.
        crate::test_support::terminally_unresolved_zero()
    }

    /// Check strict Unknown for unresolved multiplier, normal, and offset components.
    #[test]
    fn unresolved_farkas_certificate_components_propagate() {
        let unresolved_multiplier = HalfspaceInfeasibilityCertificate {
            active_planes: [None; 4],
            multipliers: [terminal_zero(), 0.into(), 0.into(), 0.into()],
            offset_sum: 0.into(),
        };
        assert!(matches!(
            unresolved_multiplier.validate_against_planes(&[], PredicatePolicy::STRICT),
            PredicateOutcome::Unknown { .. }
        ));

        let unresolved_normal = [Plane3::new(
            point(terminal_zero(), 0.into(), 0.into()),
            1.into(),
        )];
        let certificate = HalfspaceInfeasibilityCertificate {
            active_planes: [Some(0), None, None, None],
            multipliers: [1.into(), 0.into(), 0.into(), 0.into()],
            offset_sum: 1.into(),
        };
        assert!(matches!(
            certificate.validate_against_planes(&unresolved_normal, PredicatePolicy::STRICT),
            PredicateOutcome::Unknown { .. }
        ));

        let unresolved_offset = [Plane3::new(p3(0, 0, 0), terminal_zero())];
        let certificate = HalfspaceInfeasibilityCertificate {
            offset_sum: terminal_zero(),
            ..certificate
        };
        assert!(matches!(
            certificate.validate_against_planes(&unresolved_offset, PredicatePolicy::STRICT),
            PredicateOutcome::Unknown { .. }
        ));
    }

    /// Check positive/negative ray handling, mixed/all-zero rejection, and strict Unknown.
    #[test]
    fn multiplier_orientation_covers_both_rays_mixed_zero_and_unknown() {
        assert!(matches!(
            orient_nonnegative_multipliers([1.into(), 2.into(), 0.into(), 0.into()], APPROX),
            MultiplierOrientation::Oriented(_)
        ));
        let negative =
            orient_nonnegative_multipliers([(-1).into(), (-2).into(), 0.into(), 0.into()], APPROX);
        assert!(matches!(
            negative,
            MultiplierOrientation::Oriented(values)
                if values[0] == Real::from(1) && values[1] == Real::from(2)
        ));
        assert!(matches!(
            orient_nonnegative_multipliers([1.into(), (-1).into(), 0.into(), 0.into()], APPROX,),
            MultiplierOrientation::NotConePositive
        ));
        assert!(matches!(
            orient_nonnegative_multipliers([0.into(), 0.into(), 0.into(), 0.into()], APPROX),
            MultiplierOrientation::NotConePositive
        ));
        assert!(matches!(
            orient_nonnegative_multipliers(
                [terminal_zero(), 0.into(), 0.into(), 0.into()],
                PredicatePolicy::STRICT,
            ),
            MultiplierOrientation::Unknown { .. }
        ));
    }

    /// Check rational projections, singular skips, and symbolic uncertainty branches.
    ///
    /// The deep-positive controls also compare constructed coordinates with exact two
    /// and three. These are value/branch assertions, not dispatch or cost measurements.
    #[test]
    fn active_set_candidate_construction_covers_point_skip_and_unknown() {
        assert!(matches!(
            closest_point_on_plane(&plane(0, 0, 0, 1), APPROX),
            CandidateConstruction::Skip
        ));
        assert!(matches!(
            closest_point_on_plane(&plane(1, 0, 0, -2), APPROX),
            CandidateConstruction::Point(value) if value == p3(2, 0, 0)
        ));
        let unresolved_plane = Plane3::new(point(terminal_zero(), 0.into(), 0.into()), 1.into());
        assert!(matches!(
            closest_point_on_plane(&unresolved_plane, PredicatePolicy::STRICT),
            CandidateConstruction::Unknown { .. }
        ));
        assert!(matches!(
            classify_halfspace_feasibility3_impl(
                core::slice::from_ref(&unresolved_plane),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        assert!(matches!(
            closest_point_on_plane_pair(&plane(1, 0, 0, 0), &plane(2, 0, 0, 1), APPROX),
            CandidateConstruction::Skip
        ));
        assert!(matches!(
            closest_point_on_plane_pair(&plane(1, 0, 0, -2), &plane(0, 1, 0, -3), APPROX),
            CandidateConstruction::Point(value) if value == p3(2, 3, 0)
        ));

        let deep_positive = exact_normal_positive();
        let deep_plane = Plane3::new(
            point(deep_positive.clone(), Real::zero(), Real::zero()),
            -(deep_positive.clone() * Real::from(2)),
        );
        let CandidateConstruction::Point(deep_point) =
            closest_point_on_plane(&deep_plane, PredicatePolicy::STRICT)
        else {
            panic!("policy-certified norm should construct its closest point");
        };
        assert_eq!(
            crate::compare_reals(&deep_point.x, &Real::from(2), PredicatePolicy::STRICT).value(),
            Some(core::cmp::Ordering::Equal)
        );

        let first_deep = deep_plane;
        let second_deep = Plane3::new(
            point(Real::zero(), deep_positive.clone(), Real::zero()),
            -(deep_positive * Real::from(3)),
        );
        let CandidateConstruction::Point(deep_point) =
            closest_point_on_plane_pair(&first_deep, &second_deep, PredicatePolicy::STRICT)
        else {
            panic!("policy-certified Gram determinant should construct its closest point");
        };
        assert_eq!(
            crate::compare_reals(&deep_point.x, &Real::from(2), PredicatePolicy::STRICT).value(),
            Some(core::cmp::Ordering::Equal)
        );
        assert_eq!(
            crate::compare_reals(&deep_point.y, &Real::from(3), PredicatePolicy::STRICT).value(),
            Some(core::cmp::Ordering::Equal)
        );
        let almost_parallel = Plane3::new(point(1.into(), terminal_zero(), 0.into()), 0.into());
        assert!(matches!(
            closest_point_on_plane_pair(
                &plane(1, 0, 0, 0),
                &almost_parallel,
                PredicatePolicy::STRICT,
            ),
            CandidateConstruction::Unknown { .. }
        ));

        let first = plane(1, 0, 0, 1);
        let second = Plane3::new(point((-1).into(), terminal_zero(), 0.into()), 1.into());
        assert!(matches!(
            classify_halfspace_feasibility3_impl(&[first, second], PredicatePolicy::STRICT),
            PredicateOutcome::Unknown { .. }
        ));

        assert!(matches!(
            affine_point_with_policy(
                &HomogeneousPoint3::new(1.into(), 2.into(), 3.into(), 0.into()),
                PredicatePolicy::STRICT,
            ),
            CandidateConstruction::Skip
        ));
        assert!(matches!(
            affine_point_with_policy(
                &HomogeneousPoint3::new(1.into(), 2.into(), 3.into(), terminal_zero()),
                PredicatePolicy::STRICT,
            ),
            CandidateConstruction::Unknown { .. }
        ));
    }

    /// Require a strict feasible triple witness at `(1/3,3,4)` with the expected indices.
    ///
    /// Also replay membership; the assertions do not count proof-cache reuse or dispatch.
    #[test]
    fn active_set_reuses_policy_certified_nonzero_triple_weight() {
        let deep_positive = exact_normal_positive();
        let x_target = (Real::from(1) / Real::from(3)).unwrap();
        let planes = [
            Plane3::new(
                point(-deep_positive.clone(), Real::zero(), Real::zero()),
                deep_positive.clone() * &x_target,
            ),
            Plane3::new(
                point(Real::zero(), -deep_positive.clone(), Real::zero()),
                deep_positive.clone() * Real::from(3),
            ),
            Plane3::new(
                point(Real::zero(), Real::zero(), -deep_positive.clone()),
                deep_positive * Real::from(4),
            ),
        ];

        let report = classify_halfspace_feasibility3_with_policy(&planes, PredicatePolicy::STRICT)
            .value()
            .expect("the exact-normal triple weight should construct a feasible vertex");

        assert_eq!(report.status, HalfspaceFeasibility::Feasible);
        assert_eq!(report.active_planes, [Some(0), Some(1), Some(2)]);
        let witness = report.witness.as_ref().expect("feasible witness");
        for (coordinate, expected) in [
            (&witness.x, &x_target),
            (&witness.y, &Real::from(3)),
            (&witness.z, &Real::from(4)),
        ] {
            assert_eq!(
                crate::compare_reals(coordinate, expected, PredicatePolicy::STRICT).value(),
                Some(core::cmp::Ordering::Equal)
            );
        }
        assert_eq!(
            report
                .validate_against_planes(&planes, PredicatePolicy::STRICT)
                .value(),
            Some(true)
        );
    }

    /// Check selected dependency successes, misses, exact cofactors, and symbolic Unknown.
    #[test]
    fn dependency_search_covers_pair_triple_and_four_plane_bases() {
        assert!(matches!(
            pair_dependency(&p3(1, 0, 0), &p3(-1, 0, 0), APPROX),
            DependencySearch::Found(_)
        ));
        assert!(matches!(
            pair_dependency(&p3(0, 0, 0), &p3(0, 0, 0), APPROX),
            DependencySearch::NotFound
        ));
        assert!(matches!(
            pair_dependency(
                &point(terminal_zero(), 0.into(), 0.into()),
                &p3(1, 0, 0),
                PredicatePolicy::STRICT,
            ),
            DependencySearch::Unknown { .. }
        ));
        assert!(matches!(
            pair_dependency(
                &p3(1, 0, 0),
                &point(terminal_zero(), 0.into(), 0.into()),
                PredicatePolicy::STRICT,
            ),
            DependencySearch::Unknown { .. }
        ));

        assert!(matches!(
            triple_dependency(&p3(1, 0, 0), &p3(0, 1, 0), &p3(-1, -1, 0), APPROX),
            DependencySearch::Found(_)
        ));
        assert!(matches!(
            triple_dependency(&p3(1, 0, 0), &p3(0, 1, 0), &p3(0, 0, 1), APPROX),
            DependencySearch::NotFound
        ));

        let dependency =
            four_plane_dependency(&p3(1, 0, 0), &p3(0, 1, 0), &p3(0, 0, 1), &p3(-1, -1, -1));
        assert_eq!(
            dependency,
            [
                Real::from(-1),
                Real::from(-1),
                Real::from(-1),
                Real::from(-1),
            ]
        );

        let pi_sine = Real::pi().sin();
        let pi_cosine = Real::pi().cos();
        let independent_terminal_zero = &pi_sine * &pi_sine + &pi_cosine * &pi_cosine - Real::one();
        let symbolic_dependency = triple_dependency(
            &point(terminal_zero(), 1.into(), 0.into()),
            &p3(0, 0, 1),
            &point(neg(&independent_terminal_zero), (-1).into(), (-1).into()),
            PredicatePolicy::STRICT,
        );
        assert!(matches!(
            symbolic_dependency,
            DependencySearch::Unknown { .. }
        ));

        let unknown_multiplier = triple_dependency(
            &p3(1, 0, 0),
            &p3(0, 1, 0),
            &point(0.into(), terminal_zero(), 1.into()),
            PredicatePolicy::STRICT,
        );
        assert!(matches!(
            unknown_multiplier,
            DependencySearch::Unknown { .. }
        ));
    }

    /// Exercise constant/pair/triple/quadruple search controls and selected Unknown paths.
    ///
    /// Found and NotFound both map to Infeasible at the packaging boundary; Unknown
    /// stays Unknown. These fixtures do not enumerate all search or uncertainty paths.
    #[test]
    fn farkas_search_exercises_every_active_set_and_uncertainty_exit() {
        assert!(matches!(
            find_farkas_certificate(
                &[Plane3::new(p3(0, 0, 0), terminal_zero())],
                PredicatePolicy::STRICT,
            ),
            CertificateSearch::Unknown { .. }
        ));
        assert!(matches!(
            find_farkas_certificate(&[plane(0, 0, 0, -1)], PredicatePolicy::STRICT),
            CertificateSearch::NotFound
        ));

        assert!(matches!(
            find_farkas_certificate(
                &[
                    Plane3::new(p3(1, 0, 0), terminal_zero()),
                    plane(-1, 0, 0, 0),
                ],
                PredicatePolicy::STRICT,
            ),
            CertificateSearch::Unknown { .. }
        ));
        assert!(matches!(
            find_farkas_certificate(
                &[
                    Plane3::new(point(1.into(), terminal_zero(), 0.into()), 0.into()),
                    plane(0, 1, 0, 0),
                ],
                PredicatePolicy::STRICT,
            ),
            CertificateSearch::Unknown { .. }
        ));

        let triple = [plane(1, 0, 0, 1), plane(0, 1, 0, 1), plane(-1, -1, 0, 1)];
        assert!(matches!(
            find_farkas_certificate(&triple, PredicatePolicy::STRICT),
            CertificateSearch::Found(_)
        ));
        let triple_unknown = [
            Plane3::new(p3(1, 0, 0), terminal_zero()),
            plane(0, 1, 0, 0),
            plane(-1, -1, 0, 0),
        ];
        assert!(matches!(
            find_farkas_certificate(&triple_unknown, PredicatePolicy::STRICT),
            CertificateSearch::Unknown { .. }
        ));
        let triple_not_found = [plane(1, 0, 0, -1), plane(0, 1, 0, -1), plane(-1, -1, 0, -1)];
        assert!(matches!(
            find_farkas_certificate(&triple_not_found, PredicatePolicy::STRICT),
            CertificateSearch::NotFound
        ));
        let triple_dependency_unknown = [
            Plane3::new(point(1.into(), 0.into(), terminal_zero()), 0.into()),
            plane(1, 1, 0, 0),
            plane(1, 2, 0, 0),
        ];
        assert!(matches!(
            find_farkas_certificate(&triple_dependency_unknown, PredicatePolicy::STRICT),
            CertificateSearch::Unknown { .. }
        ));

        let four = [
            plane(1, 0, 0, 1),
            plane(0, 1, 0, 1),
            plane(0, 0, 1, 1),
            plane(-1, -1, -1, 1),
        ];
        assert!(matches!(
            find_farkas_certificate(&four, PredicatePolicy::STRICT),
            CertificateSearch::Found(_)
        ));
        let four_unknown = [
            Plane3::new(p3(1, 0, 0), terminal_zero()),
            plane(0, 1, 0, 0),
            plane(0, 0, 1, 0),
            plane(-1, -1, -1, 0),
        ];
        assert!(matches!(
            find_farkas_certificate(&four_unknown, PredicatePolicy::STRICT),
            CertificateSearch::Unknown { .. }
        ));
        let four_not_found = [
            plane(1, 0, 0, -1),
            plane(0, 1, 0, -1),
            plane(0, 0, 1, -1),
            plane(-1, -1, -1, -1),
        ];
        assert!(matches!(
            find_farkas_certificate(&four_not_found, PredicatePolicy::STRICT),
            CertificateSearch::NotFound
        ));

        let found = find_farkas_certificate(&[plane(0, 0, 0, 1)], PredicatePolicy::STRICT);
        assert_eq!(
            infeasible_outcome_from_certificate_search(found)
                .value()
                .map(|report| report.status),
            Some(HalfspaceFeasibility::Infeasible)
        );
        assert_eq!(
            infeasible_outcome_from_certificate_search(CertificateSearch::NotFound)
                .value()
                .map(|report| report.status),
            Some(HalfspaceFeasibility::Infeasible)
        );
        assert!(matches!(
            infeasible_outcome_from_certificate_search(CertificateSearch::Unknown {
                needed: RefinementNeed::RealRefinement,
                stage: Escalation::Undecided,
            }),
            PredicateOutcome::Unknown { .. }
        ));
    }

    /// Check accepted contradiction data, bad indices/nonzero normals, and unresolved parts.
    #[test]
    fn farkas_dependency_acceptance_checks_indices_normal_and_offset() {
        let planes = [plane(0, 0, 0, 1)];
        assert!(matches!(
            accept_farkas_dependency(
                &planes,
                [Some(0), None, None, None],
                [1.into(), 0.into(), 0.into(), 0.into()],
                APPROX,
            ),
            CertificateSearch::Found(_)
        ));
        assert!(matches!(
            accept_farkas_dependency(
                &planes,
                [Some(9), None, None, None],
                [1.into(), 0.into(), 0.into(), 0.into()],
                APPROX,
            ),
            CertificateSearch::NotFound
        ));
        assert!(matches!(
            accept_farkas_dependency(
                &[plane(1, 0, 0, 1)],
                [Some(0), None, None, None],
                [1.into(), 0.into(), 0.into(), 0.into()],
                APPROX,
            ),
            CertificateSearch::NotFound
        ));
        assert!(matches!(
            accept_farkas_dependency(
                &planes,
                [Some(0), None, None, None],
                [terminal_zero(), 0.into(), 0.into(), 0.into()],
                PredicatePolicy::STRICT,
            ),
            CertificateSearch::Unknown { .. }
        ));

        let unresolved_normal = [Plane3::new(
            point(terminal_zero(), 0.into(), 0.into()),
            1.into(),
        )];
        assert!(matches!(
            accept_farkas_dependency(
                &unresolved_normal,
                [Some(0), None, None, None],
                [1.into(), 0.into(), 0.into(), 0.into()],
                PredicatePolicy::STRICT,
            ),
            CertificateSearch::Unknown { .. }
        ));
        let unresolved_offset = [Plane3::new(p3(0, 0, 0), terminal_zero())];
        assert!(matches!(
            accept_farkas_dependency(
                &unresolved_offset,
                [Some(0), None, None, None],
                [1.into(), 0.into(), 0.into(), 0.into()],
                PredicatePolicy::STRICT,
            ),
            CertificateSearch::Unknown { .. }
        ));
    }

    /// Check selected Unknown propagation, empty certificate search, and receipt ranks.
    ///
    /// The assertions cover finite cases, not total decision of arbitrary Real inputs.
    #[test]
    fn candidate_acceptance_zero_checks_and_trace_ranking_are_total() {
        let unresolved = Plane3::new(p3(0, 0, 0), terminal_zero());
        assert!(matches!(
            point_satisfies_halfspaces(&p3(0, 0, 0), &[unresolved], PredicatePolicy::STRICT),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            accept_candidate(
                &p3(0, 0, 0),
                [None; 3],
                &[Plane3::new(p3(0, 0, 0), terminal_zero())],
                PredicatePolicy::STRICT,
            ),
            Some(PredicateOutcome::Unknown { .. })
        ));
        assert!(matches!(
            point_zero(
                &point(terminal_zero(), 0.into(), 0.into()),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        assert!(matches!(
            find_farkas_certificate(&[], APPROX),
            CertificateSearch::NotFound
        ));
        assert!(matches!(
            find_farkas_certificate(
                &[Plane3::new(
                    point(terminal_zero(), 0.into(), 0.into()),
                    0.into(),
                )],
                PredicatePolicy::STRICT,
            ),
            CertificateSearch::Unknown { .. }
        ));

        assert_eq!(
            max_certainty(Certainty::Exact, Certainty::Filtered),
            Certainty::Filtered
        );
        assert_eq!(
            max_certainty(Certainty::Filtered, Certainty::Approximate),
            Certainty::Approximate
        );
        assert_eq!(stage_rank(Escalation::Structural), 0);
        assert_eq!(stage_rank(Escalation::Filter), 1);
        assert_eq!(stage_rank(Escalation::Exact), 2);
        assert_eq!(stage_rank(Escalation::Refined), 3);
        assert_eq!(stage_rank(Escalation::Undecided), 4);
        assert_eq!(
            max_stage(Escalation::Filter, Escalation::Exact),
            Escalation::Exact
        );
    }
}
